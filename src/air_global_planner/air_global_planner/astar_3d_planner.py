import heapq
import math
from itertools import product

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class AStar3DPlanner(Node):
    def __init__(self):
        super().__init__("air_global_planner")
        self._declare_parameters()
        self.start = None
        self.goal = None
        self.obstacle_boxes = []
        self.occupied = set()
        self.grid_dims = self._grid_dims()
        self.neighbors = self._neighbor_offsets()

        self.path_pub = self.create_publisher(Path, "/air/global_path", 10)
        self.status_pub = self.create_publisher(String, "/air/planner_status", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.start_sub = self.create_subscription(PoseStamped, "/air/start", self.start_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/air/goal", self.goal_cb, 10)
        self.obs_sub = self.create_subscription(MarkerArray, "/air/occupancy_markers", self.obstacles_cb, 10)
        self.get_logger().info("3D A* planner ready.")

    def _declare_parameters(self):
        defaults = {
            "map.x_min": -10.0,
            "map.x_max": 10.0,
            "map.y_min": -10.0,
            "map.y_max": 10.0,
            "map.z_min": 0.5,
            "map.z_max": 6.0,
            "map.resolution": 0.5,
            "map.inflation_radius": 0.5,
            "planner.heuristic_weight": 1.0,
            "planner.allow_diagonal": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.x_min = self.get_parameter("map.x_min").value
        self.x_max = self.get_parameter("map.x_max").value
        self.y_min = self.get_parameter("map.y_min").value
        self.y_max = self.get_parameter("map.y_max").value
        self.z_min = self.get_parameter("map.z_min").value
        self.z_max = self.get_parameter("map.z_max").value
        self.resolution = self.get_parameter("map.resolution").value
        self.inflation_radius = self.get_parameter("map.inflation_radius").value
        self.heuristic_weight = self.get_parameter("planner.heuristic_weight").value
        self.allow_diagonal = self.get_parameter("planner.allow_diagonal").value

    def _grid_dims(self):
        return (
            int(round((self.x_max - self.x_min) / self.resolution)) + 1,
            int(round((self.y_max - self.y_min) / self.resolution)) + 1,
            int(round((self.z_max - self.z_min) / self.resolution)) + 1,
        )

    def _neighbor_offsets(self):
        offsets = []
        for dx, dy, dz in product([-1, 0, 1], repeat=3):
            if dx == dy == dz == 0:
                continue
            if not self.allow_diagonal and sum(v != 0 for v in (dx, dy, dz)) > 1:
                continue
            offsets.append((dx, dy, dz))
        return offsets

    def start_cb(self, msg):
        self.start = msg
        self.plan_if_ready()

    def goal_cb(self, msg):
        self.goal = msg
        self.plan_if_ready()

    def obstacles_cb(self, msg):
        boxes = []
        for marker in msg.markers:
            if marker.type != Marker.CUBE or marker.ns != "air_world_obstacles":
                continue
            if marker.id == 0 or marker.pose.position.x > 50.0:
                continue
            boxes.append(marker)
        self.obstacle_boxes = boxes
        self.occupied = self._build_occupied_grid(boxes)
        self.plan_if_ready()

    def _build_occupied_grid(self, boxes):
        occupied = set()
        for marker in boxes:
            cx = marker.pose.position.x
            cy = marker.pose.position.y
            cz = marker.pose.position.z
            sx = marker.scale.x * 0.5 + self.inflation_radius
            sy = marker.scale.y * 0.5 + self.inflation_radius
            sz = marker.scale.z * 0.5 + self.inflation_radius
            min_i = self.world_to_grid((cx - sx, cy - sy, cz - sz), clamp=True)
            max_i = self.world_to_grid((cx + sx, cy + sy, cz + sz), clamp=True)
            for ix in range(min_i[0], max_i[0] + 1):
                for iy in range(min_i[1], max_i[1] + 1):
                    for iz in range(min_i[2], max_i[2] + 1):
                        occupied.add((ix, iy, iz))
        return occupied

    def world_to_grid(self, p, clamp=False):
        ix = int(round((p[0] - self.x_min) / self.resolution))
        iy = int(round((p[1] - self.y_min) / self.resolution))
        iz = int(round((p[2] - self.z_min) / self.resolution))
        if clamp:
            ix = min(max(ix, 0), self.grid_dims[0] - 1)
            iy = min(max(iy, 0), self.grid_dims[1] - 1)
            iz = min(max(iz, 0), self.grid_dims[2] - 1)
        return (ix, iy, iz)

    def grid_to_world(self, idx):
        return (
            self.x_min + idx[0] * self.resolution,
            self.y_min + idx[1] * self.resolution,
            self.z_min + idx[2] * self.resolution,
        )

    def in_bounds(self, idx):
        return all(0 <= idx[i] < self.grid_dims[i] for i in range(3))

    def heuristic(self, a, b):
        return self.heuristic_weight * self.distance(a, b)

    def distance(self, a, b):
        return self.resolution * math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))

    def plan_if_ready(self):
        if self.start is None or self.goal is None or not self.obstacle_boxes:
            return
        start_idx = self.world_to_grid(self._pose_xyz(self.start), clamp=False)
        goal_idx = self.world_to_grid(self._pose_xyz(self.goal), clamp=False)
        if not self.in_bounds(start_idx):
            self.publish_status(f"ERROR: start out of bounds {start_idx}")
            return
        if not self.in_bounds(goal_idx):
            self.publish_status(f"ERROR: goal out of bounds {goal_idx}")
            return
        if start_idx in self.occupied:
            self.publish_status("ERROR: start is inside inflated obstacle")
            return
        if goal_idx in self.occupied:
            self.publish_status("ERROR: goal is inside inflated obstacle")
            return

        path_idx = self.astar(start_idx, goal_idx)
        if not path_idx:
            self.publish_status("ERROR: 3D A* failed to find a collision-free path")
            return
        path_idx = self._simplify_collinear(path_idx)
        path = self._to_path(path_idx)
        self.path_pub.publish(path)
        self.publish_status(
            f"OK: 3D A* path generated with {len(path.poses)} waypoints, z range "
            f"{min(p.pose.position.z for p in path.poses):.2f}-{max(p.pose.position.z for p in path.poses):.2f} m"
        )

    def astar(self, start_idx, goal_idx):
        open_heap = [(self.heuristic(start_idx, goal_idx), 0.0, start_idx)]
        came_from = {}
        g_score = {start_idx: 0.0}
        closed = set()
        while open_heap:
            _, current_g, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            if current == goal_idx:
                return self.reconstruct(came_from, current)
            closed.add(current)
            for off in self.neighbors:
                nb = (current[0] + off[0], current[1] + off[1], current[2] + off[2])
                if not self.in_bounds(nb) or nb in self.occupied or nb in closed:
                    continue
                tentative = current_g + self.distance(current, nb)
                if tentative < g_score.get(nb, float("inf")):
                    came_from[nb] = current
                    g_score[nb] = tentative
                    heapq.heappush(open_heap, (tentative + self.heuristic(nb, goal_idx), tentative, nb))
        return []

    def reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def _simplify_collinear(self, path):
        if len(path) < 3:
            return path
        out = [path[0]]
        prev_dir = None
        for a, b in zip(path, path[1:]):
            direction = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
            if prev_dir is not None and direction != prev_dir:
                out.append(a)
            prev_dir = direction
        out.append(path[-1])
        return out

    def _to_path(self, path_idx):
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for idx in path_idx:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y, p.pose.position.z = self.grid_to_world(idx)
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def _pose_xyz(self, msg):
        return (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def publish_status(self, text):
        status = String()
        status.data = text
        self.status_pub.publish(status)
        self._publish_status_marker(text)
        if text.startswith("ERROR"):
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _publish_status_marker(self, text):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "air_status"
        marker.id = 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -9.5
        marker.pose.position.y = -9.5
        marker.pose.position.z = 6.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.r = 0.1
        marker.color.g = 0.9 if text.startswith("OK") else 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = text
        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = AStar3DPlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
