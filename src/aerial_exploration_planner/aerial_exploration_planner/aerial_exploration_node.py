import json
import math
from heapq import heappop, heappush

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from .grid_model import GridSpec, grid_to_world, ground_to_occupied_voxels, make_dense50_ground_footprint, world_to_grid


class AerialExplorationNode(Node):
    def __init__(self):
        super().__init__("aerial_exploration_node")
        self.declare_parameter("exploration.plan_period_sec", 0.5)
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.declare_parameter("aerial_corridor_mode", True)
        self.declare_parameter("aerial_corridor_min_z", 0.8)
        self.declare_parameter("aerial_corridor_max_z", 2.2)
        self.declare_parameter("aerial_corridor_default_z", 1.4)
        self.declare_parameter("aerial_corridor_clearance", 0.25)
        self.declare_parameter("avoid_flying_above_obstacle_top", True)
        self.declare_parameter("max_above_obstacle_margin", 0.2)
        self.declare_parameter("exploration.goal_reached_radius", 0.8)
        self.declare_parameter("exploration.sensor_mapping_patrol_every_n_goals", 3)
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
        )
        self.corridor_min_z = float(self.get_parameter("aerial_corridor_min_z").value)
        self.corridor_max_z = float(self.get_parameter("aerial_corridor_max_z").value)
        self.corridor_default_z = self._clamp_z(float(self.get_parameter("aerial_corridor_default_z").value))
        self.corridor_clearance = float(self.get_parameter("aerial_corridor_clearance").value)
        self.goal_reached_radius = float(self.get_parameter("exploration.goal_reached_radius").value)
        self.sensor_patrol_every = int(self.get_parameter("exploration.sensor_mapping_patrol_every_n_goals").value)
        self.ground_occupied = make_dense50_ground_footprint(self.spec)
        self.occupied_voxels = ground_to_occupied_voxels(self.spec, self.ground_occupied)
        self.pose = None
        self.map_state = None
        self.current_goal = None
        self.goal_index = 0
        self.coverage_pub = self.create_publisher(Float32, "/aerial_exploration/coverage", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/aerial_exploration/goal", 10)
        self.path_pub = self.create_publisher(Path, "/aerial_exploration/path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "/aerial_exploration/frontiers", 10)
        self.viewpoint_pub = self.create_publisher(MarkerArray, "/aerial_exploration/viewpoints", 10)
        self.selected_goal_pub = self.create_publisher(Marker, "/aerial_exploration/selected_goal_marker", 10)
        self.coverage_marker_pub = self.create_publisher(Marker, "/aerial_exploration/coverage_marker", 10)
        self.ground_marker_pub = self.create_publisher(Marker, "/aerial_exploration/ground_footprint_marker", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Odometry, "/state_estimation", self.odom_cb, 10)
        self.create_subscription(str_msg(), "/aerial_exploration/map_state", self.map_cb, 10)
        self.timer = self.create_timer(float(self.get_parameter("exploration.plan_period_sec").value), self.tick)
        self.get_logger().info(
            "Pose/map-driven aerial exploration node started "
            f"with corridor z=[{self.corridor_min_z:.2f}, {self.corridor_max_z:.2f}], "
            f"default_z={self.corridor_default_z:.2f}"
        )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def map_cb(self, msg):
        self.map_state = json.loads(msg.data)

    def tick(self):
        if not self.map_state:
            return
        coverage = float(self.map_state.get("observed_coverage", self.map_state.get("coverage", 0.0)))
        synthetic_coverage = float(self.map_state.get("synthetic_coverage", self.map_state.get("coverage", coverage)))
        coverage_msg = Float32()
        coverage_msg.data = coverage
        self.coverage_pub.publish(coverage_msg)
        self.coverage_marker_pub.publish(
            self._text_marker("coverage", 9001, f"observed_coverage={coverage:.3f} synthetic={synthetic_coverage:.3f}", (0.0, -11.0, 3.0))
        )
        gr = float(self.map_state.get("ground_footprint_occupancy_ratio", 0.0))
        self.ground_marker_pub.publish(self._text_marker("ground_footprint", 9002, f"dense50 footprint={gr:.3f}", (0.0, -11.0, 2.5)))
        if self.pose is None or coverage >= self.done_threshold:
            return
        if self.current_goal is None or math.dist(self.pose, self.current_goal) <= self.goal_reached_radius:
            goal = self._select_goal()
            if goal is None:
                return
            self.current_goal = goal
            self.goal_index += 1
            self.goal_pub.publish(self._goal_pose(goal))
        path = self._path_to_goal(self.current_goal)
        self.path_pub.publish(path)
        self.frontier_pub.publish(self._frontier_markers())
        self.viewpoint_pub.publish(self._viewpoint_markers())
        self.selected_goal_pub.publish(self._goal_marker(self.current_goal))

    def _select_goal(self):
        frontiers = self.map_state.get("frontier_cells", [])
        if self.map_state.get("mapping_source") == "sensor_driven_local_simulated_lidar_camera" and self.sensor_patrol_every > 0:
            if self.goal_index % self.sensor_patrol_every == 0:
                return self._nearest_free_lawnmower_goal()
        if not frontiers:
            return self._lawnmower_goal()
        candidates = []
        for cell in frontiers:
            p = self._corridor_point_from_cell(tuple(cell))
            if self.is_point_in_aerial_free_space(*p):
                candidates.append(p)
        if not candidates:
            return self._nearest_free_lawnmower_goal()
        patrol_anchor = self._lawnmower_goal()
        candidates.sort(key=lambda p: (math.dist(p, patrol_anchor), -math.dist(p, self.pose), p[0], p[1], p[2]))
        for offset in range(len(candidates)):
            candidate = candidates[(self.goal_index + offset) % len(candidates)]
            if self.is_segment_collision_free_3d(self.pose, candidate):
                return candidate
        return candidates[min(self.goal_index % max(1, len(candidates)), len(candidates) - 1)]

    def _lawnmower_goal(self):
        cols = [-8.0, -4.0, 0.0, 4.0, 8.0]
        y = 8.0 if self.goal_index % 2 == 0 else -8.0
        return (cols[self.goal_index % len(cols)], y, self.corridor_default_z)

    def _nearest_free_lawnmower_goal(self):
        base = self._lawnmower_goal()
        if self.is_point_in_aerial_free_space(*base):
            return base
        bx, by, _ = world_to_grid(self.spec, base)
        options = []
        for radius in range(1, max(self.spec.x_cells, self.spec.y_cells)):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    ix, iy = bx + dx, by + dy
                    if 0 <= ix < self.spec.x_cells and 0 <= iy < self.spec.y_cells:
                        p = self._corridor_point_from_cell((ix, iy, 0))
                        if self.is_point_in_aerial_free_space(*p):
                            options.append(p)
            if options:
                options.sort(key=lambda p: math.dist(p, base))
                return options[0]
        return base

    def _path_to_goal(self, goal):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        start = self._corridor_point(self.pose or (-9.0, -9.0, self.corridor_default_z))
        goal = self._corridor_point(goal)
        points = self._plan_corridor_path(start, goal)
        if len(points) < 2:
            points = [start, goal]
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _plan_corridor_path(self, start, goal):
        if self.is_segment_collision_free_3d(start, goal):
            return self._sample_segment(start, goal)
        start_idx = self._nearest_free_xy(world_to_grid(self.spec, start)[:2])
        goal_idx = self._nearest_free_xy(world_to_grid(self.spec, goal)[:2])
        if start_idx is None or goal_idx is None:
            return self._sample_segment(start, goal)
        cells = self._astar_xy(start_idx, goal_idx)
        if not cells:
            return self._sample_segment(start, goal)
        waypoints = [(grid_to_world(self.spec, (ix, iy, 0))[0], grid_to_world(self.spec, (ix, iy, 0))[1], self.corridor_default_z) for ix, iy in cells]
        waypoints[0] = start
        waypoints[-1] = goal
        return self._densify_path(waypoints)

    def _astar_xy(self, start, goal):
        frontier = []
        heappush(frontier, (0.0, start))
        came_from = {start: None}
        cost_so_far = {start: 0.0}
        neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
        while frontier:
            _, current = heappop(frontier)
            if current == goal:
                break
            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)
                if not self._is_free_xy(nxt):
                    continue
                new_cost = cost_so_far[current] + 1.0
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                    heappush(frontier, (priority, nxt))
                    came_from[nxt] = current
        if goal not in came_from:
            return []
        cells = []
        current = goal
        while current is not None:
            cells.append(current)
            current = came_from[current]
        cells.reverse()
        return cells

    def _nearest_free_xy(self, idx):
        if self._is_free_xy(idx):
            return idx
        for radius in range(1, max(self.spec.x_cells, self.spec.y_cells)):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    candidate = (idx[0] + dx, idx[1] + dy)
                    if self._is_free_xy(candidate):
                        return candidate
        return None

    def _is_free_xy(self, idx):
        if not (0 <= idx[0] < self.spec.x_cells and 0 <= idx[1] < self.spec.y_cells):
            return False
        point = grid_to_world(self.spec, (idx[0], idx[1], 0))
        return self.is_point_in_aerial_free_space(point[0], point[1], self.corridor_default_z)

    def _sample_segment(self, start, goal):
        steps = max(2, int(math.dist(start, goal) / 0.75) + 1)
        points = []
        for idx in range(steps):
            t = idx / max(1, steps - 1)
            points.append(
                (
                    start[0] + (goal[0] - start[0]) * t,
                    start[1] + (goal[1] - start[1]) * t,
                    self._clamp_z(start[2] + (goal[2] - start[2]) * t),
                )
            )
        return points

    def _densify_path(self, waypoints):
        points = []
        for a, b in zip(waypoints, waypoints[1:]):
            segment = self._sample_segment(a, b)
            if points:
                segment = segment[1:]
            points.extend(segment)
        return points

    def _corridor_point_from_cell(self, cell):
        x, y, _ = grid_to_world(self.spec, (cell[0], cell[1], 0))
        return (x, y, self.corridor_default_z)

    def _corridor_point(self, point):
        return (point[0], point[1], self._clamp_z(point[2]))

    def _clamp_z(self, z):
        return min(self.corridor_max_z, max(self.corridor_min_z, z))

    def is_point_in_obstacle(self, x, y, z):
        idx = world_to_grid(self.spec, (x, y, z))
        return idx in self.occupied_voxels

    def is_point_in_aerial_free_space(self, x, y, z):
        if z < self.corridor_min_z or z > self.corridor_max_z:
            return False
        gx, gy, gz = world_to_grid(self.spec, (x, y, z))
        if not (0 <= gx < self.spec.x_cells and 0 <= gy < self.spec.y_cells):
            return False
        radius = int(math.ceil(self.corridor_clearance / self.spec.resolution))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if math.hypot(dx * self.spec.resolution, dy * self.spec.resolution) > self.corridor_clearance + self.spec.resolution * 0.5:
                    continue
                if (gx + dx, gy + dy, gz) in self.occupied_voxels:
                    return False
        return True

    def is_segment_collision_free_3d(self, a, b):
        steps = max(2, int(math.dist(a, b) / 0.25) + 1)
        for idx in range(steps):
            t = idx / max(1, steps - 1)
            p = (
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                self._clamp_z(a[2] + (b[2] - a[2]) * t),
            )
            if not self.is_point_in_aerial_free_space(*p):
                return False
        return True

    def _goal_pose(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
        msg.pose.orientation.w = 1.0
        return msg

    def _frontier_markers(self):
        arr = MarkerArray()
        marker = self._cube_list("frontiers", 7001, (0.0, 0.6, 1.0, 0.75), 0.35)
        for cell in self.map_state.get("frontier_cells", [])[:80]:
            p = self._corridor_point_from_cell(tuple(cell))
            if not self.is_point_in_aerial_free_space(*p):
                continue
            point = Point()
            point.x, point.y, point.z = p
            marker.points.append(point)
        arr.markers.append(marker)
        return arr

    def _viewpoint_markers(self):
        arr = MarkerArray()
        for idx, cell in enumerate(self.map_state.get("frontier_cells", [])[:12]):
            p = self._corridor_point_from_cell(tuple(cell))
            if not self.is_point_in_aerial_free_space(*p):
                continue
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "candidate_viewpoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = p
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, 1.0, 0.2, 0.82)
            arr.markers.append(marker)
        return arr

    def _goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "selected_goal"
        marker.id = 8001
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.75
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
        return marker

    def _text_marker(self, ns, marker_id, text, xyz):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.45
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 1.0, 1.0, 1.0)
        marker.text = text
        return marker

    def _cube_list(self, ns, marker_id, color, scale):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def str_msg():
    from std_msgs.msg import String

    return String


def main(args=None):
    rclpy.init(args=args)
    node = AerialExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
