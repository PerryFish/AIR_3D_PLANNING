import random
from collections import deque
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple
    size: tuple
    color: tuple


class AirWorldProvider(Node):
    def __init__(self):
        super().__init__("air_world_provider")
        self._declare_parameters()
        latched_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.obstacle_pub = self.create_publisher(MarkerArray, "/air/occupancy_markers", 10)
        self.visual_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.status_pub = self.create_publisher(String, "/air/world_status", latched_qos)
        self.status_marker_pub = self.create_publisher(Marker, "/air/world_status_marker", latched_qos)
        self.boxes = []
        self.occupied_cells = set()
        self.status_text = ""
        self._build_world()
        self.timer = self.create_timer(1.0, self.publish_world)
        self.get_logger().info(f"Air 3D world provider started: {self.status_text}")

    def _declare_parameters(self):
        defaults = {
            "frame_id": "map",
            "map.x_min": -10.0,
            "map.x_max": 10.0,
            "map.y_min": -10.0,
            "map.y_max": 10.0,
            "map.z_min": 0.5,
            "map.z_max": 6.0,
            "map.resolution": 0.5,
            "map.inflation_radius": 0.5,
            "world.scenario_type": "deterministic_boxes",
            "world.occupancy_ratio": 0.5,
            "world.random_seed": 42,
            "world.ensure_connectivity": True,
            "world.max_generation_attempts": 100,
            "world.publish_inflated_obstacles": True,
            "uav.initial_x": -8.0,
            "uav.initial_y": -8.0,
            "uav.initial_z": 1.0,
            "mission.goal_x": 8.0,
            "mission.goal_y": 8.0,
            "mission.goal_z": 4.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.frame_id = self.get_parameter("frame_id").value
        self.x_min = float(self.get_parameter("map.x_min").value)
        self.x_max = float(self.get_parameter("map.x_max").value)
        self.y_min = float(self.get_parameter("map.y_min").value)
        self.y_max = float(self.get_parameter("map.y_max").value)
        self.z_min = float(self.get_parameter("map.z_min").value)
        self.z_max = float(self.get_parameter("map.z_max").value)
        self.resolution = float(self.get_parameter("map.resolution").value)
        self.inflation_radius = float(self.get_parameter("map.inflation_radius").value)
        self.scenario_type = self.get_parameter("world.scenario_type").value
        self.occupancy_ratio = float(self.get_parameter("world.occupancy_ratio").value)
        self.random_seed = int(self.get_parameter("world.random_seed").value)
        self.ensure_connectivity = bool(self.get_parameter("world.ensure_connectivity").value)
        self.max_generation_attempts = int(self.get_parameter("world.max_generation_attempts").value)
        self.start_xyz = (
            float(self.get_parameter("uav.initial_x").value),
            float(self.get_parameter("uav.initial_y").value),
            float(self.get_parameter("uav.initial_z").value),
        )
        self.goal_xyz = (
            float(self.get_parameter("mission.goal_x").value),
            float(self.get_parameter("mission.goal_y").value),
            float(self.get_parameter("mission.goal_z").value),
        )
        self.grid_dims = (
            int(round((self.x_max - self.x_min) / self.resolution)) + 1,
            int(round((self.y_max - self.y_min) / self.resolution)) + 1,
            int(round((self.z_max - self.z_min) / self.resolution)) + 1,
        )

    def _build_world(self):
        if self.scenario_type == "random_occupancy_3d":
            self.occupied_cells, attempts, connected = self._build_random_occupancy()
            self.boxes = []
            self._set_status(attempts, connected)
            return
        if self.scenario_type == "narrow_passage_3d":
            self.occupied_cells = self._build_narrow_passage()
            self.boxes = []
            self._set_status(1, self._connected(self.occupied_cells))
            return
        if self.scenario_type == "wall_with_vertical_gap":
            self.occupied_cells = self._build_wall_with_vertical_gap()
            self.boxes = []
            self._set_status(1, self._connected(self.occupied_cells))
            return
        self.boxes = self._build_deterministic_boxes()
        self.occupied_cells = set()
        self._set_status(1, True)

    def _build_deterministic_boxes(self):
        obstacle_color = (0.75, 0.18, 0.12, 0.82)
        inflated_color = (1.0, 0.55, 0.05, 0.24)
        return [
            Box("ground", (0.0, 0.0, 0.0), (22.0, 22.0, 0.08), (0.35, 0.35, 0.35, 0.45)),
            Box("pillar_a", (-4.0, -2.0, 2.5), (1.0, 1.0, 5.0), obstacle_color),
            Box("pillar_b", (1.5, 2.0, 3.0), (1.2, 1.2, 6.0), obstacle_color),
            Box("pillar_c", (5.0, -1.5, 2.2), (1.0, 1.0, 4.4), obstacle_color),
            Box("middle_wall", (0.0, 0.0, 2.0), (6.0, 0.7, 3.0), obstacle_color),
            Box("floating_block", (-2.0, 5.0, 4.1), (3.0, 2.0, 1.2), obstacle_color),
            Box("gate_left", (4.0, 4.0, 2.0), (0.8, 3.0, 4.0), obstacle_color),
            Box("gate_right", (7.0, 4.0, 2.0), (0.8, 3.0, 4.0), obstacle_color),
            Box("gate_top", (5.5, 4.0, 5.2), (3.8, 3.0, 0.8), obstacle_color),
            Box("inflation_hint", (100.0, 100.0, 100.0), (0.1, 0.1, 0.1), inflated_color),
        ]

    def _build_random_occupancy(self):
        free_mandatory = self._protected_cells(clearance_cells=2)
        free_mandatory |= self._corridor_cells(width_cells=2)
        all_cells = self._all_cells()
        candidates = list(all_cells - free_mandatory)
        target_count = int(round(len(all_cells) * self.occupancy_ratio))
        target_count = min(target_count, len(candidates))
        for attempt in range(1, self.max_generation_attempts + 1):
            rng = random.Random(self.random_seed + attempt - 1)
            occupied = set(rng.sample(candidates, target_count))
            connected = self._connected(occupied)
            if connected or not self.ensure_connectivity:
                return occupied, attempt, connected
        return occupied, self.max_generation_attempts, False

    def _build_narrow_passage(self):
        free = self._protected_cells(clearance_cells=2) | self._corridor_cells(width_cells=1)
        occupied = self._all_cells() - free
        return occupied

    def _build_wall_with_vertical_gap(self):
        occupied = set()
        wall_x = self.world_to_grid((0.0, 0.0, self.z_min), clamp=True)[0]
        y_range = range(4, self.grid_dims[1] - 4)
        z_range = range(0, self.grid_dims[2])
        for iy in y_range:
            for iz in z_range:
                world = self.grid_to_world((wall_x, iy, iz))
                in_gap = -1.5 <= world[1] <= 1.5 and 3.8 <= world[2] <= 5.5
                if not in_gap:
                    occupied.add((wall_x, iy, iz))
                    occupied.add((max(0, wall_x - 1), iy, iz))
                    occupied.add((min(self.grid_dims[0] - 1, wall_x + 1), iy, iz))
        return occupied - self._protected_cells(clearance_cells=2)

    def _all_cells(self):
        return {
            (ix, iy, iz)
            for ix in range(self.grid_dims[0])
            for iy in range(self.grid_dims[1])
            for iz in range(self.grid_dims[2])
        }

    def _protected_cells(self, clearance_cells):
        protected = set()
        for center in (self.world_to_grid(self.start_xyz, True), self.world_to_grid(self.goal_xyz, True)):
            for ix in range(center[0] - clearance_cells, center[0] + clearance_cells + 1):
                for iy in range(center[1] - clearance_cells, center[1] + clearance_cells + 1):
                    for iz in range(center[2] - clearance_cells, center[2] + clearance_cells + 1):
                        idx = (ix, iy, iz)
                        if self.in_bounds(idx):
                            protected.add(idx)
        return protected

    def _corridor_cells(self, width_cells):
        start = self.world_to_grid(self.start_xyz, True)
        goal = self.world_to_grid(self.goal_xyz, True)
        steps = max(abs(goal[i] - start[i]) for i in range(3)) + 1
        corridor = set()
        for step in range(steps + 1):
            t = step / max(1, steps)
            center = tuple(round(start[i] + (goal[i] - start[i]) * t) for i in range(3))
            for ix in range(center[0] - width_cells, center[0] + width_cells + 1):
                for iy in range(center[1] - width_cells, center[1] + width_cells + 1):
                    for iz in range(center[2] - width_cells, center[2] + width_cells + 1):
                        idx = (ix, iy, iz)
                        if self.in_bounds(idx):
                            corridor.add(idx)
        return corridor

    def _connected(self, occupied):
        start = self.world_to_grid(self.start_xyz, True)
        goal = self.world_to_grid(self.goal_xyz, True)
        if start in occupied or goal in occupied:
            return False
        q = deque([start])
        visited = {start}
        while q:
            current = q.popleft()
            if current == goal:
                return True
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nb = (current[0] + dx, current[1] + dy, current[2] + dz)
                        if self.in_bounds(nb) and nb not in occupied and nb not in visited:
                            visited.add(nb)
                            q.append(nb)
        return False

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

    def _set_status(self, attempts, connected):
        total = len(self._all_cells())
        actual = len(self.occupied_cells) / total if total else 0.0
        self.status_text = (
            f"scenario_type={self.scenario_type} requested_occupancy_ratio={self.occupancy_ratio:.2f} "
            f"actual_occupancy_ratio={actual:.3f} random_seed={self.random_seed} "
            f"connected={str(connected).lower()} generation_attempts={attempts}"
        )

    def _marker(self, box, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "air_world_obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(box.center[0])
        marker.pose.position.y = float(box.center[1])
        marker.pose.position.z = float(box.center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(box.size[0])
        marker.scale.y = float(box.size[1])
        marker.scale.z = float(box.size[2])
        marker.color.r = float(box.color[0])
        marker.color.g = float(box.color[1])
        marker.color.b = float(box.color[2])
        marker.color.a = float(box.color[3])
        return marker

    def publish_world(self):
        msg = MarkerArray()
        if self.occupied_cells:
            msg.markers.append(self._cube_list_marker())
        else:
            # The planner treats all CUBE markers except the ground and hint marker as obstacles.
            for i, box in enumerate(self.boxes):
                msg.markers.append(self._marker(box, i))
        self.obstacle_pub.publish(msg)
        self.visual_pub.publish(msg)
        status = String()
        status.data = self.status_text
        self.status_pub.publish(status)
        self.status_marker_pub.publish(self._status_marker())

    def _cube_list_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "air_world_obstacles"
        marker.id = 100
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution
        marker.color.r = 0.75
        marker.color.g = 0.16
        marker.color.b = 0.12
        marker.color.a = 0.45
        for idx in sorted(self.occupied_cells):
            p = Point()
            p.x, p.y, p.z = self.grid_to_world(idx)
            marker.points.append(p)
        return marker

    def _status_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "air_world_status"
        marker.id = 9000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = -9.5
        marker.pose.position.y = 8.8
        marker.pose.position.z = 6.5
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = self.status_text.replace(" ", "\n")
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = AirWorldProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
