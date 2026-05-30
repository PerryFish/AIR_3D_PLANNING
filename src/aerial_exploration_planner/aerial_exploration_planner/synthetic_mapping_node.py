import json
import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .grid_model import GridSpec, grid_to_world, ground_to_occupied_voxels, in_bounds, make_dense50_ground_footprint, world_to_grid


class SyntheticMappingNode(Node):
    def __init__(self):
        super().__init__("synthetic_mapping_node")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("grid.origin_x", 0.0)
        self.declare_parameter("grid.origin_y", 0.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.declare_parameter("mapping.publish_period_sec", 0.25)
        self.declare_parameter("mapping.sensor_range", 3.2)
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
            float(self.get_parameter("grid.origin_x").value),
            float(self.get_parameter("grid.origin_y").value),
        )
        self.ground_occupied = make_dense50_ground_footprint(self.spec)
        self.occupied_voxels = ground_to_occupied_voxels(self.spec, self.ground_occupied)
        self.observed = set()
        self.last_pose = None
        self.current_pose = None
        self.newly_observed = 0
        self.sensor_range = float(self.get_parameter("mapping.sensor_range").value)
        self.map_pub = self.create_publisher(String, "/aerial_exploration/map_state", 10)
        self.ground_pub = self.create_publisher(String, "/aerial_exploration/ground_footprint_occupancy", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/aerial_exploration/map_markers", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.timer = self.create_timer(float(self.get_parameter("mapping.publish_period_sec").value), self.publish_state)
        self.get_logger().info("Pose-driven dense50 mapping node started")

    def odom_cb(self, msg):
        pose = msg.pose.pose.position
        self.current_pose = (pose.x, pose.y, pose.z)
        if self.last_pose is not None and math.dist(self.current_pose, self.last_pose) < 0.05:
            self.newly_observed = 0
            return
        before = len(self.observed)
        center = world_to_grid(self.spec, self.current_pose)
        radius = max(1, int(math.ceil(self.sensor_range / self.spec.resolution)))
        for ix in range(center[0] - radius, center[0] + radius + 1):
            for iy in range(center[1] - radius, center[1] + radius + 1):
                for iz in range(center[2] - radius, center[2] + radius + 1):
                    idx = (ix, iy, iz)
                    if in_bounds(self.spec, idx) and math.dist(grid_to_world(self.spec, idx), self.current_pose) <= self.sensor_range:
                        self.observed.add(idx)
        self.newly_observed = len(self.observed) - before
        self.last_pose = self.current_pose

    def publish_state(self):
        ground_count = len(self.ground_occupied)
        explored = len(self.observed)
        occupied = len(self.observed & self.occupied_voxels)
        free = explored - occupied
        unknown = self.spec.total_voxels - explored
        payload = {
            "x_cells": self.spec.x_cells,
            "y_cells": self.spec.y_cells,
            "z_cells": self.spec.z_cells,
            "resolution": self.spec.resolution,
            "total_voxels": self.spec.total_voxels,
            "ground_total_cells": self.spec.ground_total_cells,
            "ground_occupied_footprint_cells": ground_count,
            "ground_footprint_occupancy_ratio": ground_count / self.spec.ground_total_cells,
            "target_ground_footprint_occupancy_ratio": self.spec.target_ground_ratio,
            "coverage": explored / self.spec.total_voxels,
            "explored_voxels": explored,
            "free_voxels": free,
            "occupied_voxels": occupied,
            "unknown_voxels": unknown,
            "newly_observed_voxels": self.newly_observed,
            "robot_x": self.current_pose[0] if self.current_pose else 0.0,
            "robot_y": self.current_pose[1] if self.current_pose else 0.0,
            "robot_z": self.current_pose[2] if self.current_pose else 0.0,
            "frontier_cells": self._frontiers(),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.map_pub.publish(msg)
        self.ground_pub.publish(msg)
        self.marker_pub.publish(self._map_markers())

    def _frontiers(self):
        frontiers = []
        for idx in self.observed:
            for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                nb = (idx[0] + d[0], idx[1] + d[1], idx[2] + d[2])
                if in_bounds(self.spec, nb) and nb not in self.observed and nb not in self.occupied_voxels:
                    frontiers.append(nb)
                    break
        return frontiers[:80]

    def _map_markers(self):
        arr = MarkerArray()
        header_stamp = self.get_clock().now().to_msg()
        obs = Marker()
        obs.header.frame_id = "map"
        obs.header.stamp = header_stamp
        obs.ns = "dense50_obstacles"
        obs.id = 1
        obs.type = Marker.CUBE_LIST
        obs.action = Marker.ADD
        obs.scale.x = self.spec.resolution
        obs.scale.y = self.spec.resolution
        obs.scale.z = self.spec.resolution
        obs.color.r = 0.8
        obs.color.g = 0.18
        obs.color.b = 0.08
        obs.color.a = 0.72
        for idx in sorted(self.occupied_voxels):
            p = grid_to_world(self.spec, idx)
            point = __import__("geometry_msgs.msg").msg.Point()
            point.x, point.y, point.z = p
            obs.points.append(point)
        arr.markers.append(obs)
        seen = Marker()
        seen.header.frame_id = "map"
        seen.header.stamp = header_stamp
        seen.ns = "observed_voxels"
        seen.id = 2
        seen.type = Marker.CUBE_LIST
        seen.action = Marker.ADD
        seen.scale.x = self.spec.resolution * 0.45
        seen.scale.y = self.spec.resolution * 0.45
        seen.scale.z = self.spec.resolution * 0.45
        seen.color.r = 0.0
        seen.color.g = 0.75
        seen.color.b = 1.0
        seen.color.a = 0.28
        for idx in sorted(self.observed):
            p = grid_to_world(self.spec, idx)
            point = __import__("geometry_msgs.msg").msg.Point()
            point.x, point.y, point.z = p
            seen.points.append(point)
        arr.markers.append(seen)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
