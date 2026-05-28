import json
import math
from pathlib import Path

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from .grid_model import GridSpec, grid_to_world, ground_to_occupied_voxels, in_bounds, make_dense50_ground_footprint, world_to_grid


class SensorMappingNode(Node):
    def __init__(self):
        super().__init__("sensor_mapping_node")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.declare_parameter("sensor_mapping.publish_period_sec", 0.25)
        self.declare_parameter("sensor_mapping.lidar_range", 5.8)
        self.declare_parameter("sensor_mapping.lidar_initial_range", 5.5)
        self.declare_parameter("sensor_mapping.lidar_step_deg", 8.0)
        self.declare_parameter("sensor_mapping.camera_range", 4.2)
        self.declare_parameter("sensor_mapping.camera_initial_range", 3.5)
        self.declare_parameter("sensor_mapping.camera_fov_deg", 80.0)
        self.declare_parameter("sensor_mapping.camera_enabled", True)
        self.declare_parameter("sensor_mapping.range_warmup_sec", 8.0)
        self.declare_parameter("sensor_mapping.export_dir", "results/maps")
        self.declare_parameter("sensor_mapping.metrics_csv_path", "results/map_metrics.csv")
        self.frame_id = self.get_parameter("frame_id").value
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
        )
        self.ground_occupied = make_dense50_ground_footprint(self.spec)
        self.truth_occupied = ground_to_occupied_voxels(self.spec, self.ground_occupied)
        self.free = set()
        self.occupied = set()
        self.observed = set()
        self.frontiers = set()
        self.latest_pose = None
        self.last_pose = None
        self.trajectory = []
        self.last_scan_points = []
        self.newly_observed = 0
        self.lidar_range = float(self.get_parameter("sensor_mapping.lidar_range").value)
        self.lidar_initial_range = float(self.get_parameter("sensor_mapping.lidar_initial_range").value)
        self.lidar_step_deg = float(self.get_parameter("sensor_mapping.lidar_step_deg").value)
        self.camera_range = float(self.get_parameter("sensor_mapping.camera_range").value)
        self.camera_initial_range = float(self.get_parameter("sensor_mapping.camera_initial_range").value)
        self.camera_fov = math.radians(float(self.get_parameter("sensor_mapping.camera_fov_deg").value))
        self.camera_enabled = self._as_bool(self.get_parameter("sensor_mapping.camera_enabled").value)
        self.range_warmup_sec = float(self.get_parameter("sensor_mapping.range_warmup_sec").value)
        self.start_time = self.get_clock().now()
        self.map_pub = self.create_publisher(String, "/aerial_exploration/map_state", 10)
        self.real_metrics_pub = self.create_publisher(String, "/exploration/map_metrics", 10)
        self.coverage_pub = self.create_publisher(Float32, "/exploration/coverage_real", 10)
        self.observed_cloud_pub = self.create_publisher(PointCloud2, "/exploration/observed_cloud", 10)
        self.occupied_cloud_pub = self.create_publisher(PointCloud2, "/exploration/occupied_cloud", 10)
        self.free_cloud_pub = self.create_publisher(PointCloud2, "/exploration/free_cloud", 10)
        self.frontier_cloud_pub = self.create_publisher(PointCloud2, "/exploration/unknown_frontiers", 10)
        self.scan_cloud_pub = self.create_publisher(PointCloud2, "/exploration/lidar_scan_points", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/exploration/sensor_map_markers", 10)
        self.coverage_marker_pub = self.create_publisher(Marker, "/exploration/observed_coverage_marker", 10)
        self.synthetic_marker_pub = self.create_publisher(Marker, "/exploration/synthetic_coverage_marker", 10)
        self.create_subscription(Odometry, "/state_estimation", self.odom_cb, 10)
        self.create_subscription(String, "/exploration/save_map", self.save_cb, 10)
        self.timer = self.create_timer(float(self.get_parameter("sensor_mapping.publish_period_sec").value), self.publish_state)
        self.metrics_path = Path(self.get_parameter("sensor_mapping.metrics_csv_path").value)
        self.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_file = self.metrics_path.open("w")
        self.metrics_file.write(
            "time,robot_x,robot_y,robot_z,synthetic_coverage,observed_coverage,observed_voxels,free_voxels,"
            "occupied_voxels,unknown_voxels,frontier_count,path_length,sensor_cloud_points\n"
        )
        self.metrics_file.flush()
        self.get_logger().info("Sensor-driven mapping baseline started with local simulated LiDAR/camera frustum")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        pose = (p.x, p.y, p.z)
        self.latest_pose = pose
        if self.last_pose is None or math.dist(pose, self.last_pose) > 0.05:
            self.trajectory.append(pose)
            self._integrate_local_lidar(pose)
            if self.camera_enabled:
                self._integrate_camera_frustum(pose)
            self._update_frontiers()
            self.last_pose = pose
        else:
            self.newly_observed = 0

    def _integrate_local_lidar(self, pose):
        before = len(self.observed)
        self.last_scan_points = []
        horizontal = int(max(12, round(360.0 / self.lidar_step_deg)))
        vertical_angles = [math.radians(a) for a in (-28, -14, 0, 14, 28)]
        for h in range(horizontal):
            yaw = 2.0 * math.pi * h / horizontal
            for pitch in vertical_angles:
                direction = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
                hit = self._cast_ray(pose, direction, self._effective_lidar_range())
                if hit:
                    self.last_scan_points.append(grid_to_world(self.spec, hit))
        self.newly_observed = len(self.observed) - before

    def _integrate_camera_frustum(self, pose):
        before = len(self.observed)
        forward_yaw = self._trajectory_yaw()
        yaw_steps = 9
        pitch_steps = 5
        for yi in range(yaw_steps):
            yaw = forward_yaw - self.camera_fov * 0.5 + self.camera_fov * yi / max(1, yaw_steps - 1)
            for pi in range(pitch_steps):
                pitch = math.radians(-18.0) + math.radians(36.0) * pi / max(1, pitch_steps - 1)
                direction = (math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch))
                self._cast_ray(pose, direction, self._effective_camera_range())
        self.newly_observed += len(self.observed) - before

    def _cast_ray(self, origin, direction, max_range):
        step = max(0.2, self.spec.resolution * 0.25)
        last_idx = None
        dist = 0.0
        while dist <= max_range:
            p = (origin[0] + direction[0] * dist, origin[1] + direction[1] * dist, origin[2] + direction[2] * dist)
            idx = world_to_grid(self.spec, p)
            if not in_bounds(self.spec, idx):
                break
            if idx != last_idx:
                self.observed.add(idx)
                if idx in self.truth_occupied:
                    self.occupied.add(idx)
                    self.free.discard(idx)
                    return idx
                self.free.add(idx)
                last_idx = idx
            dist += step
        return None

    def _update_frontiers(self):
        frontiers = set()
        for idx in self.free:
            for d in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)):
                nb = (idx[0] + d[0], idx[1] + d[1], idx[2] + d[2])
                if in_bounds(self.spec, nb) and nb not in self.observed:
                    frontiers.add(idx)
                    break
        self.frontiers = frontiers

    def publish_state(self):
        if self.latest_pose is None:
            return
        observed = len(self.observed)
        occupied = len(self.occupied)
        free = len(self.free)
        unknown = self.spec.total_voxels - observed
        observed_coverage = observed / self.spec.total_voxels
        synthetic_coverage = observed_coverage
        payload = {
            "mapping_source": "sensor_driven_local_simulated_lidar_camera",
            "synthetic_coverage": synthetic_coverage,
            "observed_coverage": observed_coverage,
            "coverage": observed_coverage,
            "total_voxels": self.spec.total_voxels,
            "explored_voxels": observed,
            "observed_voxels": observed,
            "free_voxels": free,
            "occupied_voxels": occupied,
            "unknown_voxels": unknown,
            "frontier_count": len(self.frontiers),
            "frontier_cells": list(sorted(self.frontiers))[:160],
            "newly_observed_voxels": self.newly_observed,
            "sensor_cloud_points": len(self.last_scan_points),
            "ground_total_cells": self.spec.ground_total_cells,
            "ground_occupied_footprint_cells": len(self.ground_occupied),
            "ground_footprint_occupancy_ratio": len(self.ground_occupied) / self.spec.ground_total_cells,
            "target_ground_footprint_occupancy_ratio": self.spec.target_ground_ratio,
            "robot_x": self.latest_pose[0],
            "robot_y": self.latest_pose[1],
            "robot_z": self.latest_pose[2],
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.map_pub.publish(msg)
        self.real_metrics_pub.publish(msg)
        cov = Float32()
        cov.data = observed_coverage
        self.coverage_pub.publish(cov)
        self._publish_clouds()
        self.marker_pub.publish(self._markers())
        self.coverage_marker_pub.publish(self._text_marker("observed_coverage", 9101, f"observed_coverage={observed_coverage:.3f}", (0.0, -12.0, 3.2)))
        self.synthetic_marker_pub.publish(self._text_marker("synthetic_coverage", 9102, f"synthetic_coverage={synthetic_coverage:.3f}", (0.0, -12.0, 2.7)))
        self._write_metrics_row(payload)

    def _publish_clouds(self):
        self.observed_cloud_pub.publish(self._cloud([grid_to_world(self.spec, idx) for idx in self.observed]))
        self.occupied_cloud_pub.publish(self._cloud([grid_to_world(self.spec, idx) for idx in self.occupied]))
        self.free_cloud_pub.publish(self._cloud([grid_to_world(self.spec, idx) for idx in self.free]))
        self.frontier_cloud_pub.publish(self._cloud([grid_to_world(self.spec, idx) for idx in self.frontiers]))
        self.scan_cloud_pub.publish(self._cloud(self.last_scan_points))

    def _cloud(self, points):
        header = __import__("std_msgs.msg").msg.Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        return point_cloud2.create_cloud_xyz32(header, [(float(x), float(y), float(z)) for x, y, z in points])

    def _markers(self):
        arr = MarkerArray()
        arr.markers.append(self._cube_list("observed_free", 3001, self.free, (0.0, 0.55, 1.0, 0.22), 0.42))
        arr.markers.append(self._cube_list("observed_occupied", 3002, self.occupied, (1.0, 0.08, 0.0, 0.85), 0.82))
        arr.markers.append(self._cube_list("unknown_frontiers", 3003, self.frontiers, (1.0, 0.9, 0.0, 0.9), 0.36))
        return arr

    def _cube_list(self, ns, marker_id, cells, color, scale):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        for idx in sorted(cells)[:1500]:
            p = grid_to_world(self.spec, idx)
            point = Point()
            point.x, point.y, point.z = p
            marker.points.append(point)
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

    def _write_metrics_row(self, payload):
        path_length = sum(math.dist(a, b) for a, b in zip(self.trajectory, self.trajectory[1:]))
        self.metrics_file.write(
            f"{self.get_clock().now().nanoseconds / 1e9:.3f},{self.latest_pose[0]:.3f},{self.latest_pose[1]:.3f},{self.latest_pose[2]:.3f},"
            f"{payload['synthetic_coverage']:.6f},{payload['observed_coverage']:.6f},{payload['observed_voxels']},"
            f"{payload['free_voxels']},{payload['occupied_voxels']},{payload['unknown_voxels']},{payload['frontier_count']},"
            f"{path_length:.3f},{payload['sensor_cloud_points']}\n"
        )
        self.metrics_file.flush()

    def save_cb(self, _msg):
        self.save_map()

    def save_map(self):
        out = Path(self.get_parameter("sensor_mapping.export_dir").value)
        out.mkdir(parents=True, exist_ok=True)
        self._write_pcd(out / "observed_occupied_cloud.pcd", [grid_to_world(self.spec, idx) for idx in self.occupied])
        self._write_pcd(out / "observed_free_cloud.pcd", [grid_to_world(self.spec, idx) for idx in self.free])
        self._write_pcd(out / "observed_all_cloud.pcd", [grid_to_world(self.spec, idx) for idx in self.observed])
        self._write_pcd(out / "frontier_cloud.pcd", [grid_to_world(self.spec, idx) for idx in self.frontiers])
        with (out / "trajectory.csv").open("w") as f:
            f.write("x,y,z\n")
            for p in self.trajectory:
                f.write(f"{p[0]:.3f},{p[1]:.3f},{p[2]:.3f}\n")
        (out / "map_metrics.csv").write_text(self.metrics_path.read_text())
        self.get_logger().info(f"Saved sensor-driven exploration map to {out}")

    def _write_pcd(self, path, points):
        with path.open("w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n")
            f.write("TYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
            for x, y, z in points:
                f.write(f"{x:.3f} {y:.3f} {z:.3f}\n")

    def _trajectory_yaw(self):
        if len(self.trajectory) < 2:
            return 0.0
        a, b = self.trajectory[-2], self.trajectory[-1]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def _warmup_fraction(self):
        if self.range_warmup_sec <= 0.0:
            return 1.0
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        return min(1.0, max(0.0, elapsed / self.range_warmup_sec))

    def _effective_lidar_range(self):
        t = self._warmup_fraction()
        return self.lidar_initial_range + (self.lidar_range - self.lidar_initial_range) * t

    def _effective_camera_range(self):
        t = self._warmup_fraction()
        return self.camera_initial_range + (self.camera_range - self.camera_initial_range) * t

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).lower() in ("1", "true", "yes", "on")

    def destroy_node(self):
        try:
            self.save_map()
            self.metrics_file.flush()
            self.metrics_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
