from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from visualization_msgs.msg import Marker


class TareRvizReplayBridge(Node):
    def __init__(self):
        super().__init__("tare_rviz_replay_bridge_node")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("reference_cloud_path", "")
        self.declare_parameter("sensor_range", 8.0)
        self.declare_parameter("terrain_ext_stride", 2)
        self.declare_parameter("garage_v1.start_pose.x", -23.817)
        self.declare_parameter("garage_v1.start_pose.y", -46.018)
        self.declare_parameter("garage_v1.start_pose.z", 1.6)
        self.declare_parameter("garage_v1.start_pose.yaw", 0.0)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.sensor_range = float(self.get_parameter("sensor_range").value)
        self.start_pose = (
            float(self.get_parameter("garage_v1.start_pose.x").value),
            float(self.get_parameter("garage_v1.start_pose.y").value),
            float(self.get_parameter("garage_v1.start_pose.z").value),
            float(self.get_parameter("garage_v1.start_pose.yaw").value),
        )
        self.latest_pose = None
        self.explored_keys = set()
        self.explored_points = []
        self.reference_points = self._load_reference_cloud()
        self.surface_debug_points = self._load_surface_debug_cloud()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.overall_map_pub = self.create_publisher(PointCloud2, "/overall_map", latched)
        self.registered_scan_pub = self.create_publisher(PointCloud2, "/registered_scan", 10)
        self.explored_areas_pub = self.create_publisher(PointCloud2, "/explored_areas", 10)
        self.terrain_map_pub = self.create_publisher(PointCloud2, "/terrain_map", 10)
        self.terrain_map_ext_pub = self.create_publisher(PointCloud2, "/terrain_map_ext", 10)
        self.edge_cloud_pub = self.create_publisher(PointCloud2, "/exploration/garage_edge_cloud", latched)
        self.debug_surface_pub = self.create_publisher(PointCloud2, "/exploration/debug_surface_cloud", latched)
        self.free_paths_pub = self.create_publisher(PointCloud2, "/free_paths", 10)
        self.uncovered_cloud_pub = self.create_publisher(PointCloud2, "/uncovered_cloud", 10)
        self.uncovered_frontier_pub = self.create_publisher(PointCloud2, "/uncovered_frontier_cloud", 10)
        self.path_pub = self.create_publisher(NavPath, "/path", 10)
        self.local_path_pub = self.create_publisher(NavPath, "/local_path", 10)
        self.global_path_pub = self.create_publisher(NavPath, "/global_path", 10)
        self.waypoint_pub = self.create_publisher(PointStamped, "/way_point", 10)
        self.local_horizon_pub = self.create_publisher(Marker, "/tare_visualizer/local_planning_horizon", 10)
        self.exploring_subspaces_pub = self.create_publisher(Marker, "/tare_visualizer/exploring_subspaces", 10)
        self.start_marker_pub = self.create_publisher(Marker, "/exploration/start_pose_marker", 10)

        self.create_subscription(Odometry, "/state_estimation", self._odom_cb, 10)
        self.create_subscription(PointCloud2, "/exploration/frontier_cloud", self._frontier_cb, 10)
        self.create_subscription(NavPath, "/exploration/trajectory_path", self._trajectory_cb, 10)
        self.create_subscription(NavPath, "/aerial_exploration/path", self._planner_path_cb, 10)
        self.create_subscription(PoseStamped, "/aerial_exploration/goal", self._goal_cb, 10)
        self.create_subscription(Marker, "/exploration/local_planning_box", self._local_box_cb, 10)

        self.timer = self.create_timer(float(self.get_parameter("publish_period_sec").value), self.publish_static)
        self.publish_static()
        self.get_logger().info(f"TARE edge replay bridge started with {len(self.reference_points)} edge /overall_map points")

    def _load_reference_cloud(self):
        explicit = str(self.get_parameter("reference_cloud_path").value).strip()
        candidates = []
        if explicit:
            candidates.append(Path(explicit))
        candidates.extend(self._cloud_candidates())
        for path in candidates:
            if not path.exists():
                continue
            try:
                points = self._load_cloud(path)
            except Exception as exc:
                self.get_logger().warn(f"Could not load reference cloud {path}: {exc}")
                continue
            if points:
                self.get_logger().info(f"Loaded TARE replay reference cloud: {path}")
                return points
        self.get_logger().warn("No TARE replay reference cloud found; /overall_map will be empty")
        return []

    def _cloud_candidates(self):
        rels = [
            Path("virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.pcd"),
            Path("virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.xyz"),
        ]
        roots = []
        try:
            roots.append(Path(get_package_share_directory("aerial_exploration_planner")))
        except Exception:
            pass
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src"))
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/install/aerial_exploration_planner/share/aerial_exploration_planner"))
        return [root / rel for root in roots for rel in rels]

    def _load_surface_debug_cloud(self):
        rels = [
            Path("virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.pcd"),
            Path("virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.xyz"),
            Path("virtual_env/garage_v1/maps/garage_structure_from_tare.pcd"),
            Path("virtual_env/garage_v1/maps/garage_structure_from_tare.xyz"),
        ]
        roots = []
        try:
            roots.append(Path(get_package_share_directory("aerial_exploration_planner")))
        except Exception:
            pass
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src"))
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/install/aerial_exploration_planner/share/aerial_exploration_planner"))
        for path in (root / rel for root in roots for rel in rels):
            if not path.exists():
                continue
            try:
                points = self._load_cloud(path)
            except Exception:
                continue
            if points:
                self.get_logger().info(f"Loaded debug surface cloud: {path}")
                return points
        return []

    def _load_cloud(self, path):
        if path.suffix.lower() == ".pcd":
            return self._load_ascii_pcd(path)
        return self._load_xyz(path)

    def _load_xyz(self, path):
        points = []
        with path.open() as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    points.append((float(parts[0]), float(parts[1]), float(parts[2])))
        return points

    def _load_ascii_pcd(self, path):
        points = []
        data = False
        with path.open(errors="ignore") as f:
            for line in f:
                stripped = line.strip()
                if not data:
                    if stripped.lower().startswith("data"):
                        if "ascii" not in stripped.lower():
                            raise ValueError("only ASCII PCD is supported")
                        data = True
                    continue
                if stripped:
                    parts = stripped.split()
                    if len(parts) >= 3:
                        points.append((float(parts[0]), float(parts[1]), float(parts[2])))
        return points

    def publish_static(self):
        if self.reference_points:
            cloud = self._cloud(self.reference_points)
            self.overall_map_pub.publish(cloud)
            self.edge_cloud_pub.publish(cloud)
            self.start_marker_pub.publish(self._start_pose_marker())
            if self.surface_debug_points:
                self.debug_surface_pub.publish(self._cloud(self.surface_debug_points))
            if self.latest_pose is None:
                self.terrain_map_pub.publish(cloud)
                self.terrain_map_ext_pub.publish(self._cloud(self.reference_points[:: max(1, int(self.get_parameter("terrain_ext_stride").value))]))
            else:
                local = self._local_edge_points(self.latest_pose, self.sensor_range)
                if len(local) < 50:
                    local = self._local_edge_points(self.latest_pose, self.sensor_range * 2.0)
                if len(local) < 50:
                    local = self._local_edge_points(self.latest_pose, 30.0)
                local_cloud = self._cloud(local)
                self.registered_scan_pub.publish(local_cloud)
                self.terrain_map_pub.publish(local_cloud)
                for point in local:
                    key = tuple(round(v, 2) for v in point)
                    if key not in self.explored_keys:
                        self.explored_keys.add(key)
                        self.explored_points.append(point)
                explored_cloud = self._cloud(self.explored_points)
                self.explored_areas_pub.publish(explored_cloud)
                ext_points = self._expanded_points(local)
                self.terrain_map_ext_pub.publish(self._cloud(ext_points))

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.latest_pose = (p.x, p.y, p.z)

    def _frontier_cb(self, msg):
        self.uncovered_cloud_pub.publish(msg)
        self.uncovered_frontier_pub.publish(msg)

    def _trajectory_cb(self, msg):
        self.path_pub.publish(self._with_frame(msg))
        self.free_paths_pub.publish(self._path_points(msg))

    def _planner_path_cb(self, msg):
        path = self._with_frame(msg)
        self.local_path_pub.publish(path)
        self.global_path_pub.publish(path)

    def _goal_cb(self, msg):
        out = PointStamped()
        out.header = msg.header
        out.header.frame_id = out.header.frame_id or self.frame_id
        out.point = Point(x=msg.pose.position.x, y=msg.pose.position.y, z=msg.pose.position.z)
        self.waypoint_pub.publish(out)

    def _local_box_cb(self, msg):
        marker = msg
        marker.header.frame_id = marker.header.frame_id or self.frame_id
        self.local_horizon_pub.publish(marker)
        self.exploring_subspaces_pub.publish(marker)

    def _local_edge_points(self, pose, max_range):
        max_range_sq = max_range * max_range
        local = []
        for point in self.reference_points:
            dx = point[0] - pose[0]
            dy = point[1] - pose[1]
            dz = point[2] - pose[2]
            if dx * dx + dy * dy + dz * dz <= max_range_sq:
                local.append(point)
        return local

    def _expanded_points(self, points):
        expanded = []
        for x, y, z in points[::2]:
            expanded.append((x, y, z))
            expanded.append((x, y, z + 0.08))
        return expanded

    def _with_frame(self, path):
        path.header.frame_id = path.header.frame_id or self.frame_id
        for pose in path.poses:
            pose.header.frame_id = pose.header.frame_id or path.header.frame_id
        return path

    def _path_points(self, path):
        return self._cloud([(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in path.poses])

    def _cloud(self, points):
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        return point_cloud2.create_cloud_xyz32(header, [(float(x), float(y), float(z)) for x, y, z in points])

    def _start_pose_marker(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "garage_start_pose"
        marker.id = 8101
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.start_pose[0]
        marker.pose.position.y = self.start_pose[1]
        marker.pose.position.z = self.start_pose[2]
        import math

        marker.pose.orientation.z = math.sin(self.start_pose[3] * 0.5)
        marker.pose.orientation.w = math.cos(self.start_pose[3] * 0.5)
        marker.scale.x = 0.85
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 0.95)
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = TareRvizReplayBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
