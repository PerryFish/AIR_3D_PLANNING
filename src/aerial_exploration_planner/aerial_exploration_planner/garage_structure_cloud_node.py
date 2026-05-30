import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class GarageStructureCloudNode(Node):
    def __init__(self):
        super().__init__("garage_structure_cloud_node")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("point_spacing", 0.12)
        self.declare_parameter("cloud_file", "")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.points, self.source = self._load_preferred_cloud()
        if not self.points:
            self.source = "generated_wall_proxy_fallback"
            self.points = self._build_proxy_structure(float(self.get_parameter("point_spacing").value))
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(PointCloud2, "/exploration/garage_edge_cloud", qos)
        self.compat_pub = self.create_publisher(PointCloud2, "/exploration/garage_structure_cloud", qos)
        self.alias_pub = self.create_publisher(PointCloud2, "/exploration/static_garage_structure_cloud", qos)
        self.debug_surface_pub = self.create_publisher(PointCloud2, "/exploration/debug_surface_cloud", qos)
        self.timer = self.create_timer(float(self.get_parameter("publish_period_sec").value), self.publish)
        bounds = self._bounds(self.points)
        self.get_logger().info(
            f"Publishing static garage structure cloud from {self.source} with {len(self.points)} points; "
            f"bounds x=[{bounds[0][0]:.2f},{bounds[1][0]:.2f}] y=[{bounds[0][1]:.2f},{bounds[1][1]:.2f}] "
            f"z=[{bounds[0][2]:.2f},{bounds[1][2]:.2f}]"
        )
        self.publish()

    def _load_preferred_cloud(self):
        explicit = str(self.get_parameter("cloud_file").value).strip()
        candidates = []
        if explicit:
            candidates.append(Path(explicit))
        candidates.extend(self._package_cloud_candidates())
        for path in candidates:
            if not path.exists():
                continue
            try:
                points = self._load_cloud_file(path)
            except Exception as exc:
                self.get_logger().warn(f"Could not load garage structure cloud {path}: {exc}")
                continue
            if points:
                return points, str(path)
        return [], "none"

    def _package_cloud_candidates(self):
        rels = [
            Path("virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.pcd"),
            Path("virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.xyz"),
            Path("virtual_env/garage_v1/maps/garage_structure_from_mesh.pcd"),
            Path("virtual_env/garage_v1/maps/garage_structure_from_mesh.xyz"),
        ]
        roots = []
        try:
            roots.append(Path(get_package_share_directory("aerial_exploration_planner")))
        except Exception:
            pass
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src"))
        roots.append(Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/install/aerial_exploration_planner/share/aerial_exploration_planner"))
        return [root / rel for root in roots for rel in rels]

    def _load_cloud_file(self, path):
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
                parts = line.replace(",", " ").split()
                if len(parts) < 3:
                    continue
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
                if not stripped:
                    continue
                parts = stripped.split()
                if len(parts) >= 3:
                    points.append((float(parts[0]), float(parts[1]), float(parts[2])))
        return points

    def _bounds(self, points):
        if not points:
            return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return (
            tuple(min(p[i] for p in points) for i in range(3)),
            tuple(max(p[i] for p in points) for i in range(3)),
        )

    def _build_proxy_structure(self, spacing):
        spacing = max(0.05, spacing)
        walls = [
            (0.0, 15.0, 32.0, 0.25, 3.0),
            (0.0, -15.0, 32.0, 0.25, 3.0),
            (16.0, 0.0, 0.25, 30.0, 3.0),
            (-16.0, 0.0, 0.25, 30.0, 3.0),
            (-7.0, 0.0, 0.18, 22.0, 2.8),
            (2.0, 2.0, 0.18, 20.0, 2.8),
            (9.0, -3.0, 0.18, 18.0, 2.8),
            (-3.0, -6.0, 18.0, 0.18, 2.7),
            (4.0, 7.0, 20.0, 0.18, 2.7),
        ]
        points = []
        for cx, cy, sx, sy, sz in walls:
            points.extend(self._wall_edge_points(cx, cy, sx, sy, sz, spacing))
        return points

    def _wall_edge_points(self, cx, cy, sx, sy, sz, spacing):
        x_values = self._samples(cx - sx * 0.5, cx + sx * 0.5, spacing)
        y_values = self._samples(cy - sy * 0.5, cy + sy * 0.5, spacing)
        z_values = self._samples(0.0, sz, spacing)
        points = []
        xmin, xmax = cx - sx * 0.5, cx + sx * 0.5
        ymin, ymax = cy - sy * 0.5, cy + sy * 0.5
        for x in (xmin, xmax):
            for y in (ymin, ymax):
                for z in z_values:
                    points.append((x, y, z))
        for z in (0.0, sz):
            for x in x_values:
                for y in (ymin, ymax):
                    points.append((x, y, z))
            for y in y_values:
                for x in (xmin, xmax):
                    points.append((x, y, z))
        return points

    def _samples(self, start, end, spacing):
        count = max(2, int(math.ceil((end - start) / spacing)) + 1)
        if count == 1:
            return [start]
        step = (end - start) / (count - 1)
        return [start + i * step for i in range(count)]

    def publish(self):
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        msg = point_cloud2.create_cloud_xyz32(header, self.points)
        self.pub.publish(msg)
        self.compat_pub.publish(msg)
        self.alias_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GarageStructureCloudNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
