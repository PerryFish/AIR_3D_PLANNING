import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("air_trajectory_generator")
        self.declare_parameter("sample_spacing", 0.5)
        self.sample_spacing = self.get_parameter("sample_spacing").value
        self.path_pub = self.create_publisher(Path, "/air/smoothed_path", 10)
        self.traj_pub = self.create_publisher(Path, "/air/trajectory", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.sub = self.create_subscription(Path, "/air/global_path", self.path_cb, 10)
        self.get_logger().info("3D trajectory generator ready.")

    def path_cb(self, msg):
        if not msg.poses:
            self.get_logger().warning("Received empty global path; ignoring.")
            return
        simplified = self._line_simplify(msg.poses)
        sampled = self._sample_path(simplified, msg.header)
        self.path_pub.publish(sampled)
        self.traj_pub.publish(sampled)
        self._publish_current_waypoint(sampled)
        self.get_logger().info(f"Published smoothed 3D path with {len(sampled.poses)} sampled waypoints.")

    def _line_simplify(self, poses):
        # Keep direction changes from A* while preserving all 3D coordinates.
        if len(poses) < 3:
            return poses
        out = [poses[0]]
        prev = None
        for a, b in zip(poses, poses[1:]):
            direction = self._unit_direction(a, b)
            if prev is not None and direction != prev:
                out.append(a)
            prev = direction
        out.append(poses[-1])
        return out

    def _unit_direction(self, a, b):
        dx = b.pose.position.x - a.pose.position.x
        dy = b.pose.position.y - a.pose.position.y
        dz = b.pose.position.z - a.pose.position.z
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        if norm < 1e-6:
            return (0, 0, 0)
        return (round(dx / norm, 3), round(dy / norm, 3), round(dz / norm, 3))

    def _sample_path(self, poses, header):
        out = Path()
        out.header.frame_id = "map"
        out.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(poses) - 1):
            a = poses[i].pose.position
            b = poses[i + 1].pose.position
            dist = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
            steps = max(1, int(math.ceil(dist / self.sample_spacing)))
            for step in range(steps):
                t = step / steps
                p = PoseStamped()
                p.header = out.header
                p.pose.position.x = a.x + (b.x - a.x) * t
                p.pose.position.y = a.y + (b.y - a.y) * t
                p.pose.position.z = a.z + (b.z - a.z) * t
                p.pose.orientation.w = 1.0
                out.poses.append(p)
        last = PoseStamped()
        last.header = out.header
        last.pose = poses[-1].pose
        out.poses.append(last)
        return out

    def _publish_current_waypoint(self, path):
        if not path.poses:
            return
        marker = Marker()
        marker.header = path.header
        marker.ns = "air_current_waypoint"
        marker.id = 2000
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = path.poses[0].pose
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.05
        marker.color.a = 1.0
        arr = MarkerArray()
        arr.markers.append(marker)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenerator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
