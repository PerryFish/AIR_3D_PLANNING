import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker


class SimpleUavFollowerNode(Node):
    def __init__(self):
        super().__init__("simple_uav_follower_node")
        self.declare_parameter("uav.initial_x", -9.0)
        self.declare_parameter("uav.initial_y", -9.0)
        self.declare_parameter("uav.initial_z", 1.4)
        self.declare_parameter("uav.initial_yaw", 0.0)
        self.declare_parameter("garage_v1.start_pose.x", self.get_parameter("uav.initial_x").value)
        self.declare_parameter("garage_v1.start_pose.y", self.get_parameter("uav.initial_y").value)
        self.declare_parameter("garage_v1.start_pose.z", self.get_parameter("uav.initial_z").value)
        self.declare_parameter("garage_v1.start_pose.yaw", self.get_parameter("uav.initial_yaw").value)
        self.declare_parameter("uav.max_speed", 2.2)
        self.declare_parameter("uav.max_vertical_speed", 0.8)
        self.declare_parameter("uav.goal_tolerance", 0.35)
        self.declare_parameter("altitude_planning.adaptive_z_enabled", False)
        self.declare_parameter("aerial_corridor_min_z", 0.8)
        self.declare_parameter("aerial_corridor_max_z", 2.2)
        self.declare_parameter("aerial_corridor_default_z", 1.4)
        self.corridor_min_z = float(self.get_parameter("aerial_corridor_min_z").value)
        self.corridor_max_z = float(self.get_parameter("aerial_corridor_max_z").value)
        self.corridor_default_z = self._clamp_z(float(self.get_parameter("aerial_corridor_default_z").value))
        self.position = [
            float(self.get_parameter("garage_v1.start_pose.x").value),
            float(self.get_parameter("garage_v1.start_pose.y").value),
            self._clamp_z(float(self.get_parameter("garage_v1.start_pose.z").value)),
        ]
        self.max_speed = float(self.get_parameter("uav.max_speed").value)
        self.max_vertical_speed = float(self.get_parameter("uav.max_vertical_speed").value)
        self.adaptive_z_enabled = self._as_bool(self.get_parameter("altitude_planning.adaptive_z_enabled").value)
        self.goal_tolerance = float(self.get_parameter("uav.goal_tolerance").value)
        self.path = []
        self.target_index = 0
        self.yaw = float(self.get_parameter("garage_v1.start_pose.yaw").value)
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.state_pub = self.create_publisher(Odometry, "/state_estimation", 10)
        self.marker_pub = self.create_publisher(Marker, "/aerial_exploration/uav_marker", 10)
        self.create_subscription(Path, "/aerial_exploration/path", self.path_cb, 10)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info(
            "Simple UAV follower publishing /odom and TF "
            f"with corridor z=[{self.corridor_min_z:.2f}, {self.corridor_max_z:.2f}], "
            f"default_z={self.corridor_default_z:.2f}, adaptive_z_enabled={self.adaptive_z_enabled}"
        )

    def path_cb(self, msg):
        pts = []
        for pose in msg.poses:
            raw_z = pose.pose.position.z
            z = self._clamp_z(raw_z)
            if abs(z - raw_z) > 1e-3:
                self.get_logger().warning(f"Clamped waypoint z from {raw_z:.3f} to {z:.3f}")
            pts.append((pose.pose.position.x, pose.pose.position.y, z))
        if pts:
            self.path = pts
            self.target_index = min(1, len(pts) - 1)

    def step(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        if self.path and self.target_index < len(self.path):
            self._move(self.path[self.target_index], dt)
        self._publish(now)

    def _move(self, target, dt):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        dz = target[2] - self.position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < self.goal_tolerance:
            self.target_index += 1
            return
        step = min(dist, self.max_speed * dt)
        self.position[0] += dx / dist * step
        self.position[1] += dy / dist * step
        target_z = self._clamp_z(target[2])
        z_delta = target_z - self.position[2]
        z_step = max(-self.max_vertical_speed * dt, min(self.max_vertical_speed * dt, z_delta))
        self.position[2] = self._clamp_z(self.position[2] + z_step)
        self.yaw = math.atan2(dy, dx)

    def _clamp_z(self, z):
        return min(self.corridor_max_z, max(self.corridor_min_z, z))

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        return str(value).lower() in ("1", "true", "yes", "on")

    def _publish(self, now):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = now.to_msg()
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = self.position
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.odom_pub.publish(odom)
        self.state_pub.publish(odom)
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = self.position
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(tf)
        marker = Marker()
        marker.header = odom.header
        marker.ns = "uav"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = odom.pose.pose
        marker.scale.x = 0.9
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.0, 0.35, 1.0, 1.0)
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleUavFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
