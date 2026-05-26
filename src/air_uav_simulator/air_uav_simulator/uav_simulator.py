import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class UavSimulator(Node):
    def __init__(self):
        super().__init__("air_uav_simulator")
        self._declare_parameters()
        self.position = [self.initial_x, self.initial_y, self.initial_z]
        self.yaw = 0.0
        self.path = []
        self.target_index = 0
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, "/air/state_estimation", 10)
        self.uav_marker_pub = self.create_publisher(Marker, "/air/uav_marker", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.path_sub = self.create_subscription(Path, "/air/smoothed_path", self.path_cb, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info("3D UAV kinematic simulator ready.")

    def _declare_parameters(self):
        self.declare_parameter("uav.max_speed", 1.0)
        self.declare_parameter("uav.goal_tolerance", 0.3)
        self.declare_parameter("uav.initial_x", -8.0)
        self.declare_parameter("uav.initial_y", -8.0)
        self.declare_parameter("uav.initial_z", 1.0)
        self.max_speed = self.get_parameter("uav.max_speed").value
        self.goal_tolerance = self.get_parameter("uav.goal_tolerance").value
        self.initial_x = self.get_parameter("uav.initial_x").value
        self.initial_y = self.get_parameter("uav.initial_y").value
        self.initial_z = self.get_parameter("uav.initial_z").value

    def path_cb(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.target_index = 0
        self.get_logger().info(f"UAV accepted 3D trajectory with {len(self.path)} waypoints.")

    def step(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        if self.path and self.target_index < len(self.path):
            self._move_toward(self.path[self.target_index], dt)
        self.publish_state(now)

    def _move_toward(self, target, dt):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        dz = target[2] - self.position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= self.goal_tolerance:
            self.target_index += 1
            return
        step = min(self.max_speed * dt, dist)
        self.position[0] += dx / dist * step
        self.position[1] += dy / dist * step
        self.position[2] += dz / dist * step
        self.yaw = math.atan2(dy, dx)

    def publish_state(self, stamp):
        odom = Odometry()
        odom.header.frame_id = "map"
        odom.header.stamp = stamp.to_msg()
        odom.child_frame_id = "uav_base_link"
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.position.z = self.position[2]
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "uav_base_link"
        tf.transform.translation.x = self.position[0]
        tf.transform.translation.y = self.position[1]
        tf.transform.translation.z = self.position[2]
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        marker = self._uav_marker(odom)
        self.uav_marker_pub.publish(marker)
        arr = MarkerArray()
        arr.markers.append(marker)
        if self.path and self.target_index < len(self.path):
            arr.markers.append(self._current_waypoint_marker(odom.header))
        self.marker_pub.publish(arr)

    def _uav_marker(self, odom):
        marker = Marker()
        marker.header = odom.header
        marker.ns = "air_uav"
        marker.id = 3000
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = odom.pose.pose
        marker.scale.x = 0.8
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = 0.05
        marker.color.g = 0.45
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _current_waypoint_marker(self, header):
        target = self.path[self.target_index]
        marker = Marker()
        marker.header = header
        marker.ns = "air_current_waypoint"
        marker.id = 2001
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = target[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = UavSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
