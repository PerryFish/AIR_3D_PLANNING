import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        self.path_signature = None
        self.target_index = 0
        self.status = "WAITING_FOR_TRAJECTORY"
        self.last_status_log_time = self.get_clock().now()
        self.last_trail_point = None
        self.trail = Path()
        self.trail.header.frame_id = "map"
        self.last_time = self.get_clock().now()
        self.odom_pub = self.create_publisher(Odometry, "/air/state_estimation", 10)
        self.uav_marker_pub = self.create_publisher(Marker, "/air/uav_marker", 10)
        self.trail_pub = self.create_publisher(Path, "/air/uav_trail", 10)
        self.current_waypoint_pub = self.create_publisher(Marker, "/air/current_waypoint_marker", 10)
        self.status_marker_pub = self.create_publisher(Marker, "/air/uav_status_marker", 10)
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
        new_path = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        if not new_path:
            self.get_logger().warning("Received empty trajectory; keep current path.")
            return
        new_signature = self._trajectory_signature(new_path)
        if self.path_signature == new_signature:
            self.get_logger().info(
                f"Ignored duplicate trajectory, keep tracking current target_idx={self.target_index}/{len(self.path)}"
            )
            return
        self.path = new_path
        self.path_signature = new_signature
        self.target_index = self._nearest_forward_waypoint_index(new_path)
        self.status = "TRACKING"
        self.get_logger().info(
            f"Accepted new trajectory, nearest forward waypoint index={self.target_index}, "
            f"waypoints={len(self.path)}"
        )

    def step(self):
        now = self.get_clock().now()
        dt = max(0.001, (now - self.last_time).nanoseconds * 1e-9)
        self.last_time = now
        if self.path and self.target_index < len(self.path):
            self._move_toward(self.path[self.target_index], dt)
        elif self.path and self.target_index >= len(self.path):
            self.status = "GOAL_REACHED"
        else:
            self.status = "WAITING_FOR_TRAJECTORY"
        self.publish_state(now)
        self._log_status(now)

    def _move_toward(self, target, dt):
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        dz = target[2] - self.position[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= self.goal_tolerance:
            self.target_index += 1
            if self.target_index >= len(self.path):
                self.status = "GOAL_REACHED"
                self.get_logger().info("GOAL_REACHED")
            else:
                self.status = "TRACKING"
            return
        step = min(self.max_speed * dt, dist)
        self.position[0] += dx / dist * step
        self.position[1] += dy / dist * step
        self.position[2] += dz / dist * step
        self.yaw = math.atan2(dy, dx)
        self.status = "TRACKING"

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
        self._update_trail(odom)

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
            current_marker = self._current_waypoint_marker(odom.header)
            self.current_waypoint_pub.publish(current_marker)
            arr.markers.append(current_marker)
        status_marker = self._status_marker(odom.header)
        self.status_marker_pub.publish(status_marker)
        arr.markers.append(status_marker)
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
        marker.scale.y = 0.35
        marker.scale.z = 0.35
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

    def _status_marker(self, header):
        marker = Marker()
        marker.header = header
        marker.ns = "air_uav_status"
        marker.id = 5000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2] + 0.9
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.r = 0.05
        marker.color.g = 0.9
        marker.color.b = 1.0
        marker.color.a = 1.0
        dist = self._distance_to_current_target()
        marker.text = (
            f"{self.status}\n"
            f"pos=({self.position[0]:.2f},{self.position[1]:.2f},{self.position[2]:.2f})\n"
            f"target={self.target_index}/{len(self.path)} dist={dist:.2f}"
        )
        return marker

    def _update_trail(self, odom):
        current = tuple(self.position)
        if self.last_trail_point is None or math.dist(current, self.last_trail_point) >= 0.1:
            pose = PoseStamped()
            pose.header = odom.header
            pose.pose = odom.pose.pose
            self.trail.header = odom.header
            self.trail.poses.append(pose)
            self.trail.poses = self.trail.poses[-2000:]
            self.last_trail_point = current
        self.trail_pub.publish(self.trail)

    def _trajectory_signature(self, path):
        sample_indices = sorted(set([0, len(path) // 4, len(path) // 2, (len(path) * 3) // 4, len(path) - 1]))
        signature = [len(path)]
        for i in sample_indices:
            signature.append(tuple(round(v, 2) for v in path[i]))
        return tuple(signature)

    def _nearest_forward_waypoint_index(self, path):
        nearest = min(range(len(path)), key=lambda i: math.dist(tuple(self.position), path[i]))
        # If already close to the nearest point and there is a next point, start tracking forward.
        if nearest + 1 < len(path) and math.dist(tuple(self.position), path[nearest]) <= self.goal_tolerance:
            return nearest + 1
        return nearest

    def _distance_to_current_target(self):
        if not self.path or self.target_index >= len(self.path):
            return 0.0
        return math.dist(tuple(self.position), self.path[self.target_index])

    def _log_status(self, now):
        elapsed = (now - self.last_status_log_time).nanoseconds * 1e-9
        if elapsed < 1.0:
            return
        self.last_status_log_time = now
        if not self.path:
            self.get_logger().info("WAITING_FOR_TRAJECTORY")
            return
        target = self.path[min(self.target_index, len(self.path) - 1)]
        self.get_logger().info(
            f"UAV pos=({self.position[0]:.2f},{self.position[1]:.2f},{self.position[2]:.2f}), "
            f"target_idx={self.target_index}/{len(self.path)}, "
            f"target=({target[0]:.2f},{target[1]:.2f},{target[2]:.2f}), "
            f"speed={self.max_speed:.2f}, dist={self._distance_to_current_target():.2f}, status={self.status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = UavSimulator()
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
