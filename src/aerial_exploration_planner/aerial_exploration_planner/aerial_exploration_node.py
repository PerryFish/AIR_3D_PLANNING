import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


class AerialExplorationNode(Node):
    def __init__(self):
        super().__init__("aerial_exploration_node")
        self.declare_parameter("exploration.step_period_sec", 0.25)
        self.declare_parameter("exploration.coverage_increment", 0.045)
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("frame_id", "map")
        self.coverage = 0.0
        self.step = 0
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.increment = float(self.get_parameter("exploration.coverage_increment").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.coverage_pub = self.create_publisher(Float32, "/aerial_exploration/coverage", 10)
        self.path_pub = self.create_publisher(Path, "/aerial_exploration/path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "/aerial_exploration/frontiers", 10)
        self.viewpoint_pub = self.create_publisher(MarkerArray, "/aerial_exploration/viewpoints", 10)
        self.timer = self.create_timer(float(self.get_parameter("exploration.step_period_sec").value), self.tick)
        self.get_logger().info("Aerial exploration node started")

    def tick(self):
        if self.coverage < self.done_threshold:
            self.coverage = min(1.0, self.coverage + self.increment)
            self.step += 1
        coverage_msg = Float32()
        coverage_msg.data = float(self.coverage)
        self.coverage_pub.publish(coverage_msg)
        self.path_pub.publish(self._path())
        self.frontier_pub.publish(self._markers("frontier", (0.1, 0.6, 1.0, 0.9)))
        self.viewpoint_pub.publish(self._markers("viewpoint", (0.0, 1.0, 0.25, 0.9)))

    def _path(self):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        count = max(2, self.step + 1)
        for idx in range(count):
            t = idx / max(1, count - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = -8.0 + 16.0 * t
            pose.pose.position.y = -6.0 + 12.0 * t
            pose.pose.position.z = 1.0 + 2.0 * math.sin(t * math.pi)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _markers(self, namespace, color):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for idx in range(4):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = namespace
            marker.id = idx
            marker.type = Marker.SPHERE if namespace == "viewpoint" else Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = -6.0 + idx * 4.0
            marker.pose.position.y = 5.0 - idx * 2.0
            marker.pose.position.z = 1.0 + (idx % 3)
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.35
            marker.scale.y = 0.35
            marker.scale.z = 0.35
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            arr.markers.append(marker)
        return arr


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
