import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager_node")
        self.pub = self.create_publisher(String, "/aerial_exploration/mode", 10)
        self.timer = self.create_timer(1.0, self.publish_mode)
        self.get_logger().info("Mode manager set to autonomous exploration")

    def publish_mode(self):
        msg = String()
        msg.data = "AUTONOMOUS_EXPLORATION"
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
