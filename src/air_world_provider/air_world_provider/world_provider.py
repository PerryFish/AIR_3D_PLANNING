from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple
    size: tuple
    color: tuple


class AirWorldProvider(Node):
    def __init__(self):
        super().__init__("air_world_provider")
        self.declare_parameter("frame_id", "map")
        self.frame_id = self.get_parameter("frame_id").value
        self.obstacle_pub = self.create_publisher(MarkerArray, "/air/occupancy_markers", 10)
        self.visual_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.boxes = self._build_world()
        self.timer = self.create_timer(1.0, self.publish_world)
        self.get_logger().info("Air 3D world provider started with deterministic box obstacles.")

    def _build_world(self):
        obstacle_color = (0.75, 0.18, 0.12, 0.82)
        inflated_color = (1.0, 0.55, 0.05, 0.24)
        return [
            Box("ground", (0.0, 0.0, 0.0), (22.0, 22.0, 0.08), (0.35, 0.35, 0.35, 0.45)),
            Box("pillar_a", (-4.0, -2.0, 2.5), (1.0, 1.0, 5.0), obstacle_color),
            Box("pillar_b", (1.5, 2.0, 3.0), (1.2, 1.2, 6.0), obstacle_color),
            Box("pillar_c", (5.0, -1.5, 2.2), (1.0, 1.0, 4.4), obstacle_color),
            Box("middle_wall", (0.0, 0.0, 2.0), (6.0, 0.7, 3.0), obstacle_color),
            Box("floating_block", (-2.0, 5.0, 4.1), (3.0, 2.0, 1.2), obstacle_color),
            Box("gate_left", (4.0, 4.0, 2.0), (0.8, 3.0, 4.0), obstacle_color),
            Box("gate_right", (7.0, 4.0, 2.0), (0.8, 3.0, 4.0), obstacle_color),
            Box("gate_top", (5.5, 4.0, 5.2), (3.8, 3.0, 0.8), obstacle_color),
            Box("inflation_hint", (100.0, 100.0, 100.0), (0.1, 0.1, 0.1), inflated_color),
        ]

    def _marker(self, box, marker_id):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "air_world_obstacles"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = float(box.center[0])
        marker.pose.position.y = float(box.center[1])
        marker.pose.position.z = float(box.center[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(box.size[0])
        marker.scale.y = float(box.size[1])
        marker.scale.z = float(box.size[2])
        marker.color.r = float(box.color[0])
        marker.color.g = float(box.color[1])
        marker.color.b = float(box.color[2])
        marker.color.a = float(box.color[3])
        return marker

    def publish_world(self):
        msg = MarkerArray()
        # The planner treats all CUBE markers except the ground and hint marker as obstacles.
        for i, box in enumerate(self.boxes):
            msg.markers.append(self._marker(box, i))
        self.obstacle_pub.publish(msg)
        self.visual_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AirWorldProvider()
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
