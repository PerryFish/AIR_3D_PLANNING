import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class MissionManager(Node):
    def __init__(self):
        super().__init__("air_mission_manager")
        self._declare_parameters()
        self.start_pub = self.create_publisher(PoseStamped, "/air/start", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/air/goal", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/air/visualization/markers", 10)
        self.goal_sub = self.create_subscription(PoseStamped, "/air/goal", self.external_goal_cb, 10)
        self.timer = self.create_timer(1.0, self.publish_initial_mission_once)
        self.published_initial_goal = False
        self.goal_duplicate_threshold = 0.05
        self.get_logger().info("Mission manager ready.")

    def _declare_parameters(self):
        self.declare_parameter("uav.initial_x", -8.0)
        self.declare_parameter("uav.initial_y", -8.0)
        self.declare_parameter("uav.initial_z", 1.0)
        self.declare_parameter("mission.goal_x", 8.0)
        self.declare_parameter("mission.goal_y", 8.0)
        self.declare_parameter("mission.goal_z", 4.0)
        self.declare_parameter("mission.auto_start", True)
        self.start_xyz = (
            self.get_parameter("uav.initial_x").value,
            self.get_parameter("uav.initial_y").value,
            self.get_parameter("uav.initial_z").value,
        )
        self.goal_xyz = (
            self.get_parameter("mission.goal_x").value,
            self.get_parameter("mission.goal_y").value,
            self.get_parameter("mission.goal_z").value,
        )
        self.auto_start = self.get_parameter("mission.auto_start").value

    def external_goal_cb(self, msg):
        new_goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        if self._distance(new_goal, self.goal_xyz) < self.goal_duplicate_threshold:
            return
        self.goal_xyz = new_goal
        self._publish_markers(self._pose(self.start_xyz), self._pose(self.goal_xyz))
        self.get_logger().info("Received external goal, requesting replan")

    def publish_initial_mission_once(self):
        if not self.auto_start or self.published_initial_goal:
            self.timer.cancel()
            return
        start = self._pose(self.start_xyz)
        goal = self._pose(self.goal_xyz)
        self.start_pub.publish(start)
        self.goal_pub.publish(goal)
        self._publish_markers(start, goal)
        self.published_initial_goal = True
        self.get_logger().info("Published initial 3D mission goal once")
        self.timer.cancel()

    def _pose(self, xyz):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_markers(self, start, goal):
        arr = MarkerArray()
        arr.markers.append(self._sphere(start, "air_start_goal", 4000, (0.0, 1.0, 0.15, 1.0), 0.45))
        arr.markers.append(self._sphere(goal, "air_start_goal", 4001, (1.0, 0.1, 0.1, 1.0), 0.55))
        self.marker_pub.publish(arr)

    def _sphere(self, pose, ns, marker_id, color, scale):
        marker = Marker()
        marker.header = pose.header
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def _distance(self, a, b):
        return sum((a[i] - b[i]) ** 2 for i in range(3)) ** 0.5


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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
