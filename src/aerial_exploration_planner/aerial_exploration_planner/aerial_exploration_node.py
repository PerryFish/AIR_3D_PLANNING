import json
import math

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray

from .grid_model import GridSpec, grid_to_world


class AerialExplorationNode(Node):
    def __init__(self):
        super().__init__("aerial_exploration_node")
        self.declare_parameter("exploration.plan_period_sec", 0.5)
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
        )
        self.pose = None
        self.map_state = None
        self.current_goal = None
        self.goal_index = 0
        self.coverage_pub = self.create_publisher(Float32, "/aerial_exploration/coverage", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/aerial_exploration/goal", 10)
        self.path_pub = self.create_publisher(Path, "/aerial_exploration/path", 10)
        self.frontier_pub = self.create_publisher(MarkerArray, "/aerial_exploration/frontiers", 10)
        self.viewpoint_pub = self.create_publisher(MarkerArray, "/aerial_exploration/viewpoints", 10)
        self.selected_goal_pub = self.create_publisher(Marker, "/aerial_exploration/selected_goal_marker", 10)
        self.coverage_marker_pub = self.create_publisher(Marker, "/aerial_exploration/coverage_marker", 10)
        self.ground_marker_pub = self.create_publisher(Marker, "/aerial_exploration/ground_footprint_marker", 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Odometry, "/state_estimation", self.odom_cb, 10)
        self.create_subscription(str_msg(), "/aerial_exploration/map_state", self.map_cb, 10)
        self.timer = self.create_timer(float(self.get_parameter("exploration.plan_period_sec").value), self.tick)
        self.get_logger().info("Pose/map-driven aerial exploration node started")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def map_cb(self, msg):
        self.map_state = json.loads(msg.data)

    def tick(self):
        if not self.map_state:
            return
        coverage = float(self.map_state.get("coverage", 0.0))
        coverage_msg = Float32()
        coverage_msg.data = coverage
        self.coverage_pub.publish(coverage_msg)
        self.coverage_marker_pub.publish(self._text_marker("coverage", 9001, f"coverage={coverage:.3f}", (0.0, -11.0, 3.0)))
        gr = float(self.map_state.get("ground_footprint_occupancy_ratio", 0.0))
        self.ground_marker_pub.publish(self._text_marker("ground_footprint", 9002, f"dense50 footprint={gr:.3f}", (0.0, -11.0, 2.5)))
        if self.pose is None or coverage >= self.done_threshold:
            return
        goal = self._select_goal()
        if goal is None:
            return
        if self.current_goal is None or math.dist(goal, self.current_goal) > 1.0:
            self.current_goal = goal
            self.goal_index += 1
            self.goal_pub.publish(self._goal_pose(goal))
        path = self._path_to_goal(goal)
        self.path_pub.publish(path)
        self.frontier_pub.publish(self._frontier_markers())
        self.viewpoint_pub.publish(self._viewpoint_markers())
        self.selected_goal_pub.publish(self._goal_marker(goal))

    def _select_goal(self):
        frontiers = self.map_state.get("frontier_cells", [])
        if not frontiers:
            return self._lawnmower_goal()
        candidates = [grid_to_world(self.spec, tuple(cell)) for cell in frontiers]
        candidates.sort(key=lambda p: (-math.dist(p, self.pose), p[0], p[1], p[2]))
        return candidates[min(self.goal_index % max(1, len(candidates)), len(candidates) - 1)]

    def _lawnmower_goal(self):
        z = 2.5
        cols = [-8.0, -4.0, 0.0, 4.0, 8.0]
        y = 8.0 if self.goal_index % 2 == 0 else -8.0
        return (cols[self.goal_index % len(cols)], y, z)

    def _path_to_goal(self, goal):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        start = self.pose or (-9.0, -9.0, 1.5)
        steps = max(2, int(math.dist(start, goal) / 0.75) + 1)
        for idx in range(steps):
            t = idx / max(1, steps - 1)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = start[0] + (goal[0] - start[0]) * t
            pose.pose.position.y = start[1] + (goal[1] - start[1]) * t
            pose.pose.position.z = start[2] + (goal[2] - start[2]) * t
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _goal_pose(self, goal):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
        msg.pose.orientation.w = 1.0
        return msg

    def _frontier_markers(self):
        arr = MarkerArray()
        marker = self._cube_list("frontiers", 7001, (0.0, 0.6, 1.0, 0.75), 0.35)
        for cell in self.map_state.get("frontier_cells", [])[:80]:
            p = grid_to_world(self.spec, tuple(cell))
            point = Point()
            point.x, point.y, point.z = p
            marker.points.append(point)
        arr.markers.append(marker)
        return arr

    def _viewpoint_markers(self):
        arr = MarkerArray()
        for idx, cell in enumerate(self.map_state.get("frontier_cells", [])[:12]):
            p = grid_to_world(self.spec, tuple(cell))
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "candidate_viewpoints"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = p
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.45
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (0.1, 1.0, 0.2, 0.82)
            arr.markers.append(marker)
        return arr

    def _goal_marker(self, goal):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "selected_goal"
        marker.id = 8001
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = goal
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.75
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.85, 0.0, 1.0)
        return marker

    def _text_marker(self, ns, marker_id, text, xyz):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.45
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 1.0, 1.0, 1.0)
        marker.text = text
        return marker

    def _cube_list(self, ns, marker_id, color, scale):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker


def str_msg():
    from std_msgs.msg import String

    return String


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
