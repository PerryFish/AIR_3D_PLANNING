import csv
import json
import math
from pathlib import Path
from pathlib import Path as FilesystemPath

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class ExplorationMetricsNode(Node):
    def __init__(self):
        super().__init__("exploration_metrics_node")
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("metrics.csv_path", "results/metrics_dense50.csv")
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.csv_path = FilesystemPath(self.get_parameter("metrics.csv_path").value)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.map_state = {}
        self.planner_state = {}
        self.pose = (0.0, 0.0, 0.0)
        self.last_pose = None
        self.goal = (0.0, 0.0, 0.0)
        self.last_goal = None
        self.path_length = 0.0
        self.done = False
        self.failed_goals = 0
        self.stuck_events = 0
        self.max_no_progress_duration = 0.0
        self.done_pub = self.create_publisher(Bool, "/aerial_exploration/done", 10)
        self.coverage_pub = self.create_publisher(Float32, "/aerial_exploration/coverage_metrics", 10)
        self.create_subscription(String, "/aerial_exploration/map_state", self.map_cb, 10)
        self.create_subscription(String, "/aerial_exploration/planner_state", self.planner_state_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(PoseStamped, "/aerial_exploration/goal", self.goal_cb, 10)
        self.create_subscription(NavPath, "/aerial_exploration/path", self.path_cb, 10)
        self.timer = self.create_timer(0.25, self.publish_metrics)
        self._open_csv()
        self.get_logger().info(f"Exploration metrics node writing {self.csv_path}")

    def _open_csv(self):
        self.csv_file = self.csv_path.open("w", newline="")
        self.fields = [
            "timestamp",
            "coverage",
            "synthetic_coverage",
            "observed_coverage",
            "done",
            "explored_voxels",
            "free_voxels",
            "occupied_voxels",
            "unknown_voxels",
            "ground_total_cells",
            "ground_occupied_footprint_cells",
            "ground_footprint_occupancy_ratio",
            "target_ground_footprint_occupancy_ratio",
            "failed_goals",
            "stuck_events",
            "backtrack_events",
            "frontier_goals",
            "goal_source",
            "frontier_cluster_id",
            "branch_point_count",
            "dead_end_count",
            "backtrack_count",
            "returned_to_branch_count",
            "current_region",
            "start_pose_x",
            "start_pose_y",
            "start_pose_z",
            "start_pose_yaw",
            "distance_from_start",
            "max_no_progress_duration",
            "robot_x",
            "robot_y",
            "robot_z",
            "goal_x",
            "goal_y",
            "goal_z",
            "path_length",
            "goal_changed",
            "pose_changed",
            "newly_observed_voxels",
            "observed_voxels",
            "frontier_count",
            "sensor_cloud_points",
            "mapping_source",
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fields)
        self.writer.writeheader()
        self.csv_file.flush()

    def map_cb(self, msg):
        self.map_state = json.loads(msg.data)

    def planner_state_cb(self, msg):
        self.planner_state = json.loads(msg.data)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def goal_cb(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y, p.z)

    def path_cb(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses]
        self.path_length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))

    def publish_metrics(self):
        if not self.map_state:
            return
        observed_coverage = float(self.map_state.get("observed_coverage", self.map_state.get("coverage", 0.0)))
        synthetic_coverage = float(self.map_state.get("synthetic_coverage", self.map_state.get("coverage", observed_coverage)))
        coverage = observed_coverage
        self.done = self.done or coverage >= self.done_threshold
        done_msg = Bool()
        done_msg.data = bool(self.done)
        self.done_pub.publish(done_msg)
        cov_msg = Float32()
        cov_msg.data = coverage
        self.coverage_pub.publish(cov_msg)
        pose_changed = self.last_pose is None or math.dist(self.pose, self.last_pose) > 0.02
        goal_changed = self.last_goal is None or math.dist(self.goal, self.last_goal) > 0.2
        row = {
            "timestamp": f"{self.get_clock().now().nanoseconds / 1e9:.3f}",
            "coverage": f"{coverage:.6f}",
            "synthetic_coverage": f"{synthetic_coverage:.6f}",
            "observed_coverage": f"{observed_coverage:.6f}",
            "done": str(bool(self.done)),
            "explored_voxels": int(self.map_state.get("explored_voxels", 0)),
            "free_voxels": int(self.map_state.get("free_voxels", 0)),
            "occupied_voxels": int(self.map_state.get("occupied_voxels", 0)),
            "unknown_voxels": int(self.map_state.get("unknown_voxels", 0)),
            "ground_total_cells": int(self.map_state.get("ground_total_cells", 0)),
            "ground_occupied_footprint_cells": int(self.map_state.get("ground_occupied_footprint_cells", 0)),
            "ground_footprint_occupancy_ratio": f"{float(self.map_state.get('ground_footprint_occupancy_ratio', 0.0)):.6f}",
            "target_ground_footprint_occupancy_ratio": f"{float(self.map_state.get('target_ground_footprint_occupancy_ratio', 0.0)):.6f}",
            "failed_goals": int(self.planner_state.get("failed_goals", self.failed_goals)),
            "stuck_events": int(self.planner_state.get("stuck_events", self.stuck_events)),
            "backtrack_events": int(self.planner_state.get("backtrack_events", 0)),
            "frontier_goals": int(self.planner_state.get("frontier_goals", 0)),
            "goal_source": self.planner_state.get("goal_source", "unknown"),
            "frontier_cluster_id": int(self.planner_state.get("frontier_cluster_id", -1)),
            "branch_point_count": int(self.planner_state.get("branch_point_count", self.planner_state.get("branch_points", 0))),
            "dead_end_count": int(self.planner_state.get("dead_end_count", 0)),
            "backtrack_count": int(self.planner_state.get("backtrack_count", self.planner_state.get("backtrack_events", 0))),
            "returned_to_branch_count": int(self.planner_state.get("returned_to_branch_count", 0)),
            "current_region": self.planner_state.get("current_region", "unknown"),
            "start_pose_x": f"{float(self.planner_state.get('start_pose_x', 0.0)):.3f}",
            "start_pose_y": f"{float(self.planner_state.get('start_pose_y', 0.0)):.3f}",
            "start_pose_z": f"{float(self.planner_state.get('start_pose_z', 0.0)):.3f}",
            "start_pose_yaw": f"{float(self.planner_state.get('start_pose_yaw', 0.0)):.3f}",
            "distance_from_start": f"{float(self.planner_state.get('distance_from_start', 0.0)):.3f}",
            "max_no_progress_duration": f"{self.max_no_progress_duration:.3f}",
            "robot_x": f"{self.pose[0]:.3f}",
            "robot_y": f"{self.pose[1]:.3f}",
            "robot_z": f"{self.pose[2]:.3f}",
            "goal_x": f"{self.goal[0]:.3f}",
            "goal_y": f"{self.goal[1]:.3f}",
            "goal_z": f"{self.goal[2]:.3f}",
            "path_length": f"{self.path_length:.3f}",
            "goal_changed": str(goal_changed),
            "pose_changed": str(pose_changed),
            "newly_observed_voxels": int(self.map_state.get("newly_observed_voxels", 0)),
            "observed_voxels": int(self.map_state.get("observed_voxels", self.map_state.get("explored_voxels", 0))),
            "frontier_count": int(self.map_state.get("frontier_count", len(self.map_state.get("frontier_cells", [])))),
            "sensor_cloud_points": int(self.map_state.get("sensor_cloud_points", 0)),
            "mapping_source": self.map_state.get("mapping_source", "synthetic_mapping_node"),
        }
        self.writer.writerow(row)
        self.csv_file.flush()
        self.last_pose = self.pose
        self.last_goal = self.goal

    def destroy_node(self):
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
