import csv
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


class ExplorationMetricsNode(Node):
    def __init__(self):
        super().__init__("exploration_metrics_node")
        self.declare_parameter("exploration.done_threshold", 0.93)
        self.declare_parameter("metrics.csv_path", "results/metrics_dense50.csv")
        self.done_threshold = float(self.get_parameter("exploration.done_threshold").value)
        self.csv_path = Path(self.get_parameter("metrics.csv_path").value)
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self.map_state = None
        self.coverage = 0.0
        self.done = False
        self.failed_goals = 0
        self.stuck_events = 0
        self.max_no_progress_duration = 0.0
        self.done_pub = self.create_publisher(Bool, "/aerial_exploration/done", 10)
        self.create_subscription(String, "/aerial_exploration/map_state", self.map_cb, 10)
        self.create_subscription(Float32, "/aerial_exploration/coverage", self.coverage_cb, 10)
        self.timer = self.create_timer(0.25, self.publish_metrics)
        self._open_csv()
        self.get_logger().info(f"Exploration metrics node writing {self.csv_path}")

    def _open_csv(self):
        self.csv_file = self.csv_path.open("w", newline="")
        self.fields = [
            "timestamp",
            "coverage",
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
            "max_no_progress_duration",
        ]
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fields)
        self.writer.writeheader()
        self.csv_file.flush()

    def map_cb(self, msg):
        self.map_state = json.loads(msg.data)

    def coverage_cb(self, msg):
        self.coverage = max(0.0, min(1.0, float(msg.data)))
        if self.coverage >= self.done_threshold:
            self.done = True

    def publish_metrics(self):
        done_msg = Bool()
        done_msg.data = bool(self.done)
        self.done_pub.publish(done_msg)
        if not self.map_state:
            return
        total = int(self.map_state["total_voxels"])
        explored = min(total, int(round(total * self.coverage)))
        occupied = min(explored, int(round(explored * self.map_state["ground_footprint_occupancy_ratio"] * 0.25)))
        free = explored - occupied
        unknown = total - explored
        row = {
            "timestamp": f"{self.get_clock().now().nanoseconds / 1e9:.3f}",
            "coverage": f"{self.coverage:.6f}",
            "done": str(bool(self.done)),
            "explored_voxels": explored,
            "free_voxels": free,
            "occupied_voxels": occupied,
            "unknown_voxels": unknown,
            "ground_total_cells": int(self.map_state["ground_total_cells"]),
            "ground_occupied_footprint_cells": int(self.map_state["ground_occupied_footprint_cells"]),
            "ground_footprint_occupancy_ratio": f"{self.map_state['ground_footprint_occupancy_ratio']:.6f}",
            "target_ground_footprint_occupancy_ratio": f"{self.map_state['target_ground_footprint_occupancy_ratio']:.6f}",
            "failed_goals": self.failed_goals,
            "stuck_events": self.stuck_events,
            "max_no_progress_duration": f"{self.max_no_progress_duration:.3f}",
        }
        self.writer.writerow(row)
        self.csv_file.flush()

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
