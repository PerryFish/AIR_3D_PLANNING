import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .grid_model import GridSpec, make_dense50_ground_footprint


class SyntheticMappingNode(Node):
    def __init__(self):
        super().__init__("synthetic_mapping_node")
        self.declare_parameter("grid.x_cells", 20)
        self.declare_parameter("grid.y_cells", 20)
        self.declare_parameter("grid.z_cells", 6)
        self.declare_parameter("grid.resolution", 1.0)
        self.declare_parameter("dense50.ground_footprint_occupancy_ratio", 0.50)
        self.declare_parameter("mapping.publish_period_sec", 0.25)
        self.spec = GridSpec(
            int(self.get_parameter("grid.x_cells").value),
            int(self.get_parameter("grid.y_cells").value),
            int(self.get_parameter("grid.z_cells").value),
            float(self.get_parameter("grid.resolution").value),
            float(self.get_parameter("dense50.ground_footprint_occupancy_ratio").value),
        )
        self.ground_occupied = make_dense50_ground_footprint(self.spec)
        self.map_pub = self.create_publisher(String, "/aerial_exploration/map_state", 10)
        self.ground_pub = self.create_publisher(String, "/aerial_exploration/ground_footprint_occupancy", 10)
        self.timer = self.create_timer(float(self.get_parameter("mapping.publish_period_sec").value), self.publish_state)
        self.get_logger().info("Synthetic dense50 mapping node started")

    def publish_state(self):
        ground_count = len(self.ground_occupied)
        payload = {
            "x_cells": self.spec.x_cells,
            "y_cells": self.spec.y_cells,
            "z_cells": self.spec.z_cells,
            "resolution": self.spec.resolution,
            "total_voxels": self.spec.total_voxels,
            "ground_total_cells": self.spec.ground_total_cells,
            "ground_occupied_footprint_cells": ground_count,
            "ground_footprint_occupancy_ratio": ground_count / self.spec.ground_total_cells,
            "target_ground_footprint_occupancy_ratio": self.spec.target_ground_ratio,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.map_pub.publish(msg)
        self.ground_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
