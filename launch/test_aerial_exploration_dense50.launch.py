from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("aerial_exploration_planner"), "config", "aerial_exploration.yaml"])
    common = {"parameters": [config], "output": "screen"}
    return LaunchDescription(
        [
            Node(package="aerial_exploration_planner", executable="synthetic_mapping_node", name="synthetic_mapping_node", **common),
            Node(package="aerial_exploration_planner", executable="aerial_exploration_node", name="aerial_exploration_node", **common),
            Node(package="aerial_exploration_planner", executable="exploration_metrics_node", name="exploration_metrics_node", **common),
            Node(package="aerial_exploration_planner", executable="mode_manager_node", name="mode_manager_node", **common),
        ]
    )
