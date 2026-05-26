from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("air_bringup"), "config", "air_planning.yaml"])
    rviz_config = PathJoinSubstitution([FindPackageShare("air_bringup"), "rviz", "air_planning.rviz"])
    use_rviz = LaunchConfiguration("rviz")

    common = {"parameters": [config], "output": "screen"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz2."),
            Node(package="air_world_provider", executable="world_provider", name="air_world_provider", **common),
            Node(package="air_global_planner", executable="astar_3d_planner", name="air_global_planner", **common),
            Node(
                package="air_trajectory_generator",
                executable="trajectory_generator",
                name="air_trajectory_generator",
                **common,
            ),
            Node(package="air_uav_simulator", executable="uav_simulator", name="air_uav_simulator", **common),
            Node(package="air_mission_manager", executable="mission_manager", name="air_mission_manager", **common),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
