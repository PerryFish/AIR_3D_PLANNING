from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("aerial_exploration_planner")
    config = PathJoinSubstitution([pkg, "config", "aerial_exploration.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "aerial_exploration.rviz"])
    gazebo_launch = PathJoinSubstitution([pkg, "launch", "gazebo_dense50.launch.py"])
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    common = {"parameters": [config], "output": "screen"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"gui": gui, "use_sim_time": "true"}.items(),
            ),
            Node(package="aerial_exploration_planner", executable="synthetic_mapping_node", name="synthetic_mapping_node", **common),
            Node(package="aerial_exploration_planner", executable="simple_uav_follower_node", name="simple_uav_follower_node", **common),
            Node(package="aerial_exploration_planner", executable="aerial_exploration_node", name="aerial_exploration_node", **common),
            Node(package="aerial_exploration_planner", executable="exploration_metrics_node", name="exploration_metrics_node", **common),
            Node(package="aerial_exploration_planner", executable="mode_manager_node", name="mode_manager_node", **common),
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config], output="screen", condition=IfCondition(rviz)),
        ]
    )
