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
    simple_uav_sdf = PathJoinSubstitution([pkg, "models", "simple_uav", "model.sdf"])
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    gazebo_uav_visual = LaunchConfiguration("gazebo_uav_visual")
    gazebo_trail_visual = LaunchConfiguration("gazebo_trail_visual")
    gazebo_waypoint_visual = LaunchConfiguration("gazebo_waypoint_visual")
    common = {"parameters": [config], "output": "screen"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gazebo_uav_visual", default_value="true"),
            DeclareLaunchArgument("gazebo_trail_visual", default_value="true"),
            DeclareLaunchArgument("gazebo_waypoint_visual", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"gui": gui, "use_sim_time": "true"}.items(),
            ),
            Node(package="aerial_exploration_planner", executable="synthetic_mapping_node", name="synthetic_mapping_node", **common),
            Node(package="aerial_exploration_planner", executable="simple_uav_follower_node", name="simple_uav_follower_node", **common),
            Node(package="aerial_exploration_planner", executable="aerial_exploration_node", name="aerial_exploration_node", **common),
            Node(package="aerial_exploration_planner", executable="exploration_metrics_node", name="exploration_metrics_node", **common),
            Node(package="aerial_exploration_planner", executable="mode_manager_node", name="mode_manager_node", **common),
            Node(
                package="aerial_exploration_planner",
                executable="gazebo_uav_visualizer",
                name="gazebo_uav_visualizer",
                parameters=[
                    config,
                    {
                        "pose_topic": "/state_estimation",
                        "model_sdf_path": simple_uav_sdf,
                        "model_name": "simple_uav",
                        "visual_z_offset": 0.25,
                    },
                ],
                output="screen",
                condition=IfCondition(gazebo_uav_visual),
            ),
            Node(
                package="aerial_exploration_planner",
                executable="gazebo_trail_visualizer",
                name="gazebo_trail_visualizer",
                parameters=[
                    config,
                    {
                        "pose_topic": "/state_estimation",
                        "goal_topic": "/aerial_exploration/goal",
                        "max_points": 300,
                        "enable_waypoint_marker": gazebo_waypoint_visual,
                    },
                ],
                output="screen",
                condition=IfCondition(gazebo_trail_visual),
            ),
            ExecuteProcess(cmd=["rviz2", "-d", rviz_config], output="screen", condition=IfCondition(rviz)),
        ]
    )
