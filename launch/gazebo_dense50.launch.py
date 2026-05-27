from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    default_world = PathJoinSubstitution(
        [FindPackageShare("aerial_exploration_planner"), "worlds", "dense50_ground_footprint.world"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            ExecuteProcess(cmd=["gazebo", "--verbose", world], output="screen", condition=IfCondition(gui)),
            ExecuteProcess(cmd=["gzserver", "--verbose", world], output="screen", condition=UnlessCondition(gui)),
        ]
    )
