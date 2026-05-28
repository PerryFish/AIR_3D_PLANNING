from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    model_path = LaunchConfiguration("gazebo_model_path")
    default_world = PathJoinSubstitution(
        [FindPackageShare("aerial_exploration_planner"), "worlds", "dense50_ground_footprint.world"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gazebo_model_path", default_value="/usr/share/gazebo-11/models"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
            ExecuteProcess(
                cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", world],
                output="screen",
                condition=IfCondition(gui),
            ),
            ExecuteProcess(
                cmd=["gzserver", "--verbose", "-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so", world],
                output="screen",
                condition=UnlessCondition(gui),
            ),
        ]
    )
