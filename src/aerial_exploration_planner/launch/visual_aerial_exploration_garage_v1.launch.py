import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


SOURCE_WORLD = "/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/worlds/garage_v1.world"


def garage_world_path():
    share = get_package_share_directory("aerial_exploration_planner")
    install_world = os.path.join(share, "virtual_env", "garage_v1", "worlds", "garage_v1.world")
    if os.path.exists(install_world):
        return install_world
    if os.path.exists(SOURCE_WORLD):
        return SOURCE_WORLD
    raise FileNotFoundError(f"garage_v1 world not found: {install_world} or {SOURCE_WORLD}")


def rviz_config_path(context):
    share = get_package_share_directory("aerial_exploration_planner")
    profile = context.perform_substitution(LaunchConfiguration("rviz_profile")).strip().lower()
    legacy_mode = context.perform_substitution(LaunchConfiguration("rviz_view_mode")).strip().lower()
    names = {
        "tare_edge_replay": "garage_v1_tare_edge_replay.rviz",
        "tare_edge_like": "garage_v1_tare_edge_replay.rviz",
        "tare_v1_replay": "garage_v1_tare_v1_replay.rviz",
        "clean": "garage_v1_tare_reference.rviz",
        "tare_reference": "garage_v1_tare_reference.rviz",
        "tare_like": "garage_v1_tare_like.rviz",
        "air_debug": "garage_v1_debug.rviz",
        "debug": "garage_v1_debug.rviz",
        "voxel": "garage_v1_voxel.rviz",
    }
    mode = legacy_mode if legacy_mode != "clean" and profile == "tare_v1_replay" else profile
    if mode not in names:
        raise RuntimeError(f"Unsupported rviz_profile '{mode}'. Use tare_edge_replay, tare_v1_replay, clean, air_debug, debug, or voxel.")
    path = os.path.join(share, "rviz", names[mode])
    if not os.path.exists(path):
        raise FileNotFoundError(f"Garage RViz config not found: {path}")
    return path


def rviz_process(context):
    return [ExecuteProcess(cmd=["rviz2", "-d", rviz_config_path(context)], output="screen")]


def use_tare_replay(context):
    return context.perform_substitution(LaunchConfiguration("rviz_profile")).strip().lower() in ("tare_edge_replay", "tare_edge_like", "tare_v1_replay")


def generate_launch_description():
    pkg = FindPackageShare("aerial_exploration_planner")
    config = PathJoinSubstitution([pkg, "config", "garage_v1_exploration.yaml"])
    gazebo_launch = PathJoinSubstitution([pkg, "launch", "gazebo_garage_v1.launch.py"])
    simple_uav_sdf = PathJoinSubstitution([pkg, "models", "simple_uav", "model.sdf"])
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    sensor_mapping = LaunchConfiguration("sensor_mapping")
    synthetic_mapping = LaunchConfiguration("synthetic_mapping")
    gazebo_uav_visual = LaunchConfiguration("gazebo_uav_visual")
    gazebo_trail_visual = LaunchConfiguration("gazebo_trail_visual")
    gazebo_waypoint_visual = LaunchConfiguration("gazebo_waypoint_visual")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_z = LaunchConfiguration("start_z")
    start_yaw = LaunchConfiguration("start_yaw")
    start_x_float = ParameterValue(start_x, value_type=float)
    start_y_float = ParameterValue(start_y, value_type=float)
    start_z_float = ParameterValue(start_z, value_type=float)
    start_yaw_float = ParameterValue(start_yaw, value_type=float)
    start_overrides = {
        "garage_v1.start_pose.x": start_x_float,
        "garage_v1.start_pose.y": start_y_float,
        "garage_v1.start_pose.z": start_z_float,
        "garage_v1.start_pose.yaw": start_yaw_float,
        "uav.initial_x": start_x_float,
        "uav.initial_y": start_y_float,
        "uav.initial_z": start_z_float,
        "uav.initial_yaw": start_yaw_float,
        "grid.origin_x": start_x_float,
        "grid.origin_y": start_y_float,
    }
    common = {"parameters": [config, start_overrides], "output": "screen"}
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("sensor_mapping", default_value="true"),
            DeclareLaunchArgument("observed_coverage", default_value="true"),
            DeclareLaunchArgument("synthetic_mapping", default_value="false"),
            DeclareLaunchArgument("gazebo_uav_visual", default_value="true"),
            DeclareLaunchArgument("gazebo_trail_visual", default_value="true"),
            DeclareLaunchArgument("gazebo_waypoint_visual", default_value="true"),
            DeclareLaunchArgument("rviz_profile", default_value="tare_edge_replay"),
            DeclareLaunchArgument("rviz_view_mode", default_value="clean"),
            DeclareLaunchArgument("tare_rviz_replay_bridge", default_value="true"),
            DeclareLaunchArgument("env", default_value="garage_v1"),
            DeclareLaunchArgument("world", default_value=garage_world_path()),
            DeclareLaunchArgument("start_x", default_value="-23.817"),
            DeclareLaunchArgument("start_y", default_value="-46.018"),
            DeclareLaunchArgument("start_z", default_value="1.6"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={"gui": gui, "world": LaunchConfiguration("world"), "use_garage_wall_proxy": "true"}.items(),
            ),
            Node(
                package="aerial_exploration_planner",
                executable="sensor_mapping_node",
                name="sensor_mapping_node",
                condition=IfCondition(sensor_mapping),
                **common,
            ),
            Node(
                package="aerial_exploration_planner",
                executable="synthetic_mapping_node",
                name="synthetic_mapping_node",
                condition=IfCondition(synthetic_mapping),
                **common,
            ),
            Node(package="aerial_exploration_planner", executable="simple_uav_follower_node", name="simple_uav_follower_node", **common),
            Node(package="aerial_exploration_planner", executable="aerial_exploration_node", name="aerial_exploration_node", **common),
            Node(package="aerial_exploration_planner", executable="exploration_metrics_node", name="exploration_metrics_node", **common),
            Node(package="aerial_exploration_planner", executable="garage_structure_cloud_node", name="garage_structure_cloud_node", **common),
            Node(
                package="aerial_exploration_planner",
                executable="tare_rviz_replay_bridge_node",
                name="tare_rviz_replay_bridge_node",
                condition=IfCondition(LaunchConfiguration("tare_rviz_replay_bridge")),
                **common,
            ),
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
                        "model_name": "simple_uav_garage_v1",
                        "visual_z_offset": 0.0,
                        "spawn_x": start_x_float,
                        "spawn_y": start_y_float,
                        "spawn_z": start_z_float,
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
                        "min_distance": 0.25,
                        "trail_radius": 0.06,
                        "goal_radius": 0.18,
                        "trail_z_offset": 0.0,
                        "goal_z_offset": 0.0,
                        "enable_waypoint_marker": gazebo_waypoint_visual,
                    },
                ],
                output="screen",
                condition=IfCondition(gazebo_trail_visual),
            ),
            OpaqueFunction(function=rviz_process, condition=IfCondition(rviz)),
        ]
    )
