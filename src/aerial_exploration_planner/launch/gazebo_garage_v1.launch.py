import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


SOURCE_WORLD = "/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/worlds/garage_v1.world"
SOURCE_MODELS = "/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/models"
SOURCE_PROXY_MODELS = "/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/models"


def _model_path():
    share = get_package_share_directory("aerial_exploration_planner")
    install_models = os.path.join(share, "virtual_env", "garage_v1", "models")
    return ":".join([SOURCE_MODELS, install_models, "/usr/share/gazebo-11/models"])


def _garage_world():
    share = get_package_share_directory("aerial_exploration_planner")
    install_world = os.path.join(
        share,
        "virtual_env",
        "garage_v1",
        "worlds",
        "garage_v1.world",
    )
    if os.path.exists(install_world):
        return install_world
    if os.path.exists(SOURCE_WORLD):
        return SOURCE_WORLD
    raise FileNotFoundError(f"garage_v1 world not found: {install_world} or {SOURCE_WORLD}")


def _proxy_model_sdf():
    share = get_package_share_directory("aerial_exploration_planner")
    install_proxy = os.path.join(
        share,
        "virtual_env",
        "garage_v1",
        "models",
        "garage_wall_proxy",
        "model.sdf",
    )
    source_proxy = os.path.join(
        SOURCE_PROXY_MODELS,
        "garage_wall_proxy",
        "model.sdf",
    )
    return install_proxy if os.path.exists(install_proxy) else source_proxy


def _spawn_proxy_process(context):
    if not _as_bool(context.perform_substitution(LaunchConfiguration("use_garage_wall_proxy"))):
        return []
    model_sdf = _proxy_model_sdf()
    if not os.path.exists(model_sdf):
        raise FileNotFoundError(f"garage wall proxy model not found: {model_sdf}")
    return [
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "gazebo_ros",
                "spawn_entity.py",
                "-entity",
                "garage_wall_proxy",
                "-file",
                model_sdf,
                "-x",
                "0",
                "-y",
                "0",
                "-z",
                "0",
            ],
            output="screen",
        )
    ]


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _gazebo_process(context):
    gui = _as_bool(context.perform_substitution(LaunchConfiguration("gui")))
    verbose = _as_bool(context.perform_substitution(LaunchConfiguration("verbose")))
    paused = _as_bool(context.perform_substitution(LaunchConfiguration("paused")))
    world = context.perform_substitution(LaunchConfiguration("world"))
    if world in ("", "garage_v1"):
        world = _garage_world()
    if not world.endswith(".world") or not os.path.exists(world):
        raise FileNotFoundError(f"Invalid garage_v1 world path passed to Gazebo: {world}")
    cmd = ["gazebo" if gui else "gzserver"]
    if verbose:
        cmd.append("--verbose")
    cmd.extend(["-s", "libgazebo_ros_init.so", "-s", "libgazebo_ros_factory.so"])
    if paused:
        cmd.append("--pause")
    cmd.append(world)
    return [ExecuteProcess(cmd=cmd, output="screen")]


def generate_launch_description():
    garage_world = _garage_world()
    world = LaunchConfiguration("world")
    model_path = LaunchConfiguration("gazebo_model_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("verbose", default_value="true"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("world", default_value=garage_world),
            DeclareLaunchArgument("use_garage_wall_proxy", default_value="true"),
            DeclareLaunchArgument("gazebo_model_path", default_value=_model_path()),
            LogInfo(msg=f"Using garage_v1 world: {garage_world}"),
            SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path),
            OpaqueFunction(function=_gazebo_process),
            OpaqueFunction(function=_spawn_proxy_process),
        ]
    )
