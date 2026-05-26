# AIR_3D_PLANNING

AIR_3D_PLANNING is a ROS2 Humble UAV 3D aerial path planning MVP. It supports 3D A*, 3D obstacle visualization, path smoothing, UAV kinematic simulation, dynamic 3D goal replanning, UAV motion diagnostics, and RViz2 visualization.

This repository is intentionally lightweight: it does not depend on PX4, Gazebo, MAVROS, OctoMap, or ESDF. The goal is a reproducible ROS2 Humble aerial planning demo that can be cloned, built, and launched quickly.

## System Requirements

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- colcon
- RViz2

## Quick Clone

```bash
git clone https://github.com/PerryFish/AIR_3D_PLANNING.git
cd AIR_3D_PLANNING
```

## Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

If `rosdep` reports that `ament_python` cannot be resolved on your machine, use the provided build script; it skips that non-system rosdep key.

## Build

```bash
./scripts/build_air.sh
```

## Launch RViz Demo

```bash
./scripts/launch_air_demo.sh
```

Equivalent manual command:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch air_bringup air_planning_demo.launch.py
```

Headless launch without RViz:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch air_bringup air_planning_demo.launch.py rviz:=false
```

## Check Topics

Run this while the demo is active:

```bash
./scripts/check_air_topics.sh
```

## Check UAV Motion

Run this while the demo is active:

```bash
./scripts/check_uav_motion.sh
```

The script samples `/air/state_estimation` for 10 seconds and reports total motion and z change.

## Send A New 3D Goal

Run this while the demo is active:

```bash
./scripts/send_new_goal.sh
```

The example goal is:

```text
x = -6.0
y = 7.0
z = 5.0
```

## 50% 障碍物占据率空中规划测试

Dense50 测试用于验证高占据率 3D voxel 场景下的空中规划能力。核心指标包括 `/air/world_status` 中的实际占据率、`/air/planner_status` 中的规划状态，以及 `/air/planning_metrics` 中的规划时间、扩展节点数、路径长度和 z 范围。

启动 dense50 RViz demo：

```bash
./scripts/launch_dense50_demo.sh
```

检查 dense50 状态：

```bash
./scripts/check_dense50_status.sh
```

运行多 seed benchmark：

```bash
./scripts/run_dense50_benchmark.sh
```

benchmark 输出：

- `benchmark_results/dense50_benchmark.csv`
- `benchmark_results/dense50_benchmark_report.md`

判断成功的依据：

- `/air/world_status` 显示 `actual_occupancy_ratio` 接近 `0.50`
- `/air/planner_status` 显示 `PLAN_SUCCESS`
- `/air/planning_metrics` 包含 `planning_time_sec`、`expanded_nodes`、`path_length_m`
- UAV 能沿路径运动并接近 goal
- RViz 中可见 dense 3D obstacle field、global path、smoothed path 和 UAV trail

常见失败原因：

- 50% 随机地图本身不连通
- `inflation_radius` 太大导致通道被膨胀封闭
- `resolution` 太小导致搜索慢
- `max_planning_time` 太短
- `heuristic_weight` 太小导致扩展节点过多

## Main ROS2 Packages

- `air_planning_msgs`
- `air_world_provider`
- `air_global_planner`
- `air_trajectory_generator`
- `air_uav_simulator`
- `air_mission_manager`
- `air_bringup`

## Architecture

```text
air_mission_manager
  -> /air/start
  -> /air/goal

air_world_provider
  -> /air/occupancy_markers
  -> /air/visualization/markers
  -> /air/world_status
  -> /air/world_status_marker

air_global_planner
  <- /air/start
  <- /air/goal
  <- /air/occupancy_markers
  -> /air/global_path
  -> /air/planner_status
  -> /air/planning_metrics

air_trajectory_generator
  <- /air/global_path
  -> /air/smoothed_path
  -> /air/trajectory

air_uav_simulator
  <- /air/smoothed_path
  -> /air/state_estimation
  -> /air/uav_marker
  -> /air/uav_trail
  -> /air/current_waypoint_marker
  -> /air/uav_status_marker
  -> /tf
```

## Main Topics

- `/air/start`
- `/air/goal`
- `/air/global_path`
- `/air/smoothed_path`
- `/air/state_estimation`
- `/air/planner_status`
- `/air/planning_metrics`
- `/air/world_status`
- `/air/world_status_marker`
- `/air/uav_marker`
- `/air/uav_trail`
- `/air/current_waypoint_marker`
- `/air/uav_status_marker`
- `/air/occupancy_markers`
- `/air/visualization/markers`

## Current Support

- 3D A* with 26-connected neighbor expansion.
- 3D obstacle visualization.
- Collision checking with obstacle inflation.
- Path smoothing and sampled waypoint generation.
- UAV kinematic simulation in x/y/z/yaw.
- Dynamic goal replanning through `/air/goal`.
- RViz visualization.
- UAV trail and status marker visualization.
- UAV motion diagnostics script.

## Current Limitations

- No real PX4 flight controller.
- No Gazebo real dynamics.
- No real sensor-based mapping.
- No OctoMap or ESDF real-time map.
- No motor-level control.
- No physical UAV safety stack.

## Reproducible Demo Workflow

1. Build the workspace:

```bash
./scripts/build_air.sh
```

2. Launch the demo:

```bash
./scripts/launch_air_demo.sh
```

3. In another terminal, verify topics:

```bash
cd AIR_3D_PLANNING
./scripts/check_air_topics.sh
```

4. Verify UAV movement:

```bash
./scripts/check_uav_motion.sh
```

5. Send a new 3D goal:

```bash
./scripts/send_new_goal.sh
```

6. In RViz, observe the obstacle boxes, global path, smoothed path, UAV marker, and orange `/air/uav_trail`.

## Troubleshooting

### `ros2 launch` Cannot Find A Package

Build and source the workspace:

```bash
./scripts/build_air.sh
source install/setup.bash
ros2 pkg list | grep air_
```

### RViz Is Blank

Check that the fixed frame is `map`, then verify marker topics:

```bash
ros2 topic list | grep /air
ros2 topic echo /air/occupancy_markers --once
```

### UAV Looks Stationary

Check odometry and motion:

```bash
ros2 topic echo /air/state_estimation
./scripts/check_uav_motion.sh
```

Also check that logs are not repeatedly printing `UAV accepted 3D trajectory`; repeated trajectory acceptance can reset tracking progress. Confirm RViz displays `/air/uav_trail`.

### `/air/state_estimation` Has No Data

Confirm the simulator is running:

```bash
ros2 node list | grep air_uav_simulator
ros2 topic hz /air/state_estimation
```

### `colcon build` Fails

Source ROS2 first and rebuild:

```bash
source /opt/ros/humble/setup.bash
./scripts/build_air.sh
```

### Forgot To Source The Workspace

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Ctrl+C Exit

Use Ctrl+C once in the launch terminal. The Python nodes use guarded shutdown logic to avoid `rcl_shutdown already called` tracebacks.

## Difference From TARE_V1

TARE_V1 used `vehicle_simulator`, `localPlanner`, `pathFollower`, and fixed-height or ground-like execution assumptions. AIR_3D_PLANNING implements a standalone ROS2 Humble 3D aerial planner and simple UAV kinematic simulator. It does not modify TARE_V1.

## Documentation

- `AIR_PLANNING_ARCHITECTURE.md`
- `DEPLOY_REPORT.md`
- `PX4_INTEGRATION_PLAN.md`
- `UAV_MOTION_DEBUG_REPORT.md`
- `GITHUB_DEPLOYMENT.md`

## Future Roadmap

- dense50 occupancy benchmark.
- Weighted A*.
- Bidirectional A*.
- RRT* fallback.
- PX4 SITL bridge.
- Bimodal ground-air mode manager.
