# TARE_V2_AIR

TARE_V2_AIR is a ROS2 Humble aerial 3D path planning MVP for Ubuntu 22.04. It is an independent workspace under `/home/nuaa/ZHY/TARE_V2_AIR` and does not modify TARE_V1.

## Difference From TARE_V1

TARE_V1 ran a TARE-style exploration demo with `vehicle_simulator`, `localPlanner`, `pathFollower`, and fixed-height or ground-like execution assumptions. TARE_V2_AIR implements a 3D aerial planner, 3D obstacle checking, path smoothing, and a simple UAV kinematic simulator that tracks changing z coordinates.

## Architecture

```text
air_mission_manager -> /air/start, /air/goal
air_world_provider  -> /air/occupancy_markers, /air/visualization/markers
air_global_planner  -> /air/global_path, /air/planner_status
air_trajectory_generator -> /air/smoothed_path, /air/trajectory
air_uav_simulator   -> /air/state_estimation, /tf, /air/uav_marker
air_bringup         -> launch, config, RViz
```

## Dependencies

- Ubuntu 22.04
- ROS2 Humble
- `python3-colcon-common-extensions`
- Standard ROS2 message packages: `geometry_msgs`, `nav_msgs`, `std_msgs`, `visualization_msgs`, `tf2_ros`

## Build

```bash
cd /home/nuaa/ZHY/TARE_V2_AIR
./scripts/build_air.sh
```

## Launch Demo

```bash
cd /home/nuaa/ZHY/TARE_V2_AIR
./scripts/launch_air_demo.sh
```

Equivalent command:

```bash
source /opt/ros/humble/setup.bash
source /home/nuaa/ZHY/TARE_V2_AIR/install/setup.bash
ros2 launch air_bringup air_planning_demo.launch.py
```

Headless mode:

```bash
ros2 launch air_bringup air_planning_demo.launch.py rviz:=false
```

## Check Topics

```bash
./scripts/check_air_topics.sh
```

Important topics:

- `/air/occupancy_markers`
- `/air/global_path`
- `/air/smoothed_path`
- `/air/trajectory`
- `/air/state_estimation`
- `/air/planner_status`
- `/air/uav_marker`
- `/air/uav_trail`
- `/air/current_waypoint_marker`
- `/air/uav_status_marker`
- `/air/visualization/markers`

## Send A New 3D Goal

```bash
./scripts/send_new_goal.sh
```

The example goal is `x=-6.0, y=7.0, z=5.0`.

## UAV Looks Stationary

If RViz shows the path but the UAV looks stationary:

1. Check state output:

```bash
ros2 topic echo /air/state_estimation
```

2. Run the motion checker while the demo is already running:

```bash
./scripts/check_uav_motion.sh
```

3. Check whether the simulator log repeatedly prints `UAV accepted 3D trajectory`. Repeated acceptance of the same path means trajectory execution is being reset.

4. Confirm RViz displays `/air/uav_trail`; this orange path shows the actual flown trajectory.

5. Check `uav.max_speed` in `src/air_bringup/config/air_planning.yaml`; too small a value makes motion hard to see.

6. Check `uav.goal_tolerance`; too large a value can skip nearby waypoints too aggressively.

7. Check whether some node is continuously publishing the same `/air/goal` or `/air/start`, which can force repeated replanning.

## Current Support

- 3D A* with 26-connected neighbor expansion.
- 3D obstacle boxes with inflation.
- UAV kinematic simulation in x/y/z/yaw.
- RViz visualization.
- Path simplification and sampling.
- Dynamic goal replanning through `/air/goal`.

## Current Limitations

- No real PX4 flight controller.
- No real UAV dynamics or motor control.
- No real sensor loop or online mapping.
- No ESDF, OctoMap, Voxblox, or dynamic map updates.
- The world model is deterministic box obstacles, not a live perception map.

## Future Extensions

- Connect PX4 SITL.
- Add MAVROS or `px4_ros_com` bridge.
- Add OctoMap, Voxblox, or ESDF map input.
- Replace A* with RRT*, kinodynamic A*, or trajectory optimization.
- Add a dual-mode `mode_manager` for simulation and flight-controller execution.
