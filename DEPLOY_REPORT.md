# Deployment Report

Date: 2026-05-26

## Result

- Workspace created: yes, `/home/nuaa/ZHY/TARE_V2_AIR`.
- TARE_V1 modified: no.
- Packages created:
  - `air_planning_msgs`
  - `air_world_provider`
  - `air_global_planner`
  - `air_trajectory_generator`
  - `air_uav_simulator`
  - `air_mission_manager`
  - `air_bringup`
- Build result: success.
- Build log: `/home/nuaa/ZHY/TARE_V2_AIR/build_log.txt`.
- Headless demo result: success.
- RViz config created: `/home/nuaa/ZHY/TARE_V2_AIR/src/air_bringup/rviz/air_planning.rviz`.

## Validation Evidence

The final headless test started:

- `air_world_provider`
- `air_global_planner`
- `air_trajectory_generator`
- `air_uav_simulator`
- `air_mission_manager`

Observed topics:

- `/air/global_path`
- `/air/goal`
- `/air/occupancy_markers`
- `/air/planner_status`
- `/air/smoothed_path`
- `/air/start`
- `/air/state_estimation`
- `/air/trajectory`
- `/air/uav_marker`
- `/air/visualization/markers`
- `/tf`

`/air/state_estimation` published at about 20 Hz.

Planner status:

```text
OK: 3D A* path generated with 18 waypoints, z range 1.00-4.50 m
```

The smoothed path contained sampled z values from 1.0 m through 4.5 m and ended at 4.0 m, so z is not fixed.

## RViz

The launch file starts RViz by default:

```bash
ros2 launch air_bringup air_planning_demo.launch.py
```

The headless validation used `rviz:=false` because automated terminal validation does not need a GUI. The RViz config contains displays for obstacle markers, global path, smoothed path, trajectory, UAV marker, and TF with fixed frame `map`.

## External Code

No external GitHub repository was imported or compiled into this workspace. PX4 offboard examples, Fast-Planner, EGO-Planner, FUEL, and OctoMap-style planners were treated as references only. This avoids pulling incompatible ROS1 or dependency-heavy code into the Humble MVP.

## Current Limits

- Kinematic UAV simulator only.
- Static obstacle boxes only.
- No PX4 bridge yet.
- No live 3D mapping.
- No dynamic obstacle avoidance.

## Next Steps

1. Add `air_px4_bridge` for PX4 offboard setpoints.
2. Add frame conversion and failsafe handling.
3. Replace static box map with OctoMap, Voxblox, or ESDF input.
4. Add planner unit tests for blocked start/goal, no-path cases, and new-goal replanning.
