# Garage V1 RViz Clean TARE Style

## View Modes

Garage launch now supports:

```bash
rviz_view_mode:=clean
rviz_view_mode:=debug
rviz_view_mode:=voxel
```

Default:

```bash
rviz_view_mode:=clean
```

## Clean Mode

Config:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_reference.rviz
```

Clean mode is the recommended operator view. It opens:

- Garage Static Structure Cloud from the TARE preview PLY
- Observed Structure Cloud from online simulated local sensor observations
- Local Sensor Cloud
- Frontier Candidates, small/muted
- UAV Odometry
- UAV Trajectory
- Planned Path
- Current Goal
- Coverage Text

Clean mode closes:

- Debug Local Planning Box
- Debug Frontier Viewpoints
- Observed Free Voxel Cloud
- Full voxel cube markers

## Debug Mode

Config:

```text
src/aerial_exploration_planner/rviz/garage_v1_debug.rviz
```

Debug mode enables the local planning box, local obstacle cloud, free cloud, frontier viewpoints, and other details useful while tuning planner behavior.

## Voxel Mode

Config:

```text
src/aerial_exploration_planner/rviz/garage_v1_voxel.rviz
```

Voxel mode intentionally shows observed/free/occupied/frontier voxel centers and cube markers. It is for coverage debugging, not for normal visual inspection.

## TARE Reference

The TARE reference files copied into AIR are:

- `src/virtual_env/garage_v1/rviz/tare_planner_ground.rviz`
- `src/virtual_env/garage_v1/rviz/vehicle_simulator.rviz`

The AIR clean mode follows the same readability direction: dark background, small structure points, path overlays, and optional local planning markers.

The static cloud is now generated from:

```text
/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply
```

and transformed with the same `(-23.817, -46.018, 0.073)` pose used by Gazebo.
