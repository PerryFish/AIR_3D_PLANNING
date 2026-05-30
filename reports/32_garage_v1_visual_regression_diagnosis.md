# Garage V1 Visual Regression Diagnosis

## RViz Display Problem

The previous `garage_v1_tare_like.rviz` still enabled too many debug layers by default. It showed:

- `/exploration/structure_cloud`: occupied voxel centers from the simulated local sensor map.
- `/exploration/garage_structure_cloud`: previously also occupied voxel centers, not a static garage reference map.
- `/exploration/local_sensor_cloud`: current simulated local scan points.
- `/exploration/local_obstacle_cloud`: local occupied voxel points.
- `/exploration/frontier_cloud`: frontier candidates.
- `/exploration/local_planning_box`: green wireframe local planning window.

This made the view look like mixed white/colored sparse voxels instead of a TARE-like structural point cloud. The green box came from `/exploration/local_planning_box`; it means the planner's local search volume around the UAV, not a garage wall or room boundary. The red/orange goal/path markers came from `/aerial_exploration/selected_goal_marker`, `/aerial_exploration/path`, and frontier/obstacle debug layers.

The copied TARE reference configs are:

- `src/virtual_env/garage_v1/rviz/tare_planner_ground.rviz`
- `src/virtual_env/garage_v1/rviz/vehicle_simulator.rviz`

TARE's style is closer to a black background, small grey/black structural point clouds, path overlays, selected viewpoints, and optional local horizon markers. Heavy voxel blocks are not the default visual focus.

## Default Layer Decision

Default open:

- Grid
- TF
- UAV Odometry
- static Garage Structure Cloud
- observed occupied structure cloud
- Local Sensor Cloud
- Frontier Candidates, small and muted
- Trajectory Path
- Planned Path
- Current Goal
- Coverage Text

Default disabled or weakened:

- Debug Local Planning Box
- Debug Frontier Viewpoints
- Observed Free Voxel Cloud
- Full voxel cube marker displays

## Gazebo Diagnosis

`garage_v1.world` already loads `model://garage` with a recenter pose:

```xml
<pose>-23.817 -46.018 0.073 0 0 0</pose>
```

The wall proxy was visual-only and had no collision, so it should not block physics, but it was visually too strong. It also had no default camera in the world file, so users could open Gazebo at an unhelpful view and need to zoom/pan manually.

Fixes:

- Keep wall proxy collision disabled.
- Make proxy walls thinner and darker.
- Disable proxy cast shadows.
- Add a `garage_v1_overview` GUI camera in `garage_v1.world`.

## Remaining Gap

The new static structure cloud is generated from the AIR wall proxy geometry. It improves readability and approximates TARE's structural map style, but it is not yet a real registered scan, terrain map, or FAST-LIVO/SLAM output.
