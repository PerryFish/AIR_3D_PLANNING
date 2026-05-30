# Garage V1 TARE Reference RViz View

## Config

Default clean mode now loads:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_reference.rviz
```

Launch mode mapping:

```text
rviz_view_mode:=clean          -> garage_v1_tare_reference.rviz
rviz_view_mode:=tare_reference -> garage_v1_tare_reference.rviz
rviz_view_mode:=tare_like      -> garage_v1_tare_like.rviz
rviz_view_mode:=debug          -> garage_v1_debug.rviz
rviz_view_mode:=voxel          -> garage_v1_voxel.rviz
```

## Default On

```text
Garage Static Structure Cloud  /exploration/garage_structure_cloud
Observed Structure Cloud       /exploration/observed_structure_cloud
Local Sensor Cloud             /exploration/local_sensor_cloud
UAV Trajectory                 /exploration/trajectory_path
Planned Path                   /aerial_exploration/path
Current Goal                   /aerial_exploration/selected_goal_marker
Coverage Text                  /exploration/coverage_text
```

## Default Off

```text
Frontier Candidates
Debug Local Planning Box
Debug Observed Free Voxel Cloud
Debug Frontier Viewpoints
```

## Visual Style

- Background: black.
- Static garage cloud: dark grey points, `0.02 m`.
- Observed structure cloud: light grey/white points, `0.02 m`.
- Local sensor cloud: pale blue points, `0.03 m`.
- Trajectory: thin blue line, `0.02 m`.
- Planned path: thin green line, `0.025 m`.

This restores the TARE-like visual hierarchy: structure first, then live observed scan/map, then lightweight path overlays.

## Difference From TARE Original

AIR still does not run the full TARE vehicle simulator, terrain mapper, or FAST-LIVO pipeline. The static structure background is generated from the TARE garage preview point cloud, and the observed map is produced by AIR's simulated local sensor fallback.
