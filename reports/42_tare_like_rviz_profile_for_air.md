# TARE-like RViz Profile For AIR

## Profile

Added earlier:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_v1_replay.rviz
```

Launch with:

```bash
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=true rviz:=true rviz_profile:=tare_v1_replay
```

Current default has moved to `garage_v1_tare_edge_replay.rviz` via `rviz_profile:=tare_edge_replay`.

## Default Enabled

- Grid and TF.
- `/overall_map`: grey/white static garage structure, point size `0.012`.
- `/registered_scan`: white live local scan, point size `0.04`, decay `10`.
- `/explored_areas`: blue observed structure, point size `0.014`.
- `/path`: green trajectory path, width `0.05`.
- `/local_path`: blue planner path, width `0.08`.
- `/global_path`: cyan planner path, width `0.08`.
- `/way_point`: current goal point.
- `/free_paths`: blue sparse path points.

## Default Disabled

- `/uncovered_cloud`.
- `/uncovered_frontier_cloud`.
- `/tare_visualizer/local_planning_horizon`.
- `/tare_visualizer/exploring_subspaces`.
- image and navigation boundary placeholders.
- AIR voxel cube debug displays.

## Remaining Difference From TARE

The old `tare_v1_replay` profile remains available, but the recommended profile is now edge-based. AIR still uses a replay/simulated local sensor fallback. It is not a complete TARE planner, vehicle simulator, LOAM, FAST-LIVO, or terrain analysis stack. The next step is to replace the simulated local sensor with real Gazebo LiDAR/camera or FAST-LIVO2 output and publish those directly to `/registered_scan`; `/overall_map` should remain a reference/accumulated map rather than a dense face sample.
