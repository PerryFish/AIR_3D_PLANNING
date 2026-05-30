# TARE Edge Replay RViz Profile

## Profile

New default profile:

```text
rviz_profile:=tare_edge_replay
src/aerial_exploration_planner/rviz/garage_v1_tare_edge_replay.rviz
```

`scripts/run_garage_v1_visual_exploration.sh` now defaults to this profile.

## Default Layers

Enabled:

- `/overall_map`: edge cloud, grey/white points.
- `/registered_scan`: local visible edge points, white, decay 10.
- `/explored_areas`: accumulated observed edge points, cyan/blue.
- `/terrain_map`: local edge-derived terrain-like cloud.
- `/path`, `/local_path`, `/global_path`.
- `/way_point`, `/free_paths`, `/exploration/coverage_text`.
- `/exploration/start_pose_marker`: small yellow start arrow at the transformed TARE garage start.

Disabled:

- `/exploration/debug_surface_cloud`.
- voxel cube displays.
- local planning horizon/box.
- uncovered/frontier debug surfaces.

## Data Sources

`tare_rviz_replay_bridge_node.py` now loads `garage_edge_cloud.pcd` first. `/overall_map` and `/exploration/garage_edge_cloud` publish this edge cloud. `/registered_scan` filters edge points around `/state_estimation`, so it is local rather than full-map. `/explored_areas` accumulates only edge points observed by this local filter. `/terrain_map_ext` publishes a lightweight expanded local edge cloud.

The previous full preview/surface cloud is retained only as `/exploration/debug_surface_cloud` and is disabled in the new profile.

The profile still keeps dense surface and voxel debug displays disabled by default after the start-pose alignment update.
