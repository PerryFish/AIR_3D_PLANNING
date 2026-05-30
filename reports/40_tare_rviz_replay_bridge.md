# TARE RViz Replay Bridge

## Node

Added:

```text
src/aerial_exploration_planner/aerial_exploration_planner/tare_rviz_replay_bridge_node.py
```

This is a visualization bridge only. It does not change exploration planning or mapping decisions.

## Topic Mapping

| TARE topic | AIR source |
|---|---|
| `/overall_map` | extracted TARE garage edge cloud |
| `/registered_scan` | local range-filtered edge cloud around `/state_estimation` |
| `/explored_areas` | accumulated observed edge cloud |
| `/terrain_map` | local edge scan cloud |
| `/terrain_map_ext` | lightly expanded local edge scan cloud |
| `/path` | `/exploration/trajectory_path` |
| `/local_path` | `/aerial_exploration/path` |
| `/global_path` | `/aerial_exploration/path` fallback |
| `/way_point` | `/aerial_exploration/goal` converted to PointStamped |
| `/free_paths` | trajectory path sampled as a PointCloud2 approximation |
| `/uncovered_cloud` | `/exploration/frontier_cloud` |
| `/uncovered_frontier_cloud` | `/exploration/frontier_cloud` |
| `/tare_visualizer/local_planning_horizon` | `/exploration/local_planning_box` |
| `/tare_visualizer/exploring_subspaces` | `/exploration/local_planning_box` fallback |

## Notes

The bridge now keeps dense preview/surface data only on `/exploration/debug_surface_cloud`. `/overall_map` no longer uses the full surface cloud by default.

`/free_paths` and `/global_path` are approximations because AIR currently does not publish the same TARE local planner candidate-path set. The bridge keeps the TARE topic contract available for RViz style reproduction.
