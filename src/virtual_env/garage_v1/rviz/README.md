# TARE V1 Garage RViz Reference Bundle

## Purpose

This directory collects RViz display assets from `/home/nuaa/ZHY/TARE_V1` for the garage scenario. It is intended for later analysis and reproduction of the original TARE_V1 garage RViz style inside AIR_3D_PLANNING_CLEAN.

This collection pass only copied and analyzed display resources. It did not modify AIR exploration algorithms or launch behavior.

## Directory Layout

```text
tare_v1_original_rviz/        copied .rviz files
tare_v1_launch_reference/     garage/RViz/planner/simulator launch references
tare_v1_config_reference/     garage and related TARE planner yaml configs
tare_v1_docs_reference/       TARE run notes, reports, and README references
tare_v1_pointcloud_reference/ small pointcloud/path/boundary references
extracted_topic_summary/      parsed RViz display/topic summary
manifest.yaml                 machine-readable inventory
```

## Copied RViz Files

- `tare_v1_original_rviz/install/tare_planner/share/tare_planner/tare_planner_ground.rviz`
- `tare_v1_original_rviz/install/vehicle_simulator/share/vehicle_simulator/rviz/vehicle_simulator.rviz`
- `tare_v1_original_rviz/install/velodyne_description/share/velodyne_description/rviz/example.rviz`
- `tare_v1_original_rviz/src/autonomous_exploration_development_environment/src/vehicle_simulator/rviz/vehicle_simulator.rviz`
- `tare_v1_original_rviz/src/autonomous_exploration_development_environment/src/velodyne_simulator/velodyne_description/rviz/example.rviz`
- `tare_v1_original_rviz/src/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz`

Most important files:

- `tare_planner_ground.rviz`: TARE planner view, including map, local planning horizon, viewpoints, global/local paths, waypoint, explored/uncovered clouds.
- `vehicle_simulator.rviz`: vehicle simulator view, including `/registered_scan`, `/terrain_map`, `/terrain_map_ext`, `/free_paths`, `/trajectory`, camera image, and waypoint.

## Copied Launch Files

- `tare_v1_launch_reference/install/loam_interface/share/loam_interface/launch/loam_interface.launch`
- `tare_v1_launch_reference/install/local_planner/share/local_planner/launch/local_planner.launch`
- `tare_v1_launch_reference/install/sensor_scan_generation/share/sensor_scan_generation/launch/sensor_scan_generation.launch`
- `tare_v1_launch_reference/install/tare_planner/share/tare_planner/explore.launch`
- `tare_v1_launch_reference/install/tare_planner/share/tare_planner/explore_garage.launch`
- `tare_v1_launch_reference/install/terrain_analysis/share/terrain_analysis/launch/terrain_analysis.launch`
- `tare_v1_launch_reference/install/terrain_analysis_ext/share/terrain_analysis_ext/launch/terrain_analysis_ext.launch`
- `tare_v1_launch_reference/install/vehicle_simulator/share/vehicle_simulator/launch/system_garage.launch`
- `tare_v1_launch_reference/install/vehicle_simulator/share/vehicle_simulator/launch/vehicle_simulator.launch`
- `tare_v1_launch_reference/install/visualization_tools/share/visualization_tools/launch/visualization_tools.launch`
- `tare_v1_launch_reference/install/waypoint_example/share/waypoint_example/launch/waypoint_example_garage.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/loam_interface/launch/loam_interface.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/local_planner/launch/local_planner.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/sensor_scan_generation/launch/sensor_scan_generation.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/terrain_analysis/launch/terrain_analysis.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/terrain_analysis_ext/launch/terrain_analysis_ext.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/system_garage.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/vehicle_simulator.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/visualization_tools/launch/visualization_tools.launch`
- `tare_v1_launch_reference/src/autonomous_exploration_development_environment/src/waypoint_example/launch/waypoint_example_garage.launch`
- `tare_v1_launch_reference/src/tare_planner/src/tare_planner/launch/explore.launch`
- `tare_v1_launch_reference/src/tare_planner/src/tare_planner/launch/explore_garage.launch`

Most important launch references:

- `system_garage.launch`: starts the TARE vehicle simulator garage world and optionally RViz.
- `vehicle_simulator.launch`: lower-level vehicle simulator launch with world/scenario parameters.
- `explore_garage.launch` and `explore.launch`: start TARE planner RViz and garage scenario parameters.
- `terrain_analysis.launch`, `terrain_analysis_ext.launch`, `sensor_scan_generation.launch`, `local_planner.launch`: likely producers/consumers for map, scan, path, and planning visualization topics.

## Copied Config Files

- `tare_v1_config_reference/backups_before_P4_escape_rescan_20260526_163703/garage.yaml`
- `tare_v1_config_reference/backups_before_watchdog_blacklist_fix_20260526_133714/garage.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/campus.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/forest.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/garage.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/indoor.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/matterport.yaml`
- `tare_v1_config_reference/install/tare_planner/share/tare_planner/tunnel.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/campus.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/forest.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/garage.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/indoor.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/matterport.yaml`
- `tare_v1_config_reference/src/tare_planner/src/tare_planner/config/tunnel.yaml`

Most important config:

- `garage.yaml`: declares TARE subscriptions and publisher topics such as `/registered_scan`, `/terrain_map`, `/terrain_map_ext`, `/state_estimation_at_scan`, `/way_point`, plus local planning horizon and viewpoint visualization parameters.

## Copied Docs

- `tare_v1_docs_reference/DEPLOY_NOTES.md`
- `tare_v1_docs_reference/DEPLOY_REPORT.md`
- `tare_v1_docs_reference/TARE_INTERFACE_FOR_BIMODAL_UAV.md`
- `tare_v1_docs_reference/fix_reports/P2_P3_watchdog_blacklist/P2_P3_WATCHDOG_BLACKLIST_FIX_REPORT.md`
- `tare_v1_docs_reference/fix_reports/P2_P3_watchdog_blacklist/runtime_test_summary.md`
- `tare_v1_docs_reference/fix_reports/P4_escape_rescan/P4_ESCAPE_RESCAN_FIX_REPORT.md`
- `tare_v1_docs_reference/fix_reports/P4_escape_rescan/runtime_test_summary.md`
- `tare_v1_docs_reference/fix_reports/runtime_stability/RUNTIME_STABILITY_FIX_REPORT.md`
- `tare_v1_docs_reference/fix_reports/runtime_stability/launch_script_runtime_fix.md`
- `tare_v1_docs_reference/fix_reports/runtime_stability/stable_p4_test_summary.md`
- `tare_v1_docs_reference/fix_reports/runtime_test_after_nan_guard.md`
- `tare_v1_docs_reference/review_reports/COMMAND_LOG.md`
- `tare_v1_docs_reference/review_reports/TARE_STUCK_AND_NAN_REVIEW_REPORT.md`
- `tare_v1_docs_reference/review_reports/WAYPOINT_GENERATION_TRACE.md`
- `tare_v1_docs_reference/src/autonomous_exploration_development_environment/README.md`
- `tare_v1_docs_reference/src/tare_planner/README.md`

## Copied Pointcloud / Map References

Small files copied:

- `tare_v1_pointcloud_reference/install/local_planner/share/local_planner/paths/pathList.ply`
- `tare_v1_pointcloud_reference/install/local_planner/share/local_planner/paths/paths.ply`
- `tare_v1_pointcloud_reference/install/local_planner/share/local_planner/paths/startPaths.ply`
- `tare_v1_pointcloud_reference/install/tare_planner/share/tare_planner/boundary.ply`
- `tare_v1_pointcloud_reference/install/waypoint_example/share/waypoint_example/data/boundary_garage.ply`
- `tare_v1_pointcloud_reference/install/waypoint_example/share/waypoint_example/data/waypoints_garage.ply`
- `tare_v1_pointcloud_reference/src/autonomous_exploration_development_environment/src/local_planner/paths/pathList.ply`
- `tare_v1_pointcloud_reference/src/autonomous_exploration_development_environment/src/local_planner/paths/startPaths.ply`
- `tare_v1_pointcloud_reference/src/autonomous_exploration_development_environment/src/waypoint_example/data/boundary_garage.ply`
- `tare_v1_pointcloud_reference/src/autonomous_exploration_development_environment/src/waypoint_example/data/waypoints_garage.ply`
- `tare_v1_pointcloud_reference/src/tare_planner/src/tare_planner/data/boundary.ply`

Large files not copied:

- `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply` (`53M`): original garage preview cloud. This is display-critical but omitted from this bundle to keep it small. AIR already has a derived/downsampled structure cloud under `src/virtual_env/garage_v1/maps/garage_structure_from_tare.pcd`.

## Core TARE V1 RViz Topics

Structure / scan:

- `/registered_scan`: enabled in `vehicle_simulator.rviz`, white points, size `0.05`, decay `10`; most likely live grey-white local structure scan.
- `/overall_map`: enabled in `tare_planner_ground.rviz`, white points, size `0.01`; most likely accumulated structure/map background.
- `/planner_cloud`: disabled by default in `tare_planner_ground.rviz`, white flat squares, likely planner internal collision/surface cloud.
- `/terrain_map`, `/terrain_map_ext`: available in `vehicle_simulator.rviz`, disabled by default; likely terrain/collision map products.

Terrain / map:

- `/terrain_map`
- `/terrain_map_ext`
- `/overall_map`
- `/explored_areas`
- `/uncovered_cloud`
- `/uncovered_frontier_cloud`

Odometry:

- `/state_estimation`
- `/state_estimation_at_scan`

Path:

- `/path`
- `/global_path`
- `/local_path`
- `/exploration_path`

Waypoint / viewpoint:

- `/way_point`
- `/viewpoint_vis_cloud`
- `/selected_viewpoint_vis_cloud`
- `/lookahead_point_cloud`

Local planning:

- `/tare_visualizer/local_planning_horizon`
- `/tare_visualizer/exploring_subspaces`
- `/planner_cloud`
- `/free_paths`

Image:

- `/camera/image_raw`

Fixed Frame:

- TARE planner and vehicle simulator RViz configs use `map`.
- Velodyne example RViz uses `lidar`.

## Most Likely Original Visual Sources

- Garage grey/white structure: `/registered_scan` for live local scan and `/overall_map` for accumulated map.
- Already explored area: `/explored_areas` and possibly `/overall_map`.
- Local planning box/range: `/tare_visualizer/local_planning_horizon`.
- Local/global paths: `/local_path`, `/global_path`, `/path`.
- Key waypoint: `/way_point`.
- Viewpoint candidates and selected viewpoints: `/viewpoint_vis_cloud`, `/selected_viewpoint_vis_cloud`, `/lookahead_point_cloud`.

## Recommended AIR Topic Mapping

| TARE topic | AIR equivalent or follow-up |
|---|---|
| `/registered_scan` | `/exploration/local_sensor_cloud` or `/exploration/observed_structure_cloud` |
| `/overall_map` | `/exploration/garage_structure_cloud` + `/exploration/observed_structure_cloud` |
| `/terrain_map` | `/exploration/structure_cloud` or `/exploration/occupied_cloud` |
| `/terrain_map_ext` | `/exploration/local_obstacle_cloud` or `/exploration/occupied_cloud` |
| `/global_path` | `/aerial_exploration/path` |
| `/local_path` | `/aerial_exploration/path` or future local path publisher |
| `/path` | `/exploration/trajectory_path` |
| `/way_point` | `/aerial_exploration/goal` or `/aerial_exploration/selected_goal_marker` |
| `/tare_visualizer/local_planning_horizon` | `/exploration/local_planning_box` |
| `/viewpoint_vis_cloud` | `/exploration/frontier_cloud` or future viewpoint candidate cloud |
| `/selected_viewpoint_vis_cloud` | `/aerial_exploration/selected_goal_marker` or future selected viewpoint cloud |
| `/navigation_boundary` | future `/exploration/navigation_boundary` polygon |
| `/camera/image_raw` | future Gazebo camera topic |

## Follow-up Reproduction Strategy

1. Use `extracted_topic_summary/rviz_topic_summary.md` as the exact RViz display inventory.
2. Recreate the TARE visual hierarchy in AIR: black background, `/overall_map`-like fine grey points, `/registered_scan`-like live local scan, then thin path/waypoint/local horizon overlays.
3. Add AIR topic aliases/remaps only after deciding whether to match TARE topic names directly or keep AIR names and update RViz.
4. If exact TARE map appearance is required, separately copy or regenerate the 53M garage preview PLY and publish it as an `/overall_map`-compatible static cloud.
