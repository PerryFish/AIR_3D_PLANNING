# TARE V1 RViz Reference Analysis

## Source Files

Read from:

```text
src/virtual_env/garage_v1/rviz/extracted_topic_summary/rviz_topic_summary.md
src/virtual_env/garage_v1/rviz/manifest.yaml
src/virtual_env/garage_v1/rviz/README.md
```

## Core Vehicle Simulator Displays

`vehicle_simulator.rviz` uses `map` fixed frame and emphasizes:

- `/registered_scan`: white PointCloud2, size `0.05`, decay `10`.
- `/path`: green Path, line width `0.05`.
- `/free_paths`: blue PointCloud2, size `0.02`.
- `/way_point`: PointStamped.
- `/added_obstacles`: red/orange PointCloud2, size `0.05`.
- `/trajectory`: white PointCloud2, size `0.01`.
- `/camera/image_raw`: Image.
- `/navigation_boundary`: Polygon.

## Core Planner Displays

`tare_planner_ground.rviz` also uses `map` fixed frame and emphasizes:

- `/overall_map`: white PointCloud2, size `0.01`.
- `/explored_areas`: blue PointCloud2, size `0.01`.
- `/uncovered_cloud`: red PointCloud2.
- `/uncovered_frontier_cloud`: red PointCloud2.
- `/global_path`, `/local_path`, `/path`.
- `/way_point`.
- `/tare_visualizer/local_planning_horizon`.
- `/tare_visualizer/exploring_subspaces`.
- `/selected_viewpoint_vis_cloud`.

## Interpretation

The TARE visual style is mostly fine point clouds and thin path overlays. The most important structure layers are `/registered_scan` for live local scan and `/overall_map` for global reference structure. `/registered_scan` is produced from Gazebo Velodyne ray returns via `vehicleSimulator`, while `/overall_map` is a downsampled `preview/pointcloud.ply` published by `visualizationTools`. The local planning horizon is a planner debug volume, not a building outline.

For AIR garage_v1, directly rendering dense DAE/preview surface samples as the default RViz structure makes the garage look like a filled white sheet. The new default uses an extracted edge cloud and scan-like local replay instead.
