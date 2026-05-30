# TARE V1 Visualization Mechanism Study

## Direct Findings

`/registered_scan` is published by `vehicle_simulator/src/vehicleSimulator.cpp`. The source is Gazebo `/velodyne_points` from the VLP-16 ray sensor in `vehicle_simulator/urdf/lidar.urdf.xacro`; `vehicleSimulator` transforms each scan into `map` and republishes it.

`/overall_map` is published by `visualization_tools/src/visualizationTools.cpp`. It reads `vehicle_simulator/mesh/garage/preview/pointcloud.ply`, downsamples it, and republishes it as a static map.

`/terrain_map` is published by `terrain_analysis/src/terrainAnalysis.cpp`. `/terrain_map_ext` is published by `terrain_analysis_ext/src/terrainAnalysisExt.cpp`. Both subscribe to `/registered_scan`, keep rolling local scan voxels, compute terrain elevation/collision intensity, and publish filtered local terrain clouds.

## Visual Mechanism

The fine TARE visual style is a combination of:

- `/registered_scan`: live simulated LiDAR ray returns, white, decay 10.
- `/terrain_map` and `/terrain_map_ext`: filtered rolling products from registered scan.
- `/overall_map`: static preview point cloud, shown with very small points and low alpha.
- `/explored_areas`: accumulated observed scan points from `visualization_tools`.

TARE does not publish garage structure by sampling `garage.dae` faces inside RViz. The mesh is used by Gazebo collision/visual rendering, and the VLP-16 ray sensor produces scan-like point returns. The `preview/pointcloud.ply` is a reference map used for `/overall_map`, not a planner scan source.

## Answers

1. `/registered_scan`: `vehicleSimulator`.
2. Source: simulated LiDAR `/velodyne_points`, not direct mesh/PLY publication.
3. `/overall_map`: `visualizationTools`.
4. `/terrain_map`: `terrainAnalysis`; `/terrain_map_ext`: `terrainAnalysisExt`.
5. Thin line/scan structure most likely comes from `/registered_scan`, plus filtered `/terrain_map(_ext)`.
6. TARE uses Gazebo ray sensor scanning against the garage mesh, not a custom offline DAE face sampler.
7. TARE has `pointcloud.ply`, `garage.dae`, and `vehicleSimulator`; point projection happens through Gazebo ray scans and the vehicle simulator transform.
8. RViz combines static map points and scan/terrain points; the desired live look is scan-line/edge-like, not dense face sampling.
9. AIR should复刻 `/overall_map`, `/registered_scan`, `/explored_areas`, `/terrain_map`, `/terrain_map_ext`, `/path`, `/local_path`, `/global_path`, `/way_point`, `/free_paths`.
10. Terrain nodes crop by relative z/range, voxelize, estimate terrain elevation, apply connectivity/ceiling filtering in ext, and use intensity as elevation/collision signal. They are not explicit mesh edge extractors.
