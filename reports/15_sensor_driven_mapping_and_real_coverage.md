# Sensor Driven Mapping And Real Coverage Baseline

## Why Previous Coverage Was Not Visually Trustworthy

The previous visual demo showed coverage above 0.95 while the visible UAV trajectory did not obviously scan most of the environment. The root issue was semantic: coverage came from an internal synthetic map update, not from explicit LiDAR/camera observations.

## Old Coverage Meaning

Old `coverage` meant observed voxels in the synthetic mapping node. The update depended on UAV pose and a sensor radius, but it did not publish or accumulate point clouds and did not ray cast camera/LiDAR observations.

## New Observed Coverage Definition

`observed_coverage = observed_free_or_occupied_voxels / total_voxels_in_planning_boundary`

Observed voxels are updated only by local simulated sensing:

- LiDAR ray casting from the current UAV pose.
- Camera frustum ray casting from the current UAV pose.
- Free voxels are cells traversed before a hit.
- Occupied voxels are local ray hits against dense50 obstacle boxes represented in the voxel map.
- Unknown voxels remain unknown until a ray traverses or hits them.

## LiDAR And Camera Baseline

This is a simulated local sensor baseline, not full SLAM:

- LiDAR source: local ray casting against dense50 obstacle truth from `/state_estimation`.
- Camera source: forward frustum ray casting against the same voxelized world.
- The global truth map is not directly marked observed. It is only used as the hidden environment for local sensor hit simulation.

## Map Data Structure

The `sensor_mapping_node` maintains:

- `unknown`: any voxel not in `observed`
- `free`: ray-traversed non-hit voxels
- `occupied`: ray hit voxels
- `observed`: `free union occupied`
- `frontiers`: known free voxels adjacent to unknown voxels

## Frontier Extraction And Planning

Frontiers are extracted from the observed map and published in `/aerial_exploration/map_state`. The existing planner consumes those frontier cells and selects aerial corridor goals. Path planning remains the current baseline: 3D occupied-voxel checking plus XY A* at fixed aerial corridor z.

This is a baseline frontier planner, not a full information-theoretic planner.

## Map Export

Maps are saved under `results/maps/`:

- `observed_occupied_cloud.pcd`
- `observed_free_cloud.pcd`
- `observed_all_cloud.pcd`
- `frontier_cloud.pcd`
- `trajectory.csv`
- `map_metrics.csv`

These files are generated from the online observed map, not a direct dump of the dense50 world truth.

## RViz

RViz displays:

- UAV odometry and marker
- planned path and selected goal
- local LiDAR scan points
- observed occupied cloud
- observed free cloud
- unknown frontier cloud
- observed coverage text
- synthetic coverage text

## Gazebo

Gazebo remains responsible for visualizing:

- dense50 world
- UAV model
- green trajectory breadcrumbs
- yellow goal marker

Dense map visualization is kept primarily in RViz.

## Current Limitations

- Not full FAST-LIVO, FAST-LIO, OctoMap, or SLAM.
- No real Gazebo lidar plugin is used yet.
- Camera is a frustum ray-casting baseline, not real image/depth processing.
- Pose is provided by the kinematic follower, not visual-inertial odometry.
- Planning is baseline frontier selection with corridor routing, not full kinodynamic 3D exploration planning.

## Next Stage

Replace the local simulated sensors with:

- real `sensor_msgs/PointCloud2` LiDAR input;
- depth camera or stereo camera point cloud;
- FAST-LIVO2 or another LiDAR-camera odometry/mapping backend;
- OctoMap/voxblox/ESDF map integration;
- planner over the online map.

## Tests

Expected passing tests:

- `scripts/run_sensor_mapping_smoke_test.sh`
- `scripts/check_observed_coverage_not_fake.sh`
- `scripts/check_map_export.sh`
- legacy visual/corridor/anti-fake tests

Observed latest results:

- sensor mapping smoke: PASS
- observed coverage not fake: PASS
- map export: PASS
- final observed coverage: `0.942083`
- final synthetic coverage column: `0.942083`
- final frontier count: `125`
- final sensor cloud points: `195`
- map export path: `results/maps/`

## Run Command

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true sensor_mapping:=true observed_coverage:=true
```
