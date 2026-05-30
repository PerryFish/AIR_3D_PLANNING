# Garage V1 Virtual Environment Integration

## Purpose

`garage_v1` migrates the garage scene previously used in `/home/nuaa/ZHY/TARE_V1` into AIR so the current Gazebo + RViz UAV exploration baseline can be exercised outside the dense50 world.

## Copied Resource Summary

- World: `src/virtual_env/garage_v1/worlds/garage_v1.world`
- Model: `src/virtual_env/garage_v1/models/garage`
- Mesh: `src/virtual_env/garage_v1/models/garage/meshes/garage.dae`
- Textures: `src/virtual_env/garage_v1/models/garage/meshes/*.jpg`
- TARE launch references: `src/virtual_env/garage_v1/launch_reference`
- RViz references: `src/virtual_env/garage_v1/rviz`
- TARE config reference: `src/virtual_env/garage_v1/config/tare_garage.yaml`
- Example maps: `src/virtual_env/garage_v1/maps`
- Docs: `src/virtual_env/garage_v1/docs`

The large TARE preview point cloud `preview/pointcloud.ply` was not copied because it is not a Gazebo runtime dependency.

## World And Model Dependencies

The migrated world uses:

- `model://garage`, resolved by `src/virtual_env/garage_v1/models` or the installed package share path.
- `model://sun`, resolved by `/usr/share/gazebo-11/models`.

The garage model uses:

- `model://garage/meshes/garage.dae`
- local JPG textures referenced by the DAE.

## Launch Changes

Added:

- `src/aerial_exploration_planner/launch/gazebo_garage_v1.launch.py`
- `src/aerial_exploration_planner/launch/visual_aerial_exploration_garage_v1.launch.py`

`gazebo_garage_v1.launch.py` starts Gazebo Classic 11 with the migrated garage world and sets a scoped `GAZEBO_MODEL_PATH`. It resolves the installed package-share world path first and falls back to the source path only if the installed copy is absent. It must log `Using garage_v1 world: ...garage_v1.world`; passing the bare string `garage_v1` to Gazebo is treated as an error because it can trigger an `empty.world` fallback.

`visual_aerial_exploration_garage_v1.launch.py` starts Gazebo, RViz, the simple UAV follower, AIR planner, sensor mapping node, metrics node, UAV visualizer, trail visualizer, and goal marker. It defaults to the TARE-like RViz config `src/aerial_exploration_planner/rviz/garage_v1_tare_like.rviz`.

The garage world now explicitly names and recenters the included `garage` model:

```xml
<include>
  <uri>model://garage</uri>
  <name>garage</name>
  <pose>-23.817 -46.018 0.073 0 0 0</pose>
</include>
```

This compensates for the DAE mesh coordinate offset and makes the building visible around the AIR exploration area.

A visual wall proxy is also available and enabled by default in the AIR garage launch:

```text
src/virtual_env/garage_v1/models/garage_wall_proxy
```

The proxy is only a Gazebo visual outline aid; it does not replace the original garage mesh or provide exact planning collision geometry.

## Install Configuration

`src/aerial_exploration_planner/setup.py` now recursively installs:

```text
../virtual_env/garage_v1
```

to:

```text
install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1
```

Expected installed files include:

- `install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/worlds/garage_v1.world`
- `install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/models/garage/model.sdf`

## New Scripts

- `scripts/run_garage_v1_visual_exploration.sh`
- `scripts/check_garage_v1_assets.sh`
- `scripts/run_garage_v1_gazebo_smoke_test.sh`
- `scripts/run_garage_v1_visual_exploration_smoke_test.sh`

## Gazebo Launch

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner gazebo_garage_v1.launch.py gui:=true
```

## Gazebo + RViz + Exploration Launch

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=true \
  rviz:=true
```

One-command wrapper:

```bash
./scripts/run_garage_v1_visual_exploration.sh
```

## Mapping And Planning Status

- Real Gazebo sensor: not yet integrated.
- Current mapping source: `simulated_local_sensor_fallback_garage_v1_mesh_bbox`.
- Current planner mode: online observed map and frontier cells from the AIR sensor mapping baseline, with garage-specific frontier scoring, stuck detection, temporary blacklist, and backtracking.
- RViz map style: TARE-like point clouds via `/exploration/structure_cloud`, `/exploration/garage_structure_cloud`, `/exploration/local_sensor_cloud`, `/exploration/local_obstacle_cloud`, and `/exploration/local_planning_box`.
- Adaptive altitude: enabled for garage with z levels `0.8, 1.2, 1.6, 2.0, 2.4, 2.8`.
- Dense50 truth is not used by `visual_aerial_exploration_garage_v1.launch.py`; garage uses `garage_v1_exploration.yaml` and `sensor_mapping.environment_model: garage_v1`.

## Current Limitations

- This is not complete FAST-LIVO, SLAM, OctoMap, or ESDF integration.
- The garage obstacle field used by AIR planning/mapping is a garage-like local sensor fallback, not a parsed full triangle mesh collision map.
- The UAV is controlled by the ROS2 kinematic follower, not a Gazebo physics controller.
- TARE launch files are copied for traceability only and remain tied to TARE packages.

## Follow-Up Recommendations

- Add a Gazebo lidar/camera sensor to the AIR UAV model and feed `sensor_mapping_node` from real Gazebo topics.
- Parse `garage.dae` or use Gazebo collision data to replace the current bounding-box-style fallback.
- Add a garage-specific RViz config if the default AIR visualization becomes crowded.
- Keep dense50 regression tests active when tuning garage planner parameters.
