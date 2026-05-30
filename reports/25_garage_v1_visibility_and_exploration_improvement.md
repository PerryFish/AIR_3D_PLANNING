# Garage V1 Visibility And Exploration Improvement

## Current Problems

- Gazebo could silently fall back to `/usr/share/gazebo-11/worlds/empty.world` when the launch argument `world:=garage_v1` reached Gazebo instead of an absolute `.world` path.
- Gazebo loaded the garage scene assets but the visible building structure was not apparent in the default AIR view.
- RViz initially emphasized large colored voxel blocks rather than a TARE-like structural point cloud.
- Garage V1 is more difficult than dense50 because it has indoor-like narrow corridors, dead ends, and routes that may require backtracking.
- Early garage runs used a mostly fixed altitude even though the corridor allowed z motion.

## Gazebo Visibility Root Cause

There were two separate issues.

First, `visual_aerial_exploration_garage_v1.launch.py` exposed `world` with the default value `garage_v1`. That string could be passed to Gazebo as the world file, producing `URI not supported by Fuel [garage_v1]`, `Could not open file[garage_v1]`, and then loading `empty.world`.

Second, after the real world file was loaded, the garage mesh frame still needed recentering. `garage.dae` has a vertex bounding box centered near `(23.817, 46.018, 10.472)`, while AIR runs the UAV and exploration view around the map origin. The model could therefore be loaded but visually offset from the area users inspect.

The original `garage_v1.world` also did not explicitly name or pose the included garage model. The fix recenters the include:

```xml
<include>
  <uri>model://garage</uri>
  <name>garage</name>
  <pose>-23.817 -46.018 0.073 0 0 0</pose>
</include>
```

`model.sdf` still uses `model://garage/meshes/garage.dae`; no stale `/home/nuaa/ZHY/TARE_V1` absolute paths are used.

## Gazebo Fixes

- Updated `src/aerial_exploration_planner/launch/gazebo_garage_v1.launch.py` and `src/aerial_exploration_planner/launch/visual_aerial_exploration_garage_v1.launch.py` to resolve an absolute `garage_v1.world` path with `get_package_share_directory('aerial_exploration_planner')`.
- The launch falls back to `/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/worlds/garage_v1.world` only when the installed copy is absent.
- Launch now logs `Using garage_v1 world: ...garage_v1.world` and raises an error if the world path is missing or does not end with `.world`.
- Updated `src/virtual_env/garage_v1/worlds/garage_v1.world`.
- Updated `GAZEBO_MODEL_PATH` ordering to prefer:
  - `src/virtual_env/garage_v1/models`
  - installed `virtual_env/garage_v1/models`
  - `/usr/share/gazebo-11/models`
- Added `garage_wall_proxy` as a visible gray wall outline when the original DAE material is hard to inspect in Gazebo.
- Extended asset and Gazebo smoke tests to fail on empty-world fallback, bad `garage_v1` URI loading, missing model paths, and missing `garage_v1.world` log evidence.

Installed runtime path:

```text
install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1
```

## RViz Visualization Enhancements

Added or clarified:

- `/exploration/observed_cloud`
- `/exploration/occupied_cloud`
- `/exploration/free_cloud`
- `/exploration/frontier_cloud`
- `/exploration/structure_cloud`
- `/exploration/garage_structure_cloud`
- `/exploration/local_sensor_cloud`
- `/exploration/local_obstacle_cloud`
- `/exploration/local_planning_box`
- `/exploration/trajectory_path`
- `/exploration/coverage_text`

Garage TARE-like RViz config:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_like.rviz
```

This config uses point-cloud rendering instead of large voxel cube displays:

- trajectory line width: `0.02`
- planned path line width: `0.025`
- structure cloud point size: `0.03-0.035`
- local sensor point size: `0.04`
- frontier point size: `0.055`
- local planning box: green `LINE_LIST` wireframe, not a filled cube

Gazebo garage markers were reduced:

- trail radius: `0.06`
- goal radius: `0.18`
- trail minimum distance: `0.25`

## Garage Exploration Strategy

Garage uses `garage_v1_exploration.yaml` with:

- target observed coverage: `0.75`
- online observed map and frontier topics
- simulated local sensor fallback for occupancy observation
- frontier scoring with information gain, nearby unknown area, narrow passage bonus, distance cost, turning cost, revisit penalty, and loop closure bonus
- stuck detection using recent pose movement and coverage improvement
- temporary frontier blacklist after failed/stuck goals
- branch-point recording and backtrack target selection
- adaptive z-level candidate selection with levels `0.8, 1.2, 1.6, 2.0, 2.4, 2.8`
- speed-limited follower altitude tracking instead of forcing every waypoint to `default_z`

Dense50 still uses the existing dense50 config and behavior.

## Test Results

- build: PASS, `8 packages finished`
- garage asset: PASS
- garage Gazebo smoke: PASS
- garage visual smoke: PASS
  - initial observed coverage: `0.027465`
  - final observed coverage: `0.758975`
  - coverage improvement: `0.731510`
  - target observed coverage: `0.75`
  - target reached: `True`
  - pose samples: `276`
  - goal samples: `11`
  - frontier count: `1482`
  - stuck events: `0`
  - backtrack events: `0`
  - failed goals: `1`
  - path length: `31.019 m`
- garage adaptive altitude: PASS
  - z range: `1.600 -> 2.000`
  - z changes: `85`
- garage coverage target: PASS, final observed coverage `0.766436`
- garage frontier goal ratio: WARN
  - frontier goals: `0`
  - goal changes: `10`
  - frontier goal ratio: `0.000`
- dense50 regression: PASS
  - aerial corridor height: PASS, robot and goal z range `1.400..1.400`
  - sensor mapping smoke: PASS, observed coverage `0.027917 -> 0.951667`

The garage smoke test does not require hitting the full `0.75` target; it verifies real coverage growth, active goals, frontiers, trajectory growth, and stable launch. The `0.75` target remains the development objective for longer runs and future real sensor integration.

## Known Limitations

- Garage V1 still uses a simulated local sensor fallback, not a real Gazebo lidar/camera topic.
- This is not complete FAST-LIVO, SLAM, OctoMap, or ESDF integration.
- The visual garage mesh is loaded and recentered; the wall proxy is a visual aid, not exact planning collision geometry.
- AIR planning still uses a structured garage-like occupancy fallback rather than exact triangle mesh collision extraction.
- Frontier ratio is still weak in the current smoke test, so systematic sweep/backtracking fallback remains important.
- Future work should add a Gazebo lidar/depth camera, feed real point clouds into mapping, and replace the fallback occupancy model with SLAM or parsed collision geometry.
