# Garage V1 World Loading And TARE-Like Visualization

## Empty World Fallback Root Cause

`visual_aerial_exploration_garage_v1.launch.py` declared `world` with the default string `garage_v1`. In some launch paths this string was passed to Gazebo instead of the real `.world` file. Gazebo then attempted to resolve `garage_v1` as a Fuel URI or file, failed, and fell back to `/usr/share/gazebo-11/worlds/empty.world`.

The fixed Gazebo launch now resolves an absolute world path with `get_package_share_directory('aerial_exploration_planner')` and falls back to:

```text
/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/src/virtual_env/garage_v1/worlds/garage_v1.world
```

Launch now prints:

```text
Using garage_v1 world: /absolute/path/to/garage_v1.world
```

If the world file is missing or does not end in `.world`, launch raises an error instead of silently loading empty world.

## Garage Model Loading

`garage_v1.world` includes:

```xml
<include>
  <uri>model://garage</uri>
  <name>garage</name>
  <pose>-23.817 -46.018 0.073 0 0 0</pose>
</include>
```

The pose recenters the original DAE mesh, whose vertex bounds are centered near `(23.817, 46.018, 10.472)`.

`GAZEBO_MODEL_PATH` includes:

- source garage models
- installed garage models
- `/usr/share/gazebo-11/models`

## Wall Proxy

A lightweight `garage_wall_proxy` model was added and spawned by default as a visual aid. It does not replace the original mesh; it makes the garage outline legible in Gazebo when the original DAE material/geometry is hard to inspect. The proxy is visual-only, contains no collision blocks, uses thin muted grey walls, and disables shadows.

Files:

- `src/virtual_env/garage_v1/models/garage_wall_proxy/model.config`
- `src/virtual_env/garage_v1/models/garage_wall_proxy/model.sdf`

## TARE-Like RViz

Garage launch now supports:

```text
rviz_view_mode:=clean
rviz_view_mode:=debug
rviz_view_mode:=voxel
```

The default clean view uses:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_like.rviz
```

Main clean-view topics:

- `/exploration/structure_cloud`
- `/exploration/garage_structure_cloud`
- `/exploration/local_sensor_cloud`
- `/exploration/frontier_cloud`
- `/exploration/trajectory_path`
- `/aerial_exploration/path`
- `/exploration/coverage_text`

The view emphasizes a static grey garage structure cloud, observed occupied structure, local sensor points, and thin paths rather than large colored voxel cubes. `/exploration/local_planning_box` is a planner debug local search window, not a building outline, so it is disabled in clean mode and available in debug mode.

## Test Result

- garage asset check: PASS
- garage Gazebo smoke: PASS
- garage visual smoke: PASS
- empty.world fallback: removed
- visual smoke coverage: `0.027465 -> 0.758975`

Known gap: this is still a simulated local sensor fallback and not a full TARE/FAST-LIVO map.
