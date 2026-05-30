# Garage V1 Visual Readability Fix

## Goal

This pass focuses on visual readability, not coverage. Garage V1 already reaches approximately `0.75+` observed coverage; the problem was that RViz did not clearly communicate building structure, explored map, and planning result.

## Local Planning Box Meaning

`local planning box` means the local planning window around the UAV. It shows the bounded volume where the planner evaluates near-term local space. It is not the garage building, a wall, or a room outline.

Because this marker was easy to confuse with the building structure, the clean RViz view disables it by default. It remains available in debug mode as `Debug Local Planning Box` with a thin muted green line.

## Structure Cloud Fix

Added:

```text
src/aerial_exploration_planner/aerial_exploration_planner/garage_structure_cloud_node.py
```

The node now publishes a static, dense, grey garage reference cloud generated from the TARE garage preview point cloud:

```text
/exploration/garage_structure_cloud
/exploration/static_garage_structure_cloud
```

Generated source files:

```text
src/virtual_env/garage_v1/maps/garage_structure_from_tare.pcd
src/virtual_env/garage_v1/maps/garage_structure_from_tare.xyz
```

The previous wall-proxy cloud remains only as a fallback. The default RViz structure cloud is now aligned to the same `garage.dae` coordinate frame and world include pose used by Gazebo.

This separates static building context from online observed map data:

- Static Garage Structure Cloud: visual reference for garage outline.
- Observed Structure Cloud: `/exploration/observed_structure_cloud`, occupied points observed by the online simulated local sensor map.
- Local Sensor Cloud: current local scan.
- Frontier Candidates: unknown/known boundary candidates.
- Path and trajectory: planning and motion result.

## Visual Changes

- Clean RViz uses a black background and small grey point clouds.
- Free/full voxel layers are disabled in clean mode.
- Frontier points are smaller and muted.
- Trajectory/path lines remain thin: `0.02` and `0.025`.
- Coverage text is shortened and moved to the map corner.

## Known Limitation

The static structure cloud is a TARE preview/reference cloud, not live SLAM output. It improves visual alignment with Gazebo, but does not replace future Gazebo lidar, camera depth, or FAST-LIVO output.
