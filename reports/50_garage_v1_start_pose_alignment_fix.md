# Garage V1 Start Pose Alignment Fix

## Changes

- Added unified `garage_v1.start_pose.*` parameters to both garage config files.
- Updated legacy `uav.initial_*` values to match the new start pose.
- Added launch overrides: `start_x`, `start_y`, `start_z`, `start_yaw`.
- The default run script still launches `rviz_profile:=tare_edge_replay`; extra launch args pass through unchanged.
- Added `grid.origin_x/y` and made planner/mapping grid conversions respect that origin, so the local exploration grid follows the entrance start in world coordinates.
- Added `/exploration/start_pose_marker` to RViz and the replay bridge/planner.
- Added `scripts/analyze_garage_start_region.py` and `scripts/check_garage_v1_start_pose_alignment.sh`.

## New Start

```text
x=-23.817
y=-46.018
z=1.6
yaw=0.0
region=between the two opening regions / corridor entrance area
```

This is TARE `(0,0)` transformed by the AIR garage pose recorded in the edge-cloud metadata. It is inside the edge-map bounds and outside the central 30% XY core.

## Synchronization

- `/state_estimation`: from `simple_uav_follower_node`, initialized with `garage_v1.start_pose`.
- `/odom`: same odometry message as `/state_estimation`.
- TF `map -> base_link`: same pose.
- Gazebo UAV visualizer: spawn parameters receive the same launch values, then follow `/state_estimation`.
- Planner and mapping: use `grid.origin_x/y` equal to start x/y.
- RViz marker: `/exploration/start_pose_marker`.

The garage model and edge cloud were not moved.
