# UAV Motion Debug Report

Date: 2026-05-26

## Root Cause

The UAV looked stationary because the upstream planning chain was repeatedly publishing the same trajectory and the UAV simulator reset its tracking index whenever it received `/air/smoothed_path`.

The specific chain was:

1. `air_world_provider` publishes `/air/occupancy_markers` every second.
2. The previous `air_global_planner` called `plan_if_ready()` on every obstacle message.
3. The previous `air_trajectory_generator` republished `/air/smoothed_path` on every repeated `/air/global_path`.
4. The previous `air_uav_simulator.path_cb()` always did:
   - replace `self.path`
   - set `self.target_index = 0`
5. The first waypoint equals the initial UAV pose `(-8, -8, 1)`, so the UAV repeatedly restarted at the beginning of the trajectory and looked like it barely moved.

## Repeated Goal/Start Publishing

`air_mission_manager` previously published the initial mission several times. It now publishes the default start and goal once after startup and logs:

```text
Published initial 3D mission goal once
```

External goals published to `/air/goal` remain supported and produce:

```text
Received external goal, requesting replan
```

## Repeated Planning

The planner previously replanned on every obstacle marker update. It now tracks the last planned start/goal and defaults to `planner.allow_periodic_replan=false`.

Repeated requests produce explicit skip status strings:

- `REPLAN_SKIPPED_DUPLICATE_GOAL`
- `REPLAN_SKIPPED_NO_CHANGE`

Successful plans produce:

- `PLAN_SUCCESS`

Failures produce:

- `PLAN_FAILED`

## Repeated Smoothed Path Publishing

`air_trajectory_generator` now computes a path signature from length and sampled xyz points. Duplicate global paths are ignored with:

```text
Skipped duplicate global path, keep current trajectory
```

Only changed paths are republished:

```text
Published new smoothed 3D path with N sampled waypoints
```

## Simulator Reset Behavior

The simulator no longer resets execution progress for duplicate trajectories. It now:

- accepts the first trajectory from the current UAV position,
- ignores duplicate trajectory signatures,
- accepts changed trajectories from the nearest forward waypoint,
- never resets UAV pose after startup.

New debug logs include:

- `WAITING_FOR_TRAJECTORY`
- `Accepted new trajectory, nearest forward waypoint index=...`
- `Ignored duplicate trajectory, keep tracking current target_idx=...`
- `UAV pos=(x,y,z), target_idx=i/N, target=(x,y,z), speed=..., dist=..., status=...`
- `GOAL_REACHED`

## Pose Reset

The UAV pose is initialized only once in `UavSimulator.__init__()`. No path callback writes to `self.position`.

## Start Pose Behavior

The MVP still uses the mission start pose as the planner start for the initial mission. This is acceptable for the current single-run demo. For future dynamic replanning from arbitrary in-flight positions, the planner or mission manager should update `/air/start` from `/air/state_estimation` before planning the new goal.

## Visualization Fixes

Added:

- `/air/uav_trail`: actual flown path as `nav_msgs/Path`
- `/air/current_waypoint_marker`: current target waypoint marker
- `/air/uav_status_marker`: text marker with pose, target index, distance, and state

RViz now includes displays for these topics.
