# PX4 Integration Plan

## Current MVP Output

TARE_V2_AIR currently outputs:

- `/air/smoothed_path`: sampled 3D path as `nav_msgs/Path`
- `/air/trajectory`: same sampled path, reserved as the future trajectory interface
- `/air/state_estimation`: simulated UAV odometry as `nav_msgs/Odometry`
- `/air/planner_status`: planner success/failure text

## PX4 Needs

PX4 offboard control needs a continuous stream of offboard control mode, trajectory setpoints, and vehicle commands. A future bridge should publish:

- `/fmu/in/trajectory_setpoint`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/vehicle_command`

It should consume:

- `/fmu/out/vehicle_odometry`

## Conversion From `/air/smoothed_path`

The future bridge should:

1. Subscribe to `/air/smoothed_path`.
2. Track the active segment and choose a lookahead waypoint.
3. Convert ROS ENU path coordinates into PX4-required frame conventions.
4. Publish position and optional velocity setpoints at PX4 offboard rate.
5. Hold position if the path is empty or stale.
6. Switch to landing or return-home behavior on failsafe events.

## Packages To Add Later

- `air_px4_bridge`
- `air_trajectory_setpoint_publisher`
- `air_offboard_mode_manager`
- `air_failsafe_manager`

## Safety Strategy

The PX4 integration must reject or failsafe on:

- Goal outside map bounds.
- Empty path.
- Collision risk or stale obstacle map.
- Lost localization or stale `/fmu/out/vehicle_odometry`.
- Planner timeout.
- Offboard stream interruption.

Fallback actions should include hover, land, or return home depending on mission state and vehicle health.
