# /way_point Generation Trace

## Summary

- Publisher package: `tare_planner`
- Node executable: `tare_planner_node`
- Node class: `sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D`
- Topic: `/way_point`, configurable by `pub_waypoint_topic_`
- Message type: `geometry_msgs/msg/PointStamped`
- Frame: `map`, from `kWorldFrameID`
- Main publish function: `SensorCoveragePlanner3D::PublishWaypoint()`

## Static Trace

1. `tare_planner_node.cpp` constructs `SensorCoveragePlanner3D` and spins it.
2. `sensor_coverage_planner_ground.cpp` declares `pub_waypoint_topic_` with default `/way_point` and reads it from YAML.
3. `Initialize()` creates `waypoint_pub_` as `geometry_msgs::msg::PointStamped`.
4. `execute()` updates representation, viewpoints, global planning, local planning, and concatenates `exploration_path_`.
5. `GetLookAheadPoint(exploration_path_, global_path, lookahead_point_)` selects or keeps the current lookahead target.
6. Additional watchdog logic may override `lookahead_point_` to the current robot position after timeout.
7. `PublishWaypoint()` converts `lookahead_point_` into the published `/way_point`, optionally extending it in the robot-to-lookahead direction.

## Key Source Locations

| File | Lines | Function | Role |
|---|---:|---|---|
| `src/tare_planner/src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h` | 62 | global constant | `kWorldFrameID = "map"` |
| `src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | 51 | `ReadParameters()` declaration area | default `pub_waypoint_topic_ = /way_point` |
| same | 537-538 | `Initialize()` | creates `/way_point` publisher |
| same | 1446-1648 | `PublishWaypoint()` | computes and publishes waypoint |
| same | 1765-1973 | `execute()` | planner loop and watchdog before publish |
| same | 1121-1443 | `GetLookAheadPoint()` | selects lookahead from local/global path |
| `src/tare_planner/src/tare_planner/config/garage.yaml` | 3-17 | ROS parameters | input/output topics |

## Inputs Used by TARE

From `garage.yaml`:

- State input: `/state_estimation_at_scan`
- Registered scan: `/registered_scan`
- Terrain map: `/terrain_map`
- Terrain map extended: `/terrain_map_ext`
- Coverage boundary: `/sensor_coverage_planner/coverage_boundary`
- Navigation boundary: `/navigation_boundary`
- Reset waypoint: `/reset_waypoint`

## Output Sources

The published x/y/z normally come from:

- `lookahead_point_` selected in `GetLookAheadPoint()`
- robot position extension in `PublishWaypoint()`
- return-home fallback when `exploration_finished_ && near_home_ && kRushHome`
- initial and reset callbacks that directly publish the robot or initial position

## Reset and Timeout Behavior

`PublishWaypoint()` has an absolute timeout block. When `waypoint_age > kWaypointTimeout`, it:

- logs `ABSOLUTE TIMEOUT TRIGGERED`
- adds the current waypoint region to the blacklist
- resets the published waypoint to the robot position

`execute()` has a second watchdog. When the same lookahead persists beyond `kWaypointTimeout`, it:

- logs `ABSOLUTE WATCHDOG TIMEOUT FIRED`
- forces `lookahead_point_ = robot_position_`
- adds the previous lookahead to a permanent blacklist
- calls `PublishWaypoint()`

## NaN Risk in the Generation Chain

The highest risk is the combination of watchdog reset and waypoint extension:

- `execute()` can set `lookahead_point_` exactly equal to `robot_position_`.
- `PublishWaypoint()` computes `r = sqrt(dx*dx + dy*dy)`.
- If `r == 0` and `kExtendWayPoint == true`, it executes `dx = dx / r * extend_dist` and `dy = dy / r * extend_dist`.
- That creates `NaN` and can publish `(-nan, -nan)`.

This explains the observed log line `Robot has been stuck trying to reach (-nan, -nan)`.
