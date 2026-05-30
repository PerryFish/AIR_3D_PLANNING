# Garage V1 Adaptive Altitude Planning

## Change Summary

Garage V1 now enables adaptive altitude planning with z-level candidates:

```text
0.8, 1.2, 1.6, 2.0, 2.4, 2.8
```

The planner evaluates candidate frontier goals across z levels and chooses a collision-free altitude using information gain and current-altitude preference. The follower tracks waypoint z with a vertical speed limit instead of forcing `default_z`.

## Implemented Topics And Metrics

Planner state publishes:

- `adaptive_z_enabled`
- `adaptive_z_goal_count`
- `z_min`
- `z_max`
- `stuck_events`
- `backtrack_events`
- `failed_goals`

The coverage text marker includes:

- observed coverage and target coverage
- frontier count
- path length
- current z
- adaptive z enabled

The text is intentionally shortened and moved to the map corner in the garage clean RViz view so altitude/coverage status does not cover the structure cloud.

## Test Result

`scripts/check_garage_v1_adaptive_altitude.sh` result:

- result: PASS
- observed z min: `1.600`
- observed z max: `2.000`
- z range: `0.400`
- z changes: `85`

Coverage target check:

- target: `0.75`
- initial: `0.027465`
- final: `0.766436`
- improvement: `0.738971`
- result: PASS

Frontier goal ratio:

- frontier goals: `0`
- goal changes: `10`
- frontier goal ratio: `0.000`
- result: WARN

The warning means the current baseline still relies heavily on sweep/backtracking fallback despite using online frontier data. This is acceptable for this integration stage but should be improved when real Gazebo lidar or FAST-LIVO/SLAM is connected.

## Known Limitations

- The altitude planner is layered/2.5D, not a full continuous 3D kinodynamic planner.
- Mapping still uses simulated local sensor fallback.
- The garage visual mesh and wall proxy are not yet used as exact collision geometry for planning.
- Future work should use real Gazebo point clouds or FAST-LIVO2 output as the source map.
