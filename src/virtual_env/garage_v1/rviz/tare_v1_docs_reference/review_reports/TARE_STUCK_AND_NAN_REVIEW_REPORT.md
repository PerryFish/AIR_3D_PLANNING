# TARE Stuck and NaN Review Report

## 1. Conclusion Summary

This is a mixed issue. The narrow-area hesitation begins as a planning/local-control feasibility problem, but the hard failure is most likely caused by timeout/watchdog and blacklist recovery logic. The highest-confidence NaN source is zero-distance waypoint extension after the watchdog resets the lookahead to the robot position.

Recommended baseline: keep the current GitHub sync as a runnable baseline before making fixes.

## 2. Problem Chain Inferred from Logs

Observed logs show repeated absolute timeout, stuck detection, permanent blacklist insertion, waypoint reset to robot position, and waiting for TARE to generate a new waypoint.

Likely chain:

1. Narrow geometry causes slow progress or repeated lookahead.
2. Absolute timeout interprets this as stuck after about 15 seconds.
3. The target region is permanently blacklisted.
4. Recovery sets waypoint/lookahead to robot position.
5. The next waypoint extension divides by zero because target and robot coincide.
6. `/way_point` becomes `(-nan, -nan)`.
7. NaN is not filtered, and blacklist/candidate logic keeps the planner from recovering cleanly.

Permanent blacklist can worsen the issue because narrow passages often have few valid options. Removing one or two corridor regions may remove the only feasible route.

## 3. `/way_point` Generation Chain

| Item | Finding |
|---|---|
| Publisher package | `tare_planner` |
| Publisher node | `tare_planner_node` / `SensorCoveragePlanner3D` |
| Message type | `geometry_msgs/msg/PointStamped` |
| Frame | `map` |
| Publisher creation | `sensor_coverage_planner_ground.cpp:537-538` |
| Main publish function | `SensorCoveragePlanner3D::PublishWaypoint()` |
| Lookahead source | `GetLookAheadPoint(exploration_path_, global_path, lookahead_point_)` |
| Path sources | global TSP + local coverage planner |
| Fallback/reset sources | timeout logic, watchdog, reset waypoint callback, initial waypoint |

The main computation is `lookahead_point_ -> dx/dy/r -> extended waypoint -> /way_point`.

## 4. NaN Possible Sources, Ranked

High probability:

- `PublishWaypoint()` divides by `r` when `r == 0` after lookahead equals robot position.
- `GetLookAheadPoint()` normalizes zero vectors when selected/previous lookahead equals robot position.
- NaN is not rejected before publish or blacklist checks.

Medium probability:

- Empty/too-short local/global path leaves stale invalid lookahead.
- Candidate viewpoint exhaustion after blacklist returns early without a new safe target.
- Blacklist removes all viable narrow-corridor candidates.

Low probability:

- TSP solver directly generates NaN.
- TF frame failure creates invalid coordinates.

## 5. Narrow Area Stuck Sources, Ranked

High probability:

- Absolute timeout based on target age rather than progress.
- Permanent blacklist in corridor-like geometry.
- Reset-to-robot recovery feeding zero-distance math.

Medium probability:

- Local planner/path follower oscillation in narrow passages.
- Viewpoint collision/line-of-sight filtering eliminates valid candidates.
- Multiple watchdogs interact and repeatedly reset planner state.

Low probability:

- Gazebo sensor failure, because key perception topics are generally published.
- Frame tree failure, because reported TF includes map/sensor/vehicle paths.

## 6. Files and Functions to Inspect First

| File | Function | Logic | Why suspicious |
|---|---|---|---|
| `src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | `PublishWaypoint()` | waypoint extension, timeout, blacklist, final publish | Direct zero-division NaN path and no finite guard |
| same | `execute()` | planner loop and absolute watchdog | Forces `lookahead_point_` to robot position |
| same | `GetLookAheadPoint()` | lookahead selection and vector normalization | Normalizes potentially zero vectors |
| same | `ConcatenateGlobalLocalPath()` | combines global/local paths | Empty/short paths may leave no valid target |
| `src/tare_planner/src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h` | class fields | watchdog and blacklist state | Confirms added recovery state |
| `src/tare_planner/src/tare_planner/config/garage.yaml` | parameters | timeout and blacklist config | Fixed 15s timeout and permanent-style blacklist behavior |
| `src/tare_planner/src/tare_planner/src/local_coverage_planner/local_coverage_planner.cpp` | local solver | local path feasibility | Second-stage review after waypoint safety |
| `src/tare_planner/src/tare_planner/src/viewpoint_manager/viewpoint_manager.cpp` | viewpoint filtering | collision/LOS/candidate filtering | Candidate exhaustion in narrow spaces |

## 7. Next Modification Direction

Do not implement these in this review round.

P0: Add NaN/Inf and zero-distance guards. Do not publish illegal `/way_point`. Avoid normalizing zero vectors.

P1: Fix timeout/reset logic. Timeout should not set lookahead directly to robot position unless publish logic explicitly handles it as hover/rescan.

P2: Replace permanent blacklist for timeout recovery with temporary blacklist and decay. Keep permanent blacklist only for proven invalid regions.

P3: Add narrow-area escape behavior: rotate/rescan, short reverse, local temporary goal, or frontier fallback.

P4: Add planner fallback and return-home protection. If no valid candidate exists, publish an explicit safe mode rather than preserving an invalid old waypoint.

## 8. Things Not Recommended Immediately

- Do not only increase `kWaypointTimeout`.
- Do not simply delete blacklist.
- Do not convert NaN to zero coordinates.
- Do not skip collision checking.
- Do not force the waypoint farther away without validating collision and frame consistency.
- Do not add another independent watchdog.

## 9. Minimal Future Fix Plan

P0 should be the first code change:

- Check finite values in `lookahead_point_`, robot position, and computed waypoint.
- If `r < epsilon`, publish a safe hover/rescan command or skip publication with a clear status.
- Guard every `.normalize()` / `.normalized()` call with norm checks.

Then:

- Unify timeout behavior in one function.
- Make blacklist entries temporary by default.
- Add recovery states with explicit output semantics.

## 10. Short Summary for ChatGPT

Most suspicious file:

- `src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`

Most suspicious functions:

- `PublishWaypoint()`
- `execute()`
- `GetLookAheadPoint()`

Key log:

- `ABSOLUTE TIMEOUT TRIGGERED`
- `Waypoint age > 15.0 seconds threshold`
- `Robot has been stuck trying to reach (-nan, -nan)`
- `Adding this region to PERMANENT blacklist and requesting new waypoint`
- `ABSOLUTE TIMEOUT: Waypoint reset to robot position`

Recommended priority:

1. P0 finite/zero-distance guards.
2. P1 timeout/reset correction.
3. P2 temporary blacklist.
4. P3 escape/rescan behavior.
5. P4 planner fallback and return-home safety.

Yes, keep the current GitHub-synced version as baseline before changing behavior.
