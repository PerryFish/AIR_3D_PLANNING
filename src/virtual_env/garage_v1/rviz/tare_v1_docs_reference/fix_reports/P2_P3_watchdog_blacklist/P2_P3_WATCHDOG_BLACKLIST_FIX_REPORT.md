# P2/P3 Watchdog And Blacklist Fix Report

Date: 2026-05-26

Workspace: `/home/nuaa/ZHY/TARE_V1`

## Conclusion

This round modified source code. The change is conservative and limited to watchdog/blacklist safety in `sensor_coverage_planner_ground.cpp`.

P2 is completed: `execute()` is now the single owner of strong watchdog state changes. `PublishWaypoint()` keeps P0/P1 finite/zero-distance publish guards, but its old strong timeout/reset/permanent-blacklist action is disabled and replaced by a throttled `DUPLICATE_WATCHDOG_DISABLED` warning.

P3 is completed in minimal form: ordinary timeout no longer directly creates a permanent blacklist. Timeout regions now receive a failure count; only repeated failures at the same region can participate as permanent blacklist candidates.

## Modified Files

| File | Reason | Summary |
|---|---|---|
| `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | P2/P3 watchdog and blacklist fix | Added permanent-blacklist threshold helper, disabled strong timeout action in `PublishWaypoint()`, added movement-progress refresh in `execute()`, changed ordinary timeout to failure-count tracking. |

Backed up but not modified:

- `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h`
- `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/config/garage.yaml`

Backup directory:

`/home/nuaa/ZHY/TARE_V1/backups_before_watchdog_blacklist_fix_20260526_133714`

## P2 Details

- Strong watchdog remains in `execute()`.
- `PublishWaypoint()` no longer performs:
  - `ABSOLUTE TIMEOUT TRIGGERED`
  - reset waypoint to robot position
  - permanent blacklist insertion
  - request-new-waypoint side effects
- `PublishWaypoint()` still performs:
  - finite checks
  - zero-distance checks
  - last valid waypoint fallback
  - `/way_point` publishing

## P3 Details

- Added `kTimeoutFailuresBeforePermanentBlacklist = 3`.
- Added `IsPermanentBlacklistCount()`.
- Ordinary timeout records or increments a region failure count.
- Counts below 3 produce `TIMEOUT_NO_PERMANENT_BLACKLIST`.
- Counts at or above 3 produce `PERMANENT_BLACKLIST_AFTER_REPEATED_FAILURES`.
- Blacklist redirect/all-directions checks ignore entries whose failure count is below the permanent threshold.

## Movement Progress

`execute()` now tracks robot displacement while monitoring the current lookahead. If the robot moves more than `min(kProgressDistanceThreshold, 0.3)` meters, the watchdog timer is refreshed and `WATCHDOG_PROGRESS` is logged. This avoids treating slow but real movement in narrow areas as immediate stuck timeout.

## Build Result

Build succeeded.

Command:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build log:

`/home/nuaa/ZHY/TARE_V1/fix_reports/P2_P3_watchdog_blacklist/build_after_watchdog_blacklist_fix.log`

Build summary:

- 13 packages finished.
- No compile errors.
- Existing warnings remain in `tare_planner`.

## Runtime Test Result

Headless garage test completed for more than 5 minutes.

Observed:

- Gazebo garage headless started.
- TARE planner started and explored.
- `/way_point` published normally at about 1 Hz.
- `/state_estimation`, `/registered_scan`, and `/terrain_map` were active.
- No actual coordinate NaN waypoint was observed.
- One `ZERO_DISTANCE_WAYPOINT` guard event occurred.
- `WATCHDOG_PROGRESS` appeared repeatedly.
- `DUPLICATE_WATCHDOG_DISABLED` appeared, confirming duplicate strong watchdog action was disabled.
- `ABSOLUTE TIMEOUT TRIGGERED` did not appear.
- `all directions blacklisted` did not appear.
- `PERMANENT_BLACKLIST_AFTER_REPEATED_FAILURES` did not appear.
- `TIMEOUT_NO_PERMANENT_BLACKLIST` appeared once.

Runtime summary:

`/home/nuaa/ZHY/TARE_V1/fix_reports/P2_P3_watchdog_blacklist/runtime_test_summary.md`

Runtime log:

`/home/nuaa/ZHY/TARE_V1/fix_reports/P2_P3_watchdog_blacklist/runtime_test_after_watchdog_blacklist_fix.log`

## Impact

This change should reduce narrow-area deadlock caused by duplicate timeout/reset/blacklist behavior. It does not claim to fully solve all narrow-area navigation failures, because local planner reachability and TARE candidate selection are unchanged.

The main improvement is that slow movement refreshes the watchdog, and ordinary timeout no longer immediately makes a region permanent blacklist.

## Recommendation

Accept this change as the next baseline after the P0/P1 NaN guard. It is scoped, builds successfully, and preserves original planner behavior except for watchdog/blacklist safety.

Next round should handle P4 escape/rescan only if narrow-area stalls still occur after longer scenario tests.

## Rollback

Restore the backed-up source file:

```bash
cp /home/nuaa/ZHY/TARE_V1/backups_before_watchdog_blacklist_fix_20260526_133714/sensor_coverage_planner_ground.cpp \
  /home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp
```

Then rebuild:

```bash
cd /home/nuaa/ZHY/TARE_V1
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Diff Summary

- Added permanent-blacklist threshold helper.
- Filtered blacklist redirect logic to only use permanent entries.
- Replaced `PublishWaypoint()` strong timeout/reset/blacklist block with throttled warning.
- Added progress-based watchdog refresh in `execute()`.
- Changed timeout blacklist behavior from immediate permanent insertion to failure-count tracking.
