# P4 Escape / Rescan Fix Report

Date: 2026-05-26

## 1. What Changed

This round implemented a conservative P4 recovery layer for local waypoint oscillation and robot dithering. It does not rewrite TARE, local planner, Gazebo world, or robot model.

Modified files:

- `scripts/launch_tare_sim.sh`
- `scripts/launch_tare_sim_gui.sh`
- `scripts/launch_tare_sim_headless.sh`
- `scripts/launch_tare_rviz.sh`
- `src/tare_planner/src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h`
- `src/tare_planner/src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp`

Backup directory:

- `/home/nuaa/ZHY/TARE_V1/backups_before_P4_escape_rescan_20260526_163703`

## 2. Launch Script Fix

The old `scripts/launch_tare_sim.sh` was confusing because it used `gazebo_gui:=false rviz:=true`, so it opened RViz instead of Gazebo GUI.

Current behavior:

- `scripts/launch_tare_sim.sh`: Gazebo GUI by default, `gazebo_gui:=true rviz:=false`.
- `scripts/launch_tare_sim_gui.sh`: explicit Gazebo GUI launcher.
- `scripts/launch_tare_sim_headless.sh`: headless Gazebo launcher.
- `scripts/launch_tare_rviz.sh`: RViz/TARE visualization launcher.

All scripts print the actual command before launching.

GUI sanity log:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/gui_launch_sanity.log`

Result: `scripts/launch_tare_sim_gui.sh` started `gzserver` and `gzclient`; no `rviz2` process was launched by that script.

## 3. Stagnation Definition

The P4 logic treats the old failure mode as local fake progress:

- `/way_point` keeps publishing.
- The robot is moving.
- Recent waypoints stay inside a small radius for a sustained window.
- The robot travels distance locally but has small net displacement.
- Short-term movement alone is no longer enough to refresh the stuck timer.

Default constants are currently in C++:

- oscillation window: `15.0 s`
- waypoint radius: `1.5 m`
- robot net radius: `0.8 m`
- robot travel minimum: `2.0 m`
- recovery cooldown: `20.0 s`
- temporary blacklist duration: `30.0 s`
- temporary blacklist radius: `2.0 m`
- minimum direction change: `30 deg`
- max rejected local candidates before fallback: `5`

## 4. Waypoint Oscillation Detection

The planner records recent lookahead/waypoint samples in `recent_waypoint_history_`. If enough samples remain inside the configured local radius for the configured window, it logs:

- `WAYPOINT_OSCILLATION_DETECTED`

This detection is passive until combined with recovery gating.

## 5. Robot Dithering Detection

The planner records recent robot positions in `recent_robot_history_`. If total traveled distance is large but net displacement remains small, it logs:

- `ROBOT_DITHERING_DETECTED`

This catches the case where the robot is moving but not actually leaving the local area.

## 6. Recovery Trigger

When oscillation or dithering is detected and cooldown allows it, the planner:

- sets P4 recovery active,
- adds the oscillation center to a temporary recovery blacklist,
- also blacklists the last published waypoint position temporarily,
- resets the lookahead to the robot position to force a new planning attempt,
- logs `P4_ESCAPE_RECOVERY_TRIGGERED`.

This temporary blacklist is separate from the previous permanent blacklist logic.

## 7. Temporary Recovery Blacklist

During recovery, candidate waypoints inside a temporary recovery zone are rejected. The planner logs:

- `WAYPOINT_REJECTED_IN_OSCILLATION_ZONE`
- `WAYPOINT_REJECTED_SAME_DIRECTION`

If a new candidate leaves the temporary recovery zones, recovery is accepted and logs:

- `RECOVERY_ACCEPTED_NEW_DIRECTION`

The temporary zones expire automatically.

## 8. Direction Check

When recovery is active, the candidate direction is compared against the previous oscillation direction. If the candidate is still near the oscillation zone and direction change is below the threshold, it is rejected. This check is only active in recovery mode.

## 9. Local Fallback

If TARE repeatedly proposes candidates inside the temporary recovery zone, the planner no longer accepts those candidates after the rejection limit. Instead it publishes a finite local side-step fallback waypoint near the robot and logs:

- `P4_ESCAPE_FALLBACK_WAYPOINT`

This is still routed through the normal local planner and does not bypass collision checking.

## 10. Watchdog Progress Adjustment

The watchdog now distinguishes:

- `WATCHDOG_TRUE_PROGRESS`: robot leaves the local window, gets closer to the lookahead, or the target region meaningfully changes.
- `WATCHDOG_LOCAL_MOTION_ONLY`: robot moves but remains locally trapped; this does not refresh the stuck timer.

This preserves P2/P3 behavior while fixing the false progress condition observed in GUI testing.

## 11. P0-P3 Preservation

Confirmed preserved:

- NaN/Inf guard remains.
- Zero-distance waypoint guard remains.
- `PublishWaypoint()` strong watchdog remains disabled.
- Ordinary timeout does not directly add permanent blacklist.
- Recovery blacklist is temporary and separate from permanent blacklist.

## 12. Build Result

Build command:

```bash
cd /home/nuaa/ZHY/TARE_V1
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

- success
- `Summary: 13 packages finished [51.3s]`

Build log:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/build_after_P4_escape_rescan.log`

## 13. Runtime Test Result

Runtime logs:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/runtime_test_after_P4_escape_rescan.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/runtime_test_key_events.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/runtime_test_summary.md`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/gui_launch_sanity.log`

Observed:

- `/way_point` publishes at about 1 Hz.
- No `(-nan, -nan)` waypoint observed.
- No `all directions blacklisted` observed in the final TARE log.
- No `PERMANENT blacklist` event observed in the final TARE log.
- P4 oscillation/dithering detection triggered.
- P4 temporary recovery blacklist triggered.
- Same-zone candidates were rejected.
- Local fallback waypoint was published when repeated same-zone candidates persisted.
- In the final runtime log, key event counts were:
  - `WAYPOINT_OSCILLATION_DETECTED`: 505
  - `ROBOT_DITHERING_DETECTED`: 184
  - `P4_ESCAPE_RECOVERY_TRIGGERED`: 112
  - `P4_ESCAPE_FALLBACK_WAYPOINT`: 239

Diff artifacts:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/header.diff`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/cpp.diff`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/P4_escape_rescan/launch_tare_sim.diff`

## 14. Remaining Limitation

The deeper behavior is not fully solved. Logs show the upstream TARE candidate/lookahead generation can still repeatedly return to the same local area after recovery. P4 now detects and pushes against this, but repeated fallback events mean the next likely fix is in candidate selection/scoring or a frontier fallback, not more NaN/timeout guarding.

## 15. Recommendation

Accept this patch as a conservative anti-stagnation protection layer. It is materially better than P3 because it detects false progress and triggers recovery actions. It should be treated as a baseline for the next round, not as the final narrow-corridor exploration solution.

## 16. Rollback

Restore from:

```bash
/home/nuaa/ZHY/TARE_V1/backups_before_P4_escape_rescan_20260526_163703
```

Files to restore:

- `sensor_coverage_planner_ground.cpp`
- `sensor_coverage_planner_ground.h`
- `garage.yaml`
- `scripts/launch_tare_sim.sh`

Remove the newly added scripts if needed:

- `scripts/launch_tare_sim_gui.sh`
- `scripts/launch_tare_sim_headless.sh`
- `scripts/launch_tare_rviz.sh`

## 17. Next Step

The next most suspicious cause is upstream TARE candidate generation/viewpoint scoring repeatedly selecting the same local region. Recommended next work:

- add candidate scoring penalty for recently rejected recovery zones,
- add frontier fallback when candidates remain inside recovery zones,
- expose P4 constants in `garage.yaml`,
- optionally add a controller-layer rotate/rescan behavior when no new valid region is generated.
