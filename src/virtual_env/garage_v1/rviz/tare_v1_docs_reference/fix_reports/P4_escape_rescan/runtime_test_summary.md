# P4 Runtime Test Summary

Date: 2026-05-26

## Commands

Simulation:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/launch_tare_sim_headless.sh
```

TARE:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch tare_planner explore_garage.launch rviz:=false
```

Topic checks:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 topic hz /way_point --window 10
ros2 topic echo --once /way_point
```

## Result

- Build result: success, 13 packages finished.
- Headless Gazebo started successfully.
- GUI launch sanity check passed: `scripts/launch_tare_sim_gui.sh` started both `gzserver` and `gzclient`; no `rviz2` process was launched by that script.
- Robot, lidar, camera, terrain analysis, local planner, path follower, and vehicle simulator started.
- TARE planner started and printed `Exploration Started`.
- `/way_point` continued publishing at about 1 Hz.
- One sampled `/way_point` was finite:
  - frame: `map`
  - point: `(14.697, 63.765, 7.834)`
- No `(-nan, -nan)` waypoint was observed.
- No `all directions blacklisted`, `PERMANENT blacklist`, or `ABSOLUTE TIMEOUT TRIGGERED` event was found in the final TARE log.

## P4 Events Observed

Observed in:

- `runtime_test_after_P4_escape_rescan.log`
- `runtime_test_key_events.log`
- `gui_launch_sanity.log`

Key events:

- `WAYPOINT_OSCILLATION_DETECTED`
- `ROBOT_DITHERING_DETECTED`
- `P4_ESCAPE_RECOVERY_TRIGGERED`
- `TEMP_RECOVERY_BLACKLIST_ADDED`
- `WAYPOINT_REJECTED_IN_OSCILLATION_ZONE`
- `WAYPOINT_REJECTED_SAME_DIRECTION`
- `P4_ESCAPE_FALLBACK_WAYPOINT`
- `RECOVERY_ACCEPTED_NEW_DIRECTION`
- `WATCHDOG_LOCAL_MOTION_ONLY`
- `WATCHDOG_TRUE_PROGRESS`

## Interpretation

P4 detection and recovery logic is active. The planner now detects local waypoint oscillation and robot dithering, temporarily rejects the local recovery zone, and publishes a local side-step fallback when TARE repeatedly proposes candidates inside the same recovery zone.

This improves the previous failure mode where local motion was always counted as progress and no recovery action occurred. However, the runtime log still shows repeated oscillation in later areas because the upstream TARE candidate/lookahead generation continues to propose the same local target region. The P4 patch prevents passive waiting and prevents NaN, but it does not fully solve the deeper candidate scoring/frontier fallback problem.

## Shutdown Notes

On Ctrl-C shutdown, several ROS2 nodes reported `rclcpp::exceptions::RCLError` during context destruction. This happened after the test was stopped and is consistent with previous shutdown behavior; it was not observed as a runtime planning crash.
