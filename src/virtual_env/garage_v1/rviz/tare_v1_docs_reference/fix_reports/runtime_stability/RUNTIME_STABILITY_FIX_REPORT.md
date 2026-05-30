# Runtime Stability Fix Report

Date: 2026-05-26

## 1. Algorithm Source Changes

No algorithm source was modified in this round.

Not modified:

- `sensor_coverage_planner_ground.cpp`
- `sensor_coverage_planner_ground.h`
- TARE P0/P1/P2/P3/P4 planner logic
- Gazebo world
- robot model

Only runtime scripts and reports were changed.

## 2. Scripts Modified / Added

Script backup:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/script_backups_20260526_223947`

Added:

- `/home/nuaa/ZHY/TARE_V1/scripts/clean_tare_runtime.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/diagnose_system_graphics.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/diagnose_gazebo_gui.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/diagnose_rviz_display.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/run_stable_p4_test.sh`

Updated:

- `/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_sim.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_sim_gui.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_sim_headless.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_rviz.sh`

Details:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/launch_script_runtime_fix.md`

## 3. Cleanup Script

`clean_tare_runtime.sh` was created and executed.

It now matches exact process command names instead of broad text patterns, preventing accidental self-kill when diagnostic script/log paths contain words like `gazebo`.

Cleanup log:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/clean_runtime.log`

Final process check after testing showed no residual Gazebo/RViz/TARE planner runtime processes.

## 4. System Graphics Diagnosis

Report:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/system_graphics_diagnostics.md`

Observed:

- `DISPLAY=:0`
- `XDG_SESSION_TYPE=x11`
- GPU detected: NVIDIA RTX 3090
- `nvidia-smi` could not communicate with the NVIDIA driver.
- `dmesg` access is restricted, so kernel OOM/GPU logs could not be fully inspected.
- No clear accessible OOM evidence was found.

Assessment:

- The previous `gzclient exit -9` could still be caused by graphics driver/remote desktop/OpenGL instability, but it was not reproduced in the controlled single `gzclient` test.
- For algorithm validation, headless Gazebo remains the recommended stable path.

## 5. Gazebo GUI Diagnosis

Report:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/gazebo_gui_diagnostics.log`

Result:

- `gzserver` empty world started.
- `gzclient` started and stayed alive for 20 seconds.
- `gzclient exit -9` was not reproduced in this diagnostic run.
- Memory was sufficient: about 125 GiB total, about 111 GiB free/available during the test.

Conclusion:

- Gazebo GUI is not universally broken on this machine.
- The observed `gzclient exit -9` is likely load/context dependent, such as garage world rendering load, graphics driver state, remote desktop/session behavior, or OpenGL/NVIDIA runtime instability.
- Do not rely on Gazebo GUI for unattended algorithm tests.

## 6. RViz Diagnosis

Report:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/rviz_display_diagnostics.log`

Result:

- `rviz2` starts.
- OpenGL version reported: `4.6`.
- RViz config files were found.
- Recommended RViz Fixed Frame: `map`.
- During standalone RViz diagnosis, runtime topics were absent because simulation/TARE were intentionally not running. This is not a RViz failure.

Conclusion:

- RViz itself can launch.
- If RViz shows no planning data, verify that headless simulation and TARE are running, then check `/tf`, `/way_point`, `/state_estimation`, `/registered_scan`, and `/terrain_map`.

## 7. Stable Headless P4 Test

Command:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/run_stable_p4_test.sh
```

Logs:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_test.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_sim.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_tare.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_key_events.log`
- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_test_summary.md`

Result:

- Headless Gazebo started.
- TARE planner started.
- `/state_estimation` appeared.
- `/registered_scan` appeared.
- `/terrain_map` appeared.
- `/way_point` appeared.
- `/way_point` rate sample: about `1.013 Hz`.

P4 event counts:

- `WAYPOINT_OSCILLATION_DETECTED`: 6
- `ROBOT_DITHERING_DETECTED`: 4
- `P4_ESCAPE_RECOVERY_TRIGGERED`: 3
- `P4_ESCAPE_FALLBACK_WAYPOINT`: 0 in this specific run
- `RECOVERY_ACCEPTED_NEW_DIRECTION`: 3

Safety/negative checks:

- NaN/invalid waypoint count: 0
- `all directions blacklisted` / `PERMANENT blacklist`: 0
- `ABSOLUTE TIMEOUT TRIGGERED`: 0

Note:

- The log contains many normal messages of the form `ABSOLUTE TIMEOUT: New waypoint from TARE ... Timeout timer started.` These are timer-start informational logs, not timeout-trigger failure events.

## 8. Recommended Stable Runtime

For algorithm/backend validation:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/run_stable_p4_test.sh
```

For interactive development with lower GUI risk:

Terminal 1:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/launch_tare_sim_headless.sh
```

Terminal 2:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/launch_tare_rviz.sh
```

Gazebo GUI only when needed:

```bash
cd /home/nuaa/ZHY/TARE_V1
./scripts/launch_tare_sim.sh gui
```

If `gzclient` exits with code `-9`, stop using GUI for that session and return to headless Gazebo + RViz.

## 9. Should P5 Start Now?

Recommendation: temporarily pause P5 until the runtime workflow is consistently used in headless mode.

Reason:

- P4 backend is working and stable in headless mode.
- GUI failure is not currently blocking algorithm validation.
- P5 should target candidate scoring/frontier fallback only after collecting stable headless logs that prove the remaining issue is planner candidate selection rather than runtime/GUI instability.

## 10. If GUI Must Be Used

Next steps:

- test local monitor/X11 session instead of remote desktop,
- verify NVIDIA driver state because `nvidia-smi` failed,
- inspect full system logs with sudo access for OOM/GPU kills,
- reduce Gazebo rendering load,
- avoid running Gazebo GUI and RViz simultaneously on unstable sessions.

## 11. Final Conclusion

The algorithm backend is stable under headless Gazebo. RViz can start independently. Gazebo GUI was available in the controlled empty-world diagnostic but may still be unstable under garage-world or desktop-driver load. The recommended default workflow is headless Gazebo plus optional RViz, with `run_stable_p4_test.sh` for unattended P4 verification.
