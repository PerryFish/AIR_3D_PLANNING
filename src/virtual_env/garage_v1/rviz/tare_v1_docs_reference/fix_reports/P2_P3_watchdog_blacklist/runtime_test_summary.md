# P2/P3 Headless Runtime Test Summary

Date: 2026-05-26

Workspace: `/home/nuaa/ZHY/TARE_V1`

## Commands

Simulation:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=false rviz:=false real_time_plot:=false visualization_tools:=false joy:=false
```

TARE:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch tare_planner explore_garage.launch rviz:=false
```

Observation:

```bash
ros2 node list
ros2 topic list
ros2 topic info /way_point
ros2 topic echo --once /way_point
ros2 topic hz /way_point --window 10
ros2 topic hz /state_estimation --window 10
ros2 topic hz /registered_scan --window 10
ros2 topic hz /terrain_map --window 10
```

## Result

- Test duration: more than 5 minutes headless.
- Gazebo garage headless started successfully.
- TARE planner started successfully and printed `Exploration Started`.
- `/way_point` was published as `geometry_msgs/msg/PointStamped`, frame `map`.
- `/way_point` rate was approximately 1 Hz.
- `/state_estimation`, `/registered_scan`, and `/terrain_map` were active.
- No coordinate NaN waypoint was observed.
- One `ZERO_DISTANCE_WAYPOINT` guard event occurred and skipped unsafe normalization.
- `WATCHDOG_PROGRESS` appeared repeatedly, confirming movement refresh logic.
- `DUPLICATE_WATCHDOG_DISABLED` appeared when `PublishWaypoint()` observed age timeout, confirming strong action is owned by `execute()`.
- `ABSOLUTE TIMEOUT TRIGGERED` did not appear.
- `all directions blacklisted` did not appear.
- `PERMANENT_BLACKLIST_AFTER_REPEATED_FAILURES` did not appear in this run.
- `TIMEOUT_NO_PERMANENT_BLACKLIST` appeared once, confirming ordinary timeout did not directly become permanent blacklist.

## Notes

The runtime log contains a textual warning saying `Skip publish to avoid dx/r NaN`; this is the guard message, not an actual published NaN coordinate.

Stopping the simulation with Ctrl-C produced known ROS shutdown `RCLError` messages from some simulator/local planner processes. TARE exited cleanly, and no ROS/Gazebo process remained afterward.

Runtime log:

`/home/nuaa/ZHY/TARE_V1/fix_reports/P2_P3_watchdog_blacklist/runtime_test_after_watchdog_blacklist_fix.log`

TARE node log:

`/home/nuaa/ZHY/TARE_V1/log/tare_planner_node_46619_1779775980418.log`
