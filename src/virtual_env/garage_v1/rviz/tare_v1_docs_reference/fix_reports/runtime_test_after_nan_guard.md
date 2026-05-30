# Runtime Test After NaN Guard

## Test Setup

Workspace:

- `/home/nuaa/ZHY/TARE_V1`

Build:

- `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`
- Build log: `/home/nuaa/ZHY/TARE_V1/fix_reports/build_after_nan_guard.log`

Simulation command used:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=false rviz:=false real_time_plot:=false visualization_tools:=false joy:=false
```

TARE command used:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch tare_planner explore_garage.launch rviz:=false
```

Observation log:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_observation_after_nan_guard.txt`
- TARE node log: `/home/nuaa/ZHY/TARE_V1/log/tare_planner_node_43780_1779767681731.log`

## Topic Observation

Observed:

- `/way_point` type: `geometry_msgs/msg/PointStamped`
- `/way_point` publisher count: 1
- `/way_point` subscriber count: 1
- Sample `/way_point`: finite, frame `map`
- `/way_point` rate: about 1.0 Hz
- `/state_estimation` rate: about 200 Hz
- `/registered_scan` rate: about 5.2 Hz
- `/terrain_map` rate: about 4.3 to 5.3 Hz

Sample finite waypoint:

```yaml
frame_id: map
point:
  x: 23.177046615721736
  y: 103.54330249993478
  z: 0.8367651700973511
```

## NaN Result

No runtime log match was found for:

- `(-nan`
- `x: nan`
- `y: nan`
- `z: nan`
- `Robot has been stuck trying to reach (-nan`

The new guard did trigger as intended:

- `ZERO_DISTANCE_WAYPOINT`
- `PUBLISH_LAST_VALID_WAYPOINT`

Example:

```text
ZERO_DISTANCE_WAYPOINT: robot and lookahead are too close (r=0.000000)...
PUBLISH_LAST_VALID_WAYPOINT: zero-distance waypoint before extension...
```

This confirms the zero-distance `dx/r` path was reached and blocked before producing NaN.

## Timeout / Stuck Result

Timeout still occurs:

- `ABSOLUTE WATCHDOG TIMEOUT FIRED`
- `ABSOLUTE TIMEOUT TRIGGERED`
- `REGION BLACKLIST: ALL directions blacklisted! Forcing exploration finish.`

This is expected because this round did not change timeout or blacklist strategy.

## Shutdown Note

The test was stopped with SIGINT. Some simulator-side nodes printed `rclcpp::exceptions::RCLError` during shutdown. This happened after interrupt and was not observed as a runtime failure during the active test.

## Test Conclusion

- Gazebo headless launched: yes
- TARE launched: yes
- `/way_point` published: yes
- `(-nan, -nan)` observed: no
- Timeout still observed: yes
- Blacklist deadlock still observed: yes
- P0/P1 NaN guard behavior: validated
