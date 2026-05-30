# TARE_V1 Deployment Report

Date: 2026-05-25
Result: successful with auxiliary visualization and long-run return-home limitations.

## Build Result

Command:

```bash
cd /home/nuaa/ZHY/TARE_V1
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Result:

- `rosdep`: all required rosdeps installed.
- `colcon`: 13 packages finished successfully.
- Build log saved to `/home/nuaa/ZHY/TARE_V1/build_log.txt`.
- Warnings observed from OR-Tools/PCL and minor unused-parameter warnings; no build-blocking errors.

## Runtime Result

Simulation launch:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=false rviz:=true joy:=true visualization_tools:=false
```

TARE launch:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch tare_planner explore_garage.launch rviz:=true
```

Validated:

- Gazebo `gzserver`: started.
- Robot spawn: succeeded.
- Lidar spawn: succeeded.
- Camera spawn: succeeded.
- Velodyne plugin: ready.
- RViz: started.
- TARE planner: started and printed `Exploration Started`.
- TARE generated waypoint/lookahead updates.

Auxiliary limitation:

- `visualizationTools` segfaulted when enabled.
- `realTimePlot.py` crashed at shutdown/thread finalization.
- These are visualization/statistics helpers, not core TARE planning nodes. The stable script disables `visualization_tools`.

Long-run limitation:

- During validation TARE completed exploration and entered `returning home`.
- After extended runtime, the deployed local TARE config produced repeated timeout logs for waypoint `(-nan, -nan)`.
- For bimodal integration, add a wrapper-side finite-value check before forwarding `/way_point` to the flight controller. If a non-finite waypoint appears, hold position, request replanning/reset, or switch to a configured fail-safe.

## Topic Validation

Observed important topics:

- `/registered_scan` (`sensor_msgs/msg/PointCloud2`): about 5 Hz.
- `/state_estimation` (`nav_msgs/msg/Odometry`): about 200 Hz.
- `/way_point` (`geometry_msgs/msg/PointStamped`): about 1 Hz.
- `/terrain_map` (`sensor_msgs/msg/PointCloud2`): about 3-5 Hz.
- `/global_path`, `/local_path`, `/exploration_path`: `nav_msgs/msg/Path`.
- `/tare_visualizer/*`: marker/path/point cloud visualization outputs.

Sample `/way_point`:

```yaml
header:
  frame_id: map
point:
  x: 30.266277130027838
  y: 17.420136891675384
  z: 0.8383890986442566
```

Sample `/registered_scan` header:

```yaml
frame_id: map
```

## TF Validation

`ros2 run tf2_tools view_frames` generated `frames.pdf` and reported:

- `map -> sensor` at about 200 Hz.
- `map -> sensor_at_scan` at about 5 Hz.
- `sensor -> vehicle` static.
- `sensor -> camera` static.
- `lidar -> velodyne_base_link -> velodyne` static.

`base_link` and `odom` are not present in this stack. Integration should adapt to `map` and `sensor` or add bridge transforms.

## Modified Files

- `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/system_garage.launch`
  - Added `rviz`, `joy`, and `visualization_tools` launch arguments.
  - Added conditions so unstable optional nodes can be disabled without changing core planning behavior.
- `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/*`
  - Added official downloaded Gazebo environment models from `download_environments.sh`.
  - Required to resolve `model://garage` and allow robot/lidar/camera spawn.

Generated files:

- `/home/nuaa/ZHY/TARE_V1/DEPLOY_NOTES.md`
- `/home/nuaa/ZHY/TARE_V1/DEPLOY_REPORT.md`
- `/home/nuaa/ZHY/TARE_V1/TARE_INTERFACE_FOR_BIMODAL_UAV.md`
- `/home/nuaa/ZHY/TARE_V1/build_log.txt`
- `/home/nuaa/ZHY/TARE_V1/scripts/env.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/build_tare.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_sim.sh`
- `/home/nuaa/ZHY/TARE_V1/scripts/check_tare_topics.sh`

## Reproducible Commands

Build:

```bash
/home/nuaa/ZHY/TARE_V1/scripts/build_tare.sh
```

Launch simulation:

```bash
/home/nuaa/ZHY/TARE_V1/scripts/launch_tare_sim.sh
```

Launch TARE in another terminal:

```bash
source /home/nuaa/ZHY/TARE_V1/scripts/env.sh
ros2 launch tare_planner explore_garage.launch rviz:=true
```

Check:

```bash
/home/nuaa/ZHY/TARE_V1/scripts/check_tare_topics.sh
```
