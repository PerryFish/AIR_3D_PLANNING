# TARE_V1 Deployment Notes

Date: 2026-05-25
Host: Ubuntu 22.04.5 LTS, kernel 6.8.0-111-generic, ROS 2 Humble.
Workspace: `/home/nuaa/ZHY/TARE_V1`

## Source Selection

- GitHub clone of `https://github.com/HongbiaoZ/autonomous_exploration_development_environment` was attempted but failed because the GitHub SSL connection timed out.
- A local archive `/home/nuaa/ZHY/TARE.zip` was used. It contains a ROS 2 Humble/Jazzy version of the development environment plus `tare_planner`.
- The copied source is installed under:
  - `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment`
  - `/home/nuaa/ZHY/TARE_V1/src/tare_planner`
- Official simulation environments were downloaded with:
  - `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/download_environments.sh`

## Environment Checks

- `ROS_DISTRO=humble`
- `ros2`: `/opt/ros/humble/bin/ros2`
- `colcon`: `/usr/bin/colcon`
- `cmake`: `/usr/bin/cmake`
- `git`: `/usr/bin/git`
- `gcc/g++`: 11.4.0
- `python3`: 3.10.12
- `gazebo`, `gzserver`, `gzclient`: `/usr/bin`
- `rviz2`: `/opt/ros/humble/bin/rviz2`

The user shell had an existing `A_DWA` workspace in `AMENT_PREFIX_PATH`. Build and scripts explicitly source `/opt/ros/humble/setup.bash` and this workspace setup.

## ROS Compatibility

This deployment uses ROS 2 packages:

- `package.xml` format: 3
- Build type: `ament_cmake`
- Client API: `rclcpp` / `rclpy`
- Launch files: Python launch content with `.launch` filenames
- Build tool: `colcon`

No ROS 1/catkin migration was required.

## Package / Module Inventory

- Simulation environment: `vehicle_simulator`
  - Gazebo world files: `garage.world`, `campus.world`, `forest.world`, `indoor.world`, `tunnel.world`
  - Robot/camera/lidar descriptions: `urdf/robot.sdf`, `camera.urdf.xacro`, `lidar.urdf.xacro`
  - Downloaded Gazebo models: `mesh/garage`, `mesh/campus`, `mesh/forest`, `mesh/indoor`, `mesh/tunnel`
- Terrain analysis: `terrain_analysis`, `terrain_analysis_ext`
- Local planning and path following: `local_planner`
- Waypoint examples/tools: `waypoint_example`, `waypoint_rviz_plugin`
- Sensor scan generation: `sensor_scan_generation`
- TARE planner: `tare_planner`
- Visualization: `visualization_tools`, TARE visualizer publishers inside `tare_planner`
- Robot/sensor model: `vehicle_simulator`, `velodyne_simulator`

## Launch Entry Points

- Gazebo/system launch:
  - `ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=false rviz:=true joy:=true visualization_tools:=false`
- TARE launch:
  - `ros2 launch tare_planner explore_garage.launch rviz:=true`
- Generic TARE launch:
  - `ros2 launch tare_planner explore.launch scenario:=garage rviz:=true`

`system_garage.launch` was patched to add optional `rviz`, `joy`, and `visualization_tools` launch arguments. The default behavior is preserved, but the generated script disables `visualization_tools` because that auxiliary node segfaulted during validation.

## TARE Interfaces Observed

Input topics:

- `/state_estimation_at_scan` (`nav_msgs/msg/Odometry`) used by TARE
- `/registered_scan` (`sensor_msgs/msg/PointCloud2`)
- `/terrain_map` (`sensor_msgs/msg/PointCloud2`)
- `/terrain_map_ext` (`sensor_msgs/msg/PointCloud2`)
- `/sensor_coverage_planner/coverage_boundary` (`geometry_msgs/msg/PolygonStamped`)
- `/sensor_coverage_planner/nogo_boundary` (`geometry_msgs/msg/PolygonStamped`)
- `/joy` (`sensor_msgs/msg/Joy`)
- `/reset_waypoint` (`std_msgs/msg/Empty`)

Output topics:

- `/way_point` (`geometry_msgs/msg/PointStamped`)
- `/exploration_path` (`nav_msgs/msg/Path`)
- `/global_path`, `/global_path_full`, `/local_path`, `/path` (`nav_msgs/msg/Path`)
- `/runtime` (`std_msgs/msg/Float32`)
- `/runtime_breakdown` (`std_msgs/msg/Int32MultiArray`)
- `/exploration_finish` (`std_msgs/msg/Bool`)
- TARE visualizer topics under `/tare_visualizer/*`

Frames:

- Main world frame: `map`
- Moving sensor frame: `sensor`
- Scan-time frame: `sensor_at_scan`
- Static child frames: `vehicle`, `camera`, `lidar`, `velodyne_base_link`, `velodyne`
- `base_link` and `odom` are not used by this stack.

## Important Files

- TARE config: `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/config/garage.yaml`
- TARE RViz: `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz`
- Vehicle RViz: `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/rviz/vehicle_simulator.rviz`
- Build log: `/home/nuaa/ZHY/TARE_V1/build_log.txt`
