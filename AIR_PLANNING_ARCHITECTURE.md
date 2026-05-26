# TARE_V2_AIR Architecture

## Why TARE_V1 Is Not A Complete Aerial UAV Planner

TARE_V1 was useful as a ROS2 exploration demo, but its runtime stack was closer to a ground or fixed-height robot workflow than a complete UAV aerial planning system:

- Waypoints were effectively fixed-height or near-fixed-z targets, so the execution layer did not require real 3D climb, descent, or altitude-aware tracking.
- The runtime depended on `vehicle_simulator`, which behaves like a simple robot simulator rather than a multirotor flight stack.
- `localPlanner` and `pathFollower` were used as ground/fixed-height executors and are not a 3D UAV trajectory follower.
- There was no UAV dynamics or even a UAV-specific 3D kinematic tracking model.
- There was no dedicated 3D trajectory follower that tracks x/y/z waypoints and yaw in air.
- There was no PX4, MAVROS, `px4_ros_com`, ROS2 control, or flight-controller bridge.

For these reasons TARE_V1 must not be presented as a full UAV aerial path planning system.

## TARE_V2_AIR Goals

TARE_V2_AIR is a new independent ROS2 Humble workspace under `/home/nuaa/ZHY/TARE_V2_AIR`. It does not modify TARE_V1.

The MVP goals are:

- Real 3D aerial path planning with changing x/y/z coordinates.
- Configurable 3D start and goal poses.
- 3D obstacle environment with collision checking and inflation.
- RViz visualization of obstacles, start, goal, global path, smoothed path, UAV state, current waypoint, and planner status.
- A simple UAV kinematic simulator that follows a 3D path.
- Clean extension points for PX4, MAVROS, `px4_ros_com`, ROS2 control, OctoMap, ESDF, and advanced planners.

## Recommended System Structure

```text
air_world_provider
  publishes deterministic 3D obstacle boxes as MarkerArray
  provides the obstacle source used by the planner

air_global_planner
  input: /air/start, /air/goal, /air/occupancy_markers
  output: /air/global_path, /air/planner_status
  algorithm: 3D A*, extensible to RRT* or kinodynamic A*

air_trajectory_generator
  input: /air/global_path
  output: /air/smoothed_path, /air/trajectory
  simplifies and samples the 3D path without flattening z

air_uav_simulator
  input: /air/smoothed_path
  output: /air/state_estimation, /tf, /air/uav_marker
  simple 3D UAV kinematic model with yaw aligned to motion

air_mission_manager
  publishes default /air/start and /air/goal
  supports parameter-configured mission targets and external goal replanning

air_bringup
  launch, config, RViz, and runnable scripts
```

## Topic Naming

- `/air/start`: `geometry_msgs/PoseStamped`
- `/air/goal`: `geometry_msgs/PoseStamped`
- `/air/occupancy_markers`: `visualization_msgs/MarkerArray`
- `/air/global_path`: `nav_msgs/Path`
- `/air/smoothed_path`: `nav_msgs/Path`
- `/air/trajectory`: `nav_msgs/Path`
- `/air/current_waypoint`: reserved for future explicit waypoint topic
- `/air/state_estimation`: `nav_msgs/Odometry`
- `/air/planner_status`: `std_msgs/String`
- `/air/uav_marker`: `visualization_msgs/Marker`
- `/air/visualization/markers`: `visualization_msgs/MarkerArray`

## Message Types

The MVP intentionally uses standard ROS2 messages:

- `geometry_msgs/PoseStamped`
- `geometry_msgs/PointStamped`
- `nav_msgs/Path`
- `nav_msgs/Odometry`
- `visualization_msgs/Marker`
- `visualization_msgs/MarkerArray`
- `std_msgs/String`

`air_planning_msgs` is reserved for future custom messages once the interface needs exceed standard ROS2 messages.
