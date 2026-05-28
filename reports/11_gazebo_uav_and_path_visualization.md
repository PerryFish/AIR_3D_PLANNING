# Gazebo UAV And Path Visualization

## Problem

Gazebo showed the dense50 obstacle world, but did not show the UAV body, current goal, or vehicle motion trail. RViz showed ROS topics such as path, goal, coverage, and markers, but Gazebo does not automatically render `nav_msgs/Path` or RViz markers.

## Root Cause

- The Gazebo world only contained ground and dense50 obstacle models.
- The exploration loop published pose and path through ROS topics.
- RViz subscribed to those topics, but Gazebo needed explicit Gazebo entities or visuals.

## Changes

Models:

- Added `models/simple_uav/model.config`
- Added `models/simple_uav/model.sdf`
- Added package-installed copy under `src/aerial_exploration_planner/models/simple_uav/`

Nodes:

- Added `gazebo_uav_visualizer`
  - spawns `simple_uav` through `/spawn_entity`
  - subscribes to `/state_estimation`
  - updates the Gazebo entity through `/gazebo/set_entity_state`
- Added `gazebo_trail_visualizer`
  - subscribes to `/state_estimation`
  - spawns green breadcrumb spheres named `uav_trail_*`
  - subscribes to `/aerial_exploration/goal`
  - spawns and updates yellow `current_exploration_goal`

Launch and install:

- Updated `visual_aerial_exploration_dense50.launch.py`
- Added launch arguments:
  - `gazebo_uav_visual:=true`
  - `gazebo_trail_visual:=true`
  - `gazebo_waypoint_visual:=true`
- Updated `setup.py` to install models and console entry points.
- Updated `package.xml` with `gazebo_msgs`.

## Pose Source

Gazebo UAV visualization follows `/state_estimation`.

- type: `nav_msgs/msg/Odometry`
- publisher: `simple_uav_follower_node`
- frame: `map`, treated as Gazebo `world`
- visual z offset: `0.0 m`

`/odom` is also published, but `/state_estimation` was selected because it is the existing state-estimation-style topic and is already consumed by the exploration node.

## Trail Visualization

Gazebo path visualization uses breadcrumb spheres, not a Gazebo line primitive.

- trail prefix: `uav_trail`
- default max points: `300`
- min distance: `0.8 m`
- spawn period limit: `0.5 s`
- goal marker: `current_exploration_goal`

## Verification

Build: PASS

Installed model check: PASS

Visual smoke test: PASS

- Gazebo UAV model visible: True
- Gazebo trail breadcrumbs visible: True
- Gazebo goal marker visible: True
- `/odom` and `/state_estimation` changed during exploration: True
- `/aerial_exploration/path` published: True
- `/aerial_exploration/goal` changed: True
- final coverage: `0.940000`
- dense50 footprint ratio: `0.500000`

GUI launch check:

- Gazebo started.
- RViz started.
- `simple_uav` spawned successfully.
- `/get_model_list` returned `simple_uav`, `current_exploration_goal`, and many `uav_trail_*` models.

## Known Limitations

- Gazebo UAV movement is kinematic visualization driven by ROS odometry and `/gazebo/set_entity_state`; it is not Gazebo physics flight control.
- Gazebo trail is sampled pose breadcrumbs, not the planner's internal `nav_msgs/Path` rendered as a continuous line.
- The current corridor fix keeps UAV, goal, and breadcrumbs at the real `/state_estimation` and goal z. It does not visually lift them above obstacles.
- The dense sensor-built map is visualized primarily in RViz through `/exploration/*` clouds and markers; Gazebo stays focused on world, UAV, trajectory breadcrumbs, and goal marker.

## Commands

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true
```

Checks:

```bash
ros2 node list | grep gazebo
ros2 topic list | grep aerial
ros2 service list | grep -E "spawn|state|model"
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```
