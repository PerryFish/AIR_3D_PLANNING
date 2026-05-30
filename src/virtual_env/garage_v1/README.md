# Garage V1 Virtual Environment

`garage_v1` is a migrated Gazebo Classic garage scene from `/home/nuaa/ZHY/TARE_V1`.

The runtime world is:

```bash
src/virtual_env/garage_v1/worlds/garage_v1.world
```

The required custom Gazebo model path is:

```bash
src/virtual_env/garage_v1/models
```

The world uses `model://garage`, and `garage/model.sdf` references `model://garage/meshes/garage.dae`.
System model `model://sun` is resolved from `/usr/share/gazebo-11/models`.

TARE launch files, RViz configs, planner config, waypoint examples, and deployment notes are copied as reference material only. AIR does not launch TARE planner nodes for this baseline.

Quick launch after building:

```bash
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true
```

Current limitation: this is a Gazebo/RViz exploration baseline with a local simulated sensor fallback, not full FAST-LIVO/SLAM.
