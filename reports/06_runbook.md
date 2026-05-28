# Runbook

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
python3 scripts/generate_dense50_gazebo_world.py
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install
set +u
source install/setup.bash
set -u
python3 scripts/validate_dense50_ground_ratio.py --config config/aerial_exploration.yaml
timeout -s INT -k 10s 180s scripts/run_aerial_exploration_tests.sh
timeout -s INT -k 10s 120s scripts/run_anti_fake_coverage_tests.sh
timeout -s INT -k 10s 180s scripts/run_visual_exploration_smoke_test.sh
timeout -s INT -k 10s 120s scripts/check_aerial_corridor_height.sh
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true
ros2 launch aerial_exploration_planner gazebo_dense50.launch.py gui:=true
```

Gazebo visual exploration should show the dense50 world, blue `simple_uav` model, green breadcrumb trail spheres, and a yellow current goal marker. The UAV, breadcrumbs, and goal should stay in the aerial corridor at about `1.4 m`, with valid z range `0.8-2.2 m`, rather than flying above the full obstacle field. If the UAV is not visible:

```bash
ros2 node list | grep gazebo
ros2 service list | grep -E "spawn|state|model"
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
echo "$GAZEBO_MODEL_PATH"
```

If the trail is not visible, check that `/state_estimation` is publishing, `gazebo_trail_visualizer` is running, and `min_distance` or `max_points` have not filtered out new breadcrumbs.

Recommended GUI launch:

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true
```
