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
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true
ros2 launch aerial_exploration_planner gazebo_dense50.launch.py gui:=true
```
