# Runbook

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
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
```
