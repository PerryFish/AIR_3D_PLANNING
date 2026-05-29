# GitHub Backup and Reuse

## Backup Metadata

- Backup branch: `backup/air3d-visual-sensor-mapping-demo`
- Backup data commit: `c69f080`
- Backup tag: `v0.4.0-visual-sensor-mapping-demo-backup`
- GitHub remote: `https://github.com/PerryFish/AIR_3D_PLANNING.git`

## Clone and Checkout

```bash
git clone https://github.com/PerryFish/AIR_3D_PLANNING.git
cd AIR_3D_PLANNING
git checkout backup/air3d-visual-sensor-mapping-demo
```

## Build

```bash
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install

set +u
source install/setup.bash
set -u
```

## Quick Demo

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN

pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f gazebo || true
pkill -9 -f rviz2 || true
pkill -9 -f "ros2 launch" || true
sudo fuser -k 11345/tcp || true
sudo fuser -k 11346/tcp || true
sleep 2

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
ros2 launch aerial_exploration_planner \
  visual_aerial_exploration_dense50.launch.py \
  gui:=true \
  rviz:=true
```

## Current Functionality

- dense50 Gazebo scene.
- UAV Gazebo visualization.
- Green trajectory points.
- Yellow goal marker.
- Aerial corridor height.
- Sensor-driven mapping baseline.
- Observed coverage.
- RViz visualization.
- Map export.

## Known Limitations

- This is not a complete FAST-LIVO/SLAM implementation.
- Some sensors use a simulated local sensor fallback.
- The planner may still need further work to eliminate ground truth leakage.
- This version is a demo and future development baseline.
