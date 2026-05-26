# GitHub Deployment Guide

## Repository Purpose

`AIR_3D_PLANNING` is a ROS2 Humble UAV 3D aerial path planning MVP. It provides a reproducible demo with 3D A*, 3D obstacles, smoothed path generation, a simple UAV kinematic simulator, RViz visualization, and motion diagnostics.

## Clone

```bash
git clone https://github.com/PerryFish/AIR_3D_PLANNING.git
cd AIR_3D_PLANNING
```

## Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

The repository scripts are designed for Ubuntu 22.04 and ROS2 Humble.

## Build

```bash
./scripts/build_air.sh
```

## Launch

```bash
./scripts/launch_air_demo.sh
```

Headless launch:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch air_bringup air_planning_demo.launch.py rviz:=false
```

## Check UAV Motion

Run this while the demo is active:

```bash
./scripts/check_uav_motion.sh
```

Expected result:

```text
PASS: UAV is moving
PASS: UAV z is changing
```

Depending on when the checker starts, z may already be near cruise altitude; in that case `WARN: z change not obvious yet` can appear even though x/y motion is valid.

## Send A New Goal

Run this while the demo is active:

```bash
./scripts/send_new_goal.sh
```

The script publishes a 3D goal on `/air/goal`.

## Confirm Z Coordinate Change

Use either the motion checker or echo the smoothed path:

```bash
ros2 topic echo /air/smoothed_path --once
ros2 topic echo /air/state_estimation
```

The default path climbs from `z=1.0` to approximately `z=4.5` and then approaches the goal at `z=4.0`.

## View In RViz

The default launch opens RViz2 with:

- 3D obstacle boxes
- global path
- smoothed path
- UAV marker
- UAV trail
- current waypoint marker
- UAV status marker

If RViz is blank, set fixed frame to `map` and confirm `/air/visualization/markers` and `/air/uav_trail` have data.

## Common Troubleshooting

If ROS2 cannot find packages:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg list | grep air_
```

If build fails:

```bash
source /opt/ros/humble/setup.bash
./scripts/build_air.sh
```

If UAV appears stationary:

```bash
./scripts/check_uav_motion.sh
ros2 topic hz /air/state_estimation
```

If `/air/state_estimation` has no data, confirm `air_uav_simulator` is running:

```bash
ros2 node list | grep air_uav_simulator
```

## Directories Not Uploaded

The following generated directories should not be committed:

- `build/`
- `install/`
- `log/`

Other ignored files include Python caches, rosbag files, editor metadata, swap files, and generic `*.log` files.

## Submit A New Version

```bash
git status
git add .
git commit -m "Describe the change"
git push
```

Do not commit credentials, private keys, build products, install products, ROS logs, or rosbags.
