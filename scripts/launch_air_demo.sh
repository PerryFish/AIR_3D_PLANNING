#!/usr/bin/env bash
set -eo pipefail

cd /home/nuaa/ZHY/TARE_V2_AIR
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch air_bringup air_planning_demo.launch.py
