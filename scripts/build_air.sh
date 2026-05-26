#!/usr/bin/env bash
set -eo pipefail

cd /home/nuaa/ZHY/TARE_V2_AIR
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee build_log.txt
