#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/nuaa/ZHY/TARE_V2_AIR/install/setup.bash

ros2 topic pub --once /air/goal geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: -6.0, y: 7.0, z: 5.0},
    orientation: {w: 1.0}
  }
}"
