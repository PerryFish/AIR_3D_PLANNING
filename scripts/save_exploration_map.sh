#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 topic pub --once /exploration/save_map std_msgs/msg/String "{data: save}"
sleep 1
echo "Saved map files under results/maps"
