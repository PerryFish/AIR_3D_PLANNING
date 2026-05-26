#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
ros2 topic list
timeout -s INT -k 2s 8s ros2 topic hz /air/state_estimation --window 5 || true
timeout -s INT -k 2s 8s ros2 topic echo /air/planner_status --once || true
timeout -s INT -k 2s 8s ros2 topic echo /air/global_path --once || true
timeout -s INT -k 2s 8s ros2 topic echo /air/smoothed_path --once || true
