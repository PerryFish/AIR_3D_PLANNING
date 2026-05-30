#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

pkill -9 -x gzserver || true
pkill -9 -x gzclient || true
pkill -9 -x gazebo || true
pkill -9 -x rviz2 || true
pkill -9 -f "ros2 launch aerial_exploration_planner" || true
fuser -k 11345/tcp >/dev/null 2>&1 || true
fuser -k 11346/tcp >/dev/null 2>&1 || true
sleep 2

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

SOURCE_MODELS="$ROOT/src/virtual_env/garage_v1/models"
INSTALL_MODELS="$ROOT/install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/models"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export GAZEBO_MODEL_PATH="${SOURCE_MODELS}:${INSTALL_MODELS}:/usr/share/gazebo-11/models"
mkdir -p "$ROOT/logs/ros"
mkdir -p "$ROOT/logs/gazebo_home"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"

if [ "$#" -eq 0 ]; then
  set -- gui:=true rviz:=true rviz_profile:=tare_edge_replay
fi

ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py "$@"
