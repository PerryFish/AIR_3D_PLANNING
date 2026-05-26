#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p log

ros2 launch air_bringup air_planning_demo.launch.py rviz:=false > log/headless_air_demo.log 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  sleep 2
  kill -TERM "$LAUNCH_PID" >/dev/null 2>&1 || true
  wait "$LAUNCH_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

sleep 8
./scripts/check_air_topics.sh | tee log/headless_air_check.log
sleep 22
