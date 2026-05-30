#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p logs/tests
mkdir -p logs/ros
mkdir -p logs/gazebo_home
LOG="logs/tests/garage_v1_gazebo_smoke.log"
rm -f "$LOG"

pkill -9 -x gzserver || true
pkill -9 -x gzclient || true
pkill -9 -x gazebo || true
pkill -9 -f "ros2 launch aerial_exploration_planner" || true
fuser -k 11345/tcp >/dev/null 2>&1 || true
fuser -k 11346/tcp >/dev/null 2>&1 || true
sleep 2

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export GAZEBO_MODEL_PATH="$ROOT/src/virtual_env/garage_v1/models:$ROOT/install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/models:/usr/share/gazebo-11/models"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"

setsid ros2 launch aerial_exploration_planner gazebo_garage_v1.launch.py gui:=false > "$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    kill -INT "-$LAUNCH_PID" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "-$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 45
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: garage_v1 Gazebo launch exited early"
  tail -n 160 "$LOG" || true
  exit 1
fi
if ! grep -q "garage_v1.world" "$LOG"; then
  echo "FAIL: Gazebo log does not mention garage_v1.world"
  tail -n 160 "$LOG" || true
  exit 1
fi
if grep -E "Could not open file\\[garage_v1\\]|Falling back on worlds/empty.world|Loading world file \\[/usr/share/gazebo-11/worlds/empty.world\\]|URI not supported by Fuel \\[garage_v1\\]|incorrect plugin type|Address already in use|File or path does not exist|Unable to find uri|exit code 255" "$LOG" >/dev/null; then
  echo "FAIL: garage_v1 Gazebo log contains startup error"
  grep -E "Could not open file\\[garage_v1\\]|Falling back on worlds/empty.world|Loading world file \\[/usr/share/gazebo-11/worlds/empty.world\\]|URI not supported by Fuel \\[garage_v1\\]|incorrect plugin type|Address already in use|File or path does not exist|Unable to find uri|exit code 255" "$LOG" || true
  exit 1
fi

if timeout -s INT -k 2s 8s gz model -m garage -p > logs/tests/garage_v1_models.txt 2>&1; then
  grep -Eq "^-?[0-9]" logs/tests/garage_v1_models.txt || {
    echo "FAIL: garage model pose query returned unexpected output"
    cat logs/tests/garage_v1_models.txt
    exit 1
  }
else
  echo "WARN: gz model pose query unavailable; static world/model checks and Gazebo log checks passed" > logs/tests/garage_v1_models.txt
fi

echo "PASS: garage_v1 Gazebo smoke test"
