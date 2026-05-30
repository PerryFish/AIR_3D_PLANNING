#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p logs/tests logs/ros

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"

pkill -9 -x gzserver || true
pkill -9 -x gzclient || true
pkill -9 -x rviz2 || true
fuser -k 11346/tcp >/dev/null 2>&1 || true

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=false rviz:=false rviz_profile:=tare_v1_replay > logs/tests/tare_rviz_replay_topics.log 2>&1 &
PID=$!

cleanup() {
  if kill -0 "$PID" >/dev/null 2>&1; then
    kill -INT "-$PID" >/dev/null 2>&1 || true
    sleep 2
    kill -TERM "-$PID" >/dev/null 2>&1 || true
    wait "$PID" >/dev/null 2>&1 || true
  fi
  pkill -9 -x gzserver || true
  pkill -9 -x gzclient || true
}
trap cleanup EXIT

sleep 20

for topic in /overall_map /registered_scan /explored_areas /path /local_path /way_point /free_paths /uncovered_frontier_cloud; do
  if ! timeout -s INT -k 2s 10s ros2 topic echo "$topic" --once >/tmp/tare_replay_topic.txt 2>/tmp/tare_replay_topic.err; then
    echo "FAIL: no data on $topic"
    cat /tmp/tare_replay_topic.err || true
    exit 1
  fi
done

python3 - <<'PY'
from pathlib import Path
log = Path("logs/tests/tare_rviz_replay_topics.log").read_text(errors="ignore")
bad = ["Falling back on worlds/empty.world", "Could not open file[garage_v1]", "URI not supported by Fuel [garage_v1]"]
for token in bad:
    if token in log:
        raise SystemExit(f"FAIL: Gazebo world loading issue: {token}")
print("PASS: TARE RViz replay topics")
PY
