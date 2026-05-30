#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p logs/tests
mkdir -p logs/ros
LOG="logs/tests/tare_edge_replay_topics.log"
rm -f "$LOG"

set +u
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
set -u

python3 scripts/generate_garage_edge_cloud_from_tare_pointcloud.py >/tmp/tare_edge_topics_generate.json
export GAZEBO_MODEL_PATH="$PWD/src/virtual_env/garage_v1/models:$PWD/install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/models:/usr/share/gazebo-11/models"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export ROS_LOG_DIR="$PWD/logs/ros"

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=false rviz:=false rviz_profile:=tare_edge_replay > "$LOG" 2>&1 &
PID=$!
cleanup() {
  kill -INT "-$PID" >/dev/null 2>&1 || true
  sleep 2
  kill -TERM "-$PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT
sleep 25

for topic in /overall_map /registered_scan /explored_areas /terrain_map /path /local_path /way_point; do
  timeout -s INT -k 2s 12s ros2 topic echo "$topic" --once >/tmp/tare_edge_topic.txt
done

python3 - <<'PY'
from pathlib import Path
log = Path("logs/tests/tare_edge_replay_topics.log").read_text(errors="ignore")
bad = ["Falling back on worlds/empty.world", "Could not open file[garage_v1]", "URI not supported by Fuel [garage_v1]", "Traceback"]
for item in bad:
    if item in log:
        raise SystemExit(f"FAIL: launch log contains {item}")
print("PASS: TARE edge replay topics publish data")
PY
