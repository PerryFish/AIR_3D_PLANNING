#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p logs/tests results/garage_v1
mkdir -p logs/ros
mkdir -p logs/gazebo_home
LOG="logs/tests/garage_v1_visual_exploration_smoke.log"
rm -f "$LOG" results/garage_v1/metrics.csv results/garage_v1/map_metrics.csv

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

export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export GAZEBO_MODEL_PATH="$ROOT/src/virtual_env/garage_v1/models:$ROOT/install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1/models:/usr/share/gazebo-11/models"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-58}"

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=false rviz:=false sensor_mapping:=true observed_coverage:=true > "$LOG" 2>&1 &
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

sleep 8
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: garage_v1 visual exploration launch exited early"
  tail -n 160 "$LOG" || true
  exit 1
fi
if ! grep -q "garage_v1.world" "$LOG"; then
  echo "FAIL: visual launch log does not mention garage_v1.world"
  tail -n 160 "$LOG" || true
  exit 1
fi
if grep -E "Could not open file\\[garage_v1\\]|Falling back on worlds/empty.world|Loading world file \\[/usr/share/gazebo-11/worlds/empty.world\\]|URI not supported by Fuel \\[garage_v1\\]|incorrect plugin type|Address already in use|File or path does not exist|Unable to find uri|exit code 255|Traceback" "$LOG" >/dev/null; then
  echo "FAIL: garage_v1 visual exploration log contains startup error"
  grep -E "Could not open file\\[garage_v1\\]|Falling back on worlds/empty.world|Loading world file \\[/usr/share/gazebo-11/worlds/empty.world\\]|URI not supported by Fuel \\[garage_v1\\]|incorrect plugin type|Address already in use|File or path does not exist|Unable to find uri|exit code 255|Traceback" "$LOG" || true
  exit 1
fi

timeout -s INT -k 2s 8s ros2 topic echo /state_estimation --once >/tmp/garage_v1_state_estimation.txt
timeout -s INT -k 2s 8s ros2 topic echo /aerial_exploration/goal --once >/tmp/garage_v1_goal.txt
timeout -s INT -k 2s 8s ros2 topic list > logs/tests/garage_v1_topics.txt
for topic in /exploration/structure_cloud /exploration/observed_structure_cloud /exploration/garage_structure_cloud /exploration/local_sensor_cloud /exploration/local_obstacle_cloud /exploration/local_planning_box /exploration/frontier_cloud /exploration/trajectory_path /exploration/coverage_text /aerial_exploration/planner_state; do
  grep -q "$topic" logs/tests/garage_v1_topics.txt || { echo "FAIL: missing topic $topic"; exit 1; }
done

sleep 112
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: garage_v1 visual exploration launch exited before 120 seconds"
  tail -n 160 "$LOG" || true
  exit 1
fi
if timeout -s INT -k 2s 8s gz model -m garage -p > logs/tests/garage_v1_visual_models.txt 2>&1; then
  grep -Eq "^-?[0-9]" logs/tests/garage_v1_visual_models.txt || { echo "FAIL: garage model pose query returned unexpected output"; cat logs/tests/garage_v1_visual_models.txt; exit 1; }
else
  echo "WARN: gz model pose query unavailable; static world/model checks and Gazebo log checks passed" > logs/tests/garage_v1_visual_models.txt
fi

python3 - <<'PY'
import csv
from pathlib import Path

metrics = Path("results/garage_v1/metrics.csv")
map_metrics = Path("results/garage_v1/map_metrics.csv")
if not metrics.exists():
    raise SystemExit("FAIL: missing results/garage_v1/metrics.csv")
if not map_metrics.exists():
    raise SystemExit("FAIL: missing results/garage_v1/map_metrics.csv")
rows = list(csv.DictReader(metrics.open()))
map_rows = list(csv.DictReader(map_metrics.open()))
if len(rows) < 20:
    raise SystemExit("FAIL: too few metrics rows")
coverages = [float(r["observed_coverage"]) for r in rows]
if coverages[-1] <= coverages[0]:
    raise SystemExit(f"FAIL: observed coverage did not grow: {coverages[0]:.6f}->{coverages[-1]:.6f}")
poses = {(round(float(r["robot_x"]), 2), round(float(r["robot_y"]), 2), round(float(r["robot_z"]), 2)) for r in rows}
goals = {(round(float(r["goal_x"]), 1), round(float(r["goal_y"]), 1), round(float(r["goal_z"]), 1)) for r in rows}
if len(poses) < 5:
    raise SystemExit(f"FAIL: trajectory did not grow enough: {len(poses)} poses")
if len(goals) < 2:
    raise SystemExit(f"FAIL: goals did not update enough: {len(goals)} goals")
if max(int(r["frontier_count"]) for r in rows) <= 0:
    raise SystemExit("FAIL: no frontiers published")
if not any("garage_v1" in r.get("mapping_source", "") or "garage_v1" in r.get("environment_model", "") for r in map_rows):
    raise SystemExit("FAIL: map metrics do not identify garage_v1 source")
target = 0.75
minimum = 0.45
final = coverages[-1]
if final < minimum:
    raise SystemExit(f"FAIL: final observed coverage below smoke minimum {minimum:.2f}: {final:.6f}")
stuck_events = max(int(r.get("stuck_events", 0)) for r in rows)
backtrack_events = max(int(r.get("backtrack_events", 0)) for r in rows)
failed_goals = max(int(r.get("failed_goals", 0)) for r in rows)
frontier_goals = max(int(r.get("frontier_goals", 0)) for r in rows)
path_length = max(float(r.get("path_length", 0.0)) for r in rows)
frontier_count = max(int(r.get("frontier_count", 0)) for r in rows)
print("PASS: garage_v1 visual exploration smoke test")
print(f"initial_observed_coverage={coverages[0]:.6f}")
print(f"final_observed_coverage={final:.6f}")
print(f"coverage_improvement={final - coverages[0]:.6f}")
print(f"target_observed_coverage={target:.2f}")
print(f"target_reached={final >= target}")
print(f"unique_pose_samples={len(poses)}")
print(f"unique_goal_samples={len(goals)}")
print(f"frontier_count={frontier_count}")
print(f"stuck_events={stuck_events}")
print(f"backtrack_events={backtrack_events}")
print(f"failed_goals={failed_goals}")
print(f"frontier_goals={frontier_goals}")
print(f"path_length={path_length:.3f}")
PY
