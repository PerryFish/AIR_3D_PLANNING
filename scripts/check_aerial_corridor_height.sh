#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p logs/tests logs/ros logs/gazebo_home reports
rm -f logs/tests/aerial_corridor_height.log logs/tests/aerial_corridor_height_metrics.csv

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11347}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-58}"

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=false rviz:=false > logs/tests/aerial_corridor_height.log 2>&1 &
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

sleep 4
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: launch exited before height check"
  tail -n 120 logs/tests/aerial_corridor_height.log || true
  exit 1
fi

python3 scripts/wait_for_exploration_done.py --timeout 120 --coverage-threshold 0.93 >> logs/tests/aerial_corridor_height.log 2>&1
tail -n 80 results/metrics_dense50.csv > logs/tests/aerial_corridor_height_metrics.csv

python3 - <<'PY'
import csv
from pathlib import Path

min_z = 0.8
max_z = 2.2
obstacle_top_z = 2.5
max_above_obstacle_margin = 0.2
rows = list(csv.DictReader(Path("results/metrics_dense50.csv").open()))
if len(rows) < 10:
    raise SystemExit("FAIL: too few metrics rows for corridor height validation")

robot_z = [float(r["robot_z"]) for r in rows if float(r["robot_z"]) > 0.0]
goal_z = [float(r["goal_z"]) for r in rows if abs(float(r["goal_x"])) + abs(float(r["goal_y"])) + abs(float(r["goal_z"])) > 0.0]
if not robot_z:
    raise SystemExit("FAIL: no non-zero robot_z samples")
if not goal_z:
    raise SystemExit("FAIL: no goal_z samples")

bad_robot = [z for z in robot_z if z < min_z - 1e-3 or z > max_z + 1e-3]
bad_goal = [z for z in goal_z if z < min_z - 1e-3 or z > max_z + 1e-3]
too_high_robot = [z for z in robot_z if z > obstacle_top_z + max_above_obstacle_margin]
too_high_goal = [z for z in goal_z if z > obstacle_top_z + max_above_obstacle_margin]
if bad_robot:
    raise SystemExit(f"FAIL: robot_z outside corridor [{min_z}, {max_z}]: {bad_robot[:8]}")
if bad_goal:
    raise SystemExit(f"FAIL: goal_z outside corridor [{min_z}, {max_z}]: {bad_goal[:8]}")
if too_high_robot or too_high_goal:
    raise SystemExit("FAIL: robot/goal z is above obstacle-top margin")

Path("reports/12_aerial_corridor_height_check.md").write_text(
    "\n".join([
        "# Aerial Corridor Height Check",
        "",
        "- result: PASS",
        f"- corridor_min_z: {min_z}",
        f"- corridor_max_z: {max_z}",
        "- corridor_default_z: 1.4",
        f"- dense50_obstacle_height_range_m: 0.7-2.5",
        f"- robot_z_min: {min(robot_z):.3f}",
        f"- robot_z_max: {max(robot_z):.3f}",
        f"- goal_z_min: {min(goal_z):.3f}",
        f"- goal_z_max: {max(goal_z):.3f}",
        f"- final_coverage: {float(rows[-1]['coverage']):.6f}",
        f"- done: {rows[-1]['done']}",
        "",
    ])
)
print("PASS: aerial corridor height check")
print(f"robot_z_range={min(robot_z):.3f}..{max(robot_z):.3f}")
print(f"goal_z_range={min(goal_z):.3f}..{max(goal_z):.3f}")
PY
