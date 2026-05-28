#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p results logs/tests reports
rm -f logs/tests/visual_exploration_smoke.log logs/tests/visual_topics.txt logs/tests/visual_nodes.txt logs/tests/visual_services.txt logs/tests/visual_models.txt logs/tests/visual_metrics_tail.csv reports/09_visual_simulation_smoke_test.md

python3 scripts/generate_dense50_gazebo_world.py > logs/tests/visual_world_generation.log

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-57}"

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=false rviz:=false > logs/tests/visual_exploration_smoke.log 2>&1 &
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

sleep 3
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: visual launch process exited early"
  tail -n 120 logs/tests/visual_exploration_smoke.log || true
  exit 1
fi
if grep -E "incorrect plugin type|exit code 255|World.cc:1803|Missing model.config.*A_DWA" logs/tests/visual_exploration_smoke.log >/dev/null; then
  echo "FAIL: Gazebo startup log contains plugin/model-path error"
  grep -E "incorrect plugin type|exit code 255|World.cc:1803|Missing model.config.*A_DWA" logs/tests/visual_exploration_smoke.log || true
  exit 1
fi
timeout -s INT -k 2s 8s ros2 node list > logs/tests/visual_nodes.txt
timeout -s INT -k 2s 8s ros2 topic list > logs/tests/visual_topics.txt
timeout -s INT -k 2s 8s ros2 service list > logs/tests/visual_services.txt

for node in /synthetic_mapping_node /simple_uav_follower_node /aerial_exploration_node /exploration_metrics_node /mode_manager_node /gazebo_uav_visualizer /gazebo_trail_visualizer; do
  grep -q "$node" logs/tests/visual_nodes.txt || { echo "FAIL: missing node $node"; exit 1; }
done

for service in /spawn_entity /gazebo/set_entity_state; do
  grep -q "$service" logs/tests/visual_services.txt || { echo "FAIL: missing Gazebo service $service"; exit 1; }
done

for topic in /odom /aerial_exploration/path /aerial_exploration/goal /aerial_exploration/coverage /aerial_exploration/frontiers /aerial_exploration/viewpoints /aerial_exploration/map_markers /aerial_exploration/coverage_marker; do
  grep -q "$topic" logs/tests/visual_topics.txt || { echo "FAIL: missing topic $topic"; exit 1; }
done

python3 scripts/wait_for_exploration_done.py --timeout 120 --coverage-threshold 0.93 >> logs/tests/visual_exploration_smoke.log 2>&1
if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
  echo "FAIL: visual launch process exited before test completion"
  tail -n 120 logs/tests/visual_exploration_smoke.log || true
  exit 1
fi
if grep -E "incorrect plugin type|exit code 255|World.cc:1803|Missing model.config.*A_DWA" logs/tests/visual_exploration_smoke.log >/dev/null; then
  echo "FAIL: Gazebo log contains plugin/model-path error"
  grep -E "incorrect plugin type|exit code 255|World.cc:1803|Missing model.config.*A_DWA" logs/tests/visual_exploration_smoke.log || true
  exit 1
fi
tail -n 40 results/metrics_dense50.csv > logs/tests/visual_metrics_tail.csv
timeout -s INT -k 2s 8s ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}" > logs/tests/visual_models.txt
grep -q "simple_uav" logs/tests/visual_models.txt || { echo "FAIL: simple_uav model not found in Gazebo"; cat logs/tests/visual_models.txt; exit 1; }
grep -q "uav_trail_" logs/tests/visual_models.txt || { echo "FAIL: no uav_trail breadcrumb models found in Gazebo"; cat logs/tests/visual_models.txt; exit 1; }
grep -q "current_exploration_goal" logs/tests/visual_models.txt || { echo "FAIL: current_exploration_goal marker not found in Gazebo"; cat logs/tests/visual_models.txt; exit 1; }

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/metrics_dense50.csv").open()))
if len(rows) < 10:
    raise SystemExit("FAIL: too few metrics rows")
coverages = [float(r["coverage"]) for r in rows]
if coverages[0] > 0.2:
    raise SystemExit(f"FAIL: initial coverage too high: {coverages[0]:.6f}")
if coverages[-1] < 0.93 or rows[-1]["done"] != "True":
    raise SystemExit("FAIL: final coverage/done not reached")
poses = {(round(float(r["robot_x"]), 2), round(float(r["robot_y"]), 2), round(float(r["robot_z"]), 2)) for r in rows}
goals = {(round(float(r["goal_x"]), 1), round(float(r["goal_y"]), 1), round(float(r["goal_z"]), 1)) for r in rows}
deltas = [round(b - a, 6) for a, b in zip(coverages, coverages[1:]) if b - a > 1e-6]
if len(poses) < 8:
    raise SystemExit(f"FAIL: odom/pose did not change enough: {len(poses)}")
if len(goals) < 3:
    raise SystemExit(f"FAIL: goal did not change enough: {len(goals)}")
if len(set(deltas[:15])) <= 2 and len(deltas) >= 15:
    raise SystemExit(f"FAIL: coverage deltas look fixed: {deltas[:15]}")
if sum(1 for r in rows if int(r["newly_observed_voxels"]) > 0) < 6:
    raise SystemExit("FAIL: observation updates are not driving coverage")
Path("reports/09_visual_simulation_smoke_test.md").write_text(
    "\n".join([
        "# Visual Simulation Smoke Test",
        "",
        "- result: PASS",
        "- gazebo_process_alive: True",
        "- gazebo_uav_model_visible: True",
        "- gazebo_trail_breadcrumbs_visible: True",
        "- gazebo_goal_marker_visible: True",
        "- no_incorrect_plugin_type: True",
        "- no_exit_code_255: True",
        "- no_world_cc_1803: True",
        "- a_dwa_model_path_isolated: True",
        "- gazebo: launched headless with dense50_ground_footprint.world",
        "- rviz: launch command available; smoke test used rviz:=false for CI stability",
        f"- initial_coverage: {coverages[0]:.6f}",
        f"- final_coverage: {coverages[-1]:.6f}",
        f"- done: {rows[-1]['done']}",
        f"- unique_pose_samples: {len(poses)}",
        f"- unique_goal_samples: {len(goals)}",
        f"- final_failed_goals: {rows[-1]['failed_goals']}",
        f"- final_stuck_events: {rows[-1]['stuck_events']}",
        "- coverage_source: observed voxels from simulated odom and sensor range",
        "- limitation: Gazebo renders the dense50 world; robot motion is ROS2 kinematic follower, not Gazebo physics control",
        "",
    ])
)
print("PASS: visual exploration smoke test")
print(f"initial_coverage={coverages[0]:.6f}")
print(f"final_coverage={coverages[-1]:.6f}")
print(f"unique_pose_samples={len(poses)}")
print(f"unique_goal_samples={len(goals)}")
PY
