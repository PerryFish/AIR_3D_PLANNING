#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash

RESULT_DIR="$REPO_ROOT/benchmark_results"
CSV="$RESULT_DIR/dense50_benchmark.csv"
REPORT="$RESULT_DIR/dense50_benchmark_report.md"
mkdir -p "$RESULT_DIR"

SEEDS=(${SEEDS:-1 2 3 4 5 6 7 8 9 10})
OCCUPANCY="${OCCUPANCY:-0.50}"
RUN_TIMEOUT="${RUN_TIMEOUT:-60}"

echo "seed,occupancy_ratio,success,failure_reason,planning_time_sec,expanded_nodes,path_waypoints,path_length_m,z_min,z_max,reached_goal,total_runtime_sec" > "$CSV"

cleanup_launch() {
  local pid="${1:-}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  fi
}

get_field() {
  local text="$1"
  local key="$2"
  echo "$text" | tr ' ' '\n' | awk -F= -v k="$key" '$1 == k {print $2; exit}'
}

wait_topic_field() {
  local topic="$1"
  local field="$2"
  local timeout_sec="$3"
  timeout "$timeout_sec"s ros2 topic echo "$topic" --once --field "$field" 2>/dev/null || true
}

for seed in "${SEEDS[@]}"; do
  echo "== dense50 seed $seed =="
  start_epoch="$(date +%s)"
  set +e
  setsid ros2 launch air_bringup air_dense50_demo.launch.py rviz:=false random_seed:="$seed" occupancy_ratio:="$OCCUPANCY" > "$RESULT_DIR/seed_${seed}.log" 2>&1 &
  launch_pid=$!
  set -e

  world_status="$(wait_topic_field /air/world_status data 15)"
  planner_status="$(wait_topic_field /air/planner_status data 20)"
  metrics="$(wait_topic_field /air/planning_metrics data 20)"

  success="$(get_field "$metrics" success)"
  failure_reason="$(get_field "$metrics" failure_reason)"
  planning_time="$(get_field "$metrics" planning_time_sec)"
  expanded_nodes="$(get_field "$metrics" expanded_nodes)"
  path_waypoints="$(get_field "$metrics" path_waypoints)"
  path_length="$(get_field "$metrics" path_length_m)"
  z_min="$(get_field "$metrics" z_min)"
  z_max="$(get_field "$metrics" z_max)"
  actual_occupancy="$(get_field "$world_status" actual_occupancy_ratio)"

  success="${success:-false}"
  failure_reason="${failure_reason:-missing_metrics}"
  planning_time="${planning_time:-0}"
  expanded_nodes="${expanded_nodes:-0}"
  path_waypoints="${path_waypoints:-0}"
  path_length="${path_length:-0}"
  z_min="${z_min:-0}"
  z_max="${z_max:-0}"
  actual_occupancy="${actual_occupancy:-$OCCUPANCY}"

  reached_goal=false
  deadline=$((start_epoch + RUN_TIMEOUT))
  while [[ "$(date +%s)" -lt "$deadline" ]]; do
    if grep -q "GOAL_REACHED" "$RESULT_DIR/seed_${seed}.log"; then
      reached_goal=true
      break
    fi
    sleep 1
  done

  runtime=$(( $(date +%s) - start_epoch ))
  cleanup_launch "$launch_pid"
  echo "$seed,$actual_occupancy,$success,$failure_reason,$planning_time,$expanded_nodes,$path_waypoints,$path_length,$z_min,$z_max,$reached_goal,$runtime" >> "$CSV"
done

python3 - "$CSV" "$REPORT" <<'PY'
import csv
import datetime as dt
import sys

csv_path, report_path = sys.argv[1], sys.argv[2]
rows = list(csv.DictReader(open(csv_path, newline="", encoding="utf-8")))
success_rows = [r for r in rows if r["success"] == "true"]
reached_rows = [r for r in rows if r["reached_goal"] == "true"]

def avg(key, rows_):
    vals = [float(r[key]) for r in rows_ if r.get(key)]
    return sum(vals) / len(vals) if vals else 0.0

failures = [f'seed {r["seed"]}: {r["failure_reason"]}' for r in rows if r["success"] != "true"]
with open(report_path, "w", encoding="utf-8") as f:
    f.write("# Dense50 Benchmark Report\n\n")
    f.write(f"- Test date: {dt.datetime.now().isoformat(timespec='seconds')}\n")
    f.write("- Configuration: random_occupancy_3d, occupancy_ratio=0.50, weighted_astar_3d\n")
    f.write(f"- Seeds: {len(rows)}\n")
    f.write(f"- PLAN_SUCCESS count: {len(success_rows)}\n")
    f.write(f"- PLAN_SUCCESS rate: {len(success_rows) / len(rows) * 100.0 if rows else 0.0:.1f}%\n")
    f.write(f"- Average planning time: {avg('planning_time_sec', success_rows):.3f} s\n")
    f.write(f"- Average expanded nodes: {avg('expanded_nodes', success_rows):.1f}\n")
    f.write(f"- Average path length: {avg('path_length_m', success_rows):.3f} m\n")
    f.write(f"- UAV reached goal count: {len(reached_rows)}\n")
    f.write(f"- UAV reached goal rate: {len(reached_rows) / len(rows) * 100.0 if rows else 0.0:.1f}%\n")
    f.write(f"- Failed seeds: {', '.join(failures) if failures else 'none'}\n\n")
    f.write("## Suggestions\n\n")
    f.write("- Tune inflation_radius and heuristic_weight for tighter passages.\n")
    f.write("- Add bidirectional A* or hierarchical planning if dense maps become larger.\n")
PY

echo "CSV: $CSV"
echo "Report: $REPORT"
