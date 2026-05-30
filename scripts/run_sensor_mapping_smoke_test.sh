#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p results logs/tests logs/ros logs/gazebo_home reports results/maps
rm -f logs/tests/sensor_mapping_smoke.log logs/tests/sensor_mapping_topics.txt logs/tests/sensor_mapping_metrics_tail.csv
rm -f results/metrics_dense50.csv results/map_metrics.csv

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11348}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-59}"

setsid ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=false rviz:=false sensor_mapping:=true observed_coverage:=true > logs/tests/sensor_mapping_smoke.log 2>&1 &
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

sleep 5
timeout -s INT -k 2s 8s ros2 topic list > logs/tests/sensor_mapping_topics.txt
for topic in /exploration/observed_cloud /exploration/occupied_cloud /exploration/free_cloud /exploration/unknown_frontiers /exploration/coverage_real /exploration/map_metrics; do
  grep -q "$topic" logs/tests/sensor_mapping_topics.txt || { echo "FAIL: missing topic $topic"; exit 1; }
done

python3 scripts/wait_for_exploration_done.py --timeout 120 --coverage-threshold 0.93 >> logs/tests/sensor_mapping_smoke.log 2>&1
tail -n 40 results/metrics_dense50.csv > logs/tests/sensor_mapping_metrics_tail.csv

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/metrics_dense50.csv").open()))
map_rows = list(csv.DictReader(Path("results/map_metrics.csv").open()))
if len(rows) < 10 or len(map_rows) < 10:
    raise SystemExit("FAIL: too few mapping metrics rows")
observed = [float(r["observed_coverage"]) for r in rows]
synthetic = [float(r["synthetic_coverage"]) for r in rows]
if observed[0] > 0.3:
    raise SystemExit(f"FAIL: observed coverage starts too high: {observed[0]:.6f}")
if observed[-1] < 0.93 or rows[-1]["done"] != "True":
    raise SystemExit("FAIL: observed coverage did not reach done threshold")
if max(observed[:5]) > 0.5:
    raise SystemExit("FAIL: observed coverage jumps too fast in early rows")
if int(rows[-1]["sensor_cloud_points"]) <= 0:
    raise SystemExit("FAIL: no local lidar scan points recorded")
if int(rows[-1]["frontier_count"]) <= 0:
    raise SystemExit("FAIL: no frontiers recorded")
if int(rows[-1]["occupied_voxels"]) <= 0 or int(rows[-1]["free_voxels"]) <= 0:
    raise SystemExit("FAIL: occupied/free observed map is empty")
Path("reports/16_sensor_mapping_smoke_test.md").write_text(
    "\n".join([
        "# Sensor Mapping Smoke Test",
        "",
        "- result: PASS",
        f"- initial_observed_coverage: {observed[0]:.6f}",
        f"- final_observed_coverage: {observed[-1]:.6f}",
        f"- final_synthetic_coverage: {synthetic[-1]:.6f}",
        f"- final_frontier_count: {rows[-1]['frontier_count']}",
        f"- final_sensor_cloud_points: {rows[-1]['sensor_cloud_points']}",
        f"- final_free_voxels: {rows[-1]['free_voxels']}",
        f"- final_occupied_voxels: {rows[-1]['occupied_voxels']}",
        "- lidar_source: local simulated ray casting from current UAV pose",
        "- camera_source: simulated camera frustum ray casting from current UAV pose",
        "",
    ])
)
print("PASS: sensor mapping smoke test")
print(f"initial_observed_coverage={observed[0]:.6f}")
print(f"final_observed_coverage={observed[-1]:.6f}")
print(f"final_synthetic_coverage={synthetic[-1]:.6f}")
PY
