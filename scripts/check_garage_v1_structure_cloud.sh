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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-67}"

setsid ros2 run aerial_exploration_planner garage_structure_cloud_node > logs/tests/garage_v1_structure_cloud.log 2>&1 &
NODE_PID=$!

cleanup() {
  if kill -0 "$NODE_PID" >/dev/null 2>&1; then
    kill -INT "-$NODE_PID" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "-$NODE_PID" >/dev/null 2>&1 || true
    wait "$NODE_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

sleep 2
timeout -s INT -k 2s 8s ros2 topic echo /exploration/garage_structure_cloud --once > /tmp/garage_v1_structure_cloud.txt
python3 - <<'PY'
from pathlib import Path
import re

pcd = Path("src/virtual_env/garage_v1/maps/garage_structure_from_tare.pcd")
if not pcd.exists():
    raise SystemExit("FAIL: missing TARE-derived garage structure PCD")
header = []
with pcd.open() as f:
    for line in f:
        header.append(line.strip())
        if line.strip().lower().startswith("data"):
            break
points_line = next((line for line in header if line.startswith("POINTS ")), None)
if not points_line:
    raise SystemExit("FAIL: PCD POINTS header missing")
file_points = int(points_line.split()[1])
if file_points < 5000:
    raise SystemExit(f"FAIL: TARE-derived structure cloud too sparse: {file_points}")

text = Path("/tmp/garage_v1_structure_cloud.txt").read_text()
if "frame_id: map" not in text:
    raise SystemExit("FAIL: garage structure cloud frame_id is not map")
width = None
for line in text.splitlines():
    if line.strip().startswith("width:"):
        width = int(line.split(":", 1)[1].strip())
        break
if width is None:
    raise SystemExit("FAIL: could not parse cloud width")
if width < 5000:
    raise SystemExit(f"FAIL: structure cloud too sparse: {width}")
print("PASS: garage_v1 structure cloud")
print(f"point_count={width}")
print(f"file_point_count={file_points}")
PY
