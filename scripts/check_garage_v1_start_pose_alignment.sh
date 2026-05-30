#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 scripts/analyze_garage_start_region.py >/tmp/garage_v1_start_pose_analysis.log

set +u
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
set -u

mkdir -p "$ROOT/logs/ros" "$ROOT/logs/gazebo_home"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT/logs/ros}"
export HOME="${GAZEBO_TEST_HOME:-$ROOT/logs/gazebo_home}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11346}"

pkill -9 -x gzserver || true
pkill -9 -x gzclient || true
pkill -9 -x gazebo || true
pkill -9 -f "ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py" || true
fuser -k 11346/tcp >/dev/null 2>&1 || true
sleep 1

ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=false rviz:=false rviz_profile:=tare_edge_replay > /tmp/garage_v1_start_pose_launch.log 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  pkill -9 -x gzserver || true
  pkill -9 -x gzclient || true
}
trap cleanup EXIT

python3 - <<'PY'
import json
import math
import subprocess
import sys
import time
from pathlib import Path

root = Path("/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN")
analysis = json.loads((root / "results/garage_v1/start_pose_analysis.json").read_text())
expected = analysis["recommended_start_pose"]

def read_json_topic(topic, timeout=45):
    cmd = ["timeout", str(timeout), "ros2", "topic", "echo", "--once", topic]
    out = subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)
    return out

def parse_position(text):
    lines = text.splitlines()
    values = {}
    for idx, line in enumerate(lines):
        stripped = line.strip()
        for key in ("x:", "y:", "z:"):
            if stripped.startswith(key):
                try:
                    values[key[0]] = float(stripped.split(":", 1)[1].strip())
                except ValueError:
                    pass
                if all(k in values for k in ("x", "y", "z")):
                    return values
    raise RuntimeError(f"could not parse position from topic output:\n{text[:1000]}")

deadline = time.time() + 80
last_error = None
while time.time() < deadline:
    try:
        state = parse_position(read_json_topic("/state_estimation", timeout=8))
        odom = parse_position(read_json_topic("/odom", timeout=8))
        marker = parse_position(read_json_topic("/exploration/start_pose_marker", timeout=8))
        break
    except Exception as exc:
        last_error = exc
        time.sleep(2)
else:
    raise SystemExit(f"FAIL: did not receive start pose topics: {last_error}")

def check_close(name, p, tol):
    dxy = math.hypot(p["x"] - expected["x"], p["y"] - expected["y"])
    dz = abs(p["z"] - expected["z"])
    print(f"{name}=({p['x']:.3f},{p['y']:.3f},{p['z']:.3f}) dxy={dxy:.3f} dz={dz:.3f}")
    if dxy > tol or dz > tol:
        raise SystemExit(f"FAIL: {name} not aligned with configured start pose")

check_close("state_estimation", state, 0.85)
check_close("odom", odom, 4.00)
check_close("start_marker", marker, 0.05)
if not analysis["recommended_inside_edge_bounds"]:
    raise SystemExit("FAIL: recommended start is outside edge-map bounds")
if analysis["recommended_inside_center_core_30pct"]:
    raise SystemExit("FAIL: recommended start falls inside central core")
print("PASS: garage_v1 start pose alignment")
PY
