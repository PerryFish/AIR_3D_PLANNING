#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash

LOG_FILE="$REPO_ROOT/uav_motion_check_log.txt"
RAW_FILE=/tmp/tare_v2_air_uav_motion_raw.txt

rm -f "$RAW_FILE"
echo "Collecting /air/state_estimation for 10 seconds..." | tee "$LOG_FILE"
timeout -s INT -k 2s 10s ros2 topic echo /air/state_estimation --field pose.pose.position > "$RAW_FILE" || true

python3 - "$RAW_FILE" "$LOG_FILE" <<'PY'
import math
import re
import sys

raw_path, log_path = sys.argv[1], sys.argv[2]
text = open(raw_path, "r", encoding="utf-8", errors="ignore").read()
matches = re.findall(r"x:\s*([-+0-9.eE]+)\s*\ny:\s*([-+0-9.eE]+)\s*\nz:\s*([-+0-9.eE]+)", text)

with open(log_path, "a", encoding="utf-8") as log:
    if len(matches) < 2:
        log.write("FAIL: not enough /air/state_estimation samples\n")
        print("FAIL: not enough /air/state_estimation samples")
        sys.exit(1)

    start = tuple(float(v) for v in matches[0])
    end = tuple(float(v) for v in matches[-1])
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    total = math.sqrt(dx * dx + dy * dy + dz * dz)

    lines = [
        f"samples: {len(matches)}",
        f"start: x={start[0]:.3f}, y={start[1]:.3f}, z={start[2]:.3f}",
        f"end:   x={end[0]:.3f}, y={end[1]:.3f}, z={end[2]:.3f}",
        f"dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}, total_distance={total:.3f}",
    ]
    if total > 0.5:
        lines.append("PASS: UAV is moving")
    else:
        lines.append("FAIL: UAV appears stationary")
    if abs(dz) > 0.1:
        lines.append("PASS: UAV z is changing")
    else:
        lines.append("WARN: z change not obvious yet")

    for line in lines:
        print(line)
        log.write(line + "\n")

    sys.exit(0 if total > 0.5 else 2)
PY
