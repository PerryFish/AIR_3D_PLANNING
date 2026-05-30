#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

RVIZ="src/aerial_exploration_planner/rviz/garage_v1_tare_reference.rviz"
LAUNCH="src/aerial_exploration_planner/launch/visual_aerial_exploration_garage_v1.launch.py"

fail() {
  echo "FAIL: $*"
  exit 1
}

[ -f "$RVIZ" ] || fail "missing TARE reference RViz config"
grep -q '"clean": "garage_v1_tare_reference.rviz"' "$LAUNCH" || fail "clean RViz mode does not use TARE reference config"
grep -q "/exploration/garage_structure_cloud" "$RVIZ" || fail "static garage structure cloud missing"
grep -q "/exploration/observed_structure_cloud" "$RVIZ" || fail "observed structure cloud missing"
grep -q "/exploration/local_sensor_cloud" "$RVIZ" || fail "local sensor cloud missing"
grep -q "Background Color: 0; 0; 0" "$RVIZ" || fail "background is not black"

python3 - <<'PY'
from pathlib import Path

lines = Path("src/aerial_exploration_planner/rviz/garage_v1_tare_reference.rviz").read_text().splitlines()

def enabled_for(name):
    for i, line in enumerate(lines):
        if line.strip() == f"Name: {name}":
            for j in range(i, max(-1, i - 14), -1):
                stripped = lines[j].strip()
                if stripped.startswith("Enabled:"):
                    return stripped.split(":", 1)[1].strip()
    raise SystemExit(f"FAIL: display not found: {name}")

expected_on = [
    "Garage Static Structure Cloud",
    "Observed Structure Cloud",
    "Local Sensor Cloud",
    "UAV Trajectory",
    "Planned Path",
    "Coverage Text",
]
for name in expected_on:
    if enabled_for(name) != "true":
        raise SystemExit(f"FAIL: {name} should be enabled")

expected_off = [
    "Frontier Candidates",
    "Debug Local Planning Box",
    "Debug Observed Free Voxel Cloud",
]
for name in expected_off:
    if enabled_for(name) != "false":
        raise SystemExit(f"FAIL: {name} should be disabled")

widths = [float(line.split(":", 1)[1].strip()) for line in lines if "Line Width:" in line]
if max(widths) > 0.03:
    raise SystemExit(f"FAIL: line width too large: {max(widths)}")
sizes = [float(line.split(":", 1)[1].strip()) for line in lines if "Size (m):" in line]
if max(sizes) > 0.035:
    raise SystemExit(f"FAIL: point size too large: {max(sizes)}")

print("PASS: garage_v1 TARE reference RViz view")
print(f"max_path_line_width={max(widths):.3f}")
print(f"max_point_size={max(sizes):.3f}")
PY
