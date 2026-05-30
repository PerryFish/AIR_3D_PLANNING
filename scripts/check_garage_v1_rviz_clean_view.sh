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

[ -f "$RVIZ" ] || fail "missing clean RViz config"
grep -q 'DeclareLaunchArgument("rviz_view_mode", default_value="clean")' "$LAUNCH" || fail "garage launch does not default rviz_view_mode to clean"
grep -q '"clean": "garage_v1_tare_reference.rviz"' "$LAUNCH" || fail "clean mode does not use TARE reference RViz"
grep -q "Garage Static Structure Cloud" "$RVIZ" || fail "missing Garage Static Structure Cloud display"
grep -q "/exploration/garage_structure_cloud" "$RVIZ" || fail "garage structure cloud topic missing"
grep -q "/exploration/observed_structure_cloud" "$RVIZ" || fail "observed structure cloud topic missing"
grep -q "Coverage Text" "$RVIZ" || fail "coverage text display missing"

python3 - <<'PY'
from pathlib import Path
text = Path("src/aerial_exploration_planner/rviz/garage_v1_tare_reference.rviz").read_text().splitlines()

def enabled_for(name):
    for i, line in enumerate(text):
        if line.strip() == f"Name: {name}":
            for j in range(i, max(-1, i - 12), -1):
                stripped = text[j].strip()
                if stripped.startswith("Enabled:"):
                    return stripped.split(":", 1)[1].strip()
    raise SystemExit(f"FAIL: display not found: {name}")

if enabled_for("Debug Local Planning Box") != "false":
    raise SystemExit("FAIL: local planning box must be disabled in clean view")
if enabled_for("Debug Observed Free Voxel Cloud") != "false":
    raise SystemExit("FAIL: free voxel cloud must be disabled in clean view")
if enabled_for("Garage Static Structure Cloud") != "true":
    raise SystemExit("FAIL: garage structure cloud must be enabled in clean view")

widths = []
for line in text:
    if "Line Width:" in line:
        widths.append(float(line.split(":", 1)[1].strip()))
if not widths:
    raise SystemExit("FAIL: no path line widths found")
if max(widths) > 0.03:
    raise SystemExit(f"FAIL: path line width too large: {max(widths)}")
print("PASS: garage_v1 clean RViz view")
print(f"max_path_line_width={max(widths):.3f}")
PY
