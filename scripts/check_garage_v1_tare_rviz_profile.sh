#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

RVIZ="src/aerial_exploration_planner/rviz/garage_v1_tare_v1_replay.rviz"
LAUNCH="src/aerial_exploration_planner/launch/visual_aerial_exploration_garage_v1.launch.py"
RUN="scripts/run_garage_v1_visual_exploration.sh"

fail() { echo "FAIL: $*"; exit 1; }

[ -f "$RVIZ" ] || fail "missing garage_v1_tare_v1_replay.rviz"
grep -q 'DeclareLaunchArgument("rviz_profile", default_value="tare_v1_replay")' "$LAUNCH" || fail "launch does not default rviz_profile to tare_v1_replay"
grep -q "tare_rviz_replay_bridge_node" "$LAUNCH" || fail "launch missing replay bridge node"
grep -q "rviz_profile:=tare_v1_replay" "$RUN" || fail "run script does not default to tare_v1_replay"

python3 - <<'PY'
from pathlib import Path
lines = Path("src/aerial_exploration_planner/rviz/garage_v1_tare_v1_replay.rviz").read_text().splitlines()

def enabled_for(name):
    for i, line in enumerate(lines):
        if line.strip() == f"Name: {name}":
            for j in range(i, max(-1, i - 16), -1):
                stripped = lines[j].strip()
                if stripped.startswith("Enabled:"):
                    return stripped.split(":", 1)[1].strip()
    raise SystemExit(f"FAIL: display not found: {name}")

for name in ("OverallMap", "RegScan", "ExploredAreas", "Path", "LocalPath", "GlobalPath", "Waypoint", "FreePaths"):
    if enabled_for(name) != "true":
        raise SystemExit(f"FAIL: {name} must be enabled")
for name in ("UncoveredCloud", "UncoveredFrontierCloud", "LocalPlanningHorizon", "ExploringSubspaces"):
    if enabled_for(name) != "false":
        raise SystemExit(f"FAIL: {name} must be disabled/weak by default")
if any("CUBE_LIST" in line for line in lines):
    raise SystemExit("FAIL: replay RViz should not include voxel cube display")
print("PASS: garage_v1 TARE RViz profile")
PY
