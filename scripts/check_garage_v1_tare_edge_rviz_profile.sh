#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

RVIZ="src/aerial_exploration_planner/rviz/garage_v1_tare_edge_replay.rviz"
LAUNCH="src/aerial_exploration_planner/launch/visual_aerial_exploration_garage_v1.launch.py"
RUN="scripts/run_garage_v1_visual_exploration.sh"

fail() { echo "FAIL: $*"; exit 1; }
[ -f "$RVIZ" ] || fail "missing $RVIZ"
grep -q 'DeclareLaunchArgument("rviz_profile", default_value="tare_edge_replay")' "$LAUNCH" || fail "launch default is not tare_edge_replay"
grep -q '"tare_edge_replay": "garage_v1_tare_edge_replay.rviz"' "$LAUNCH" || fail "launch profile map missing tare_edge_replay"
grep -q 'rviz_profile:=tare_edge_replay' "$RUN" || fail "run script does not default to tare_edge_replay"
grep -q '/overall_map' "$RVIZ" || fail "overall map missing"
grep -q '/registered_scan' "$RVIZ" || fail "registered scan missing"
grep -q '/explored_areas' "$RVIZ" || fail "explored areas missing"
grep -q '/terrain_map' "$RVIZ" || fail "terrain map missing"
grep -q '/exploration/debug_surface_cloud' "$RVIZ" || fail "debug surface cloud missing"

python3 - <<'PY'
from pathlib import Path
text = Path("src/aerial_exploration_planner/rviz/garage_v1_tare_edge_replay.rviz").read_text().splitlines()
def enabled_for(name):
    for i, line in enumerate(text):
        if line.strip() == f"Name: {name}":
            for j in range(i, -1, -1):
                if text[j].startswith("    - Class:") and j < i:
                    break
                if text[j].strip().startswith("Enabled:"):
                    return text[j].split(":", 1)[1].strip()
    raise SystemExit(f"FAIL: display {name} not found")
for name in ("OverallMapEdgeCloud", "RegisteredScanLocalEdge", "ExploredAreas", "Path", "LocalPath", "GlobalPath", "Waypoint"):
    if enabled_for(name) != "true":
        raise SystemExit(f"FAIL: {name} should be enabled")
for name in ("DebugSurfaceCloud", "DebugVoxelOccupied", "LocalPlanningHorizon"):
    if enabled_for(name) != "false":
        raise SystemExit(f"FAIL: {name} should be disabled")
print("PASS: garage_v1 TARE edge RViz profile")
PY
