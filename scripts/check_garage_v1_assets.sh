#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WORLD="src/virtual_env/garage_v1/worlds/garage_v1.world"
MODELS="src/virtual_env/garage_v1/models"
INSTALL_BASE="install/aerial_exploration_planner/share/aerial_exploration_planner/virtual_env/garage_v1"

fail() {
  echo "FAIL: $*"
  exit 1
}

[ -d "src/virtual_env/garage_v1/worlds" ] || fail "missing worlds directory"
[ -f "$WORLD" ] || fail "missing $WORLD"
[ -d "$MODELS" ] || fail "missing models directory"
[ -f "$MODELS/garage/model.config" ] || fail "missing garage model.config"
[ -f "$MODELS/garage/model.sdf" ] || fail "missing garage model.sdf"
[ -f "$MODELS/garage/meshes/garage.dae" ] || fail "missing garage mesh"
grep -q "<name>garage</name>" "$WORLD" || fail "world include does not name garage model"
grep -q "<pose>-23.817 -46.018 0.073 0 0 0</pose>" "$WORLD" || fail "world does not recenter garage mesh"
grep -q "model://garage/meshes/garage.dae" "$MODELS/garage/model.sdf" || fail "garage model.sdf does not reference model://garage/meshes/garage.dae"

python3 - <<'PY'
from pathlib import Path
import re
import sys

world = Path("src/virtual_env/garage_v1/worlds/garage_v1.world")
models = Path("src/virtual_env/garage_v1/models")
system = Path("/usr/share/gazebo-11/models")
uris = re.findall(r"<uri>\s*model://([^/<\s]+)", world.read_text())
uris += re.findall(r"<uri>\s*model://([^/<\s]+)", Path("src/virtual_env/garage_v1/models/garage/model.sdf").read_text())
missing = []
for name in sorted(set(uris)):
    if not (models / name).exists() and not (system / name).exists():
        missing.append(name)
if missing:
    raise SystemExit("FAIL: unresolved model:// references: " + ", ".join(missing))
print("Resolved model:// references:", ", ".join(sorted(set(uris))))
PY

[ -f "$INSTALL_BASE/worlds/garage_v1.world" ] || fail "missing installed garage_v1.world; run colcon build first"
[ -d "$INSTALL_BASE/models/garage" ] || fail "missing installed garage model; run colcon build first"

echo "PASS: garage_v1 assets complete"
