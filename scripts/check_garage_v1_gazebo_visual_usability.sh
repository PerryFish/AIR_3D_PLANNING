#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
PROXY="src/virtual_env/garage_v1/models/garage_wall_proxy/model.sdf"
WORLD="src/virtual_env/garage_v1/worlds/garage_v1.world"

fail() {
  echo "FAIL: $*"
  exit 1
}

[ -f "$PROXY" ] || fail "missing wall proxy model.sdf"
[ -f "$WORLD" ] || fail "missing garage_v1.world"
grep -q "<static>true</static>" "$PROXY" || fail "wall proxy is not static"
if grep -q "<collision" "$PROXY"; then
  fail "wall proxy contains collision; visual proxy should not interfere with mouse selection"
fi
grep -q "<cast_shadows>false</cast_shadows>" "$PROXY" || fail "wall proxy should disable cast shadows"
grep -q "<gui fullscreen='0'>" "$WORLD" || fail "world missing default GUI camera"
grep -q "garage_v1_overview" "$WORLD" || fail "world missing garage overview camera"
grep -q "<pose>-23.817 -46.018 0.073 0 0 0</pose>" "$WORLD" || fail "garage mesh is not recentered"

python3 - <<'PY'
from pathlib import Path
import re
text = Path("src/virtual_env/garage_v1/models/garage_wall_proxy/model.sdf").read_text()
sizes = []
for raw in re.findall(r"<size>([^<]+)</size>", text):
    sizes.append(tuple(float(x) for x in raw.split()))
if not sizes:
    raise SystemExit("FAIL: no proxy wall sizes")
max_dim = max(max(s) for s in sizes)
min_thickness = min(min(s[0], s[1]) for s in sizes)
if max_dim > 40.0:
    raise SystemExit(f"FAIL: proxy appears oversized: max_dim={max_dim}")
if min_thickness > 0.12:
    raise SystemExit(f"FAIL: proxy wall too thick: min_thickness={min_thickness}")
print("PASS: garage_v1 Gazebo visual usability")
print("wall_proxy_collision=disabled")
print(f"max_proxy_dimension={max_dim:.2f}")
print(f"min_wall_thickness={min_thickness:.2f}")
PY
