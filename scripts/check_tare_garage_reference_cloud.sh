#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PCD="src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.pcd"
XYZ="src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.xyz"
JSON="src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_bounds.json"
ORIG="src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_original.ply"

fail() { echo "FAIL: $*"; exit 1; }

[ -f "$PCD" ] || fail "missing downsampled PCD"
[ -f "$XYZ" ] || fail "missing downsampled XYZ"
[ -f "$JSON" ] || fail "missing bounds json"

python3 - <<'PY'
from pathlib import Path
import json
data = json.loads(Path("src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_bounds.json").read_text())
count = int(data["point_count"])
if count < 5000:
    raise SystemExit(f"FAIL: point_count too low: {count}")
if count > 200000:
    raise SystemExit(f"FAIL: point_count unexpectedly high: {count}")
b = data["bounds"]
if b["x_max"] - b["x_min"] < 20 or b["y_max"] - b["y_min"] < 20 or b["z_max"] - b["z_min"] < 2:
    raise SystemExit(f"FAIL: bounds too small: {b}")
print("PASS: TARE garage reference cloud")
print(f"point_count={count}")
print(f"bounds=x[{b['x_min']:.3f},{b['x_max']:.3f}] y[{b['y_min']:.3f},{b['y_max']:.3f}] z[{b['z_min']:.3f},{b['z_max']:.3f}]")
PY

if git check-ignore -q "$ORIG"; then
  echo "original_pointcloud_gitignored=yes"
else
  fail "original 53MB pointcloud is not gitignored"
fi
