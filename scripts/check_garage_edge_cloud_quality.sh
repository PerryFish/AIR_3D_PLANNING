#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

PCD="src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.pcd"
XYZ="src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.xyz"
JSON="src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud_bounds.json"

if [ ! -f "$PCD" ] || [ ! -f "$XYZ" ] || [ ! -f "$JSON" ]; then
  python3 scripts/generate_garage_edge_cloud_from_tare_pointcloud.py >/tmp/garage_edge_generate.json
fi

python3 - <<'PY'
import json
from pathlib import Path

meta = json.loads(Path("src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud_bounds.json").read_text())
edge = int(meta["edge_point_count"])
surface = int(meta["surface_point_count"])
ratio = float(meta["edge_density_ratio"])
b = meta["edge_bounds"]
if not (5000 <= edge <= 80000):
    raise SystemExit(f"FAIL: edge point count out of range: {edge}")
if ratio >= 0.5:
    raise SystemExit(f"FAIL: edge density ratio too high: {ratio:.3f}")
if b["x_max"] - b["x_min"] < 20 or b["y_max"] - b["y_min"] < 20 or b["z_max"] - b["z_min"] < 2:
    raise SystemExit(f"FAIL: edge cloud bounds too small: {b}")

cells = set()
for line in Path("src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.xyz").read_text().splitlines():
    if not line or line.startswith("#"):
        continue
    x, y, _ = map(float, line.split()[:3])
    cells.add((round(x / 0.5), round(y / 0.5)))
xs = [c[0] for c in cells]
ys = [c[1] for c in cells]
area = (max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1)
fill = len(cells) / max(1, area)
if fill > 0.65:
    raise SystemExit(f"FAIL: XY occupancy looks too filled: {fill:.3f}")
print(f"PASS: garage edge cloud quality edge_points={edge} surface_points={surface} edge_density_ratio={ratio:.4f} xy_fill={fill:.3f} bounds={b}")
PY
