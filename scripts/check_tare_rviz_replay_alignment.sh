#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 - <<'PY'
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

world = Path("src/virtual_env/garage_v1/worlds/garage_v1.world")
dae = Path("src/virtual_env/garage_v1/models/garage/meshes/garage.dae")
bounds_json = Path("src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_bounds.json")
for path in (world, dae, bounds_json):
    if not path.exists():
        raise SystemExit(f"FAIL: missing {path}")

root = ET.parse(world).getroot()
pose = (0.0, 0.0, 0.0)
for include in root.findall(".//include"):
    if include.findtext("uri", "").strip() == "model://garage" or include.findtext("name", "").strip() == "garage":
        vals = include.findtext("pose", "0 0 0 0 0 0").split()
        pose = tuple(float(vals[i]) for i in range(3))
        break

text = dae.read_text(errors="ignore")
match = re.search(r'<float_array id="Garage-mesh-positions-array" count="\d+">(.*?)</float_array>', text, re.S)
if not match:
    raise SystemExit("FAIL: DAE garage vertex array not found")
vals = [float(v) for v in match.group(1).split()]
pts = list(zip(vals[0::3], vals[1::3], vals[2::3]))
mesh_min = [min(p[i] for p in pts) + pose[i] for i in range(3)]
mesh_max = [max(p[i] for p in pts) + pose[i] for i in range(3)]
data = json.loads(bounds_json.read_text())
b = data["bounds"]
cloud_min = [b["x_min"], b["y_min"], b["z_min"]]
cloud_max = [b["x_max"], b["y_max"], b["z_max"]]
center_err = [abs((mesh_min[i] + mesh_max[i]) * 0.5 - (cloud_min[i] + cloud_max[i]) * 0.5) for i in range(3)]
extent_err = [abs((mesh_max[i] - mesh_min[i]) - (cloud_max[i] - cloud_min[i])) for i in range(3)]
if max(center_err) > 0.5:
    raise SystemExit(f"FAIL: reference cloud center mismatch: {center_err}")
if max(extent_err) > 1.0:
    raise SystemExit(f"FAIL: reference cloud extent mismatch: {extent_err}")
print("PASS: TARE RViz replay alignment")
print(f"garage_model_bounds=x[{mesh_min[0]:.3f},{mesh_max[0]:.3f}] y[{mesh_min[1]:.3f},{mesh_max[1]:.3f}] z[{mesh_min[2]:.3f},{mesh_max[2]:.3f}]")
print(f"overall_map_bounds=x[{cloud_min[0]:.3f},{cloud_max[0]:.3f}] y[{cloud_min[1]:.3f},{cloud_max[1]:.3f}] z[{cloud_min[2]:.3f},{cloud_max[2]:.3f}]")
print(f"alignment_error_center={center_err}")
print(f"alignment_error_extent={extent_err}")
PY
