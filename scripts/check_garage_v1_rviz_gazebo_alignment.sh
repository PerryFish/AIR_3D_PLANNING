#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 - <<'PY'
from pathlib import Path
import re
import xml.etree.ElementTree as ET

world = Path("src/virtual_env/garage_v1/worlds/garage_v1.world")
dae = Path("src/virtual_env/garage_v1/models/garage/meshes/garage.dae")
pcd = Path("src/virtual_env/garage_v1/maps/garage_structure_from_tare.pcd")

for path in (world, dae, pcd):
    if not path.exists():
        raise SystemExit(f"FAIL: missing {path}")

root = ET.parse(world).getroot()
pose = None
for include in root.findall(".//include"):
    if include.findtext("uri", "").strip() == "model://garage" or include.findtext("name", "").strip() == "garage":
        vals = include.findtext("pose", "0 0 0 0 0 0").split()
        pose = tuple(float(vals[i]) for i in range(3))
        break
if pose is None:
    raise SystemExit("FAIL: garage include pose not found")

text = dae.read_text(errors="ignore")
match = re.search(r'<float_array id="Garage-mesh-positions-array" count="\d+">(.*?)</float_array>', text, re.S)
if not match:
    raise SystemExit("FAIL: garage DAE vertex array missing")
values = [float(v) for v in match.group(1).split()]
vertices = list(zip(values[0::3], values[1::3], values[2::3]))
mesh_min = tuple(min(p[i] for p in vertices) + pose[i] for i in range(3))
mesh_max = tuple(max(p[i] for p in vertices) + pose[i] for i in range(3))

points = []
data = False
with pcd.open() as f:
    for line in f:
        stripped = line.strip()
        if not data:
            if stripped.lower().startswith("data"):
                data = True
            continue
        if stripped:
            parts = stripped.split()
            points.append(tuple(float(parts[i]) for i in range(3)))
cloud_min = tuple(min(p[i] for p in points) for i in range(3))
cloud_max = tuple(max(p[i] for p in points) for i in range(3))

center_err = [
    abs(((mesh_min[i] + mesh_max[i]) * 0.5) - ((cloud_min[i] + cloud_max[i]) * 0.5))
    for i in range(3)
]
extent_err = [
    abs((mesh_max[i] - mesh_min[i]) - (cloud_max[i] - cloud_min[i]))
    for i in range(3)
]
if max(center_err) > 0.35:
    raise SystemExit(f"FAIL: structure cloud center mismatch: {center_err}")
if max(extent_err) > 0.65:
    raise SystemExit(f"FAIL: structure cloud extent mismatch: {extent_err}")

print("PASS: garage_v1 RViz/Gazebo alignment")
print(f"garage_model_bounds: x=[{mesh_min[0]:.3f},{mesh_max[0]:.3f}] y=[{mesh_min[1]:.3f},{mesh_max[1]:.3f}] z=[{mesh_min[2]:.3f},{mesh_max[2]:.3f}]")
print(f"structure_cloud_bounds: x=[{cloud_min[0]:.3f},{cloud_max[0]:.3f}] y=[{cloud_min[1]:.3f},{cloud_max[1]:.3f}] z=[{cloud_min[2]:.3f},{cloud_max[2]:.3f}]")
print(f"alignment_error_center={center_err}")
print(f"alignment_error_extent={extent_err}")
PY
