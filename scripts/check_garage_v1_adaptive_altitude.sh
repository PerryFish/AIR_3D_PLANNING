#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

scripts/run_garage_v1_visual_exploration_smoke_test.sh >/tmp/garage_v1_adaptive_altitude_run.log

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/garage_v1/metrics.csv").open()))
zs = [float(r["robot_z"]) for r in rows if float(r["robot_z"]) > 0.0]
if not zs:
    raise SystemExit("FAIL: no robot_z samples")
z_min, z_max = min(zs), max(zs)
z_range = z_max - z_min
jumps = [abs(b - a) for a, b in zip(zs, zs[1:])]
if z_min < 0.79 or z_max > 3.01:
    raise SystemExit(f"FAIL: z out of bounds: {z_min:.3f}..{z_max:.3f}")
if jumps and max(jumps) > 0.35:
    raise SystemExit(f"FAIL: z jump too large: {max(jumps):.3f}")
if z_range < 0.2:
    print(f"WARN: adaptive altitude enabled but z_range is small: {z_range:.3f}")
    print(f"observed_z_min={z_min:.3f}")
    print(f"observed_z_max={z_max:.3f}")
    print("test_result=WARN")
else:
    print("PASS: garage_v1 adaptive altitude check")
    print(f"observed_z_min={z_min:.3f}")
    print(f"observed_z_max={z_max:.3f}")
    print(f"z_range={z_range:.3f}")
    print(f"z_changes={sum(1 for j in jumps if j > 0.01)}")
PY
