#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 - <<'PY'
import csv
from pathlib import Path

csv_path = Path("results/metrics_dense50.csv")
if not csv_path.exists():
    raise SystemExit("FAIL: missing results/metrics_dense50.csv")
rows = list(csv.DictReader(csv_path.open()))
if not rows:
    raise SystemExit("FAIL: metrics CSV has no data rows")

first_done = None
for row in rows:
    coverage = float(row["coverage"])
    done = row["done"] == "True"
    if done and coverage < 0.93:
        raise SystemExit(f"FAIL: fake done detected at coverage={coverage:.6f}")
    explored = int(row["explored_voxels"])
    free = int(row["free_voxels"])
    occupied = int(row["occupied_voxels"])
    unknown = int(row["unknown_voxels"])
    total = explored + unknown
    if explored != free + occupied:
        raise SystemExit("FAIL: explored_voxels != free_voxels + occupied_voxels")
    expected = explored / total if total else 0.0
    if abs(expected - coverage) > 0.002:
        raise SystemExit(f"FAIL: coverage is not derived from explored/total voxels: {coverage:.6f} vs {expected:.6f}")
    if done and first_done is None:
        first_done = row

if first_done is None:
    raise SystemExit("FAIL: no done=True row found")

final = rows[-1]
coverages = [float(r["coverage"]) for r in rows]
deltas = [round(b - a, 6) for a, b in zip(coverages, coverages[1:]) if b - a > 1e-6]
if len(set(deltas[:12])) <= 2 and len(deltas) >= 12:
    raise SystemExit(f"FAIL: coverage deltas look fixed: {deltas[:12]}")
if float(rows[0]["coverage"]) > 0.2:
    raise SystemExit(f"FAIL: initial coverage is too high: {rows[0]['coverage']}")
pose_changes = sum(1 for r in rows if r.get("pose_changed") == "True")
goal_changes = sum(1 for r in rows if r.get("goal_changed") == "True")
observed_positive = sum(1 for r in rows if int(r.get("newly_observed_voxels", "0")) > 0)
if pose_changes < 5:
    raise SystemExit(f"FAIL: too few pose_changed rows: {pose_changes}")
if goal_changes < 2:
    raise SystemExit(f"FAIL: too few goal_changed rows: {goal_changes}")
if observed_positive < 5:
    raise SystemExit(f"FAIL: newly_observed_voxels rarely increases: {observed_positive}")
ground_ratio = float(final["ground_footprint_occupancy_ratio"])
if abs(ground_ratio - 0.50) > 0.03:
    raise SystemExit(f"FAIL: dense50 ground footprint ratio outside tolerance: {ground_ratio:.6f}")
if float(final["coverage"]) < 0.93 or final["done"] != "True":
    raise SystemExit("FAIL: final row does not satisfy coverage >= 0.93 and done=True")

print("PASS: anti-fake coverage validation")
print(f"final_coverage={float(final['coverage']):.6f}")
print(f"done={final['done']}")
print(f"failed_goals={final['failed_goals']}")
print(f"stuck_events={final['stuck_events']}")
print(f"pose_changed_rows={pose_changes}")
print(f"goal_changed_rows={goal_changes}")
print(f"new_observation_rows={observed_positive}")
PY
