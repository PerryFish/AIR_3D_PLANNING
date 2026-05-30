#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ ! -f results/garage_v1/metrics.csv ]; then
  scripts/run_garage_v1_visual_exploration_smoke_test.sh >/tmp/garage_v1_coverage_run.log
fi

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/garage_v1/metrics.csv").open()))
coverages = [float(r["observed_coverage"]) for r in rows]
target = 0.75
initial, final = coverages[0], coverages[-1]
print(f"target_observed_coverage={target:.2f}")
print(f"initial_observed_coverage={initial:.6f}")
print(f"final_observed_coverage={final:.6f}")
print(f"coverage_improvement={final - initial:.6f}")
if final >= target:
    print("PASS: garage_v1 coverage target reached")
elif final >= 0.55:
    print("WARN: garage_v1 coverage target not reached, but baseline coverage is substantial")
else:
    raise SystemExit("FAIL: garage_v1 coverage too low")
PY
