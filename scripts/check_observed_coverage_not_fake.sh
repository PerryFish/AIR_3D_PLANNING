#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/metrics_dense50.csv").open()))
if len(rows) < 20:
    raise SystemExit("FAIL: metrics file too short")
observed = [float(r["observed_coverage"]) for r in rows]
synthetic = [float(r["synthetic_coverage"]) for r in rows]
if max(observed[:20]) > 0.3:
    raise SystemExit(f"FAIL: first 5s observed coverage too high: {max(observed[:20]):.6f}")
if "observed_coverage" not in rows[0] or "synthetic_coverage" not in rows[0]:
    raise SystemExit("FAIL: observed/synthetic coverage columns are not separated")
pose_changes = sum(1 for r in rows if r["pose_changed"] == "True")
new_obs = sum(1 for r in rows if int(r["newly_observed_voxels"]) > 0)
zero_obs = sum(1 for r in rows if int(r["newly_observed_voxels"]) == 0)
if pose_changes < 10:
    raise SystemExit(f"FAIL: too few pose changes: {pose_changes}")
if new_obs < 10:
    raise SystemExit(f"FAIL: coverage not coupled to local observations: {new_obs}")
if zero_obs < 3:
    raise SystemExit("FAIL: no low/no-progress rows; mapping may be blindly growing")
deltas = [round(b - a, 6) for a, b in zip(observed, observed[1:]) if b - a > 1e-6]
if len(set(deltas[:20])) <= 2 and len(deltas) >= 20:
    raise SystemExit(f"FAIL: observed coverage deltas look fixed: {deltas[:20]}")
if rows[-1]["mapping_source"] != "sensor_driven_local_simulated_lidar_camera":
    raise SystemExit(f"FAIL: unexpected mapping source: {rows[-1]['mapping_source']}")
Path("reports/17_observed_coverage_not_fake.md").write_text(
    "\n".join([
        "# Observed Coverage Not Fake Check",
        "",
        "- result: PASS",
        f"- initial_observed_coverage: {observed[0]:.6f}",
        f"- final_observed_coverage: {observed[-1]:.6f}",
        f"- final_synthetic_coverage: {synthetic[-1]:.6f}",
        f"- pose_changed_rows: {pose_changes}",
        f"- new_observation_rows: {new_obs}",
        f"- zero_observation_rows: {zero_obs}",
        "- observed_and_synthetic_columns_separated: True",
        "",
    ])
)
print("PASS: observed coverage not fake")
print(f"final_observed_coverage={observed[-1]:.6f}")
print(f"final_synthetic_coverage={synthetic[-1]:.6f}")
PY
