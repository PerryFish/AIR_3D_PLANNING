#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

mkdir -p results/maps reports

for file in \
  results/maps/observed_occupied_cloud.pcd \
  results/maps/observed_free_cloud.pcd \
  results/maps/observed_all_cloud.pcd \
  results/maps/frontier_cloud.pcd \
  results/maps/trajectory.csv \
  results/maps/map_metrics.csv; do
  test -s "$file" || { echo "FAIL: missing or empty $file"; exit 1; }
done

python3 - <<'PY'
from pathlib import Path
files = [
    "observed_occupied_cloud.pcd",
    "observed_free_cloud.pcd",
    "observed_all_cloud.pcd",
    "frontier_cloud.pcd",
    "trajectory.csv",
    "map_metrics.csv",
]
out = Path("results/maps")
counts = {f: len((out / f).read_text().splitlines()) for f in files}
if counts["trajectory.csv"] < 5:
    raise SystemExit("FAIL: trajectory.csv too short")
Path("reports/18_map_export_check.md").write_text(
    "\n".join(["# Map Export Check", "", "- result: PASS"] + [f"- {name}_lines: {count}" for name, count in counts.items()] + [""])
)
print("PASS: map export")
for name, count in counts.items():
    print(f"{name}_lines={count}")
PY
