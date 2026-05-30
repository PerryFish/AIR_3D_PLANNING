#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [ ! -f results/garage_v1/metrics.csv ]; then
  scripts/run_garage_v1_visual_exploration_smoke_test.sh >/tmp/garage_v1_frontier_run.log
fi

python3 - <<'PY'
import csv
from pathlib import Path

rows = list(csv.DictReader(Path("results/garage_v1/metrics.csv").open()))
frontier_goals = max(int(r.get("frontier_goals", 0)) for r in rows)
failed_goals = max(int(r.get("failed_goals", 0)) for r in rows)
stuck_events = max(int(r.get("stuck_events", 0)) for r in rows)
backtrack_events = max(int(r.get("backtrack_events", 0)) for r in rows)
goal_changes = sum(1 for a, b in zip(rows, rows[1:]) if (a["goal_x"], a["goal_y"], a["goal_z"]) != (b["goal_x"], b["goal_y"], b["goal_z"]))
ratio = frontier_goals / max(1, goal_changes)
print(f"frontier_goals={frontier_goals}")
print(f"goal_changes={goal_changes}")
print(f"frontier_goal_ratio={ratio:.3f}")
print(f"failed_goals={failed_goals}")
print(f"stuck_events={stuck_events}")
print(f"backtrack_events={backtrack_events}")
if goal_changes < 2:
    raise SystemExit("FAIL: too few goal changes")
if ratio < 0.05:
    print("WARN: low frontier goal ratio; planner still relies heavily on sweep/backtracking fallback")
else:
    print("PASS: garage_v1 frontier goal ratio check")
PY
