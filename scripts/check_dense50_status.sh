#!/usr/bin/env bash
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
source install/setup.bash

pass=true

echo "== /air/world_status =="
world_status="$(timeout 8s ros2 topic echo /air/world_status --once --field data 2>/dev/null || true)"
echo "${world_status:-MISSING}"
[[ "$world_status" == *"scenario_type="* ]] || pass=false

echo "== /air/planner_status =="
planner_status="$(timeout 12s ros2 topic echo /air/planner_status --once --field data 2>/dev/null || true)"
echo "${planner_status:-MISSING}"
[[ "$planner_status" == *"PLAN_SUCCESS"* ]] || pass=false

echo "== /air/planning_metrics =="
metrics="$(timeout 12s ros2 topic echo /air/planning_metrics --once --field data 2>/dev/null || true)"
echo "${metrics:-MISSING}"
[[ "$metrics" == *"success=true"* ]] || pass=false

echo "== /air/state_estimation =="
timeout 8s ros2 topic echo /air/state_estimation --once --field pose.pose.position || pass=false

echo "== /air/uav_trail =="
timeout 8s ros2 topic echo /air/uav_trail --once --field poses || pass=false

echo "== /air/state_estimation hz =="
hz_output="$(timeout 8s ros2 topic hz /air/state_estimation --window 5 2>/dev/null || true)"
echo "${hz_output:-MISSING}"
[[ "$hz_output" == *"average rate"* ]] || pass=false

if [[ "$pass" == true ]]; then
  echo "PASS: dense50 status topics are available"
else
  echo "FAIL: dense50 status check failed"
  exit 1
fi
