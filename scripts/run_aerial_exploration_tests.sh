#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
mkdir -p results logs/tests
rm -f results/metrics_dense50.csv results/test_summary.md logs/tests/dense50_launch.log logs/tests/wait_for_done.log

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

setsid ros2 launch aerial_exploration_planner test_aerial_exploration_dense50.launch.py > logs/tests/dense50_launch.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
    kill -INT "-$LAUNCH_PID" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "$LAUNCH_PID" >/dev/null 2>&1; then
      kill -TERM "-$LAUNCH_PID" >/dev/null 2>&1 || true
    fi
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

python3 scripts/wait_for_exploration_done.py --timeout 150 --coverage-threshold 0.93 > logs/tests/wait_for_done.log 2>&1
cat logs/tests/wait_for_done.log
