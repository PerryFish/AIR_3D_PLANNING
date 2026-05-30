#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

BASE="src/virtual_env/garage_v1/rviz"
fail() { echo "FAIL: $*"; exit 1; }

find "$BASE/tare_v1_original_rviz" -path "*vehicle_simulator.rviz" | grep -q . || fail "vehicle_simulator.rviz missing"
find "$BASE/tare_v1_original_rviz" -path "*tare_planner_ground.rviz" | grep -q . || fail "tare_planner_ground.rviz missing"
[ -f "$BASE/extracted_topic_summary/rviz_topic_summary.md" ] || fail "rviz_topic_summary.md missing"
[ -f "$BASE/manifest.yaml" ] || fail "manifest.yaml missing"
grep -q "/registered_scan" "$BASE/manifest.yaml" || fail "manifest missing /registered_scan"
grep -q "/overall_map" "$BASE/manifest.yaml" || fail "manifest missing /overall_map"
grep -q "garage/preview/pointcloud.ply" "$BASE/manifest.yaml" || fail "manifest missing original pointcloud path"

echo "PASS: TARE RViz reference bundle"
