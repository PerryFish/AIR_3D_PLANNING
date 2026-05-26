# Dense50 Benchmark Report

- Test date: 2026-05-26T23:10:57
- Configuration: random_occupancy_3d, occupancy_ratio=0.50, weighted_astar_3d
- Seeds: 10
- PLAN_SUCCESS count: 10
- PLAN_SUCCESS rate: 100.0%
- Average planning time: 0.005 s
- Average expanded nodes: 32.0
- Average path length: 23.740 m
- UAV reached goal count: 10
- UAV reached goal rate: 100.0%
- Failed seeds: none

## Suggestions

- Tune inflation_radius and heuristic_weight for tighter passages.
- Add bidirectional A* or hierarchical planning if dense maps become larger.
