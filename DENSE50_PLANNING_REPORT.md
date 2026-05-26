# Dense50 Planning Report

## Why 50% Occupancy Is Difficult

A 50% occupied 3D voxel map removes roughly half of the search space before inflation. After UAV safety inflation, narrow passages can close and A* may need to expand many nodes before proving that a path exists or does not exist.

## Current Approach

- `random_occupancy_3d` creates reproducible dense voxel fields with `random_seed`.
- `ensure_connectivity` keeps start, goal, and a 3D corridor free so benchmark maps remain meaningful.
- `weighted_astar_3d` uses a configurable heuristic weight to reduce search effort.
- `max_planning_time` and `max_expanded_nodes` prevent planner hangs.
- `/air/planning_metrics` publishes machine-readable planning metrics.
- `scripts/run_dense50_benchmark.sh` runs repeatable multi-seed tests and generates CSV plus Markdown reports.

## Validation Method

Run the dense50 demo with:

```bash
./scripts/launch_dense50_demo.sh
```

Run the benchmark with:

```bash
./scripts/run_dense50_benchmark.sh
```

The benchmark records planning success, planning time, expanded nodes, path length, z range, and whether the UAV reached the goal.

## Current Limits

- The occupancy map is synthetic and static.
- Connectivity is approximated on the voxel grid and does not model full UAV dynamics.
- Weighted A* is still grid-based and can be slow for larger maps or finer resolution.
- The simulator is kinematic, not a real PX4 or dynamics simulator.

## Future Upgrades

- Bidirectional A*
- 3D Jump Point Search
- RRT* fallback
- Kinodynamic A*
- ESDF-based planning
- Hierarchical coarse-to-fine planning
