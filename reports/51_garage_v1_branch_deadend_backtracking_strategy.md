# Garage V1 Branch Dead-End Backtracking Strategy

## Planner Changes

- Frontier candidates are grouped into angular clusters around the current pose.
- Cluster scoring prefers information gain, unvisited clusters, and movement away from the start while retaining distance cost.
- Branch points are recorded when frontiers occupy multiple directions or frontier count is high.
- Stuck handling now records dead-end count and sets `goal_source=backtrack_to_branch` when returning to a branch point.
- Returning to a branch goal increments `returned_to_branch_count`.
- Lawnmower fallback is now centered on `grid.origin_x/y`, so fallback goals stay near the garage entrance-aligned local map instead of the old global origin.

## Metrics

The planner state and CSV now expose:

```text
goal_source
frontier_cluster_id
branch_point_count
dead_end_count
backtrack_count
returned_to_branch_count
current_region
start_pose_x/y/z/yaw
distance_from_start
```

## Limitations

This remains a lightweight AIR frontier/backtracking baseline. It is not full TARE graph exploration, kinodynamic planning, or FAST-LIVO/SLAM. The branch and dead-end signals are pragmatic heuristics driven by the current simulated mapping state.
