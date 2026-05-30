# AIR Garage Start Pose Mismatch Diagnosis

## Before

AIR garage configs used:

```text
uav.initial_x=-13.5
uav.initial_y=-13.5
uav.initial_z=1.6
```

That pose was read by `simple_uav_follower_node.py` and became both `/odom` and `/state_estimation`. The planner and sensor mapping consumed those topics, so the full exploration loop inherited the same misplaced start. The Gazebo UAV visualizer followed `/state_estimation`, so the Gazebo visual model also converged to that pose after spawn.

## Mismatch

The edge cloud and Gazebo garage are aligned to the AIR garage transform derived from the TARE preview cloud, but the UAV start was a separate hardcoded value. The hardcoded pose was not the TARE garage vehicle start and made RViz look like the UAV began in or near the central loop instead of at the entrance-side start region.

## Required Fix

The fix is a single garage start pose source:

```text
garage_v1.start_pose.x=-23.817
garage_v1.start_pose.y=-46.018
garage_v1.start_pose.z=1.6
garage_v1.start_pose.yaw=0.0
```

The same values must feed `uav.initial_*`, planner start metrics, grid origin, Gazebo spawn parameters, `/state_estimation`, `/odom`, TF, and RViz start marker.
