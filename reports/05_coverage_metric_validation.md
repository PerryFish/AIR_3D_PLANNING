# Coverage Metric Validation

coverage = explored_voxels / total_voxels_in_planning_boundary

explored_voxels = free_voxels + occupied_voxels

done=True is published only after coverage >= 0.93.

Observed final metrics:

- explored_voxels: 2232
- free_voxels: 1669
- occupied_voxels: 563
- unknown_voxels: 168
- total_voxels: 2400
- final_coverage: 0.930000
- done: True
- coverage_source: observed voxels updated from simulated robot odom and sensor range
