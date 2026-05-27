# Coverage Metric Validation

coverage = explored_voxels / total_voxels_in_planning_boundary

explored_voxels = free_voxels + occupied_voxels

done=True is published only after coverage >= 0.93.

Observed final metrics:

- explored_voxels: 2268
- free_voxels: 1984
- occupied_voxels: 284
- unknown_voxels: 132
- total_voxels: 2400
- final_coverage: 0.945000
- done: True
