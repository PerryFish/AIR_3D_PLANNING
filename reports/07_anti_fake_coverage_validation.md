# Anti-Fake Coverage Validation

The anti-fake coverage test verifies:

- no done=True row exists before coverage >= 0.93
- coverage matches explored_voxels / total_voxels_in_planning_boundary
- explored_voxels equals free_voxels + occupied_voxels
- dense50 ground footprint occupancy ratio remains within tolerance

Observed result:

- anti-fake coverage test: PASS
- final_coverage: 0.945000
- done: True
- failed_goals: 0
- stuck_events: 0
