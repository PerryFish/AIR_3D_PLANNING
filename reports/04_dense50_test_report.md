# Dense50 Test Report

dense50 is defined as XY ground obstacle footprint occupancy ratio of about 50 percent.
It is not defined as 50 percent 3D occupied voxel ratio.

Expected validation:

- measured_ground_footprint_occupancy_ratio within 0.50 +/- 0.03
- final_coverage >= 0.93
- done=True

Observed validation:

- measured_ground_footprint_occupancy_ratio: 0.500000
- final_coverage: 0.945000
- done: True
- failed_goals: 0
- stuck_events: 0
- metrics CSV: results/metrics_dense50.csv
- summary: results/test_summary.md
