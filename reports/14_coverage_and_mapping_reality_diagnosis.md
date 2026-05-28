# Coverage And Mapping Reality Diagnosis

Current coverage is real sensor-driven mapping coverage: **partial/no for the old system; partial baseline after this change**.

## Findings Before Sensor Mapping Baseline

1. Current legacy coverage was calculated in `synthetic_mapping_node.py` as `len(observed) / total_voxels`.
2. The node used the dense50 global obstacle model from `make_dense50_ground_footprint()` and `ground_to_occupied_voxels()`.
3. It did not subscribe to a real LiDAR point cloud or camera/depth topic.
4. It marked voxels observed by radius around UAV pose, not by LiDAR/camera ray casting.
5. It could mark space as covered even when no explicit sensor scan point hit or ray traversed that voxel.
6. RViz coverage around `0.953` was therefore a synthetic internal metric, not true perception coverage.
7. The legacy map had observed/free/occupied/unknown counts, but those states were produced from simulated pose radius plus known dense50 truth.
8. There was no point cloud accumulation.
9. There was no camera scan fusion.
10. The system could not export an online-built 3D map suitable as a mapping artifact.

## Interpretation

The old coverage was useful as a regression metric for motion-coupled exploration, but it was not enough to claim unknown-environment LiDAR/camera mapping.

## Required Correction

The new baseline separates:

- `synthetic_coverage`: compatibility metric for old tests.
- `observed_coverage`: online observed map coverage from local simulated LiDAR and camera frustum ray casting.

Ground truth is only used to simulate local sensor hits and to define the dense50 environment. It is not dumped directly into the final map as explored space.
