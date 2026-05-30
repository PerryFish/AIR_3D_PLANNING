# Garage Edge Cloud Generation

## Source

Input:

```text
/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply
```

Output:

```text
src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.pcd
src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.xyz
src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud_bounds.json
```

## Method

`scripts/generate_garage_edge_cloud_from_tare_pointcloud.py` parses the binary PLY, applies the same `model://garage` pose as `garage_v1.world`, filters to the useful garage height band, then uses z-sliced XY occupancy boundary extraction. Occupied cells adjacent to empty cells are kept; interior surface cells are dropped.

This keeps wall edges, corridor outlines, and obstacle boundaries while removing most filled wall/roof/floor surfaces.

## Metrics

Latest generated values:

- surface points: 4,617,947
- edge points: 41,110
- edge density ratio: 0.008902
- XY fill ratio in quality check: 0.103
- bounds: x `[-63.856, 63.849]`, y `[-71.337, 71.330]`, z `[-0.038, 5.000]`

`scripts/check_garage_edge_cloud_quality.sh` validates file existence, point count, density ratio `< 0.5`, bounds, and non-solid XY occupancy.
