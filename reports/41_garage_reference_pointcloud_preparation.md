# Garage Reference Pointcloud Preparation

## Source

Original TARE preview point cloud:

```text
/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply
```

Size: `53M`.

## Outputs

Prepared by:

```bash
python3 scripts/prepare_tare_garage_reference_cloud.py
```

Generated:

```text
src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_original.ply
src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.pcd
src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_downsampled.xyz
src/virtual_env/garage_v1/maps/tare_reference/garage_preview_pointcloud_bounds.json
```

The original 53 MB file is copied for local reproducibility but ignored by git.

## Downsampled Cloud

- point count: `121525`
- frame: `map`
- transform: `x=-23.817`, `y=-46.018`, `z=0.073`, scale `1.0`
- bounds: `x[-63.872,63.877] y[-71.379,71.379] z[-0.037,20.973]`

## Alignment

Gazebo garage mesh bounds after world pose:

```text
x[-64.000,64.000] y[-71.500,71.500] z[0.000,21.091]
```

The replay cloud center error is below `0.078 m`, so `/overall_map` is aligned with the Gazebo garage model.
