# Garage V1 RViz Gazebo Alignment Diagnosis

## Gazebo Source

Gazebo loads:

```text
src/virtual_env/garage_v1/worlds/garage_v1.world
```

The world includes the original TARE garage model:

```xml
<include>
  <uri>model://garage</uri>
  <name>garage</name>
  <pose>-23.817 -46.018 0.073 0 0 0</pose>
</include>
```

The model uses:

```text
src/virtual_env/garage_v1/models/garage/model.sdf
src/virtual_env/garage_v1/models/garage/meshes/garage.dae
```

No mesh scale is applied in `model.sdf`. The AIR transform is therefore the world include translation above.

## Bounds

The TARE preview cloud and DAE mesh are in the same source coordinate frame:

```text
TARE preview PLY sampled bounds:
  min [-40.060, -25.357, -0.111]
  max [ 87.689, 117.397, 20.899]

DAE mesh bounds:
  min [-40.183, -25.482, -0.073]
  max [ 87.817, 117.518, 21.018]
```

After applying the Gazebo include pose, the expected AIR/Gazebo bounds are approximately:

```text
x [-64.000, 64.000]
y [-71.500, 71.500]
z [0.000, 21.091]
```

## RViz Mismatch Root Cause

Before this pass, `/exploration/garage_structure_cloud` was generated from a hand-written wall proxy. That proxy was useful for a coarse visual outline, but it was not sampled from the actual `garage.dae` or TARE preview map. As a result, RViz showed an abstract wall layout that did not match the Gazebo garage model.

## TARE Reference

TARE original garage resources include:

```text
/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage/preview/pointcloud.ply
```

The TARE RViz and planner use topics such as:

```text
/registered_scan
/terrain_map
/terrain_map_ext
/overall_map
/planner_cloud
```

The grey/white structure view in TARE is therefore driven by point cloud map and registered scan topics, not by a manually drawn proxy.

## Fix Direction

AIR now generates an aligned, downsampled structure cloud from the TARE garage preview PLY and applies the same world translation as Gazebo. The wall proxy remains available only as a Gazebo visual aid and fallback source.
