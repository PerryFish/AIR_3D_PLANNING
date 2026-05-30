# TARE V1 Garage Start Pose And Coordinate Study

## Findings

- TARE launch: `/home/nuaa/ZHY/TARE_V1/src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/system_garage.launch`.
- TARE vehicle defaults: `vehicleX=0.0`, `vehicleY=0.0`, `vehicleZ=0.0`, `vehicleYaw=0.0`, `vehicleHeight=0.75`.
- Effective TARE garage start: `(0.0, 0.0, 0.75)`, yaw `0.0`, frame `map`.
- TARE `garage.world` loads `model://garage` without a world pose offset.
- TARE boundary reference includes lower entrance-side vertices around `(-7,-4)` to `(24,-4)` and a larger garage frame up to `(54.5,84)`.
- TARE waypoints include `(10,28,0.8)` and final `(0,0,0.8)`, consistent with the vehicle starting near the lower entrance side rather than inside the central loop.

## AIR Mapping

AIR edge-cloud metadata records the garage transform used during edge-cloud generation:

```text
x=-23.817, y=-46.018, z=0.073, yaw=0.0
```

Applying this transform to the TARE start gives the recommended AIR start:

```text
(-23.817, -46.018, 1.6), yaw=0.0
```

AIR edge-map bounds are:

```text
x [-63.856, 63.849]
y [-71.337, 71.330]
z [-0.038, 5.000]
```

The recommended start is inside the edge-map bounds and outside the central 30% XY core. The old AIR start `(-13.5,-13.5,1.6)` was not TARE-equivalent and placed the UAV much closer to the central loop region.

## Recommendation

Do not move the garage model or edge cloud. Keep Gazebo and RViz map coordinates fixed, and unify UAV, `/state_estimation`, `/odom`, planner, mapping, Gazebo visualizer spawn, and start marker around the transformed TARE start pose.
