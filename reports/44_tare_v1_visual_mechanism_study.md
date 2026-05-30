# TARE V1 Visual Mechanism Study

This report supersedes the earlier surface-cloud interpretation.

TARE_V1 garage RViz is not based on publishing a full `garage.dae` face sample as the main live structure. The live scan layer is `/registered_scan`, published by `vehicleSimulator` after it receives Gazebo Velodyne `/velodyne_points`. The global reference layer `/overall_map` is published by `visualizationTools` from `vehicle_simulator/mesh/garage/preview/pointcloud.ply`.

The important distinction for AIR is that a dense white surface cloud is visually wrong as the default structure layer. TARE's readable result comes from small point size, low alpha, live local scan decay, terrain filtering, and accumulated observed scan points. AIR now uses an edge/outline cloud for `/overall_map` and local edge filtering for `/registered_scan`.

Remaining gap: AIR still replays an edge cloud and simulated local visibility. It is not a complete Gazebo VLP-16, LOAM, FAST-LIVO, or terrain analysis stack.

Start alignment note: TARE's garage vehicle start is `(0,0,0.75)` in `map`. AIR now maps that reference through the garage edge-cloud transform to `(-23.817,-46.018,1.6)`, yaw `0.0`, so the TARE-like visual replay starts from the entrance-side region instead of the old AIR central-loop pose.
