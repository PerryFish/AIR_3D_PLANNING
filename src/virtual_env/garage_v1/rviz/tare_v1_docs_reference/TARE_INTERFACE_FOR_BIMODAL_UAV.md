# TARE Interface For Bimodal UAV

## Aerial TARE Inputs

- Localization:
  - `/state_estimation_at_scan` (`nav_msgs/msg/Odometry`) for TARE.
  - `/state_estimation` (`nav_msgs/msg/Odometry`) for terrain/local planner/simulator.
- Point cloud:
  - `/registered_scan` (`sensor_msgs/msg/PointCloud2`, frame `map`).
  - `/sensor_scan` (`sensor_msgs/msg/PointCloud2`, frame `sensor_at_scan`).
- Terrain/map:
  - `/terrain_map` (`sensor_msgs/msg/PointCloud2`).
  - `/terrain_map_ext` (`sensor_msgs/msg/PointCloud2`).
- Optional control/status:
  - `/start_exploration` (`std_msgs/msg/Bool`).
  - `/joy` (`sensor_msgs/msg/Joy`).
  - `/reset_waypoint` (`std_msgs/msg/Empty`).
- Parameters:
  - `/home/nuaa/ZHY/TARE_V1/src/tare_planner/src/tare_planner/config/garage.yaml`
- Frame requirements:
  - Main planning frame is `map`.
  - Moving body/sensor frame is `sensor`.
  - Scan-aligned frame is `sensor_at_scan`.

## Aerial TARE Outputs

- Waypoint:
  - `/way_point` (`geometry_msgs/msg/PointStamped`, frame `map`, observed about 1 Hz).
- Paths:
  - `/exploration_path` (`nav_msgs/msg/Path`).
  - `/global_path`, `/global_path_full` (`nav_msgs/msg/Path`).
  - `/local_path`, `/tare_visualizer/local_path`, `/path` (`nav_msgs/msg/Path`).
- Markers/visualization:
  - `/tare_visualizer/marker` (`visualization_msgs/msg/Marker`).
  - `/tare_visualizer/exploring_subspaces_array` (`visualization_msgs/msg/MarkerArray`).
  - `/tare_visualizer/local_planning_horizon_array` (`visualization_msgs/msg/MarkerArray`).
  - `/grid_world_marker`, `/keypose_graph_edge_marker`, `/keypose_graph_node_marker`.
- Metrics/status:
  - `/runtime` (`std_msgs/msg/Float32`).
  - `/runtime_breakdown` (`std_msgs/msg/Int32MultiArray`).
  - `/exploration_finish` (`std_msgs/msg/Bool`).
  - `/momentum_activation_count` (`std_msgs/msg/Int32`).

## Recommended Bimodal Integration

Recommended nodes:

- `mode_manager`: owns `ground` / `aerial` mode switching.
- `ground_planner`: runs A* + DWA and outputs ground waypoint / `cmd_vel`.
- `aerial_tare_planner`: wraps or remaps TARE inputs/outputs.
- `common_planning_interface`: normalizes active goal/path for the controller.
- `controller_bridge`: converts the active goal into controller-specific commands.

Recommended topics:

- `/bimodal/mode` (`std_msgs/msg/String` or custom `Mode.msg`).
- `/bimodal/planning/ground/goal` (`geometry_msgs/msg/PoseStamped`).
- `/bimodal/planning/aerial/goal` (`geometry_msgs/msg/PoseStamped`).
- `/bimodal/planning/active_waypoint` (`geometry_msgs/msg/PoseStamped`).
- `/bimodal/planning/path` (`nav_msgs/msg/Path`).
- `/bimodal/planning/status` (`std_msgs/msg/String` or custom status msg).
- `/bimodal/perception/registered_scan` (`sensor_msgs/msg/PointCloud2`).
- `/bimodal/state_estimation` (`nav_msgs/msg/Odometry`).
- `/bimodal/map` (`sensor_msgs/msg/PointCloud2` or map-specific type).

Recommended remaps for aerial mode:

- `/bimodal/state_estimation` -> `/state_estimation`
- `/bimodal/perception/registered_scan` -> `/registered_scan`
- `/bimodal/map` -> `/terrain_map` or upstream terrain processing input
- `/way_point` -> `/bimodal/planning/aerial/goal`
- `/global_path` or `/exploration_path` -> `/bimodal/planning/path`

Wrapper guard:

- Reject `/way_point` if any coordinate is NaN or infinity.
- On invalid waypoint, publish `/bimodal/planning/status=INVALID_AERIAL_WAYPOINT`, hold the previous safe target, and request reset/replan.

## Controller Handoff Items

Confirm these before connecting to the real controller:

- Waypoint coordinate frame: use `map` or provide `map -> controller_frame` TF.
- Waypoint update rate: observed about 1 Hz from TARE.
- Command representation: TARE gives points/paths, not full attitude/thrust commands.
- Yaw handling: controller or bridge should infer yaw from path tangent or target policy.
- Arrival threshold: define XYZ tolerance and timeout.
- Fail-safe behavior: hover, land, hold last waypoint, or switch to ground mode.
- Mode transition policy: ensure aerial-to-ground switch clears old TARE waypoints and stops TARE output from controlling the active controller.
