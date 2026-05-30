# Runbook

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
python3 scripts/generate_dense50_gazebo_world.py
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install
set +u
source install/setup.bash
set -u
python3 scripts/validate_dense50_ground_ratio.py --config config/aerial_exploration.yaml
timeout -s INT -k 10s 180s scripts/run_aerial_exploration_tests.sh
timeout -s INT -k 10s 120s scripts/run_anti_fake_coverage_tests.sh
timeout -s INT -k 10s 180s scripts/run_visual_exploration_smoke_test.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_assets.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_world_not_empty.sh
timeout -s INT -k 10s 90s scripts/run_garage_v1_gazebo_smoke_test.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_start_pose_alignment.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_structure_cloud.sh
timeout -s INT -k 10s 120s scripts/check_garage_edge_cloud_quality.sh
timeout -s INT -k 10s 180s scripts/check_tare_edge_replay_topics.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_tare_edge_rviz_profile.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_rviz_clean_view.sh
timeout -s INT -k 10s 120s scripts/check_garage_v1_gazebo_visual_usability.sh
timeout -s INT -k 10s 240s scripts/run_garage_v1_visual_exploration_smoke_test.sh
timeout -s INT -k 10s 240s scripts/check_garage_v1_adaptive_altitude.sh
timeout -s INT -k 10s 300s scripts/check_garage_v1_coverage_target.sh
timeout -s INT -k 10s 180s scripts/check_garage_v1_frontier_goal_ratio.sh
timeout -s INT -k 10s 120s scripts/check_aerial_corridor_height.sh
timeout -s INT -k 10s 180s scripts/run_sensor_mapping_smoke_test.sh
timeout -s INT -k 10s 120s scripts/check_observed_coverage_not_fake.sh
timeout -s INT -k 10s 120s scripts/check_map_export.sh
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true
ros2 launch aerial_exploration_planner gazebo_dense50.launch.py gui:=true
ros2 launch aerial_exploration_planner gazebo_garage_v1.launch.py gui:=true
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true rviz_profile:=tare_edge_replay start_x:=-23.817 start_y:=-46.018 start_z:=1.6 start_yaw:=0.0
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true rviz_view_mode:=debug
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true rviz_view_mode:=voxel
```

Gazebo visual exploration should show the dense50 world, blue `simple_uav` model, green breadcrumb trail spheres, and a yellow current goal marker. The UAV, breadcrumbs, and goal should stay in the aerial corridor at about `1.4 m`, with valid z range `0.8-2.2 m`, rather than flying above the full obstacle field. If the UAV is not visible:

```bash
ros2 node list | grep gazebo
ros2 service list | grep -E "spawn|state|model"
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
echo "$GAZEBO_MODEL_PATH"
```

If the trail is not visible, check that `/state_estimation` is publishing, `gazebo_trail_visualizer` is running, and `min_distance` or `max_points` have not filtered out new breadcrumbs.

Recommended GUI launch:

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
ros2 launch aerial_exploration_planner visual_aerial_exploration_dense50.launch.py gui:=true rviz:=true sensor_mapping:=true observed_coverage:=true
```

Sensor-driven mapping topics:

```bash
ros2 topic echo /exploration/coverage_real
ros2 topic echo /exploration/map_metrics
ros2 topic list | grep /exploration
```

Map export files are written to `results/maps/`.

Garage V1 migrated scene commands:

```bash
GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner gazebo_garage_v1.launch.py gui:=true

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py gui:=true rviz:=true

./scripts/run_garage_v1_visual_exploration.sh
```

Garage V1 map and metrics outputs are written under `results/garage_v1/`. Current mapping uses a simulated local sensor fallback, not full FAST-LIVO/SLAM.

Garage V1 launch must print a real `garage_v1.world` path. These strings indicate a broken launch and should fail smoke tests:

```text
URI not supported by Fuel [garage_v1]
Could not open file[garage_v1]
Falling back on worlds/empty.world
Loading world file [/usr/share/gazebo-11/worlds/empty.world]
```

Garage V1 Gazebo should load the original `garage` model and, by default, the `garage_wall_proxy` outline model. The proxy is a visual aid for the building outline when the original DAE material is hard to inspect.
The proxy is visual-only, has no collision, uses thin muted grey walls, and should not interfere with mouse selection. `garage_v1.world` defines a `garage_v1_overview` camera for an initial oblique top view.

Garage V1 RViz defaults to TARE edge replay mode:

```text
src/aerial_exploration_planner/rviz/garage_v1_tare_edge_replay.rviz
```

Recommended TARE replay launch:

```bash
./scripts/run_garage_v1_visual_exploration.sh

GAZEBO_MASTER_URI=http://127.0.0.1:11346 \
ros2 launch aerial_exploration_planner visual_aerial_exploration_garage_v1.launch.py \
  gui:=true rviz:=true rviz_profile:=tare_edge_replay

./scripts/run_garage_v1_visual_exploration.sh \
  start_x:=-23.817 start_y:=-46.018 start_z:=1.6 start_yaw:=0.0
```

Primary TARE-like topics:

```text
/overall_map
/registered_scan
/terrain_map
/terrain_map_ext
/explored_areas
/path
/global_path
/local_path
/way_point
/free_paths
/uncovered_frontier_cloud
/tare_visualizer/local_planning_horizon
/exploration/garage_edge_cloud
/exploration/debug_surface_cloud
/exploration/garage_structure_cloud
/exploration/static_garage_structure_cloud
/exploration/observed_structure_cloud
/exploration/structure_cloud
/exploration/local_sensor_cloud
/exploration/local_obstacle_cloud
/exploration/frontier_cloud
/exploration/local_planning_box
/exploration/trajectory_path
/aerial_exploration/path
/aerial_exploration/selected_goal_marker
/exploration/coverage_text
/exploration/start_pose_marker
```

The local planning box is the planner's local search window around the UAV. It is not a garage wall or building outline. It is disabled in clean mode and available as `Debug Local Planning Box` in `rviz_view_mode:=debug`.

View modes:

- `tare_edge_replay`: default, publishes original TARE topic names from an edge/scan-like garage cloud and loads `garage_v1_tare_edge_replay.rviz`.
- `tare_v1_replay`: legacy replay profile.
- `clean`: TARE-like structure cloud and essential planning overlays.
- `tare_reference`: explicitly opens the TARE preview point-cloud reference view.
- `debug`: opens local box, local obstacle/free clouds, and frontier viewpoints.
- `voxel`: opens full voxel map displays for coverage debugging.

Large free/observed voxel cube displays are intentionally disabled in clean mode.

The default static structure cloud is generated from TARE's original `vehicle_simulator/mesh/garage/preview/pointcloud.ply` by z-sliced XY occupancy boundary extraction. Outputs are `src/virtual_env/garage_v1/maps/tare_reference/garage_edge_cloud.pcd` and `.xyz`, transformed with the same `model://garage` pose as Gazebo. Dense preview/surface clouds are debug-only and are disabled in the default RViz profile.

Garage V1 start pose is aligned to the TARE garage start `(0,0,0.75)` through the AIR garage transform. The default AIR start is `(-23.817,-46.018,1.6)`, yaw `0.0`; launch overrides are `start_x`, `start_y`, `start_z`, and `start_yaw`. Planner and mapping grid origins are set to the same XY value so the local exploration grid begins from the entrance-side region instead of the old central-loop pose.

Garage V1 uses adaptive altitude candidates by default. The current validation baseline expects z to remain in `0.8-3.0 m` and normally observes a nonzero z range; the latest run measured `1.600-2.000 m`.
