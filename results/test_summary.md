# Dense50 Aerial Exploration Test Summary

- passed: True
- final_coverage: 0.937083
- done: True
- failed_goals: 2
- stuck_events: 0
- measured_ground_footprint_occupancy_ratio: 0.500000
- robot_xyz: (2.495, 2.127, 1.400)
- goal_xyz: (5.500, 5.500, 1.400)
- newly_observed_voxels: 3
- coverage_rule: done is true only when coverage >= 0.93
- dense50_definition: XY ground obstacle footprint occupancy ratio, not 3D voxel occupancy
- limitation: this summary is produced by the ROS2 simulated exploration loop, not by Gazebo physics

## Garage V1 Start Alignment Update

- recommended start pose: `(-23.817, -46.018, 1.600)`, yaw `0.0`
- source: TARE garage `(0,0,0.75)` transformed by AIR garage edge-cloud metadata
- old start pose: `(-13.5, -13.5, 1.600)`
- start analysis: generated `results/garage_v1/start_pose_analysis.json`
- start marker: `/exploration/start_pose_marker`
- build: PASS
- start pose alignment: PASS (`/state_estimation` and marker exact; `/odom` already moving from the same region)
- edge cloud quality: PASS (`edge_density_ratio=0.0089`)
- replay topics: PASS
- RViz profile: PASS
- world/assets/Gazebo smoke: PASS
- visual exploration smoke: PASS (`final_observed_coverage=0.709018`, `frontier_goals=1`, `path_length=31.131`)
- adaptive altitude: PASS (`z=1.600..2.000`)
- coverage target: WARN but non-fatal (`final_observed_coverage=0.711721`, target `0.75`)
- frontier goal ratio: WARN but non-fatal (`frontier_goal_ratio=0.048`)
- aerial corridor height: PASS on isolated rerun
- sensor mapping smoke: PASS (`final_observed_coverage=0.936250`)
