# Visual Simulation Smoke Test

- result: PASS
- gazebo: launched headless with dense50_ground_footprint.world
- rviz: launch command available; smoke test used rviz:=false for CI stability
- initial_coverage: 0.043750
- final_coverage: 0.930000
- done: True
- unique_pose_samples: 306
- unique_goal_samples: 105
- final_failed_goals: 0
- final_stuck_events: 0
- coverage_source: observed voxels from simulated odom and sensor range
- limitation: Gazebo renders the dense50 world; robot motion is ROS2 kinematic follower, not Gazebo physics control
