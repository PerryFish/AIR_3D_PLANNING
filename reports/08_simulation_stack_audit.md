# Simulation Stack Audit

The previous `aerial_exploration_planner` delivery was a synthetic mock. It generated a CSV with fixed-step coverage growth and did not launch Gazebo, RViz, a robot model, sensor input, or a closed exploration loop.

Current repository audit:

- Gazebo executable available on this machine: yes (`gazebo`, `gzserver`)
- RViz2 executable available on this machine: yes (`rviz2`)
- Existing Gazebo world in repository before this change: no
- Existing RViz config before this change: yes, `src/air_bringup/rviz/air_planning.rviz`
- Existing UAV/robot URDF/SDF model before this change: no
- Existing odom/state publisher before this change: yes, `air_uav_simulator` publishes `/air/state_estimation` and TF
- Existing PointCloud2 publisher before this change: no
- Existing map/world provider before this change: yes, `air_world_provider` publishes obstacle markers
- Existing dense50 world before this change: no Gazebo world; only ROS marker-based dense50 planning config
- Existing trajectory/path follower before this change: yes, `air_uav_simulator` is a kinematic ROS follower for `/air/smoothed_path`

Reusable packages:

- `air_world_provider`: marker-based obstacle and map visualization concepts
- `air_uav_simulator`: kinematic odom/TF/path-following pattern
- `air_bringup`: launch/RViz packaging pattern
- `air_global_planner`: path publication pattern

Missing pieces addressed in this round:

- Generated Gazebo Classic world: `worlds/dense50_ground_footprint.world`
- Gazebo launch: `gazebo_dense50.launch.py`
- RViz config for exploration markers: `config/aerial_exploration.rviz`
- One-command Gazebo/RViz launch: `visual_aerial_exploration_dense50.launch.py`
- Pose-driven observation map update
- Kinematic `/odom` and TF follower for the exploration loop
- Planner goal/path output based on pose and map frontier state
- CSV fields for robot pose, goal, path length, goal changes, pose changes, and newly observed voxels

Remaining limitation:

Gazebo renders the dense50 environment, but UAV motion is still driven by a ROS2 kinematic follower rather than Gazebo physics control or a real depth camera plugin. Coverage is now tied to simulated odom and observed voxels, not fixed time steps.
