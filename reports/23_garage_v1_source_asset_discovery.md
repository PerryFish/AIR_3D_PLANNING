# Garage V1 Source Asset Discovery

## Source And Target

- Source project: `/home/nuaa/ZHY/TARE_V1`
- Target project: `/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN`
- Target environment folder: `src/virtual_env/garage_v1`

## Garage World

- Main source world: `src/autonomous_exploration_development_environment/src/vehicle_simulator/world/garage.world`
- Installed duplicate: `install/vehicle_simulator/share/vehicle_simulator/world/garage.world`
- Main world dependency: `<uri>model://garage</uri>`
- System dependency: `<uri>model://sun</uri>` from `/usr/share/gazebo-11/models/sun`

## Related Model Directories

- Source model directory: `src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/garage`
- Installed duplicate: `install/vehicle_simulator/share/vehicle_simulator/mesh/garage`
- Model metadata: `model.config`
- Model SDF: `model.sdf`
- Model SDF mesh references:
  - `model://garage/meshes/garage.dae`

## Mesh, Material, Texture Assets

Copied runtime assets:

- `models/garage/meshes/garage.dae`
- `models/garage/meshes/blue-concrete-texture-3.jpg`
- `models/garage/meshes/imgonline-com-ua-tile-pLFAF53NCY1Spr.jpg`
- `models/garage/meshes/imgonline-com-ua-tile-RySXiqZBriXUB4J.jpg`
- `models/garage/meshes/imgonline-com-ua-tile-turvdQ7ZY0r.jpg`
- `models/garage/preview/overview.png`

Not copied:

- `models/garage/preview/pointcloud.ply`, about 55 MB, because it is a preview/reference point cloud and is not required by Gazebo.

## Launch Files

Copied as reference only:

- `src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/system_garage.launch`
- `src/autonomous_exploration_development_environment/src/vehicle_simulator/launch/vehicle_simulator.launch`
- `src/tare_planner/src/tare_planner/launch/explore_garage.launch`
- `src/tare_planner/src/tare_planner/launch/explore.launch`
- `src/autonomous_exploration_development_environment/src/waypoint_example/launch/waypoint_example_garage.launch`

These are ROS2 launch files, but they are tied to TARE packages such as `vehicle_simulator`, `local_planner`, `terrain_analysis`, `sensor_scan_generation`, `visualization_tools`, and `tare_planner`. They are useful references, not direct AIR launch dependencies.

## RViz Files

Copied as reference:

- `src/autonomous_exploration_development_environment/src/vehicle_simulator/rviz/vehicle_simulator.rviz`
- `src/tare_planner/src/tare_planner/rviz/tare_planner_ground.rviz`

AIR uses its own `aerial_exploration.rviz` for the integrated demo.

## Config And Map Files

Copied:

- `src/tare_planner/src/tare_planner/config/garage.yaml` to `config/tare_garage.yaml`
- `src/autonomous_exploration_development_environment/src/waypoint_example/data/waypoints_garage.ply`
- `src/autonomous_exploration_development_environment/src/waypoint_example/data/boundary_garage.ply`

`tare_garage.yaml` is a TARE planner reference config and is not loaded by AIR nodes.

## Documentation

Copied:

- `DEPLOY_NOTES.md`
- `DEPLOY_REPORT.md`
- `src/tare_planner/README.md`

## Dependency Summary

- `garage_v1.world` needs `model://garage` and `model://sun`.
- `model://garage` resolves through `GAZEBO_MODEL_PATH` pointing at `src/virtual_env/garage_v1/models` or the installed package share equivalent.
- `model://sun` resolves through `/usr/share/gazebo-11/models`.
- The garage model uses relative `model://garage/meshes/...` references, so preserving `models/garage/meshes` is sufficient.

## Files To Copy

- Source world and custom garage model directory.
- Mesh and texture files under `mesh/garage/meshes`.
- TARE launch/config/RViz/docs as traceable references.
- Small waypoint/boundary PLY files as examples.

## Reference Only

- TARE planner launch files and `garage.yaml`.
- Vehicle simulator launch files.
- TARE RViz configs.
- TARE deployment reports.

## Files Not To Copy

- `install/` duplicates from TARE_V1.
- `build/` and `log/` products.
- Python `__pycache__`.
- Large preview point cloud `preview/pointcloud.ply`.
- TARE planner binaries or package source trees not needed by AIR.

## Risks

- The original world included TARE/rotors plugins: `librotors_gazebo_ros_interface_plugin.so` and `libgazebo_ros_state.so`; AIR removes them and loads ROS Gazebo plugins with `gazebo -s`.
- AIR garage exploration currently uses a simulated local sensor fallback derived from a garage-like occupancy approximation, not real Gazebo lidar or full SLAM.
- TARE launch files are package-coupled and should not be invoked from AIR without porting dependencies.
- Gazebo GUI stability may still depend on local graphics drivers; headless smoke tests are the baseline verification path.
