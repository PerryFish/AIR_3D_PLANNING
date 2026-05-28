# Gazebo Error Diagnosis

Scope: diagnosis only. No source, launch, world, or commit changes were made.

## Direct Failure

Gazebo exits because the installed world file contains a ROS Gazebo system plugin as a world plugin:

- error file: `/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/install/aerial_exploration_planner/share/aerial_exploration_planner/worlds/dense50_ground_footprint.world`
- XML line: 4
- offending XML:

```xml
<plugin name='gazebo_ros_init' filename='libgazebo_ros_init.so'/>
```

Gazebo log:

```text
World[dense50_ground_footprint] is attempting to load a plugin, but detected an incorrect plugin type.
Plugin filename[libgazebo_ros_init.so] name[gazebo_ros_init]
```

Conclusion: this is the fatal error that causes `gazebo --verbose .../dense50_ground_footprint.world` to exit with code 255.

## Launch Path

The launch files start Gazebo directly:

```text
gazebo --verbose <world>
gzserver --verbose <world>
```

Relevant files:

- `launch/gazebo_dense50.launch.py`
- `src/aerial_exploration_planner/launch/gazebo_dense50.launch.py`
- installed copy under `install/aerial_exploration_planner/share/aerial_exploration_planner/launch/gazebo_dense50.launch.py`

## Source Of The Bad Line

The same plugin line exists in:

- `worlds/dense50_ground_footprint.world:4`
- `src/aerial_exploration_planner/worlds/dense50_ground_footprint.world:4`
- `install/aerial_exploration_planner/share/aerial_exploration_planner/worlds/dense50_ground_footprint.world:4`
- generator source: `scripts/generate_dense50_gazebo_world.py:73`

`libgazebo_ros_init.so` exists at `/opt/ros/humble/lib/libgazebo_ros_init.so`, but it is not valid in this XML location as a world plugin.

## A_DWA Model Warning

Current environment:

```text
GAZEBO_MODEL_PATH=/home/nuaa/ZHY/A_DWA/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models:/home/nuaa/ZHY/A_DWA/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
GAZEBO_RESOURCE_PATH=
GAZEBO_PLUGIN_PATH=
```

The A_DWA path is present in `GAZEBO_MODEL_PATH`. The directory
`/home/nuaa/ZHY/A_DWA/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models/turtlebot3_autorace_2020`
exists but has no top-level `model.config`; it contains nested model directories instead.

Conclusion: the `Missing model.config` message is caused by Gazebo scanning the old A_DWA TurtleBot3 model path. It is an environment/model database warning and is not the direct cause of this Gazebo process exit. The fatal cause is the incorrect `libgazebo_ros_init.so` plugin entry in the world file.

## Temporary Bypass Without Changing Code

Run Gazebo with a temporary world copy that removes the invalid plugin line, and avoid the stale A_DWA model path:

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
grep -v "libgazebo_ros_init.so" install/aerial_exploration_planner/share/aerial_exploration_planner/worlds/dense50_ground_footprint.world > /tmp/dense50_ground_footprint_no_ros_init.world
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models gazebo --verbose /tmp/dense50_ground_footprint_no_ros_init.world
```

Headless variant:

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
grep -v "libgazebo_ros_init.so" install/aerial_exploration_planner/share/aerial_exploration_planner/worlds/dense50_ground_footprint.world > /tmp/dense50_ground_footprint_no_ros_init.world
GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models gzserver --verbose /tmp/dense50_ground_footprint_no_ros_init.world
```

This bypass does not modify repository files.

## Future Fix Location

If code changes are allowed later, fix the generator and regenerated world files:

- remove the invalid world plugin line from `scripts/generate_dense50_gazebo_world.py`
- regenerate `worlds/dense50_ground_footprint.world`
- update `src/aerial_exploration_planner/worlds/dense50_ground_footprint.world`
- rebuild/install the package

If ROS-Gazebo integration is needed, load `libgazebo_ros_init.so` through the `gazebo_ros` launch mechanism or Gazebo system plugin arguments, not as a `<world><plugin>` entry in this SDF file.

## Fixed And Verified

Root cause:

- `libgazebo_ros_init.so` was written into the SDF world as a world plugin.
- Gazebo Classic rejected it with `incorrect plugin type` and exited with code 255.

Modified files:

- `scripts/generate_dense50_gazebo_world.py`
- `worlds/dense50_ground_footprint.world`
- `src/aerial_exploration_planner/worlds/dense50_ground_footprint.world`
- `launch/gazebo_dense50.launch.py`
- `src/aerial_exploration_planner/launch/gazebo_dense50.launch.py`
- `scripts/run_visual_exploration_smoke_test.sh`

Fix:

- removed `libgazebo_ros_init.so` from the `.world` file and generator
- load Gazebo ROS support as system plugins from launch:
  `-s libgazebo_ros_init.so -s libgazebo_ros_factory.so`
- set `GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models` in launch/test scope to avoid inheriting the old A_DWA TurtleBot3 model path
- visual smoke test now fails on `incorrect plugin type`, `World.cc:1803`, `exit code 255`, or A_DWA `Missing model.config`

Verification:

- regenerated world: PASS
- dense50 footprint ratio: `0.500000`
- source world plugin grep: PASS, no invalid plugin lines
- install world plugin grep after rebuild: PASS, no invalid plugin lines
- standalone Gazebo headless launch: PASS, world loaded and process exited cleanly after timeout SIGINT
- Gazebo GUI + RViz launch: started Gazebo and RViz successfully; command was later interrupted by timeout
- visual smoke test: PASS
- old exploration test: PASS
- anti-fake coverage test: PASS
