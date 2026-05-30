# Launch Script Runtime Fix

Date: 2026-05-26

## Scope

Only `scripts/` files were modified. No TARE planner algorithm source, Gazebo world, or robot model was changed.

Script backup:

- `/home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/script_backups_20260526_223947`

## Modified / Added Scripts

- `scripts/clean_tare_runtime.sh`
- `scripts/diagnose_system_graphics.sh`
- `scripts/diagnose_gazebo_gui.sh`
- `scripts/diagnose_rviz_display.sh`
- `scripts/run_stable_p4_test.sh`
- `scripts/launch_tare_sim.sh`
- `scripts/launch_tare_sim_gui.sh`
- `scripts/launch_tare_sim_headless.sh`
- `scripts/launch_tare_rviz.sh`

## Launcher Behavior

### Stable default

```bash
./scripts/launch_tare_sim.sh
```

Equivalent to:

```bash
./scripts/launch_tare_sim.sh stable
```

Starts headless Gazebo only:

```bash
ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=false rviz:=false joy:=false visualization_tools:=false
```

### Gazebo GUI

```bash
./scripts/launch_tare_sim.sh gui
./scripts/launch_tare_sim_gui.sh
```

Starts Gazebo GUI only, not RViz:

```bash
ros2 launch vehicle_simulator system_garage.launch gazebo_gui:=true rviz:=false joy:=true visualization_tools:=false
```

The script warns that if `gzclient` exits with code `-9`, use headless Gazebo plus RViz.

### Headless only

```bash
./scripts/launch_tare_sim.sh headless
./scripts/launch_tare_sim_headless.sh
```

Starts no `gzclient` and no `rviz2`.

### RViz / TARE

```bash
./scripts/launch_tare_rviz.sh
```

Starts TARE planner with RViz and explicitly does not start Gazebo GUI.

## Diagnostic Finding

The cleanup script was initially too broad because it matched script/log paths containing `gazebo`. It has been corrected to match exact process command names such as `gzclient`, `gzserver`, `rviz2`, `tare_planner_node`, and planner process names.
