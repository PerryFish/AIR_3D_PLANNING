# Command Log

## GitHub Sync

```bash
cd /home/nuaa/ZHY
git --version
gh --version || true
git config --global user.name || true
git config --global user.email || true
gh auth status || true
git ls-remote https://github.com/PerryFish/TARE_Workspace_V1.git || true
```

The first remote check needed network escalation and then succeeded with no refs returned.

```bash
cd /home/nuaa/ZHY/TARE_Workspace_V1_git
git status
git remote -v
git fetch origin
git branch -a
```

Remote was confirmed as `https://github.com/PerryFish/TARE_Workspace_V1.git`.

```bash
rsync -av --delete \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '.vscode/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude 'test_results/bags/' \
  /home/nuaa/ZHY/TARE_V1/ \
  /home/nuaa/ZHY/TARE_Workspace_V1_git/
```

Large non-garage mesh directories and stray temporary files were removed only from the independent sync workdir:

```bash
rm -rf /home/nuaa/ZHY/TARE_Workspace_V1_git/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/campus \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/forest \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/indoor \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/src/autonomous_exploration_development_environment/src/vehicle_simulator/mesh/tunnel \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/15.0 \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/camera、sensor \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/sensor_at_scan、sensor \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/sensor、map \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/vehicle \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/frames_2026-05-25_20.23.03.gv \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/frames_2026-05-25_20.23.03.pdf \
       /home/nuaa/ZHY/TARE_Workspace_V1_git/TARE_Workspace_V1
```

```bash
find /home/nuaa/ZHY/TARE_Workspace_V1_git -type f -size +90M -printf '%s %p\n' | sort -nr
du -sh /home/nuaa/ZHY/TARE_Workspace_V1_git
git add .
git commit -m "Sync working ROS2 Humble TARE workspace"
git push -u origin main
```

Commit succeeded locally as `1b8b903844d53c9212bcfa4a1cdb8012076e7479`. Push failed due missing GitHub HTTPS credentials.

## Read-only Review

```bash
mkdir -p /home/nuaa/ZHY/TARE_V1/review_reports/logs
mkdir -p /home/nuaa/ZHY/TARE_V1/review_reports/source_maps
cd /home/nuaa/ZHY/TARE_V1
find src/autonomous_exploration_development_environment -maxdepth 5 -type f | sort > review_reports/source_maps/source_tree.txt
```

```bash
rg -n -i "ABSOLUTE TIMEOUT|Waypoint age|PERMANENT blacklist|blacklist|stuck|Waypoint reset|\bnan\b|NaN|isnan|isfinite|waypoint|way_point|new waypoint|requesting new waypoint|return|home|exploration finished|local path|global path|viewpoint|frontier|TSP|trajectory|terrain|collision|narrow|timeout" src > review_reports/source_maps/keyword_search_results.txt
```

```bash
rg -n "way_point|create_publisher|publish|Waypoint|waypoint" src/autonomous_exploration_development_environment/src src/tare_planner/src/tare_planner > review_reports/source_maps/waypoint_static_search.txt
rg -n -i "sqrt|norm|normalize|atan2|acos|asin|size\(\)|empty\(\)|frontier|viewpoint|tsp|blacklist|timeout|return home|returning home|home" src/tare_planner/src/tare_planner > review_reports/source_maps/nan_math_search.txt
git status > review_reports/source_maps/git_status.txt 2>&1 || true
git diff > review_reports/source_maps/git_diff.txt 2>&1 || true
```

Additional read-only inspection used `nl`, `sed`, and `rg` on the TARE planner source files. No source files under `/home/nuaa/ZHY/TARE_V1/src` were modified.
