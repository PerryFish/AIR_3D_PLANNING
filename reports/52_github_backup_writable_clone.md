# GitHub Backup Via Writable Clone

## Original Repository

- path: `/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN`
- branch: `test-dense50-run-v0.2`
- remote: `https://github.com/PerryFish/AIR_3D_PLANNING.git`
- worktree writable: yes
- `.git` writable: no

The original worktree accepted normal file writes, but `.git` rejected writes with:

```text
Read-only file system
```

That prevented creating `.git/index.lock`, so `git add`, `git commit`, and `git push` could not run in the original directory.

## Writable Backup Clone

- requested clone path: `/home/nuaa/ZHY/AIR_3D_PLANNING_BACKUP_WRITABLE`
- requested clone result: failed, `/home/nuaa/ZHY` is read-only in this execution environment
- fallback writable clone path: `/tmp/AIR_3D_PLANNING_BACKUP_WRITABLE_LOCAL_20260530-2032`
- fallback method: copied the current worktree and readable `.git` from `AIR_3D_PLANNING_CLEAN` into `/tmp`, excluding build/install/log/logs
- branch: `backup/garage-start-pose-aligned-tare-edge`
- commit: `PENDING`
- pushed: `PENDING`
- tag: `PENDING`
- tag pushed: `PENDING`

## Reuse

```bash
git clone https://github.com/PerryFish/AIR_3D_PLANNING.git
cd AIR_3D_PLANNING
git checkout backup/garage-start-pose-aligned-tare-edge
```

Run the current garage demo from the working project path:

```bash
cd /home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN
./scripts/run_garage_v1_visual_exploration.sh
```

## Current Garage State

- start pose: `(-23.817, -46.018, 1.6)`, yaw `0.0`
- RViz profile: `tare_edge_replay`
- edge cloud: `/overall_map` and `/registered_scan` remain edge/scan-like, not surface cloud
- latest garage coverage target check: WARN but non-fatal, around `0.711721` observed coverage in the short validation run

## Limitations

- This backup uses a `/tmp` writable clone because the requested `/home/nuaa/ZHY` clone location is read-only in this environment.
- The original `.git` remains read-only.
- This is still an AIR edge replay and frontier/backtracking baseline, not complete TARE graph exploration or FAST-LIVO/SLAM.
