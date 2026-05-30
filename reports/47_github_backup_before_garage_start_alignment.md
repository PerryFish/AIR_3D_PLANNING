# GitHub Backup Before Garage Start Alignment

## Git State

- branch: `test-dense50-run-v0.2`
- remote: `origin https://github.com/PerryFish/AIR_3D_PLANNING.git`
- current commit: `f7830c5 Add sensor driven mapping and observed coverage baseline`
- existing tags:
  - `v0.1.0-stable-air-mvp`
  - `v0.2.0-dense50-benchmark`
  - `v0.2.0-gazebo-uav-corridor`
  - `v0.3.0-sensor-mapping-baseline`

## Commit / Push Attempt

- commit attempted: yes
- commit message: `Backup garage TARE edge replay visualization baseline`
- push attempted: no, because commit failed
- tag attempted: no, because commit failed
- requested tag: `v0.5.0-garage-tare-edge-replay`

Failure reason:

```text
fatal: Unable to create '/home/nuaa/ZHY/AIR_3D_PLANNING_CLEAN/.git/index.lock': Read-only file system
```

## Tar Backup

- backup generated: yes
- path: `/tmp/AIR_3D_PLANNING_CLEAN_v0_5_garage_tare_edge_replay_backup.tar.gz`
- size: `61M`
- copied to `~/Downloads`: failed, `~/Downloads` is on a read-only filesystem

The backup excludes `build`, `install`, `log`, and `.git`.
