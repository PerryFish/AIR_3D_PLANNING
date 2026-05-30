# Garage V1 Frontier Backtracking Optimization

## Purpose

Garage V1 has indoor-like corridors, turns, and dead ends. The dense50 strategy was too simple for this shape because a nearest-goal or sweep-only behavior can spend time in one local region and fail to return through already-known free space.

## Implemented Behavior

Garage V1 uses `src/aerial_exploration_planner/config/garage_v1_exploration.yaml` and the mirrored copy under `src/virtual_env/garage_v1/config/garage_v1_exploration.yaml`.

The garage planner configuration enables:

- target observed coverage: `0.75`
- stuck window: `8.0 s`
- failed-goal blacklist radius: `0.8 m`
- max failed goal retries: `2`
- frontier revisit cooldown: `20.0 s`
- backtracking and return-to-branch behavior
- information gain, unexplored-area, narrow-passage, distance, turning, revisit, and loop-closure terms
- adaptive altitude candidates for garage z-level exploration

## Current Result

The latest visual smoke run reached the coverage target:

- initial observed coverage: `0.027465`
- final observed coverage: `0.758975`
- improvement: `0.731510`
- target observed coverage: `0.75`
- stuck events: `0`
- backtrack events: `0`
- failed goals: `1`
- path length: `31.019 m`

The dedicated coverage check also passed:

- final observed coverage: `0.766436`

## Frontier Ratio Warning

`scripts/check_garage_v1_frontier_goal_ratio.sh` currently exits successfully but reports a warning:

- frontier goals: `0`
- goal changes: `10`
- frontier goal ratio: `0.000`

This means the garage baseline still leans heavily on the systematic sweep/backtracking fallback even though online frontier data is published and visible. It is not a fake-coverage failure, but it is a planner-quality limitation. The next planner iteration should convert more goal selections into explicit frontier-sourced goals once real Gazebo lidar/depth data or a stronger local map is available.

## Limitations

- The current map source is still simulated local sensor fallback, not real SLAM.
- The wall proxy and garage mesh are visual references; collision planning still uses the AIR garage occupancy fallback.
- Backtracking did not trigger in the latest smoke run because the run reached target coverage without a stuck event.
