# Stable P4 Test Summary

Date: Tue May 26 23:16:25 CST 2026

## Commands

```bash
./scripts/run_stable_p4_test.sh
```

## Result

- Headless simulation log: /home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_sim.log
- TARE log: /home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_tare.log
- Key events log: /home/nuaa/ZHY/TARE_V1/fix_reports/runtime_stability/stable_p4_key_events.log
- /way_point rate sample: average rate: 1.013
- WAYPOINT_OSCILLATION_DETECTED: 6
- ROBOT_DITHERING_DETECTED: 4
- P4_ESCAPE_RECOVERY_TRIGGERED: 3
- P4_ESCAPE_FALLBACK_WAYPOINT: 0
- RECOVERY_ACCEPTED_NEW_DIRECTION: 3
- NaN/invalid waypoint count: 0
- all directions/permanent blacklist count: 0
- ABSOLUTE TIMEOUT TRIGGERED count: 0

## Conclusion

- Stable headless backend test passed for algorithm runtime checks.
- This test intentionally does not start gzclient or RViz.
