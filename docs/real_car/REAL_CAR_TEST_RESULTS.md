# F1TENTH UJ: real-car test results

Running log for the tests in `tools/real_car_tests/`. Standard values:
[REAL_CAR_BASELINE_2026-11-14.md](REAL_CAR_BASELINE_2026-11-14.md). A baseline value changes only
through a row here (test, date, value, old value, decision). Raw evidence:
`~/f1tenth_hw_evidence/real_car_tests/` on the Jetson (not committed).

## Results

| Test | Date | Value | Old value | Decision |
|---|---|---|---|---|
| Dry run (software only: status stub, isolated domain 77, no VESC) | 2026-11-14 | Runner publish interval p50/p99/max 20.0/20.0/20.0 ms; /joy max gap received 42 ms (TTL 200 ms); arm -> drive -> PROFILE_DONE and DEADMAN_RELEASED both zero on the same tick; bag + analysis complete | - | Runner, bag and analysis work end to end. The dry run on the car stack is still to do. |

## Changes to shared code

### `physical_drive_mux.py` (Zaid's file): per-test caps. Applied 2026-11-14 (approved by Moatasem), in its own commit for review

- **What:** two new parameters, `max_speed_mps` and `max_steer_rad`. Defaults stay `0.35` and
  `0.10` (the old constants), so every existing launch behaves exactly as before. Values are
  checked at start-up against hard ceilings of **1.5 m/s** and **0.40 rad** (`validate_caps`);
  the node refuses to start with anything else. The status JSON now reports both caps.
  The gate logic, TTLs, LiDAR gate, deadman, ownership checks and interlocks are unchanged.
- **Why:** the tests need 0.5 / 1.0 / 1.5 m/s and the measured steering range; the caps were
  hard-coded constants. A parameter set only by `tools/real_car_tests/real_car_test.launch.py`
  avoids another copy of the mux (the trial copy with `MAX_STEER = 0.20` is how F-17 started).

## Notes for the team (not edited by these tests)

- **Do not use `src/f1tenth_bringup/launch/vesc.launch.py`.** It loads `config/vesc.yaml` with
  `brake_max 200000`, `current_max 100` and `speed_max 10000`, and no command remapping.
  Brake commands are in **amps** (`vesc_packet.cpp`: brake x 1000 -> mA), so 200000 is no limit.
- Still at the old steering offset **0.361** (baseline: 0.320):
  `src/f1tenth_bringup/config/ackermann_to_vesc.yaml`, `src/f1tenth_bringup/config/vesc_to_odom.yaml`,
  `tools/diagnostics/steering_radius_audit.py` (prints 0.361),
  `tools/diagnostics/fit_steering_measurements.py` (self-test asserts 0.361).
  The real-car tests use `config/real_car/*.yaml` (0.320) instead.
- Every active `vesc_driver` yaml has `speed_max 3500` ERPM (0.416 m/s): the 12 Nov rebound peak
  (3,911 ERPM) was **above** that clip, so the rebound did not come from a commanded speed.
- `config/vesc_real_integration.yaml` servo clip 0.30-0.42 is asymmetric about 0.320: it allows
  only 0.04 rad of left steering at gain -0.50.
- The BMI160 node is commented out in `sensors.launch.py`, and no launch in the repo publishes
  `base_link -> imu_link`. The test launch publishes it (PROVISIONAL) until we know how it was done on 14 Nov.
