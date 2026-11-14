# F1TENTH UJ: real-car test results

Running log for the tests in `tools/real_car_tests/`. Standard values:
[REAL_CAR_BASELINE_2026-11-14.md](REAL_CAR_BASELINE_2026-11-14.md). A baseline value changes only
through a row here (test, date, value, old value, decision). Raw evidence:
`~/f1tenth_hw_evidence/real_car_tests/` on the Jetson (not committed).

## Results

| Test | Date | Value | Old value | Decision |
|---|---|---|---|---|
| Dry run (software only: status stub, isolated domain 77, no VESC) | 2026-11-14 | Runner publish interval p50/p99/max 20.0/20.0/20.0 ms; /joy max gap received 42 ms (TTL 200 ms); arm -> drive -> PROFILE_DONE and DEADMAN_RELEASED both zero on the same tick; bag + analysis complete | - | Runner, bag and analysis work end to end on the stub. |
| Dry run on the car stack, disarmed (`20261114_134403_dry_run_v0.30_erpmmax3500_disarmed`) | 2026-11-14 | **FAIL as designed:** the runner's `ros2 bag record` subscribed to `/drive` and `/commands/*` -> `drive_subscriber_count=2` -> mux latched `ACTUATOR_GRAPH_OWNERSHIP_FAULT`, runner refused to arm. Otherwise: runner publish p50/p99/max 20.0/21.0/22.9 ms, /joy max gap 42 ms, 1832 scans all 1440 ranges, mux output and vesc_driver sent ERPM always 0 (1250 audit lines), VESC fault 0, 11.2 V. Runner aborted on exit (`terminate called without an active exception`) | - | Mux ownership check confirmed working. Fixed: bag never records `/drive` or `/commands/*` (self-test enforces it), command side from `/control/status` + vesc_driver `VESC_SPEED_AUDIT` log; ordered ROS shutdown (self-test probes it). Re-run needed. |
| Dry run re-run on the car stack, disarmed (`20261114_135821_dry_run_v0.30_erpmmax3500_disarmed`) | 2026-11-14 | **PASS.** Arm accepted twice, no ownership fault; mux output and vesc_driver sent ERPM always 0 (689 audit lines); runner publish max 21.8 ms; /joy max gap 42 ms (TTL 200 ms); SPACE auto-repeat delay 0.50 / 0.56 s, repeat interval ~30 ms; SPACE release -> zero 0.26 / 0.27 s; bag closed (rc 0), no abort; VESC fault 0, 11.1 V. Both repeats ended by releasing SPACE (PROFILE_DONE path verified on the stub and in the self-test) | - | Runner, bag, audit copy and analysis work on the car. Next: T1 step 0.5 m/s. |
| F-14 duplicate node name | 2026-11-14 | One rf2o process (pid 6442), `rf2o_publisher_count=1`, but `/rf2o_laser_odometry` listed twice. Cause (rf2o source): `CLaserOdometry2DNode` (an `rclcpp::Node`) holds a member `CLaserOdometry2D rf2o_ref` that is itself an `rclcpp::Node`; the launch remap `__node:=rf2o_laser_odometry` renames both. The inner node has no publishers or subscriptions | Open (MEDIUM) | **Resolved, harmless.** `preflight.sh` allows exactly this duplicate only with a single rf2o process; any other duplicate fails. rf2o not modified (official package). |
| Front LiDAR on the stand (mux status, disarmed) | 2026-11-14 | `front_valid_fraction` 0.825, nearest 4.43 m | - | Note for T1: MANUAL needs >= 0.75; margin is small. Status shows `FRONT_UNKNOWN` only because STOP mode evaluates the AUTO threshold 0.90. |

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
