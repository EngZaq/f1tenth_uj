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
| T1a step 0.5 m/s, speed0, stand (`20261114_145433_t1_stop_v0.50_speed0_erpmmax4630_armed`) | 2026-11-14 | **Not a valid gate run; F-01 reproduced.** Stop 1 (from 1486 ERPM, target 4207): sent ERPM 0 (audit), ERPM dips to ~200 at +0.09 s, then **rebound peak 1969 ERPM at +0.233 s** with motor current +9.2 A and duty +0.089 (the VESC drives the motor while the sent command is 0), decaying ~linearly to 0 by +0.46 s, tach 73. Stop 2: +267 ERPM. Repeats 1-2 ended by SPACE released: the key stream stopped at the sender (1-2 SPACE events, none arriving late), not SSH jitter. Repeats 3-5 latched `LATCH_STOP_FRONT_SCAN_NOT_CLEAR`: r3 `OBSTACLE_NEAR` 0.085 m (returns 0.10-0.12 m at +10..+19 deg in 46 % of scans, present between repeats only: likely the operator's hand/arm), r4-r5 `FRONT_UNKNOWN` (front valid fraction 0.45-0.66 < 0.75) | 12 Nov: rebound 0.16-0.27 s, 3911 ERPM (above the then 3500 speed_max) | Re-run 0.5 m/s with SSH over USB, a box 1.5-2 m ahead, hands out of the front +/-20 deg sector. Peak timing matches 12 Nov. The linear decay of duty/current after the zero fits a speed-PID setpoint ramp (`s_pid_ramp_erpms_s`, checklist row 6): hypothesis, check in VESC Tool. Step gate now counts only stops from >= 80 % of the step ERPM. |
| T1a step 0.5 m/s, speed0, stand (`20261114_150601_t1_stop_v0.50_speed0_erpmmax4630_armed`) | 2026-11-14 | **FAIL, 4 of 5 stops rebound > 300 ERPM.** Worst: stop 3 (from 3035 ERPM) rebound peak **7871 ERPM** at +0.253 s, 14.1 A, tach 185. 7871 is above vesc_driver `speed_max` 4630, so the rebound is VESC-internal, not a command. Stops 1-2 from steady speed (4202 / 4113 ERPM): +429 / +1880 ERPM. Stops 3-5 were not from steady speed | 12 Nov: 3911 ERPM peak | speed0 (the standard ackermann_to_vesc stop) is not safe. |
| T1b step 0.5 m/s, brake 5 A, stand (`20261114_150933_t1b_stop_v0.50_brake5A_erpmmax4630_armed`) | 2026-11-14 | **PASS, 5 of 5 from steady speed** (4068-4277 ERPM). Rebound <= 197 ERPM, < 100 ERPM within 0.052-0.149 s, motor current <= 3.7 A, tach 11-20, VESC fault 0. Stops 2 and 5 show a small rise (99 / 83 ERPM) at +2.13 s: the brake hold ends at 2.0 s and the converter switches to current 0 | - | Brake 5 A stops without rebound at 0.5 m/s. Step gate open for brake5A at 1.0 m/s. |
| T1b step 0.5 m/s, brake 10 A, stand (`20261114_152149_..._brake10A`, `20261114_152526_..._brake10A`) | 2026-11-14 | First run INCOMPLETE (4 steady stops, none failing). Second run **PASS, 5 of 5 from steady speed** (4006-4296 ERPM): rebound <= 246 ERPM (peak 251), < 100 ERPM within 0.061-0.123 s, motor current <= 4.2 A, tach 17-20. Measured current is no higher than with 5 A (<= 3.7 A): the stop is over in ~0.1 s and 50 Hz telemetry may miss the peak, or the firmware brake limit (`l_current_min`, checklist row 11) caps it | - | Brake 10 A also stops without rebound at 0.5 m/s; marginally faster settle than 5 A (0.123 vs 0.149 s worst), similar peak. |
| T1b step 1.0 m/s, brake 5 A, stand (`20261114_152830_t1b_stop_v1.00_brake5A_erpmmax9255_armed`) | 2026-11-14 | **INCOMPLETE: 4 steady stops of 5 needed, none failing.** Steady stops from 7292-8467 ERPM: rebound <= 137 ERPM, < 100 ERPM within 0.119-0.181 s, current <= 4.8 A, tach 36-53. Stop 4 released at 4014 ERPM (not steady, still passed: +178) | - | Repeat the 1.0 m/s brake 5 A run with 5 full holds. Every brake run shows the same small rise (77-100 ERPM) at +2.1-2.2 s: the 2.0 s brake hold ending (switch to current 0). |
| T1b step 1.0 m/s, brake 5 A, stand, second run (`20261114_153922_t1b_stop_v1.00_brake5A_erpmmax9255_armed`) | 2026-11-14 | **INCOMPLETE: 4 steady stops, none failing.** Steady stops from 8384-8502 ERPM: rebound <= 110 ERPM (peak 117), < 100 ERPM within 0.121-0.161 s, current <= 4.7 A, tach 50-54. Stop 5 released during the 2 s ramp (5037 ERPM). Both 1.0 m/s runs: 8 steady stops, 0 failing. All early endings were SPACE releases (clean key stream, max gap 37 ms, zero 0.25-0.27 s after the last key) | - | Runner now counts only repeats that reached steady speed and offers up to 3 extra repeats; step gate unchanged (5 steady stops in one run). |
| T1b step 0.5 m/s, current 0 (coast), stand (`20261114_154158_t1b_stop_v0.50_current0_erpmmax4630_armed`) | 2026-11-14 | **PASS, 5 of 5 from steady speed** (4082-4290 ERPM): rebound <= 78 ERPM (peak 91), < 100 ERPM within 0.098-0.197 s, motor current <= 1.2 A, tach 18-32. Live rebound line equal to the analysis on all 5 stops (live-display fix ae9a36f confirmed on the car) | - | Lowest rebound and current, slowest settle. It is a coast: wheels off the ground stop by friction; on the ground the car's inertia will roll much further, so it is not a stopping method for driving without a measured stop distance (T4). |
| T1b step 1.0 m/s, brake 5 A, stand: attempts not counted (`20261116_214648_...`, `20261116_214934_...`) | 2026-11-16 | Not gate runs. `214648`: runner `ARM_REFUSED` (`VESC_TELEMETRY_STALE`) on every arm request, no drive. `214934`: both repeats left manual mode 0.04 s after `DRIVE_START` (`LATCH_STOP_FRONT_SCAN_NOT_CLEAR`), ERPM never left 0; not analysed | - | Interlocks refused as designed; no change. |
| T1b step 1.0 m/s, brake 5 A, stand, third run (`20261116_215116_t1b_stop_v1.00_brake5A_erpmmax9255_armed`) | 2026-11-16 | **PASS, 5 of 5 from steady speed** (8339-8489 ERPM). Rebound rise <= 152 ERPM (peak 286), < 100 ERPM within 0.120-0.179 s, motor current <= 4.4 A, tach 43-56, VESC fault 0, battery >= 12.3 V. Stops 1, 3, 5 show a small early rise (113-152 ERPM) at +0.16 s, the same timing as the speed-0 rebound but ~50x smaller; stops 2, 4 only the hold-release rise (96-98 ERPM at +2.12-2.14 s). One front-scan latch during the run | Two INCOMPLETE runs on 14 Nov (4 steady each, 0 failing) | Step gate open for 1.5 m/s. |
| T1b step 1.5 m/s, brake 5 A, stand (`20261116_215634_t1b_stop_v1.50_brake5A_erpmmax13883_armed`) | 2026-11-16 | **PASS, 5 of 5 from steady speed** (12608-12712 ERPM). Rebound rise <= 114 ERPM (peak 220), < 100 ERPM within 0.165-0.227 s, motor current <= 4.8 A, tach 95-103, VESC fault 0, battery >= 12.3 V. Early rise only in stop 5 (114 ERPM at +0.19 s); stops 1-4 only the hold-release rise (88-101 ERPM at +2.09-2.15 s) | - | Brake 5 A stops without rebound at every test speed. |
| **T1 close-out (stand)** | 2026-11-16 | Brake 5 A: **23 steady stops at 0.5 / 1.0 / 1.5 m/s, 0 failing**, worst rebound rise 197 ERPM (criterion 300), worst settle 0.227 s, current <= 4.8 A. speed 0: 2 runs, 4 of 5 stops failing, rebound up to 7871 ERPM. Moatasem confirmed T1 complete in writing (16 Nov) | F-01 CRITICAL, open | **F-01 mitigated on the stand with brake current (5 A).** **speed 0 must never be used as a stop command** (it is the standard `ackermann_to_vesc` stop, so the standard converter must not drive the motor). Not closed yet: brake on stop exists only in the test-only T1b converter (permanent converter pending, handoff section 5); ground stopping distance unknown (T4); VESC Tool checklist rows 11 and 14 unread; the small +0.16 s rise in 4 of 10 stops at 1.0/1.5 m/s means the speed-0 cause (C-7) is suppressed, not removed. |
| T2 servo range: PS4 teleop, same YSIDO servo, `config/vesc.yaml` clip 0.1-0.6 (Moatasem) | 2026-10-06 to 2026-10-12 | `/sensors/servo_position_command` reached 0.1 and 0.6 with no binding seen or heard. That topic is the command vesc_driver echoes after its clip, not a measured position: the evidence is the operator's eyes and ears. Before the steering-play fix (14 Nov), offset then 0.361, so the linkage may differ slightly now | - | Supports widening the T2 range; does not replace T2. |
| T2 servo range: hand check on `t2_servo_only.launch.py`, clip 0.15-0.55 (Moatasem, `ros2 topic pub`, not the sweep tool) | 2026-11-18 | At 0.15 and 0.55 both wheels still moved freely: no end stop, no buzzing | Sweep approved range 0.22-0.455 | Launch clip ceiling 0.15-0.55 -> **0.10-0.60** (`d8202be`). |
| Servo horn travel estimate (Moatasem, not measured) | 2026-11-19 | Horn travel inside the car ~60 deg total (~+/-30 deg from centre). VESC firmware maps servo 0..1 to 1000-2000 us (`conf_general.h` SERVO_OUT_PULSE_MIN/MAX_US); if the YSIDO turns 180 deg over 500-2500 us, 0..1 = ~90 deg, so +/-30 deg = 0.320 -/+ 0.33: left limit below 0.0 (unreachable), right limit near 0.65. Consistent with both checks above. The 180 deg servo span is an assumption (datasheet value to confirm) | - | Mechanical limit is the linkage, not the pulse range; the VESC cannot exceed the servo's 500-2500 us. |
| **T2 approved servo range** (`vehicle_params.yaml` `t2`) | 2026-11-19 | **0.10-0.60** (source: Oct teleop + 18 Nov hand check), equal to the launch ceiling, so the whole T2 runs in one launch with clip 0.10-0.60. Short first visit (0.8 s) and the binding prompt stay on every point | 0.22-0.455 | Moatasem's decision. Any binding or buzz still ends that side at the previous point. |
| T2 sweep, steering limits (`20261119_134151_t2_servo_sweep`) | 2026-11-19 | **Left last safe point servo 0.10** (wheels 15 / 20 deg, ~17 deg); mechanical stop just beyond, **~0.08** (Moatasem). Right: still free at 0.60 (wheels -19 / -11 deg), side ended at the 0.60 clip | Approved range 0.10-0.60 | Left clip stays **0.10**. **Rule: each side's driving clip = its last free point, one step before the mechanical stop.** Right limit: short extension test (`t2_servo_sweep.py --extend-right`, 0.02 steps from 0.62, launch clip max raised to 1.0), linked to this run. |
| F-14 duplicate node name | 2026-11-14 | One rf2o process (pid 6442), `rf2o_publisher_count=1`, but `/rf2o_laser_odometry` listed twice. Cause (rf2o source): `CLaserOdometry2DNode` (an `rclcpp::Node`) holds a member `CLaserOdometry2D rf2o_ref` that is itself an `rclcpp::Node`; the launch remap `__node:=rf2o_laser_odometry` renames both. The inner node has no publishers or subscriptions | Open (MEDIUM) | **Resolved, harmless.** `preflight.sh` allows exactly this duplicate only with a single rf2o process; any other duplicate fails. rf2o not modified (official package). |
| Front LiDAR on the stand (mux status, disarmed) | 2026-11-14 | `front_valid_fraction` 0.825, nearest 4.43 m | - | Note for T1: MANUAL needs >= 0.75; margin is small. Status shows `FRONT_UNKNOWN` only because STOP mode evaluates the AUTO threshold 0.90. |

## T1 / T1b comparison (stand)

Generated by `python3 tools/real_car_tests/analyze.py --table` from every analysed armed run; regenerate
after each run. Rebound = largest rise of |ERPM| above its running minimum after the zero, counted once
|ERPM| has fallen to half its value at the zero (same definition live and offline). A stop counts as
steady only from >= 80 % of the step ERPM. Pass = >= 5 steady stops, no stop > 300 ERPM rebound or
unstopped, VESC fault 0.

| Run | Method | Speed m/s | Steady / measured stops | Stops failing | Max rebound rise (ERPM) | Max rebound peak (ERPM) | Worst settle < 100 ERPM (s) | Max current after zero (A) | Max tach after zero | Verdict |
|---|---|---|---|---|---|---|---|---|---|---|
| `20261114_145433` | speed0 | 0.50 | 0 / 3 | 1 | 1774 | 1969 | 0.513 | 11.7 | 73 | FAIL |
| `20261114_150601` | speed0 | 0.50 | 2 / 5 | 4 | 7644 | 7871 | 0.386 | 15.6 | 185 | FAIL |
| `20261114_150933` | brake5A | 0.50 | 5 / 5 | 0 | 197 | 304 | 0.149 | 3.7 | 20 | PASS |
| `20261114_152149` | brake10A | 0.50 | 4 / 5 | 0 | 193 | 243 | 0.130 | 3.4 | 25 | INCOMPLETE (4 steady) |
| `20261114_152526` | brake10A | 0.50 | 5 / 5 | 0 | 246 | 251 | 0.123 | 4.2 | 20 | PASS |
| `20261114_152830` | brake5A | 1.00 | 4 / 5 | 0 | 178 | 206 | 0.181 | 4.8 | 53 | INCOMPLETE (4 steady) |
| `20261114_153922` | brake5A | 1.00 | 4 / 5 | 0 | 130 | 177 | 0.161 | 4.7 | 54 | INCOMPLETE (4 steady) |
| `20261114_154158` | current0 | 0.50 | 5 / 5 | 0 | 78 | 91 | 0.197 | 1.2 | 32 | PASS |
| `20261116_215116` | brake5A | 1.00 | 5 / 5 | 0 | 152 | 286 | 0.179 | 4.4 | 56 | PASS |
| `20261116_215634` | brake5A | 1.50 | 5 / 5 | 0 | 114 | 220 | 0.227 | 4.8 | 103 | PASS |

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

- **Never use speed 0 as a stop command (T1, 16 Nov).** The standard `ackermann_to_vesc` stops by
  sending 0 ERPM; on the stand that rebounded up to 7871 ERPM. Until the permanent converter exists,
  the motor is driven only by the real-car test stack with the T1b converter and `stop_method:=brake`.

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
- **Servo horn is off-centre (for Zaid):** straight ahead is servo 0.320 = **1320 us**, not 1500 us
  (VESC maps 0..1 to 1000-2000 us). Left has at most 0.32 of servo travel, right 0.68. Re-mounting the
  horn so straight is near 0.5 would give symmetric travel; **T2 must be redone after that** (offset,
  gains and limits all change).
- The BMI160 node is commented out in `sensors.launch.py`, and no launch in the repo publishes
  `base_link -> imu_link`. The test launch publishes it (PROVISIONAL) until we know how it was done on 14 Nov.
