# F1TENTH UJ — Real-car baseline (standards as of 14 Nov 2026)

> Cleaned and corrected version of `F1TENTH_UJ_COMPLETE_ENGINEERING_LOG_2026-11-14.md`.
> These are the **current standard values**. A newer measured value replaces a value here only when
> it is written down with its test name and date.
> Labels: **CONFIRMED** = measured on the real car · **SIM ONLY** · **PARTIAL** · **UNRESOLVED** · **PLANNED**.
> Speeds from `/control/status` or `/exploration/drive` are *requests*, not measured ground speed.

---

## 1. Repositories and rules

| Item | Value |
|---|---|
| Physical repo | `EngZaq/f1tenth_uj`, Jetson path `~/f1tenth_uj`, integration branch `zaq/hardware-integration` |
| Simulator repo | `EngZaq/f1tenth_uj_sim`, branch `moatasem/ekf-experiments` (commits 47497bf, c10fe42 and later) |
| Physical evidence root | `~/f1tenth_hw_evidence/` on the Jetson |
| Rules | Never commit to `main`. Run `git status --short` before switching branch or pulling: the Jetson repo has teammates' **uncommitted local files**. Commit only your own files, by name (never `git add -A`). Do not fork or edit official F1TENTH packages. |

The simulator map `results/missions/autonomous_20261103_114924_818822/` is a **simulation map**. Never use it on the real car.

---

## 2. Vehicle standards (use these values)

| Item | Standard value | Status / note |
|---|---|---|
| Jetson | Orin Nano 8 GB, Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=42`, `rmw_cyclonedds_cpp` | CONFIRMED |
| Wheelbase | **0.260 m** | CONFIRMED (replaces the old rough 0.25 m) |
| Wheel diameter | 0.065 m | CONFIRMED |
| Body length × width | 0.345 × 0.197 m; front bumper at x ≈ 0.305 m in `base_link` | CONFIRMED |
| `base_link` | Centre of the rear axle | Decision |
| LiDAR | LakiBeam L1, frame `laser`, at **(0.215, 0, 0.150) m**, yaw 0; 1440 beams, 0.25°, ~29.6 Hz | CONFIRMED (replaces the old x = 0.195 m) |
| IMU | **BMI160 on the car**, `/imu/data_raw` ~100 Hz, mounted upside down: `base_link -> imu_link` = (0.091, 0.057, 0.046) m, roll 180° | CONFIRMED (stream and sign) |
| VESC | Makerbase MINI 6.7 Pro, DYS D3542 1450 KV, 7 pole pairs, **sensorless**, 3S battery, ~11.2 V idle | CONFIRMED |
| `speed_to_erpm_gain` | **8414** (check: 1 m/s ÷ (π·0.065) × 60 × 4.09 × 7 = 8412 ✓) | Standard; real ground scale not yet measured |
| Steering conversion | gain **−0.50**, straight-ahead offset **0.320** | CONFIRMED 14 Nov after the steering-play fix (replaces 0.361) |
| Steering play | Fixed mechanically (14 Nov) | Re-check under load in T3 |
| Mux limits (physical repo) | speed ≤ ~0.35 m/s, steering ±0.10 rad → ideal minimum radius 0.260 / tan 0.10 = **2.59 m** | Software limit, not mechanical. To be raised per test (launch parameter) after T1 passes |
| Target test speeds | Ground: 0.5 → 1.0 → 1.5 m/s (sim validated to 1.5 m/s), limited by the free straight length | Decision 14 Nov |
| Real steering range / minimum radius | **UNKNOWN** | PLANNED (test T3) |
| Braking / stopping distance | **UNKNOWN** | Blocked by F-01 |

**TF ownership now:** SLAM Toolbox `map -> odom`; RF2O `odom -> base_link` (`/odom_rf2o`, ~20 Hz); static `base_link -> laser`, `base_link -> imu_link`. VESC odometry is `/odom_vesc_observe` with `publish_tf=false`. EKF runs in shadow only (no TF). Each TF edge has exactly one owner.

**Command path:** `keyboard GUI / explorer -> mux (/manual/drive, /control/request_mode, /joy) -> /drive -> ackermann_to_vesc -> /commands/motor/speed -> vesc_driver`. The mux checks who owns `/drive` and `/commands/motor/speed`. **No extra publisher or subscriber on those two topics.** `REAL_ACTUATION_DISABLED` is the intended software interlock. It is **not** a physical emergency stop.

---

## 3. Measured real-car results (useful as reference values)

| Test (date) | Result |
|---|---|
| LiDAR static (11 Nov) | 29.64 Hz, 1440 ranges, ~93 % valid returns in front ±20°. One old bag had 768/1248-ray scans (F-09) |
| RF2O 0.60 m straight push (11 Nov) | forward 0.527 m, **lateral −0.245 m**, yaw −0.7°; SLAM forward 0.570 m |
| VESC vs LiDAR, 0.600 m tape push (11 Nov) | LiDAR 0.595, RF2O 0.538, VESC 0.404 m. The VESC missed the first ~0.15 m (sensorless start-up). While moving, LiDAR 0.438 ≈ VESC 0.402 m. **A hand push is not a powered calibration.** |
| 90° hand turns (11–12 Nov) | Gyro +85.2° / −87.2°, RF2O +83.7° / −84.9°, SLAM −89.4° (true angle not measured) |
| Powered pulses 0.24 m/s, 1 s (12 Nov) | Peak 2,062 ERPM (expected 8414 × 0.24 = 2,019 ✓), phase current 18.7 A, no faults. Ramp needs 0.555 s to reach 0.24 m/s, so 1 s pulses are useless for steady-speed calibration. |
| **Post-STOP rebound (12 Nov)** | After a zero command: ERPM falls, then **rises again ~0.16–0.27 s later**, worst **3,911 ERPM** (≈ 0.46 m/s, above the 0.35 m/s cap), +18 A phase current, tachometer keeps counting, ~0.1–0.2 m extra travel. Driver sent 0 correctly. The serial-write fix did **not** change it. |
| Right-steer pulses −0.06 rad (12 Nov) | Much less yaw than the bicycle model predicts, abnormal sound under load (F-02) |
| Open-area moving shadow test (14 Nov, unpowered, pushed by hand, 51.6 s) | Scan 28.5 Hz, RF2O 19 Hz, explorer proposals 19.3 Hz, 27.6 % nonzero; 50/94 diagnostics `central_front_invalid_and_no_observed_candidate`; candidate compute p95 44 ms; partial map saved. See C-4 about the map size. |
| Idle VESC telemetry (14 Nov) | 0 ERPM, 0 A, 11.2 V, fault 0 |

---

## 4. Corrections made to the original log

| # | Original statement | Correction |
|---|---|---|
| C-1 | Sim: "max planned speed cap 0.8 m/s", "A3 AMCL" as the result to port | **Outdated.** Test 6 (14 Nov): 0.8 m/s was the limit of the **old** driver guard at constant speed. Test 7 (14 Nov): closed loop at **1.5 m/s** (5/5 laps, max cross-track error 8.4 cm). Localization held in shadow up to ~3 m/s. Decision for the car: **team AMCL laser model A0** (`sigma_hit 0.10`, `z_rand 0.3`) first. A3 only if real maps are very clean. Check that `config/fusion_amcl_moatasem_A3.yaml` exists before citing it. The config in my file list is `config/fusion_amcl_moatasem.yaml`. |
| C-2 | F-03 "EKF covariance explosion → FAIL for localization", HIGH | **Severity overstated.** An odometry EKF with no absolute position input **always** has growing x, y, yaw covariance (dead reckoning). AMCL fixes the pose through `map -> odom`. Simulation EKF C fuses the **same signals** (VESC vx + gyro yaw rate) and its covariance also grows, yet it owned `odom -> base_link` for 1.5 m/s laps. The real issue is **vy unobserved**: fuse VESC `vy = 0` (nonholonomic constraint) and use `two_d_mode: true`, then judge the filter on its **mean** (pose and yaw vs RF2O/SLAM), not on P alone. Keep it shadow-only until that comparison passes. → **MEDIUM.** |
| C-3 | Standard steering offset 0.361 | **Resolved 14 Nov:** after fixing the steering play, straight ahead = **0.320**. Every active converter (standard `ackermann_to_vesc` and the trial launch) must use 0.320; check the active parameters in T0. |
| C-4 | Open-area test: "RF2O path 32.7 m, net 21.6 m", map 733 × 1346 cells | 733 × 1346 × 0.05 m = **36.7 × 67.3 m map** and 21.6 m net in 51.6 s of hand pushing. That is **suspicious for an indoor place**: it may be RF2O drift or map smearing. Verify with a tape-measured path before using this map or these distances. |
| C-5 | "0.3302 m assumption" in Moatasem's code | It is the **f1tenth_gym vehicle wheelbase** (lf 0.15875 + lr 0.17145). Other sim-only geometry: rear axle → CoG 0.17145 m, sim laser x 0.27145 m. All of them must become parameters set to the real values (wheelbase 0.260, laser x 0.215). |
| C-6 | (missing) IMU calibration | The BMI160 bench calibration (gz bias −0.067 °/s, noise R 5.4e-7 (rad/s)², scale 1.009) was done on the **ESP32 bench board**. If the car's BMI160 is a different chip, repeat a 10-minute static log and a scale test **on the car**. Re-measure noise with the motor running anyway. |
| C-7 | F-01 root cause unknown | Still unknown. A **testable hypothesis to add**: the VESC is in **speed (RPM) PID mode, sensorless**. Commanding 0 ERPM asks the speed PID to hold 0 rpm while the sensorless observer has no lock at low speed, which can cause overshoot or rebound. Compare on the stand: speed 0 vs **brake current** (`/commands/motor/brake`) vs current 0, and read the speed-PID, ramp, timeout and brake settings in VESC Tool. |

Removed from the original: chat artifact lists, resolved launch/display/copy problems (F-10, F-11, F-12, F-15), team-role table, repeated conclusions.

---

## 5. Open faults (priority order)

| ID | Severity | Fault | What closes it |
|---|---|---|---|
| F-01 | **CRITICAL** | Motor ERPM/current rebound after zero command | Stand test T1: a stop method with no rebound in ≥ 10 repeats, plus a working independent cutoff (battery disconnect / kill switch) |
| F-02 | MEDIUM | Steering play fixed (14 Nov); noise under load and left/right asymmetry not re-checked | Servo sweep with measured wheel angles (T2) + loaded circles (T3) |
| F-17 | MEDIUM | Standard converter vs trial launch may still hold different conversions | T0: both use offset 0.320 / gain −0.50; one launch stack |
| F-06 | HIGH | Steering limited to ±0.10 rad (radius 2.59 m) | Measured real steering range (T2) → choose the mux limit |
| F-13 | MEDIUM | Keyboard GUI latches `MUX_LEFT_MANUAL_MODE` | `/control/status` captured at the moment of the latch |
| F-14 | MEDIUM | Duplicate node-name warning with VESC launch | `ros2 node list -a`: one driver, one converter |
| F-03 | MEDIUM | Shadow EKF: vy unobserved (see C-2) | Fuse vy = 0, two_d_mode; mean pose/yaw vs RF2O/SLAM on real bags |
| F-04/05 | MEDIUM | Explorer blocks often; 44 ms compute | Bag analysis of blocked intervals (planning team) |
| F-07 | MEDIUM | VESC odom misses slow motion (sensorless) | Powered straight-line calibration (T4), after F-01 |
| F-08 | MEDIUM | RF2O lateral drift on a straight push | Repeat with taped straight line |
| F-09 | LOW | Old scans with 768/1248 rays | Log scan lengths in every new bag |

---

## 6. Status (14 Nov)

| Subsystem | Status |
|---|---|
| LiDAR, RF2O, SLAM (loop closing OFF), partial map save | PASS (accuracy PARTIAL) |
| BMI160 stream and frame sign | PASS |
| Shadow EKF | Runs; vy fix + real-data comparison PLANNED (C-2) |
| VESC telemetry | PASS idle |
| **Powered driving / stopping** | **NO-GO (F-01)** |
| Keyboard GUI (`continuous_keyboard_mux.py`) | PASS self-test, disarmed only |
| AMCL + profiled Pure Pursuit | SIM ONLY (validated to 1.5 m/s) |
| Autonomous lap on the real car | NOT DEMONSTRATED. **No closed track available.** |

---

## 7. Standard terminal setup (Jetson)

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Read-only sensors + RF2O + SLAM (propulsion isolated):
```bash
ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=false enable_slam_loop_closing:=false
# with VESC telemetry: enable_vesc:=true  (command inputs are remapped to /observe_only_unused/*)
```
Never run the observe-only launch and an active VESC launch at the same time. Before each run, check that no old ROS processes are still running.
