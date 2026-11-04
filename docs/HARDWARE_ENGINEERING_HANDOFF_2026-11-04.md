# F1TENTH UJ — Physical Hardware Integration Engineering Handoff

**Report date:** 2026-11-04 (Jordan time)  
**Repository:** [EngZaq/f1tenth_uj](https://github.com/EngZaq/f1tenth_uj)  
**Working branch:** `zaq/hardware-integration` — **do not push to `main` without review**  
**Simulation reference:** [EngZaq/f1tenth_uj_sim](https://github.com/EngZaq/f1tenth_uj_sim), `main` (reference checkout used for offline planning)  
**Scope:** Physical platform bring-up, LiDAR / RF2O / SLAM, physical geometry, offline planning, VESC telemetry, kinematic conversion verification, and bench-only command safety gate.

> **Safety / readiness:** This report records software and stationary/hand-moved tests. It does **not** certify the vehicle for powered motion or autonomous driving. At close of session, the independent hardware emergency stop had **not** been installed/tested; controller-level stale-command behavior and steering/motor calibration remain unverified. The bench gate must **not** be connected directly to VESC/servo/teleop actuators.

## 1. Executive summary and disposition

The Jetson successfully reads the LakiBeam LiDAR, runs RF2O laser odometry and SLAM Toolbox, saves occupancy maps, communicates with the VESC over USB and reads 50 Hz core telemetry. The ROS VESC odometry conversion and Ackermann-to-VESC command conversion were independently checked using *synthetic inputs* and passed. Physical vehicle dimensions were measured and reflected in the hardware configuration. The simulator's centerline extraction and route-optimization modules were copied to an **offline physical-planning workspace**, with physical geometry substituted for simulator dimensions. A reference simulation route passed nominal full-body collision/steering audits; a newly extracted route was independently optimized and also passed nominal checks.

The hardware-stop-aware **offline, bench-topic-only** Python command gate passed its own logic self-test and a ROS 2 integration test after correcting an initial test-harness timing problem. The gate is explicitly a prototype, not a production safety system.

**Important distinctions:** (a) Short hand-movement tests cannot establish LiDAR odometry scale against precise ground truth; (b) a genuine physical map does not yet contain a closed track; (c) route geometry passes are against a **simulation-derived map**, not a real track; (d) gains, steering endpoints and speed envelopes are not physically calibrated.

## 2. Platform, software and ROS conventions

| Item | Recorded configuration |
| --- | --- |
| Jetson | NVIDIA Jetson Orin Nano 8 GB |
| OS / ROS | Ubuntu 22.04, ROS 2 Humble, ARM64 |
| Workspace | `/home/ftenth/f1tenth_uj` |
| User | `ftenth` |
| ROS domain | `ROS_DOMAIN_ID=42` |
| Middleware | `rmw_cyclonedds_cpp` |
| Shell | Bash — source `setup.bash` and `local_setup.bash` (not Zsh setup files) |
| LiDAR | LakiBeam L1, Ethernet `192.168.198.2`, host `192.168.198.1`, port `2368` |
| VESC USB | `/dev/ttyACM0`, STMicroelectronics / ChibiOS virtual COM |
| Persistent VESC name | `/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00` |
| Motor | DYS D3542, 1450 KV, 14 poles (7 pole pairs), user-confirmed model |
| IMU | BMI160 planned; not used in the accepted tests |
| TF convention | `base_link` at center of rear axle; X forward, Y left, Z up |

Shell initialization:

```bash
source /opt/ros/humble/setup.bash
source /home/ftenth/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`robot_localization` was installed for later EKF work, **not** used as a motion-estimation authority in these tests.

## 3. Measured physical car geometry

All dimensions are user-provided tape measurements except those explicitly labeled as assumptions.

| Parameter | Value | Evidence/status |
| --- | ---: | --- |
| Wheelbase | 0.260 m | Measured axle center to axle center |
| Tire diameter | 0.065 m | User measurement |
| Body length | 0.345 m | User measurement |
| Body width | 0.197 m | User measurement |
| Rear axle to frontmost point | +0.305 m | Measured |
| Rear axle to rearmost point | -0.040 m | Derived from total length |
| Body half-width | 0.0985 m | Derived |
| LiDAR X relative to rear axle | +0.215 m | Measured |
| LiDAR Y | 0.000 m | Centered |
| LiDAR nominal yaw | 0 rad | Points forward; fine alignment unverified |
| LiDAR Z | 0.150 m | **Prior configuration estimate, not remeasured** |
| Final drive ratio | 4.092 motor rev/wheel rev | Previous documented calculation; gear tooth count still unverified |
| Provisional steering limit | 24 degrees | Planning test parameter, **not physically validated** |

Nominal body rectangle relative to rear-axle `base_link`: `x in [-0.040, +0.305] m`, `y in [-0.0985, +0.0985] m`. Verify that these include every tire, protrusion, mounting plate, battery and sensor before treating it as a final collision body.

Analytical ERPM magnitude (assuming 14 motor poles and 4.092 reduction):

```text
ERPM / (m/s) = (60 × 7 × 4.092) / (pi × 0.065) ≈ 8417
```

Configured magnitude `8414` is close to this prediction, but neither the gear tooth counts nor motor/wheel speed has been independently calibrated.

### Local geometry changes on the Jetson (not automatically pushed by this documentation commit)

- `src/f1tenth_bringup/launch/tf.launch.py`: `lidar_x` default changed **0.195 → 0.215**.
- `src/f1tenth_bringup/config/vesc_to_odom.yaml`: `wheelbase` changed **0.25 → 0.26**.
- New `src/f1tenth_bringup/config/physical_vehicle.yaml`: geometry and documented drivetrain assumptions; calibration flags remain false.
- New `tools/physical_geometry.py`: reads physical YAML and defines `WHEELBASE`, `BODY_FRONT`, `BODY_REAR`, `BODY_HALFWIDTH`, `LASER`.
- `colcon build --symlink-install --packages-select f1tenth_bringup` **passed** (1 package; ~1.60 s). Geometry consistency assertions **passed**.

The original experimental SLAM/RF2O bag files used the then-current laser offset (0.195 m). Their results should not be silently combined with post-change runs as if extrinsics were identical.

## 4. LiDAR bring-up and calibration evidence

- Ethernet sensor connection: `192.168.198.1/24` → `192.168.198.2`; preliminary ping **4/4**, about **0.336 ms**.
- `ros2 launch f1tenth_bringup sensors.launch.py` publishes `/scan` at approximately **29.64 Hz**.
- LaserScan: **1,440** angle bins; reported span `[-pi, +pi]`, angular increment **0.004363323 rad** (0.25 degrees). One sampled scan had **1,016/1,440** usable returns (~70.6%); rear blind/invalid region is expected from the physical FOV, not proof of 360-degree visibility.
- Approx. 0.5 m static reference: mean **0.4988 m**, sample std **0.00101 m**, 50 samples.
- Approx. 1.0 m static reference: mean **1.0016 m**, sample std **0.00088 m**.
- These establish repeatability against approximate references, **not metrology-grade absolute accuracy**.
- Static TF `base_link → laser` originally (0.195, 0, 0.150) m, zero rotation, later X updated to 0.215 m from user measurement. Height and fine yaw remain to be checked.

## 5. Shadow-mode exploration and RF2O

The physical explorer was run with **drive disabled**:

```bash
python3 tools/exploration/exploration_controller_v2.py --ros-args \
  -p platform:=physical -p enable_drive:=false \
  -p scan_topic:=/scan -p proposal_topic:=/exploration/drive
```

It produced shadow-mode proposals (`/exploration/drive` about 20 Hz, example steering -0.054 rad and nominal speed 0.15 m/s; some proposals reached 0.28 m/s). No confirmed physical `/drive` actuator commands were issued. This verifies that proposals are generated, **not** that obstacle avoidance has been validated or that the car can drive itself.

RF2O was launched standalone:

```bash
ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args \
  -r __node:=rf2o_laser_odometry \
  -p laser_scan_topic:=/scan \
  -p odom_topic:=/odom_rf2o \
  -p publish_tf:=true \
  -p base_frame_id:=base_link \
  -p odom_frame_id:=odom \
  -p 'init_pose_from_topic:=""' \
  -p freq:=20.0
```

**Bash quoting note:** The empty string parameter must be quoted as above; unquoted `-p init_pose_from_topic:=''` caused ROS argument parsing failure. Do not run a second `odom → base_link` TF broadcaster. For some stationary tests, `publish_tf:=false` was used instead. A duplicate ROS node name appeared in `ros2 node list` due internal RF2O behavior, but process and publisher inspection confirmed one odometry publisher.

- RF2O output: approximately **20 Hz**; ~30.6 ms processing per nominal 50 ms period.
- 60-second stationary audit: **1,201 samples**, final 2D drift **0.00587 m**, max deviation **0.00601 m**, yaw drift **-0.1289 degrees**.
- These are stationary results, not absolute-motion accuracy measurements.

## 6. SLAM Toolbox and real-map experiments

SLAM Toolbox async mode was started with a **temporary copy** of `src/f1tenth_bringup/config/slam_toolbox.yaml`, using `do_loop_closing: false` for the experiment. **The repository default YAML was not necessarily changed.** Runtime parameters were checked: `odom` / `base_link` / `map` / `/scan`. The TF chain `map → odom → base_link → laser` became valid after initial startup transients. Map saver:

```bash
ros2 run nav2_map_server map_saver_cli -t /map -f /path/to/output/map_name
```

- First stationary real map: **157 × 92 cells**, 0.05 m/cell = **7.85 × 4.60 m**, with approximately 3,183 free, 165 occupied, and 11,096 unknown cells (about 23.18% known).
- Saved at `/home/ftenth/f1tenth_hw_evidence/first_real_map/static_map_v1.{yaml,pgm}`.
- Moving map bag: `/home/ftenth/f1tenth_hw_evidence/moving_map_20261104_100428/rosbag`.
  - **28.47 s**, **3,413 messages**, **844 scans**, **569 RF2O odom**, **6 occupancy maps**, **1,993 TF**, **1 TF static**.
  - Saved map: `moving_map_v1.{pgm,yaml}`, **225 × 347 cells at 0.05 m** = **11.25 × 17.35 m bounding box**, not evidence of that travel distance.
  - Offline RF2O audit: initial XY (-5.151, -18.030) m; final (-5.306, -18.027) m; net displacement **0.155 m**; integrated absolute step sum **1.545 m**; net yaw **-10.87 degrees**; max individual odom step **0.015 m**. Those absolute positions are an inherited odom origin, not distance traveled during that run.
  - Earlier raw `map → odom` TF translation step reached **0.251 m**; **robot-position correction at a fixed odom pose** was only **0.028 m maximum** among 1,424 TF samples; **0** events over 0.10 m. Rotational lever arm explains why raw transform translation alone was misleading.
- Constrained wired straight-line bag: `/home/ftenth/f1tenth_hw_evidence/straight_1m_20261104_101417/rosbag`.
  - **35.30585 s**, **4,234 messages**, 1,047 scans (~29.66 Hz), 706 RF2O odom (~20 Hz), 8 maps, 2,472 TF and 1 TF static.
  - Averaged first/last stationary RF2O poses: forward **+0.1359 m**, lateral **-0.0834 m**, net **0.1595 m**, heading change **+1.358 degrees**, accumulated path-length sum **1.0368 m**, largest step **0.01219 m**.
  - **Critical ground-truth correction:** Although the script initially assumed a 1.000 m reference, the user later confirmed that a tethered power connection restricted actual movement to roughly **10–15 cm**. The script's apparent **-86.41% error is invalid and must not be cited as an RF2O accuracy estimate**. Forward estimate is plausible, but the lateral error is large and needs a longer accurately measured motion trial.
  - Summing 706 short noisy steps can produce an inflated path-length value; **do not equate step-sum with ground truth**.

**Mapping outcome:** LiDAR + RF2O + SLAM integrate and can save maps. A closed physical track map has **not** been collected. Moving-scale calibration and loop consistency remain unverified.

## 7. Offline planning port and validation

Source algorithms copied from simulator `main` into `tools/physical_planning/`:

1. `extract_robust_centerline.py`
2. `optimize_vehicle_route.py`
3. `prepare_vehicle_route.py`
4. `vehicle_footprint_audit.py`

Physical adaptation:
- New `tools/physical_geometry.py` reads `physical_vehicle.yaml`.
- Optimizer uses physical body X/Y bounds and **0.26 m wheelbase** instead of simulator hardcoded 0.3302 m.
- Preparation and footprint audit reference physical geometry.
- Dependency `python3-skimage` was missing initially and installed.
- Ubuntu Pillow compatibility patched by replacing `Image.Resampling.NEAREST` with compatible accessor `getattr(Image, "Resampling", Image).NEAREST`.
- Keep unknown cells blocked; do **not** lower occupancy clearance just to force a loop. These are offline modules, not production controllers.

### 7.1 Correct rejection of incomplete physical map

On `moving_map_20261104_100428/moving_map_v1.yaml`:
- 5,645 originally observed free cells; 1,665 remained after **0.30 m** free-space erosion.
- No substantial enclosed interior and no closed centerline; extractor rejected with `No closed loop remains`.
- **Correct outcome**: do not fabricate unexplored space.

### 7.2 Extraction on known complete simulated SLAM map

Reference fixture in simulator repo:
`results/missions/20261028_112407/new_track.yaml`, image `new_track.pgm`.

- 19,262 observed free cells; 10,593 remained after 0.30 m erosion.
- Skeleton pixels 706; 690 after spur pruning; 2 candidate cycles; selected cycle length **39.97 m** before smoothing/resampling.
- Smoothing sigma 3, spline, 0.10 m spacing: first valid candidate.
- **377** points, output closed-route length approximately **37.67 m**, minimum observed free-space clearance **0.300 m**.
- Files created:
  - `/home/ftenth/f1tenth_hw_evidence/physical_planner_validation/physical_centerline_centerline.csv`
  - `.../physical_centerline_speed_profile.csv`
  - `.../physical_centerline_preview.png`
- First run reached image export and failed on Pillow `Image.Resampling`; compatibility fix resulted in **successful full export**.

### 7.3 Physical footprint audit

| Candidate route | Waypoints | Min nominal full-body clearance | Max required steering | Result |
| --- | ---: | ---: | ---: | --- |
| Newly extracted raw centerline | 377 | 0.20 m at waypoints | 36.8287 degrees | **FAIL** provisional 24-degree steering gate |
| Simulator's `new_track_mission_forward_feasible.csv` | 377 | 0.20 m at waypoints | 16.1446 degrees from CSV curvature | **PASS** with 0.10 m margin |
| Same feasible route, independent dense sweep | 3,016 interpolated poses | **0.180 m** min | **15.81 degrees** independently calculated from XY | **PASS** nominal offline gate |
| Physical optimizer's own `physical_optimized_route.csv` | 377 | **0.20 m** at waypoints | **23.6124 degrees** | **PASS** 0.15 m body margin and nominal 24-degree limit |

The optimizer's `prepare_vehicle_route.py` accepted **sigma=0** on the first attempt, generated **37.650 m** closed route, 377 waypoints, max curvature **1.6813 1/m**, output speed profile **0.300–0.800 m/s**. Output and report:
- `/home/ftenth/f1tenth_hw_evidence/physical_planner_validation/physical_optimized_route.csv`
- `.../physical_optimization_report.json`
- `.../physical_optimized_audit.json`

**Caution:** On the newly optimized physical route, the maximum required 23.61 degrees is only **0.39 degrees below provisional limit**; there is no verified steering reserve. The simulator's older feasible route has much lower nominal steering demand. Planner internal acceptance includes intermediate-sample clearance checks, but independently recorded **0.180 m dense-sweep minimum belongs to the older simulator-optimized route**, not to the newly optimized route.

**Safety interpretation:** All geometry results are nominal, on a simulation map. They do not account for physical map quality, localization error, tracking deviations, latency, tire slip, actual steering limits, real speed control, moving-obstacle clearance or emergency braking. **No route is authorized for physical execution.**

## 8. VESC bring-up and live read-only telemetry

USB identification:
- `lsusb`: `0483:5740 STMicroelectronics Virtual COM Port`.
- `/dev/ttyACM0`: permissions `root:dialout`, user `ftenth` belongs to `dialout`.
- Stable path: `/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00`.
- `ros2 pkg executables vesc_driver` and `vesc_ackermann` returned expected nodes.
- No motor/servo control processes initially.

An isolated VESC driver was launched with **all six motor/servo command subscriptions remapped** to `/vesc_test_unused/...`; the normal actuator topics were not used. This is topic isolation, **not a hardware safety mechanism**.

Firmware handshake:
```text
[vesc_driver]: -=60=- hardware paired 0
[vesc_driver]: Connected to VESC with firmware version 7.0
```

Telemetry on `/sensors/core` (`vesc_msgs/msg/VescStateStamped`): **~50.006 Hz**, input voltage **11.1 V**, motor current **0.0 A**, input current **0.0 A** at reported precision, duty cycle **0.001**, speed **3 ERPM** (near stationary), `fault_code=0`, `controller_id=13`, displacement field 0, `distance_traveled=30` (tachometer-related controller quantity, **not 30 meters**). Driver also advertised `/sensors/imu` and `/sensors/imu/raw`; message presence alone does not establish valid IMU data.

### 8.1 Stationary VESC odometry

Standalone `vesc_to_odom_node` with:
- `use_servo_cmd_to_calc_angular_velocity:=false` (diagnostic only; avoids waiting for uncalibrated servo feedback).
- `publish_tf:=false` (avoid competing RF2O/SLAM odom TF).
- `odom:=/odom_vesc_test`.

Observed: **50.003–50.015 Hz**, x/y initial 0, linear X velocity 0, yaw velocity 0, frame IDs `odom` and `base_link`; runtime `publish_tf=False`.

### 8.2 Synthetic VESC telemetry → odometry conversion

Synthetic test cases (ROS topics isolated at `/bench/...`):
- Stationary: PASS, v=0, yaw rate=0.
- Forward +8414 ERPM: PASS, v=+1.0 m/s.
- Reverse -8414 ERPM: PASS, v=-1.0 m/s.
- +8414 ERPM with servo command 0.311 (0.1 rad steering): PASS, yaw rate **+0.3859 rad/s** using wheelbase 0.26 m.

**4/4 PASS, no physical motion.** Upstream VESC odometry code uses `(-state.speed - speed_offset)/speed_gain`; this explains why hardware `vesc_to_odom.yaml` has `-8414` while `ackermann_to_vesc.yaml` has `+8414`. Do not alter signs without actual wheel direction calibration.

### 8.3 Synthetic Ackermann → VESC conversion

Isolated converter remappings: `ackermann_cmd:=/bench/drive`, speed output to `/bench/output/motor_speed`, servo output to `/bench/output/servo_position`. The first test returned previous-case data due subscriber timing/queue delay. A corrected test waited for exactly one converter, ignored early samples and examined fresh outputs.

| Command | Expected and observed ERPM | Expected and observed servo | Result |
| --- | ---: | ---: | --- |
| Neutral | 0 | 0.361 | PASS |
| Forward +0.25 m/s | +2103.5 | 0.361 | PASS |
| Reverse -0.25 m/s | -2103.5 | 0.361 | PASS |
| Left +0.1 rad | 0 | 0.311 | PASS |
| Right -0.1 rad | 0 | 0.411 | PASS |

**5/5 PASS, software only.** This establishes numerical conversion and ROS topic plumbing, **not actual motor speed, servo endpoints, steering polarity or traction direction**.

## 9. Offline command gate — code and verified behavior

Local files created:
- `tools/safety/physical_drive_gate.py`
- `tools/safety/test_physical_drive_gate.py`

These files live **only on the Jetson until explicitly committed/pushed**. The prototype deliberately uses:
- Inputs: `/bench/raw_drive`, `/bench/heartbeat`, `/bench/arm`.
- Outputs: `/bench/gated_drive`, `/bench/gate_state`.
- No connection to `/drive`, `/commands/motor/*`, `/commands/servo/*`, or actual VESC.
- Defaults: **disarmed**, requires an explicit low-to-high arm sequence with fresh heartbeat and command.
- Command and heartbeat timeout: **0.25 seconds each**.
- Bench test limits: **0.25 m/s** absolute command speed and **0.10 rad** steering. These are **not approved physical driving limits**.
- Fault states include limit violation, command timeout, heartbeat false/timeout; latched fault requires deliberate disarm/re-arm sequence with fresh inputs.
- ROS tick runs at nominal **50 Hz**, publishes zero drive while disarmed/faulted.

Validation:
1. `python3 tools/safety/physical_drive_gate.py --self-test`: **PASS**, default-disarmed, arming, stale command, no auto-rearm, limit rejection, heartbeat-false, heartbeat-timeout. Hardware not involved.
2. First integration test: **FAIL** for insufficient fresh output samples (test-harness timing).
3. Integration test timing corrected to process callbacks promptly; **FINAL RESULT: PASS**, all six scenarios:
   - Starts disarmed: PASS, `DISARMED`, 0 m/s.
   - Explicitly armed with fresh inputs: PASS, `ARMED`, +0.200 m/s on bench output.
   - Timeout stops and latches: PASS, `FAULT_HEARTBEAT_TIMEOUT`, 0 m/s.
   - No automatic re-arm: PASS, same latched fault, 0 m/s.
   - Explicit re-arm: PASS, `ARMED`, +0.200 m/s.
   - Overspeed rejected: PASS, `FAULT_LIMIT`, 0 m/s.
4. No physical actuation was performed.

**Important limitation:** ROS node scheduling, process crashes, DDS failures, disconnections and downstream motor-controller hold-last-command behavior are not solved by this prototype. A production interface requires a proven independent stop and VESC/controller-level stale-command timeout, safe power-on defaults, fault-injection tests and comprehensive logging. **Do not remap the bench gate output into the VESC's real command topics.**

## 10. Evidence index and local artifacts

```text
/home/ftenth/f1tenth_hw_evidence/
├── first_real_map/
│   ├── static_map_v1.yaml
│   └── static_map_v1.pgm
├── moving_map_20261104_100428/
│   ├── rosbag/
│   ├── moving_map_v1.yaml
│   └── moving_map_v1.pgm
├── straight_1m_20261104_101417/
│   ├── rosbag/
│   └── reference.txt  # ORIGINAL 1.0m note was INVALIDATED by later 10–15cm confirmation
├── latest_moving_run.txt
├── latest_straight_run.txt
└── physical_planner_validation/
    ├── physical_centerline_centerline.csv
    ├── physical_centerline_speed_profile.csv
    ├── physical_centerline_preview.png
    ├── new_centerline_audit.json
    ├── optimized_physical_footprint_audit.json
    ├── physical_optimized_route.csv
    ├── physical_optimization_report.json
    └── physical_optimized_audit.json
```

Other timestamped backups and temporary SLAM config copies may exist under `~/f1tenth_hw_evidence/`. Paths are recorded for team access; **the bags and preview artifacts were not uploaded with this Markdown document**. Consider archiving them separately (e.g. release assets, not normal Git history) after checking size and redacting environment-sensitive details. Never imply GitHub has the raw bag evidence unless explicitly uploaded.

## 11. Source changes to commit from the Jetson

Confirmed local pending code/config changes at the time of the session:

```text
M  src/f1tenth_bringup/config/vesc_to_odom.yaml
M  src/f1tenth_bringup/launch/tf.launch.py
?? src/f1tenth_bringup/config/physical_vehicle.yaml
?? tools/physical_geometry.py
?? tools/physical_planning/
?? tools/safety/
```

Do not stage `~/f1tenth_hw_evidence`, temporary bags, large archives, local secrets or dependency caches. Verify actual `git status` before committing. The documentation commit on GitHub is separate from the Jetson working-tree edits.

### Branch-safe commit and push workflow (after docs land remotely)

```bash
cd ~/f1tenth_uj
test "$(git branch --show-current)" = "zaq/hardware-integration" || exit 1
git status --short
python3 -m compileall -q tools/physical_geometry.py tools/physical_planning tools/safety
git diff --check

# Stage exactly intended source files.
git add \
  src/f1tenth_bringup/config/vesc_to_odom.yaml \
  src/f1tenth_bringup/launch/tf.launch.py \
  src/f1tenth_bringup/config/physical_vehicle.yaml \
  tools/physical_geometry.py \
  tools/physical_planning \
  tools/safety/physical_drive_gate.py \
  tools/safety/test_physical_drive_gate.py

git diff --cached --stat
git diff --cached --check
git commit -m "Integrate measured hardware geometry and offline bench validation"

# GitHub documentation may have advanced the same branch:
git fetch origin
git rebase origin/zaq/hardware-integration
git push -u origin zaq/hardware-integration
```

Before staging `tools/physical_planning`, review it to ensure no local intermediate/backup files are included. Never use `git push --force` to resolve conflicts. If a rebase conflict occurs, stop and inspect rather than resetting your work.

## 12. Open risks and unresolved engineering questions

| Issue | State / consequence |
| --- | --- |
| Independent E-stop / battery cutoff | **Not installed or verified**. Mandatory gate before powered motor commissioning. Must be correctly rated; deliberate reset. |
| VESC controller-level command timeout | **Not yet verified**; publishing stale commands or losing ROS may leave actuation unsafe. Must validate with VESC configuration and controlled fault injection. |
| Physical steering calibration | Neutral 0.361, gain -0.50, provisional max 24 degrees **not measured**; sweep limits and mechanical end stops pending. |
| Physical ERPM ↔ speed and polarity | Analytical magnitude ~8417, software conversions pass; physical traction direction and scale still unverified. |
| Drivetrain gear tooth counts | 4.092:1 prior design value, not physically rechecked. |
| LiDAR Z/yaw and full collision body | Unverified fine extrinsics, Z estimate, protrusions. |
| RF2O moving accuracy | Short tethered motion is insufficient; one short trial had conspicuous lateral estimate. Need accurately measured longer path. |
| SLAM loop closing | OFF for initial physical tests; compare controlled ON/OFF later. In prior simulator evaluation, enabling loop closure sometimes produced significant global TF jumps. |
| Physical track topology | No closed real map; centerline extractor correctly rejected incomplete map. |
| Physical localization fusion | IMU unavailable/not calibrated. EKF not launched. Covariances and TF ownership need engineering review. |
| Planning uncertainty budget | Nominal geometry checks only; footprint + tracking + localization + dynamic constraints not jointly bounded. |
| Autonomous controller | Pure Pursuit and exploration shadow proposals are not verified for powered hardware driving. |
| Power/tether | Wired external power limits safe physical motion; establish portable power and cable restraint for mapping tests. |
| Hardware-safe control path | Bench `/bench/*` prototype is deliberately isolated. Do not treat it as production watchdog. |

## 13. Next session: safety-first continuation order

1. **Safely stop and preserve state.** Stop bench nodes and VESC processes, verify no actuator nodes remain, inspect `git status`, and push vetted Jetson code to the same integration branch (after rebasing onto this documentation commit).
2. **E-stop design/install/verify.** Independent latching cut/inhibit suitable for the actual VESC/battery/servo power arrangement. Check fault currents, electrical arcing, connector polarity, mechanical reset and safe return to operation with knowledgeable review.
3. **VESC firmware/controller timeout review.** Verify actual fail-safe behavior in configuration and with controlled test procedures; bench gate is not sufficient.
4. **Secured-wheel stand / low-risk actuator commissioning.** Only after physical stop is validated; protect from unexpected wheel spin and steering movement; test neutral and controlled incremental commands, direction and response, measure servo endpoints. Do not extrapolate validated software command values into safe physical commands.
5. **Reconcile configuration.** Confirm motor pole pairs, ratio, duty/current/ERPM limits, servo center/gain/sign, motor and wheel speed scale. Keep only one TF owner.
6. **Portable-power measured odometry tests.** Straight 1–2 m with precise start/end markers, 90-degree turn, measured rectangle, loop closure. Record bags; compare RF2O, VESC and independent measurements.
7. **SLAM and real track mapping.** New accurate LiDAR TF, unknown treated blocked; obtain complete observed closed circuit; map quality gates and independent closure testing.
8. **Physical planner validation.** Re-run centerline extraction, footprint/dense sweep and actual steering-limit checks on the *real* map with error margins. Don't select a route merely because it passed on a simulator map.
9. **IMU + sensor fusion.** BMI160 bring-up/bias/axis calibration; proper noise measurements and `robot_localization` configuration, ensure no duplicate `odom → base_link`.
10. **Controlled tracking and supervision.** Only after motors, steering, localization, command timeout and independent stopping pass. Begin in a physically isolated and supervised test area; then tune Pure Pursuit/velocity with recorded evidence.

### Suggested status for supervisor

**Completed:** LiDAR comms and repeatability pilot, RF2O stationary odometry, SLAM map production, actual vehicle geometry measurement, offline physical-geometry planner adaptation (on simulation map), VESC firmware/50 Hz telemetry, synthetic forward/reverse/steering conversion tests, and bench-only gate logic plus ROS integration tests.  
**Not completed:** independent hardware stop, controller watchdog, calibrated actuation, accurate moving odometry, closed real track mapping, physical route/vehicle tracking, IMU/EKF integration, and autonomous driving validation.

---

**Repository hygiene:** This is a dated engineering snapshot of what was *observed*, *computed*, and *not yet proven*. Maintain later test results in new dated reports; do not silently overwrite experiment evidence or change conclusions without new measurements.
