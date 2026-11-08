# F1TENTH UJ — Physical Autonomous Racing Platform

**University of Jordan — Mechatronics Engineering Graduation Project**

Hardware repository for the F1TENTH autonomous racing car running ROS 2 Humble on an NVIDIA Jetson Orin Nano. The project is integrating a real LakiBeam LiDAR, RF2O, SLAM Toolbox, VESC drive interface, measured vehicle geometry, simulator-derived offline planning and staged safety validation.

> **CURRENT STATUS — 2026-11-06: FIRST REAL AUTO PROPULSION OBSERVED, FIRST-LAP NAVIGATION NOT YET COMPLETED.** The motor moved under the live Jetson/ROS/VESC command path; the final autonomous attempt latched STOP correctly on an observed front obstacle (0.782 m versus 0.80 m threshold; 72 swept-path blocking returns). SLAM map QoS and AUTO/hold handshaking were fixed, with fresh LiDAR, RF2O and map. Software limits are 0.35 m/s exploration and ±0.10 rad nominal steering (~2.59 m turning radius). Real steering calibration, reliable physical stopping, full first lap and saved-map localization/racing remain unverified. **Read the [Nov 6 engineering analysis and Nov 7 verification plan](docs/PHYSICAL_AUTONOMY_RECOVERY_AND_NEXT_DAY_PLAN_2026-11-06.md).** An optional longer-lookahead candidate planner has been committed but **not Jetson-tested**; it remains opt-in and preserves the independent physical obstacle guard.

## November 8 — Moatasem's newer 21-commit simulation branch, staged physical port

[Simulation findings and physical integration handoff](docs/MOATASEM_SIM_21_COMMITS_PHYSICAL_TRANSFER_2026-11-08.md).

The updated `f1tenth_uj_sim` branch `moatasem/ekf-experiments` includes simulation evidence of two closed-loop AMCL+EKF laps in **5/5 runs** at a simulated 0.8 m/s cap, with ~0.015 m mean true cross-track error. This is on a saved simulated map; it does not certify the end-to-end unknown-track mission or physical car.

The first physical port is a **read-only VESC+IMU Config C shadow EKF** (`config/ekf_moatasem_vesc_imu_shadow.yaml`, `launch/moatasem_ekf_shadow.launch.py`), with `publish_tf=false` so RF2O and SLAM keep their original `odom->base_link` and `map->odom` roles. Its real IMU topic, physical wheel-speed sign/noise, and moving performance remain unverified. A new `tools/validation/physical_mapping_correction_audit.py` checks physical `map->odom` recordings over **2.0 s** gaps after teammate tests demonstrated that the former 0.10 s audit missed loop-closure jumps. All checks are evidence only; map quality requires independent review.

**Do not** auto-merge the sim branch into the physical stack, copy the simulator's 0.1016 m wheel size or noise assumptions, disable RF2O TF until an alternative has been calibrated, or connect the teammate's racing controller directly to the VESC.

---

## November 8 engineering sprint — steering feasibility, STOP evidence and CI

[Full November 8 steering & STOP verification runbook](docs/NOVEMBER_08_PHYSICAL_STEERING_AND_STOP_SPRINT.md).

- New fault-time `last_stop_snapshot` is retained in `/control/status` and in `~/f1tenth_hw_evidence/last_physical_stop.json`, with the actual proposed steering and obstacle geometry. Stop output is published before evidence file I/O.
- New read-only `tools/diagnostics/fit_steering_measurements.py` fits the steering mapping from left/right front wheel angle measurements; it never changes active VESC parameters.
- Experimental path candidate selector now detects CPU overrun and fails closed. It still requires non-actuating shadow testing on the Jetson and physical wheel/track geometry calibration before actual autonomous ground motion.
- Added `.github/workflows/physical-offline-regression.yml` for ROS-free source checks; a commit alone is not proof CI or Jetson tests passed.

## Nov 6 → Nov 7 next tests

- Pull branch `zaq/hardware-integration` and run `python3 tools/exploration/physical_candidate_planner.py`, `python3 tools/diagnostics/test_physical_candidate_geometry.py`, `python3 tools/diagnostics/steering_radius_audit.py`, and mux `--self-test` before attempting to run the updated bringup.
- Observe candidate turns with `real_integrated.launch.py enable_real_actuation:=false enable_autonomous_actuation:=true enable_slam_loop_closing:=true experimental_candidate_planner:=true` and `ros2 topic echo /exploration/diagnostics --once --field data`. This is a **non-actuating** test.
- Check actual steering angle / turning radius and stopping behavior before physical ground driving; then attempt a supervised first lap only if the geometry and stopping gate are satisfied. Set `minimum_travel_m` from the measured real track rather than assuming the 22 m default. No physical full lap or AMCL/Pure Pursuit handoff is yet proven.

---

### Previous status — Nov 5

> **HISTORICAL STATUS — 2026-11-05: DISARMED PHYSICAL AUTONOMY INTEGRATION VERIFIED.** The ROS AUTO hold handshake, real LiDAR exploration proposals and one optimized swept-path precondition all passed on the Jetson with real actuation disabled. The mapping-only real-track rehearsal is running but is waiting for the car to depart the start: **no track has been built, no physical closed loop has been completed, and no real-track centerline has been generated yet**. The VESC Tool settings were deliberately left unchanged. Motor/brake calibration, firmware electrical safeguards and an independent physical stop are unresolved. The experimental §tools/safety/§ prototypes remain bench-only.


## End-of-day hardware milestone — 2026-11-05

**Read the full team handoff:** [Daily engineering report and resume plan](docs/DAILY_ENGINEERING_HANDOFF_2026-11-05.md). The [physical first-motion readiness notes](docs/PHYSICAL_FIRST_MOTION_READINESS_2026-11-05.md) and [no-VESC real-track mapping guide](docs/PHYSICAL_TRACK_MAPPING_REHEARSAL_2026-11-05.md) describe the implemented subsystems and the remaining work.

| Subsystem / observation | Latest grounded result | Interpretation |
| --- | --- | --- |
| Real sensors | LakiBeam ~30 Hz; RF2O ~20 Hz; SLAM publishes /map | Connected; moving-course accuracy not validated |
| Remote-free AUTO state machine | STOP -> AUTO_READY -> AUTO_ACTIVE passed; 0.8 s stable | **Disarmed only**, independent stop not certified |
| Autonomous exploration | Physical /exploration/drive generated 0.075 m/s at −0.10 rad; later 0.146 m/s at −0.0053 rad | Proposals, NOT actual wheel movement |
| Exclusive command graph | /drive and motor command topics had 1 publisher / 1 subscriber each | Snapshot ownership validated |
| Physical swept-path guard | 0 blocking returns; ~80.51% measured path rays; max blind width 0.0113 m | Experimental software precondition PASS, not obstacle safety proof |
| Guard timing on Jetson | Reduced from 78.2 ms to **4.17 ms** in sampled run | Meets configured 40 ms single-call budget for that sample |
| Read-only map rehearsal | /mission/physical_lap_progress reported AWAITING_DEPARTURE | Startup and progress publishing work, no physical lap |
| Partial stationary map | 3,228 known free cells; 203 safe cells; observed closed corridor false | **Not a completed track map** |
| Physical drive / racing | Not executed | Firmware, e-stop, steering, braking still unverified |

### Delivered code

- **ROS-integrated but normally disarmed real hardware launch:** `real_integrated.launch.py`, VESC command selector `physical_drive_mux.py`, remote-free AUTO hold, mode acknowledgments, single-owner /drive and /commands/motor/speed checks.
- **Physical path guard:** `physical_path_guard.py` now uses vectorized Ackermann swept-body geometry, reports blocking hits/unknown sectors/path width and actual `compute_ms`, and stops software acceptance if its computation exceeds budget. Missing LiDAR returns are treated as unknown, never guaranteed free.
- **Real mapping-only rehearsal:** `physical_map_rehearsal.launch.py` uses real LiDAR + RF2O + SLAM with loop closing, then physical first-lap progression and automatic map/centerline/vehicle-route preparation. It intentionally launches **no VESC or drive node**; physically isolate propulsion before manually pushing the car.
- **Documentation/tests:** disarmed AUTO handshake, near-return probe, calibration precheck, ROS-free physical guard regression and full dated engineering report.

### Resume at next session — produce a real track map

**Next practical prerequisite: build a LiDAR-visible closed course.** The initial suggested map/low-speed development layout (not built) is a 7 m outer diameter / 5 m inner diameter loop with ~1 m lane, secured walls at LiDAR height. It is **not** a >5 m/s racing circuit. With propulsion physically isolated, manually push the car around a complete closed lap; the mapping-only pipeline will attempt to create real `new_track.yaml`, `new_track.pgm`, `new_track_centerline.csv` and `new_track_body_optimized.csv`.

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch f1tenth_bringup physical_map_rehearsal.launch.py

Second sourced terminal: `ros2 topic echo /mission/physical_lap_progress --field data`. Inspect results in `~/f1tenth_hw_evidence/physical_mapping_YYYYMMDD_HHMMSS/mission_status.json`. **A COMPLETE result has not occurred yet.** Avoid repeating stationary LiDAR and AUTO-mode tests unless a new failure appears.

### Hardware speed and firmware decision

The project speed goal is **greater than 5 m/s**, subject to mechanical/electrical/control validation. Existing ±1,400 ERPM ROS command clamps are provisional and must **not** be mistaken for final firmware or speed limits. The user requested leaving all VESC Tool parameters unchanged for now; no firmware changes were made. Battery C rating is unknown and current/brake/regen/timeout settings are not approved by the software tests. Do not enable physical actuation without a verified independent propulsion cutoff and physical motor, steering and stopping commissioning.

---

## Real integrated ROS 2 launch (2026-11-05)

A new **real VESC-connected** integration is available on the hardware development branch. It uses actual VESC driver + Ackermann converter, real LiDAR/RF2O/SLAM, the original joystick deadman, and an exclusive command mux. See [real integrated hardware runbook](docs/REAL_INTEGRATED_ROS_HARDWARE_2026-11-05.md). Not autonomous-driving certified.

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
ros2 launch f1tenth_bringup real_integrated.launch.py \
  enable_real_actuation:=false enable_autonomous_actuation:=false
```

The VESC is connected to the **real command path**, but default STOP and no nonzero actuation; the remote-free disarmed AUTO handshake later passed on the Jetson. A separate opt-in live manual commissioning mode is documented **only after independent physical stop and actual firmware current/timeout checks**. The VESC firmware has previously exhibited a 7.93 A startup current; changing ROS speed limits does not enforce phase-current limits.

## Engineering documentation

- **[Complete physical hardware integration handoff — 2026-11-04](docs/HARDWARE_ENGINEERING_HANDOFF_2026-11-04.md)**: exact tests, numeric evidence, source changes, ROS commands, geometry model, map/route validation, bench safety tests, known problems, evidence locations and next steps.
- **[Simulation reference repository](https://github.com/EngZaq/f1tenth_uj_sim)**: established exploration, SLAM, centerline extraction and path optimization work. The hardware map-to-route finalizer is wired into a mapping-only physical rehearsal, but has **not yet produced a real completed-lap route**.

## System snapshot

| Subsystem | Evidence / status |
| --- | --- |
| Jetson | Orin Nano 8GB; Ubuntu 22.04, ROS 2 Humble |
| LiDAR | LakiBeam L1, Ethernet `192.168.198.2`, `/scan` ~29.6 Hz |
| Laser odometry | RF2O `/odom_rf2o` ~20 Hz; stationary 60 s drift ~6 mm |
| SLAM | Saved real occupancy maps; **no complete real closed-track map** |
| Geometry | Wheelbase **0.260 m**; body **0.345 × 0.197 m**; LiDAR X **+0.215 m** from rear axle |
| Physical planner | Centerline extraction, footprint checks and optimization passed offline using a **simulation map** |
| VESC | Firmware 7.0, `/sensors/core` ~50 Hz; live stationary telemetry |
| VESC odometry | Stationary publish and 4 synthetic kinematic conversions passed |
| Ackermann converter | 5 synthetic command conversions passed on isolated `/bench` topics |
| Command gate | Logic self-test and 6 ROS 2 bench integration scenarios passed; **not a production watchdog** |
| IMU / EKF | Not yet integrated into accepted physical tests |
| Hardware E-stop | **Pending** |

## Key source paths

```text
src/f1tenth_bringup/
  config/physical_vehicle.yaml         # physical geometry + unverified drivetrain assumptions
  config/vesc_to_odom.yaml              # wheelbase and odometry conversion
  config/ackermann_to_vesc.yaml         # ERPM / steering command conversion
  config/vesc.yaml                      # VESC device and provisional controller limits
  config/slam_toolbox.yaml              # SLAM configuration
  launch/sensors.launch.py              # LiDAR and fixed TF
  launch/tf.launch.py                   # rear-axle base_link -> laser
  launch/slam.launch.py                 # RF2O and SLAM
  launch/teleop.launch.py               # ACTUATES: do not launch before commissioning

tools/physical_geometry.py              # physical dimension loader
tools/physical_planning/               # OFFLINE simulator-derived mapping / route tooling
tools/safety/physical_drive_gate.py    # BENCH-ONLY prototype, not hardware safety
tools/safety/test_physical_drive_gate.py
```

The `physical_vehicle.yaml`, `physical_geometry.py`, `physical_planning/` and `safety/` paths are **new local Jetson work** and might not appear in this repository until the Jetson commits and pushes them. This README and the handoff were committed to GitHub separately so the documentation is not lost.

## Bash / ROS environment

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

When collecting data, run only the appropriate independent sensor and mapping nodes. Ensure exactly one owner for `odom -> base_link`. Don't combine standalone RF2O/SLAM instances with the launch file that starts those same nodes.

**Bench-only tests (do not connect to actuator output topics):**

```bash
python3 tools/safety/physical_drive_gate.py --self-test

# With the isolated offline gate running in a different terminal:
python3 tools/safety/test_physical_drive_gate.py
```

See the [engineering handoff](docs/HARDWARE_ENGINEERING_HANDOFF_2026-11-04.md) for the provenance, constraints, expected outcomes and where the real ROS bags are stored on the Jetson.

## Development policy

Work on `zaq/hardware-integration`. Do not force-push or push unreviewed changes to `main`. Do not commit raw bags or archives by accident. Before activating propulsion: independently verified physical stop, motor-controller stale-command failsafe, secured test stand, steering/motor calibration and controlled supervision are required. All simulation-derived speed and steering values are provisional until tested on the real robot.

## Integrated hardware observation — new, untested on Jetson (2026-11-04)

The new `observe_only.launch.py` and `tools/observe_only_audit.py` are **committed but still require execution on the Jetson**.

The launch starts LiDAR, laser TF, RF2O (`odom -> base_link`), SLAM Toolbox (`map -> odom`, loop closing overridden OFF), and optionally a VESC driver with all six actuator command subscriptions isolated to `/observe_only_unused/*`. VESC odometry publishes to `/odom_vesc_observe` with TF disabled.

**Do not start VESC Tool, teleop or another ROS VESC driver concurrently.** The isolated topics are not a hardware stop and can still be targeted by other publishers.

```bash
git pull --ff-only origin zaq/hardware-integration
colcon build --symlink-install --packages-select f1tenth_bringup
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=true
```

In a separate sourced terminal:

```bash
OUT="$HOME/f1tenth_hw_evidence/observe_only_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
python3 tools/observe_only_audit.py --duration 35 --with-vesc --output "$OUT/audit.json" | tee "$OUT/audit.log"
```

The audit **only subscribes**. It reports message arrival rates, observed TF edges, latest occupancy map, net RF2O motion, VESC telemetry and whether each topic was received. Passing does not validate actual wheel-speed scale, steering, motor limits, physical route tracking or braking.

**Deferred but NOT completed:** physical E-stop verification, UBEC / steering power validation, battery discharge/regen current rating, VESC electrical limits, firmware timeout fault injection and live actuator calibration. Do not mark these as passed just because the read-only integration succeeds.

## Non-authoritative shadow EKF experiment (added 2026-11-04)

**Preflight observed** 700 RF2O messages at 20.001 Hz (median header age 49.8 ms) and 1,751 VESC odometry messages at 49.992 Hz (median header age 1.35 ms). Both reported `odom`/`base_link` frames and monotonic stamps. Original RF2O pose covariance (X,Y,yaw) was **zero**, as was VESC forward-speed twist covariance. **Original topics stay unchanged.**

The new, **not-yet-tested-on-Jetson** shadow-only files are:

- `tools/shadow_covariance_adapter.py`: forwards RF2O/VESC odometry to `/shadow_inputs/odom_rf2o` and `/shadow_inputs/odom_vesc`, adding **provisional** test-only variances for fused variables.
- `src/f1tenth_bringup/config/ekf_shadow.yaml`: planar EKF; fuse RF2O X,Y,yaw and VESC forward velocity only; `publish_tf: false`, world/odom `odom`. No IMU at this stage.
- `tools/shadow_ekf_audit.py`: read-only check of incoming corrected topics and `/odometry/filtered_shadow`.

Adapter defaults use **explicitly uncalibrated test assumptions**: RF2O position standard deviation 0.25 m, yaw 15 degrees, and VESC forward-velocity standard deviation 0.4 m/s. These numbers are *not* supported by moving ground truth or sensor characterization; they are only a way to test filter data plumbing without feeding zero-variance observations into the EKF. Do **not** tune racing from them or present the resulting covariance as measured accuracy.

With the working `ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=true` running (no motor commands), run in a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 tools/shadow_covariance_adapter.py
```

In a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_shadow \
  --params-file "$HOME/f1tenth_uj/src/f1tenth_bringup/config/ekf_shadow.yaml" \
  -r odometry/filtered:=/odometry/filtered_shadow
```

In a fourth sourced terminal:

```bash
ros2 param get /ekf_shadow publish_tf   # MUST be False
ros2 topic info /odometry/filtered_shadow -v
OUT="$HOME/f1tenth_hw_evidence/shadow_ekf_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
python3 tools/shadow_ekf_audit.py --duration 35 --output "$OUT/audit.json" | tee "$OUT/audit.log"
```

If `publish_tf` is not False or the source topics/frames are wrong, stop the shadow EKF. The filter's output is **diagnostic only** and must not be used for robot navigation or physical control. RF2O remains the only publisher of `odom -> base_link` and SLAM remains the publisher of `map -> odom`.

**Deferred hardware to-do items remain open:** independently verified emergency stop, suitable UBEC output and servo electrical setup, battery C-rating / safe discharge and regen limits, controller-level command timeout verification, actual motor polarity/speed gain, physical servo calibration, and measured odometry covariance under real motion.

## Latest accepted localization milestone — stationary shadow EKF (2026-11-04)

**14/14 read-only integration checks PASSED**: RF2O 20.0 Hz, VESC wheel odom 50.0 Hz, filtered shadow EKF 30.0 Hz; valid frames/stamps, corrected test-only covariance and no EKF TF. This validates transport and stationary behavior **only**, not moving accuracy or safe driving. See the dated [shadow EKF validation and evidence report](docs/LOCALIZATION_SHADOW_EKF_VALIDATION_2026-11-04.md) for observed values, caveats and planned manual-motion comparison. Independent hardware E-stop, electrical current / regen limits, controller-level timeout and actuator calibration remain **pending**, not completed.

## Real LiDAR exploration shadow audit — 2026-11-05

The physical explorer's initial 60-second real-LiDAR evaluation had **healthy ROS transport** (1800 LiDAR scans, 1202 exploration proposals, 1200 RF2O odom messages, 12 maps), but **FAILED** independent front-view safety acceptance: 1102 proposed motions had an insufficiently valid +/-20° forward scan. The controller previously accepted only three valid beams and used the 15th-percentile forward distance; the independent audit required >=90% valid and checked the nearest finite return. The physical shadow controller was tightened to fail closed for incomplete scans, while retaining its **physical-drive interlock**. The auditor now records missing-ray classifications and flags whether zero output means no useful exploration decisions. **Jetson retest pending.**

See [2026-11-05 physical LiDAR exploration diagnostic and retest runbook](docs/PHYSICAL_EXPLORATION_SHADOW_AUDIT_2026-11-05.md). The initial 161 × 109 @ 0.05 m occupancy map was saved to `~/f1tenth_hw_evidence/exploration_shadow_20261105_092813/initial_map.{pgm,yaml}`; known area remained ~10.93 m² with the car stationary. Jetson spontaneous shutdown cause is still unknown (no persistent previous-boot journal). No powered autonomous driving is authorized.

## Offline real-LiDAR exploration comparison (2026-11-05)

First **stationary, non-actuating** raw ROS 2 bag recorded on the physical Jetson: `~/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/rosbag`. Recorder cleanly stopped after ~43 s, all five subscriptions connected (`/scan`, `/odom_rf2o`, `/map`, `/tf`, `/tf_static`); exact bag duration/message counts require `ros2 bag info`.

`tools/exploration/offline_footprint_replay.py` is a research-only script comparing the old 90%-valid ±20° criterion against **sampled hypothetical front-bumper paths** at steering angles -0.15, -0.075, 0, +0.075 and +0.15 rad. It retains `+Inf` as unknown, never publishes any ROS commands, and does **not** claim a collision-free swept volume. Wheelbase and laser/body offsets are estimates/measurements from our physical geometry record; actual servo steering calibration remains pending.

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 bag info ~/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/rosbag
python3 -m py_compile tools/exploration/offline_footprint_replay.py
python3 tools/exploration/offline_footprint_replay.py --self-test
python3 tools/exploration/offline_footprint_replay.py \
  ~/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/rosbag \
  --every 5 --preview-m 0.50 \
  --output ~/f1tenth_hw_evidence/real_exploration_baseline_20261105_101514/offline_footprint_report.json
```

**Important:** All output is diagnostic. A single laser plane, sampled leading edge and uncompensated frame cannot certify pedestrian/obstacle avoidance. No physical VESC actuation is enabled by this code. The independent hardware stop, safe controller timeout, battery limits, steering calibration and Jetson power reliability remain open.


### Fast physical commissioning (2026-11-05)

See [physical first-motion readiness](docs/PHYSICAL_FIRST_MOTION_READINESS_2026-11-05.md) for one disarmed Jetson verification, Ackermann swept-footprint guard tests, ROS steering/ERPM calibration report, and the required VESC firmware/independent-stop checks. The physical `/control/status` reports `swept_path` evidence. Real actuation remains disabled by default. The revised software guard **passed a single live disarmed Jetson snapshot** at 4.17 ms, but physical obstacle coverage, emergency stopping and braking are not certified.

## First real track map + route (no VESC Tool changes)

[Physical mapping rehearsal](docs/PHYSICAL_TRACK_MAPPING_REHEARSAL_2026-11-05.md): with propulsion physically isolated, manually push the instrumented robot once around a real closed track. The `physical_map_rehearsal.launch.py` launches **only LiDAR, RF2O, SLAM with loop closing, real lap monitor, and map/route finalizer**. It launches **no VESC, motor converter, or drive publisher**. Watch `/mission/physical_lap_progress`. An observed closed loop produces a time-stamped real map, centerline and body-size route audit; it is not an autonomous lap. The firmware is untouched. Speeds above 5m/s remain a future design objective requiring mechanical/electrical calibration and verified independent stopping.
