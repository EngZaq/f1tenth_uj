# F1TENTH UJ — Physical Autonomous Racing Platform

**University of Jordan — Mechatronics Engineering Graduation Project**

Hardware repository for the F1TENTH autonomous racing car running ROS 2 Humble on an NVIDIA Jetson Orin Nano. The project is integrating a real LakiBeam LiDAR, RF2O, SLAM Toolbox, VESC drive interface, measured vehicle geometry, simulator-derived offline planning and staged safety validation.

> **STATUS (2026-11-04): NOT APPROVED FOR POWERED AUTONOMOUS DRIVING.** The hardware emergency stop is not yet installed/tested. VESC physical direction, steering range, command timeout, wheel-speed calibration, real-track localization and tracking remain open. Safety-gate code under `tools/safety/` is **BENCH-ONLY** and must **not** be wired to the real VESC/servo command path.

## Engineering documentation

- **[Complete physical hardware integration handoff — 2026-11-04](docs/HARDWARE_ENGINEERING_HANDOFF_2026-11-04.md)**: exact tests, numeric evidence, source changes, ROS commands, geometry model, map/route validation, bench safety tests, known problems, evidence locations and next steps.
- **[Simulation reference repository](https://github.com/EngZaq/f1tenth_uj_sim)**: established exploration, SLAM, centerline extraction and path optimization work. The hardware planning copy is presently **offline only**.

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
