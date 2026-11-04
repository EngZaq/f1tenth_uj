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
