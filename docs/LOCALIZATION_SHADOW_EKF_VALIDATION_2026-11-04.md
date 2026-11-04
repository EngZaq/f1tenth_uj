# F1TENTH UJ — Read-Only Shadow EKF Integration Evidence

**Date:** 2026-11-04 (Jordan time)  
**Branch:** `zaq/hardware-integration`  
**Status:** **PASS** — ROS 2 shadow EKF data plumbing, stationary conditions only. **NOT** a localization-accuracy or driving-safety approval.

## Stack at time of test

- Jetson Orin Nano 8 GB / Ubuntu 22.04 / ROS 2 Humble.
- `observe_only.launch.py enable_vesc:=true`: LiDAR scan, RF2O, SLAM Toolbox (physical loop closing OFF), isolated VESC telemetry and isolated VESC odometry.
- RF2O owns `odom -> base_link`; SLAM owns `map -> odom`; shadow EKF **does not** publish TF (`publish_tf=False` verified with `ros2 param get`).
- Covariance adapter `tools/shadow_covariance_adapter.py`: passes RF2O X/Y/yaw and VESC vx on separate `/shadow_inputs/*` topics while adding **provisional** covariances. Original source messages are unchanged.
- `robot_localization ekf_node`: `two_d_mode=True`, `publish_tf=False`, RF2O X/Y/yaw + VESC forward speed only, output `/odometry/filtered_shadow`.
- No motor/servo commands were sent by the observation, adapter, EKF or audit scripts. **Hardware emergency stop, controller timeout, current limits and actuator calibration remain pending.**

## Preflight (saved on Jetson)

Evidence: `/home/ftenth/f1tenth_hw_evidence/localization_preflight_20261104_221149/preflight.json`.

| Metric | RF2O | VESC wheel odometry |
| --- | ---: | ---: |
| Samples in 35 s | 700 | 1751 |
| Rate | 20.001 Hz | 49.992 Hz |
| Median header age | 49.819 ms | 1.346 ms |
| P95 header age | 65.030 ms | 1.523 ms |
| Frame pair | odom/base_link | odom/base_link |
| Monotonic stamps | PASS | PASS |
| Important input covariance | Pose X/Y/yaw all **zero** | Twist vx **zero** |
| Net stationary XY | 8.58 mm | 0.0 mm |
| Net stationary yaw | +0.10656 deg | 0.0 deg |

**Decision:** Original covariance fields cannot be interpreted as calibrated measurement uncertainties. Test only with clearly labeled nonzero *provisional* values; do not overwrite original measurements or claim zero covariance means perfect certainty.

## Provisional shadow input uncertainty (NOT CALIBRATED)

| Field | Test standard deviation | Test variance |
| --- | ---: | ---: |
| RF2O planar X, Y | 0.25 m | 0.0625 m² |
| RF2O heading yaw | 15 deg | 0.068539 rad² |
| VESC forward velocity | 0.40 m/s | 0.1600 (m/s)² |

The EKF's lower posterior covariance follows from its assumptions and measurements; it is **not proof that physical error is lower**.

## Integration test results (successful final run)

Evidence: `/home/ftenth/f1tenth_hw_evidence/shadow_ekf_20261104_222310/audit.json`.

The first `shadow_ekf_audit.py` invocation failed before subscribing because `rclpy.node.Node.subscriptions` is a read-only property; the script attempted to assign a list to `self.subscriptions`. Fixed in **commit `4e34737`** by renaming the internal attribute to `self._audit_subscriptions`. This was a test harness bug, not an observed EKF failure.

With corrected script:

- **14/14 automated checks PASS**.
- Frame pairs: `odom` to `base_link` on all three topics.
- All topic stamps monotonic and reported poses finite.
- RF2O and VESC *test-only* covariance diagonal entries nonzero.
- All three message streams present.
- Shadow EKF `publish_tf=False`; `two_d_mode=True` confirmed from parameters.

| Topic | Samples | Frequency | Median message header age | Net XY during 35 s |
| --- | ---: | ---: | ---: | ---: |
| `/shadow_inputs/odom_rf2o` | 700 | 20.00 Hz | 52.0 ms | 0.009372 m |
| `/shadow_inputs/odom_vesc` | 1751 | 50.00 Hz | 2.71 ms | 0.000000 m |
| `/odometry/filtered_shadow` | 1050 | 29.999 Hz | 13.28 ms | 0.009711 m |

Last observation (not necessarily identical timestamps):
- RF2O: x=-0.002559 m, y=+0.106100 m, yaw=-0.091377 rad, vx=+0.017117 m/s (**twist not fused**).
- VESC: x=0 m, y=0 m, yaw=0 rad (**pose not fused**), vx=0.0 m/s.
- Shadow EKF: x=-0.002804 m, y=+0.105990 m, yaw=-0.090987 rad, vx=+0.000015 m/s.
- Shadow EKF pose variances: x=0.012394 m², y=0.015357 m², yaw=0.015610 rad²; forward velocity variance=0.010306 (m/s)².
- Corresponding *filter-reported* standard deviations: x~0.111 m, y~0.124 m, yaw~7.16 deg, vx~0.102 m/s. **These are not measured absolute error.**

A single earlier filtered message was x=0.004876 m, y=0.099988 m, yaw quaternion z=-0.035612, w=0.999366 and near-zero velocity. The nonzero Y and yaw represent the estimator's arbitrary initial/reference pose and must not be mistaken for drift accumulated during the 35-second audit.

## Technical caveats

1. The shadow filter includes RF2O pose observations. Agreement between the output and RF2O is **expected/correlated** and cannot be used as independent evidence of improved accuracy.
2. The VESC velocity measurement remained zero. This test establishes the data path but **does not demonstrate dynamic fusion benefit**, speed direction, gear ratio or wheel-speed scale.
3. EKF posterior uncertainties depend on assumed input covariance, process noise and measurement timing; no measured dataset has yet justified those parameters.
4. The adapter does not robustly gate sudden spikes or enforce upstream message/TF ownership at runtime. `robot_localization` is a provisional test estimator, not yet a validated navigation source.
5. Saved evidence files remain on Jetson and are **not uploaded to GitHub**; only source and Markdown documentation are committed.
6. Before powered actuation: independent verified E-stop, controller-level stale-command failsafe, suitable motor/battery/regen limits, steering and motor calibration and safe operating environment remain open.

## Next validation gates

1. Record a **measured, manually pushed** straight-line trial with the observation stack, no actuator commands. Start/end reference at rear axle, use a known travel distance (ideally >=1 m if portable power permits; 10–15 cm tethered motion is insufficient for reliable scale calibration).
2. Save `/odom_rf2o`, `/odom_vesc_observe`, `/odometry/filtered_shadow`, `/scan`, `/sensors/core`, `/tf`, `/tf_static`, `/map` and record the exact physical distance and whether steering was straight.
3. Compare relative start/end pose, yaw, integrated wheel speed and time-aligned EKF/RF2O estimates against *independently measured ground truth*. **Do not treat shared-input agreement as localization accuracy.**
4. Determine whether manually turning the unpowered drivetrain produces valid VESC ERPM, and verify sign/scale before changing odom gains. Do not assume it does.
5. Estimate empirical variances using multiple trials, stationary periods and ground-truth error; distinguish correlated RF2O pose/rate variables. Revisit EKF process noise, consistency, delay and source selection.
6. Keep TF in shadow mode until the filter has stronger evidence than the current RF2O-only setup, and until the hardware safety checklist is complete.

**Disposition:** **SHADOW EKF INTEGRATION PASS — STATIONARY / DIAGNOSTIC ONLY**.
