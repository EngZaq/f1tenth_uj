# Shadow EKF (C1): every R and Q, with its source (23 Nov evening)

Files: `config/real_car/ekf_real_shadow.yaml` (Q, sensor lines; 5 variant blocks generated from one Q) and
`tools/real_car_tests/ekf_shadow_inputs.py` (R values written into each input message; one definition for the
live node and the replays). Check tool: `tools/real_car_tests/ekf_r_nis.py` (`--r`, `--nis`). Recommended variant
C1 = `ekf_real_shadow_c1`: VESC vx (de-weighted) + vy = 0 + IMU yaw rate + RF2O vx.

## R (measurement variances, in the input messages)

| Name (`ekf_shadow_inputs.py`) | Value | Used by | Source | 23 Nov evening check |
|---|---|---|---|---|
| `R_VESC_VX` | 5.0e-3 (m/s)^2 | A, B, C* (odom0 vx) | 21 Nov, T4 cruise carpet + tile: (2 x robust std of VESC vx - LiDAR wall slope)^2; upper bound (the LiDAR reference adds ~0.03 m/s) | steady cruise self-noise 1.0e-4 (T4) / 1.9e-4 (T3): far below, but the error vs the ground is scale / slip / PID ripple (correlated); **keep** |
| `DEWEIGHT` x `R_VESC_VX` | x100 = 0.5 | C* while braking (t1b `STOP_BRAKE`) or 0 < \|ERPM\| < `SENSORLESS_ERPM` 2000 | 21 Nov T4: below 2000 ERPM the VESC reads +0.08..+0.12 m/s in the start ramp; while braking it drops to 0 with the car at 0.2-0.4 m/s | VESC open loop below 1400 ERPM (VESC Tool 23 Nov) supports the 2000 threshold |
| `R_VY` | 4.0e-4 (sigma 0.02 m/s) | all (odom0 vy = 0) | model: no lateral slip at the rear axle (base_link) | T3 22 Nov slip <= 2.3 deg -> vy <= 0.012 m/s at 0.3 m/s: consistent |
| `R_IMU_WZ` | **1.2e-3** (rad/s)^2 (was 8.0e-3) | all (imu0 vyaw) | T5 (a) 21 Nov + T3 23 Nov: (2 x sigma_hf of the worst floor, carpet 0.0174)^2; 8e-3 was RF2O noise + lag | **applied 23 Nov eve** (with the tilt-projected yaw rate) |
| `R_RF_VX` | 1.0e-2 | B, C1 (odom1 vx) | 21 Nov, T4: (2 x robust std of RF2O vx - LiDAR)^2 | steady cruise 1.3e-2 (T4) / 8.7e-3 (T3): **keep** |
| `R_RF_WZ` | 1.2e-2 | B (odom1 vyaw) | 21 Nov, T4: same rule | not in C1 |
| `R_AX` | 0.32 (m/s^2)^2 | C2 (imu0 ax) | 21 Nov, T4: sigma 0.57 m/s^2 (motor vibration) | not in C1 |
| `BIG` | 1e6 | all unused axes in the messages | robot_localization ignores them through `*_config` anyway | - |

## Q (`process_noise_covariance` diagonal, per second; robot_localization scales by dt)

| State | Value | Source |
|---|---|---|
| x, y | 0.05, 0.05 | robot_localization template default |
| z, roll, pitch | 0.06, 0.03, 0.03 | template default (unused: `two_d_mode: true`) |
| yaw | 0.06 | template default |
| **vx** | **0.05** | template 0.025, doubled in the sim Config C (`ekf_moatasem_vesc_imu_shadow.yaml`) for the speed steps |
| vy | 0.025 | template default |
| vz, vroll, vpitch | 0.04, 0.01, 0.01 | template default (unused) |
| **vyaw** | **0.02** | template default |
| ax, ay, az | 0.01, 0.01, 0.015 | template default (no accel fused in C1) |

Other filter settings: `frequency` 30 Hz, `sensor_timeout` 0.2 s (= RF2O stale limit of the inputs node),
`smooth_lagged_data` with `history_length` 0.5 s (RF2O arrives ~51 ms late), no Mahalanobis gates.

## NIS of the C1 replay and the Q proposal

`ekf_r_nis.py --nis` on the 14 T3 circle repeats (C1, YAW_MODE tilt, R_gz 1.2e-3), results doc row "C1 NIS":

| Channel | n | NIS mean | p95 | > 3.84 |
|---|---|---|---|---|
| VESC vx | 18070 | 0.006 | 0.016 | 0 % |
| VESC vx de-weighted | 2180 | 0.019 | 0.108 | 0 % |
| IMU yaw rate | 40289 | 0.179 | 0.692 | 0.2 % |
| RF2O vx | 8086 | 0.259 | 1.233 | 0.6 % |
| vy = 0 | 20250 | 0.000 | 0.000 | 0 % |

(Consistent filter: mean 1, 5 % above 3.84. The prior is approximated by the last 30 Hz output, which biases NIS low.)
Reading: the filter is conservative everywhere; each mean is about the ratio of the measured white noise to R, i.e.
R is the deliberate upper bound, not Q. **Proposal: keep Q as it is.** Next: one closed lap with a SLAM
reference, then lower one R at a time towards the measured noise (VESC 2e-4, gyro 1e-4, RF2O 9e-3) and only then
judge Q_vx (0.05) and Q_vyaw (0.02) from the NIS of the remaining channels.
