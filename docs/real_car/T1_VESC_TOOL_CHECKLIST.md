# T1: VESC Tool settings to read (F-01 post-stop rebound)

Read only; change nothing during T1. Firmware reported by vesc_driver on 14 Nov: **7.0**.
XML names are from the VESC firmware configuration; check each against your VESC Tool
version and correct this file if a name differs.

**First:** VESC Tool, File, save the motor config and the app config as XML. Copy both to
`~/f1tenth_hw_evidence/real_car_tests/vesc_config_<YYYYmmdd>/`. That backup is the record;
screenshots are optional.

| # | VESC Tool location | XML name | Value | Why it matters for F-01 |
|---|---|---|---|---|
| 1 | Motor Settings, General | `motor_type` (FOC / BLDC) | | Decides which sensorless rows below apply |
| 2 | Motor Settings, General / FOC | sensor mode: `foc_sensor_mode` (FOC) or `sensor_mode` (BLDC) | | Baseline says sensorless; confirm |
| 3 | PID Controllers, Speed | `s_pid_kp`, `s_pid_ki`, `s_pid_kd`, `s_pid_kd_filter` | | `speed 0` is a speed-PID setpoint. High gain plus a lost observer at low ERPM can overshoot through 0 |
| 4 | PID Controllers, Speed | `s_pid_min_erpm` | | Below this the speed loop does not regulate; how a 0 ERPM request is handled near it |
| 5 | PID Controllers, Speed | `s_pid_allow_braking` | | Whether the speed loop may brake to reach 0, or only drive |
| 6 | PID Controllers, Speed | `s_pid_ramp_erpms_s` | | Setpoint ramp; 12 Nov needed 0.555 s to reach 0.24 m/s |
| 7 | FOC, Sensorless | `foc_openloop_rpm`, `foc_openloop_rpm_low`, `foc_openloop_hyst`, `foc_openloop_time` | | Open-loop region at low speed can drive the rotor with no feedback: a candidate for the rise 0.16-0.27 s after zero |
| 8 | FOC, Sensorless | `foc_observer_gain`, observer type `foc_observer_type` | | Observer lock while slowing down |
| 9 | FOC, Sensorless (hybrid only) | `foc_sl_erpm` | | Sensorless ERPM threshold, if a hybrid mode is set |
| 10 | BLDC (only if BLDC) | `sl_min_erpm`, `sl_min_erpm_cycle_int_limit` | | BLDC sensorless minimum ERPM |
| 11 | Motor Settings, Current | `l_current_max`, `l_current_min` (motor brake), `l_in_current_max`, `l_in_current_min` (battery regen), `l_abs_current_max` | | Braking/regen limits; 12 Nov rebound drew +18 A phase current |
| 12 | Motor Settings, RPM | `l_max_erpm`, `l_min_erpm`, `l_erpm_start` | | Firmware ERPM limits (the ROS clip was 3500; the rebound peaked at 3911) |
| 13 | Motor Settings, Voltage | `l_battery_cut_start`, `l_battery_cut_end`, battery cell count | | The battery read 11.1 V on 14 Nov after a power loss: check the cut-off is not active |
| 14 | App Settings, General | app to use (`app_to_use`), `timeout_msec`, `timeout_brake_current` | **22 Nov (Moatasem): `timeout_msec` 300 (was unknown / default), `timeout_brake_current` 5.0 A (was 0)**; `app_to_use` not reported | What the VESC does when commands stop; the earlier stand test saw a ~1 s release |
| 15 | App Settings, General | `can_mode` / controller ID | | Confirms a single controller |

## Hypothesis being tested (baseline C-7)

The VESC is in speed (RPM) PID mode, sensorless. Commanding 0 ERPM asks the speed PID to hold
0 rpm while the observer has no lock at low speed, which can overshoot or rebound. T1b compares
`speed 0` with brake current (5 A, 10 A) and current 0 on the stand. Rows 3-9 explain a speed-0
rebound; rows 11 and 14 bound what brake and release can do.

## Report back

Paste the filled table (or the two XML files) before T1b. Do not change any value until we
have chosen a stop method; any change then gets its own T1 repeat.
