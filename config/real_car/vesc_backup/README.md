# VESC configuration backup (motor + app XML), before / after

Owner: Moatasem. Record of the VESC Tool configuration of the car's VESC (firmware 7.0, reported by vesc_driver
14 Nov). The XML exports are the record; this file is the readable diff.

## Files

| File | State | Status |
|---|---|---|
| `before_mcconf.xml`, `before_appconf.xml` | before the 22 Nov timeout change | **MISSING**: not exported before the change (results doc, 22 Nov) |
| `after_20261123_mcconf.xml`, `after_20261123_appconf.xml` | after the 23 Nov session | **MISSING on the laptop**: `~/Data/real_car_evidence/vesc_config/` was empty on 23 Nov eve |

To add them: VESC Tool, File, Save motor configuration XML / Save app configuration XML, copy both here with the
names above, then `git add config/real_car/vesc_backup/<file>` by name. Afterwards regenerate the table below from
the XMLs (`diff <(grep -o '<[a-z_]*>[^<]*' before_mcconf.xml) <(grep -o '<[a-z_]*>[^<]*' after_20261123_mcconf.xml)`).

## Diff, before -> after (values as reported by Moatasem; to be checked against the XMLs)

| Setting (VESC Tool / XML name) | Before | After (23 Nov) | Source / effect |
|---|---|---|---|
| App > General `timeout_msec` | unknown / default (24 Oct stand: release after ~1 s) | **250 ms** | Set 22 Nov, reported then as 300 ms; VESC Tool reads **250** on 23 Nov (the planned value). Stand check 22 Nov: brake onset 0.29 s = 250 ms + telemetry / converter ticks. Every command source must send at least every 0.25 s (converters 50 Hz). |
| App > General `timeout_brake_current` | 0 A (release, coast) | **5.0 A** | Set 22 Nov, verified 22 Nov (-2.44 A on the stand). |
| Motor > Current `l_current_min` (motor brake / "regen" limit) | -60 A (Zaid's 5 Nov notes, `docs/REAL_INTEGRATED_ROS_HARDWARE_2026-11-05.md`) | **-15 A** | Set 23 Nov. Caps any brake current: the converter's 5 A brake and the 5 A timeout brake are below it. |
| Motor > Voltage `l_battery_cut_end` | not read | **9.6 V** | Set 23 Nov (3S: 3.2 V per cell). `l_battery_cut_start` not reported: read it from the XML (start must be above end). |
| PID > Speed `s_pid_ramp_erpms_s` | not read | **25000 ERPM/s** (read 23 Nov) | = 25000 / 8414 = **2.97 m/s^2**: the fastest speed change a speed SETPOINT can ask for. A speed-profile deceleration commanded through speed setpoints is limited to ~3.0 m/s^2 (measured ground deceleration with brake 5 A: ~0.9 m/s^2, T4 tile). |
| FOC > Sensorless open loop (`foc_openloop_rpm`) | not read | **1400 ERPM** (read 23 Nov) | = **0.166 m/s**: below it the motor runs open loop, the ERPM reading is not observer based (the shadow EKF already de-weights the VESC below 2000 ERPM). Plan speeds >= 0.2 m/s. |

Not changed by us: everything else. Zaid's 5 Nov notes also list ~42.85 A motor and 99 A battery current limits
(`l_current_max`, `l_in_current_max`): read them from the XMLs.
