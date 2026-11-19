# Real-car tests: session handoff (2026-11-19)

For Moatasem and for the next Claude Code session. Read this, then
[REAL_CAR_BASELINE_2026-11-14.md](REAL_CAR_BASELINE_2026-11-14.md) (standard values) and
[REAL_CAR_TEST_RESULTS.md](REAL_CAR_TEST_RESULTS.md) (every measured result). Code:
`tools/real_car_tests/` (README there). Numbers: `config/real_car/vehicle_params.yaml` only.

**Status 19 Nov (end of session):**
- **T1 COMPLETE** (16 Nov, stand): brake 5 A, 23 steady stops at 0.5 / 1.0 / 1.5 m/s, 0 failing.
  F-01 mitigated on the stand; **speed 0 is never a stop command**. Permanent converter: proposal to Zaid.
- **T2 COMPLETE** (19 Nov, limits): steering driving clip **0.10-0.68** (one step before the mechanical
  stops ~0.08 / ~0.70), max left 17.15 deg, right 18.38 deg (R 0.84 / 0.78 m). Fitted gain **~-0.96 NOT
  applied** (converter stays -0.50 / 0.320): the centre is uncertain because of left-wheel play.
- **Next:** section 4, steps 1-3. Steering findings for Zaid: section 4a.

## 1. Safety rules and standing approvals (Moatasem)

| Rule | Detail |
|---|---|
| Who actuates | **Moatasem runs every launch and every runner that can power the motor or servo**, in his own SSH terminal, with a hand on the battery disconnect. Claude never starts them (not even disarmed: the test stack centres the servo) and never sets `enable_real_actuation:=true`. Claude gives exact commands, one terminal at a time. |
| F-01 | **Mitigated on the stand (T1 passed, confirmed by Moatasem in writing 16 Nov): brake 5 A, never speed 0.** Any run that powers the motor uses the T1b converter with `stop_method:=brake brake_current_a:=5.0` until the permanent converter (section 5) replaces it; the standard `ackermann_to_vesc` (speed 0 stop) never drives the motor. Ground runs only as the T3/T4 steps below, each approved per step. Stand parts: car strapped, wheels off the ground, >= 0.8 m clear in front of the LiDAR (a box 1.5-2 m ahead keeps the front valid fraction >= 0.75), hands out of the front +/-20 deg sector. |
| Mux | Never change mux gates, thresholds or interlocks. Caps change only through the per-test launch args (`max_speed_mps`, `max_steer_rad`) and the per-step `vesc_yaml`. No new publishers or subscribers on `/drive` or `/commands/*` (the bag never records them; the mux counts subscribers). |
| One stack | `preflight.sh clean` before every launch, `preflight.sh stack` (or `stack t1b`) before every runner. |
| Speeds | 0.5 -> 1.0 -> 1.5 m/s, one step only after the previous passed (runner `step_gate`, unchanged). Ground runs: LiDAR range ahead must fit ramp + 1 m + stop + 0.5 m (runner checks). |
| Git | Claude may commit **its own files by name** without asking, author `Moatasem <motasemfaousju04@gmail.com>` passed per commit with `git -c user.name=... -c user.email=...` (the Jetson's git config is Zaid's: never change it). **Never push.** Never `git add -A`. Never stash/reset/commit teammates' files. |
| Allowed without asking | Self-tests, `analyze.py`, offline replays (only when no car stack is running). |
| Reporting | After each test: pass/fail, 3-5 numbers, the next exact commands, each terminal's command on its own. |
| Sim repo | `~/f1tenth_uj_sim` (branch `moatasem/ekf-experiments`, at `9c581af`): read-only, `git pull` allowed, no commits. |

## 2. Branch and commit state

- Branch `moatasem/real-car-tests` from `zaq/hardware-integration` @ `53dfa91`. **Not pushed** (no
  upstream yet; Moatasem pushes, Claude never does).
- Commits (oldest first): `d1e0c70` runner/analysis/params · `fea48d3` **mux caps (Zaid's file, separate commit for his review)** · `8b7228c` dry-run fixes · `61dcf0e` T1/T1b · `82336a1` T2 + VESC checklist · `057995f` steady-stop gate · `ae9a36f` shared rebound definition, T1b 1.0/1.5, table · `58b2449` results · `03b2e67` steady repeats only · `6b52990` this file + current 0 · `066d104` T1 close-out ·
  `530fff8` stop-converter proposal · `36ccfa7` T2 `e` answer · `d8202be` T2 clip ceiling ·
  `7d70ed3` T2 range 0.10-0.60 · `dbf90f8` `--extend-right` · `782cc71` T2 clip 0.10-0.68 + max angles ·
  plus the commit adding this update (19 Nov).
- **Leave alone (teammates', uncommitted):** `tools/diagnostics/fit_steering_measurements.py`, and the `src/hardware/f1tenth_system` submodule (vesc_driver `VESC_SPEED_AUDIT` logging + serial fix: it is the record of sent ERPM, keep the build), untracked `teleop_tools`. `__pycache__/` is in `.git/info/exclude`.
- Local, never committed: `~/f1tenth_uj/CLAUDE.local.md` (calibration section updated to the baseline). Claude memory: `~/.claude/projects/-home-ftenth/memory/`.

## 3. T1 / T1b results (stand): T1 COMPLETE 16 Nov

`python3 tools/real_car_tests/analyze.py --table` (regenerate after each run):

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

**Close-out (16 Nov):** brake 5 A passed at 0.5, 1.0 and 1.5 m/s: 23 steady stops, 0 failing, worst
rebound rise 197 ERPM, worst settle 0.227 s, <= 4.8 A. **F-01 is mitigated on the stand with brake
current; speed 0 must never be used as a stop command.** F-01 stays open for: the permanent converter
(section 5), the ground stop distance (T4) and VESC Tool rows 11/14. A small rise (96-152 ERPM) at
+0.16 s in 4 of 10 stops at 1.0/1.5 m/s has the speed-0 rebound's timing: the cause is suppressed,
not removed.

Reading of the 14 Nov runs: **speed 0 (the standard ackermann_to_vesc stop) is not safe**: rebound up to 7871 ERPM, above
vesc_driver `speed_max` 4630, so VESC-internal; the VESC drives the motor (positive duty and current)
~0.1-0.25 s after a zero. **Brake 5 A and 10 A stop cleanly** at 0.5 m/s; brake 5 A has 8 clean steady
stops at 1.0 m/s over two runs but no single run with 5 (SPACE released early, now handled by extra
repeats). **Current 0 passes on the stand but is a coast**; on the ground it needs a measured stop
distance (T4) before it can be considered.

## 4. Next steps, in order (decided by Moatasem, 19 Nov)

1. **Push the branch and send Zaid the two documents** (Moatasem): `git push -u origin
   moatasem/real-car-tests` from his own terminal, then send
   [PERMANENT_STOP_CONVERTER_PROPOSAL.md](PERMANENT_STOP_CONVERTER_PROPOSAL.md) (5 decisions, incl. the
   `vesc_real_integration.yaml` `brake_max 0.0` that would clip the brake to 0) and the steering findings
   (section 4a). `fea48d3` (mux caps in his file) is in the same push for his review.
2. **After Zaid's mechanical fixes: re-measure the centre + short T2.** Several centre readings approached
   from each side, same ruler method; then a short sweep (e.g. 0.20 / 0.10 / 0.46 / 0.68, `n` + `s`
   elsewhere). If the centre readings agree (within ~1 deg), adopt gain/offset from the fit through a
   results row (`servo_gain` -0.50 and `servo_offset` 0.320 change only then, in both converter and
   vesc_to_odom yamls). If the horn is re-mounted, the full T2 (limits too) is redone.
3. **T4 on the ground, 0.5 m/s, brake 5 A** (straight line toward a wall, 3 repeats). Before it:
   - **The T4 runner profile does not exist yet** (`run_test.py` has `t1_stop`, `t1b_stop`,
     `t2_servo_sweep` only): Claude writes it (straight, steer 0, ground check: LiDAR range ahead >= ramp
     + 1 m + stop + 0.5 m), self-test, then a disarmed dry run on the car.
   - Stack: T1b converter `stop_method:=brake brake_current_a:=5.0` with `vesc_t1b_erpm4630.yaml`
     (brake_max 10), `max_speed_mps:=0.5`, `max_steer_rad` 0 or the minimum the mux allows.
   - Moatasem's written approval for the ground step, a free straight length checked on site, hand on the
     battery disconnect.
   - Output: stop distance and deceleration with brake 5 A (closes the F-01 ground part), VESC vs LiDAR vs
     RF2O distance (`speed_to_erpm_gain`), start delay. Lateral drift is only indicative while the centre
     is uncertain.

### 4a. Steering findings for Zaid (T2, 19 Nov)

| Finding | Evidence | Consequence / ask |
|---|---|---|
| Driving clip **0.10-0.68**, mechanical stops ~0.08 (left) / ~0.70 (right) | T2 run `20261119_134151_t2_servo_sweep` + `--extend-right` | Every servo clip in a driving config must stay inside 0.10-0.68. `config/vesc_real_integration.yaml` clip 0.30-0.42 is far inside (only 0.04 rad left at gain -0.50). |
| Max angles left 17.15 deg (R 0.84 m), right 18.38 deg (R 0.78 m) | Bicycle-equivalent of both wheels; Moatasem's wheel averages 17.5 / 18.5 deg | Fills baseline "real steering range UNKNOWN"; mux steer cap (0.10 rad, ceiling 0.40) is well inside. |
| Fitted gain **~-0.96** (left -0.87, right -1.06), offset 0.364, vs converter -0.50 / 0.320 | `fit.json` of the T2 run, rms 2.1 deg | If confirmed, a commanded angle gives about **half** the real wheel angle (e.g. mux 0.10 rad -> ~0.05 rad). Fits F-02 (much less yaw than the bicycle model). **Not applied** until the centre is confirmed. |
| **Left-wheel play / uncertain centre** | Centre readings at the same command: left wheel -5 / +3 / -3 deg, right +3 / +7 / +7 deg; fitted backlash 2.2 deg | Check the left steering linkage (knuckle, ball joints, servo saver) for play. About 8 deg toe difference at the first centre reading: check the toe setting. |
| **Servo horn off-centre** | Straight = servo 0.320 = 1320 us; VESC sends 1000-2000 us (firmware), servo accepts 500-2500 us | Left has 0.32 of servo travel, right 0.68. Re-mounting the horn so straight is near 0.5 gives symmetric travel; **T2 must be redone after that**. |
| Servo power | 6 A UBEC from the battery; only the signal comes from the VESC | A binding servo can draw up to ~6 A that the VESC does not limit (two servos already burned). |
| Old configs | `ackermann_to_vesc.yaml` / `vesc_to_odom.yaml` still at offset 0.361 | Real-car configs use 0.320; align after the centre re-check. |

## 4b. Backlog (after the next steps)

1. **Still from T1:** VESC Tool checklist (`T1_VESC_TOOL_CHECKLIST.md`, at least rows 11 and 14 and the
   two XML exports), and Zaid's decision on the permanent converter (proposal in section 5).
2. T2 reference (done 19 Nov): `t2_servo_only.launch.py` + `t2_servo_sweep.py`, `--extend-right`;
   output: servo -> angle table, limits, backlash, linear and per-side fit (decides whether a piecewise
   converter is needed; the trial gains -0.4496 / -0.8570 were measured before the play fix).
3. **T3** loaded steering on the stand, then mini-circles on the ground at 0.5 (then 1.0) m/s at 50 % and
   90 % of the measured max steer (`max_steer_rad` launch arg, approved per test); delta = atan(0.260 / R)
   vs T2. Runner profile not written yet.
4. **T4** (0.5 m/s is next step 3) straight line 0.5/1.0/1.5 m/s toward a wall, 3 repeats each (runner ground check): LiDAR vs VESC
   vs RF2O distance -> `speed_to_erpm_gain` (same value in both converter and vesc_to_odom), lateral drift
   -> steering offset, start delay, stopping distance and deceleration with the chosen stop method. Profile
   not written yet.
5. **T5** IMU on the car: 10 min static (bias, R, Allan), 8 x 360 deg hand turns (scale, Moatasem's bench
   scripts from `~/Data/kf_practice/imu/`), 2 min wheels spinning on the stand.
6. **T6** offline on T3/T4/T5 bags: VESC vx vs RF2O vs LiDAR, gyro vs RF2O yaw rate, time offsets
   (cross-correlation), R per sensor -> `vehicle_params.yaml noise`.
7. **Phase F (shadow only, no TF authority):**
   - `config/real_car/ekf_real_shadow.yaml` from `src/f1tenth_bringup/config/ekf_moatasem_vesc_imu_shadow.yaml`:
     VESC vx + **vy = 0** (via `tools/shadow_covariance_adapter.py`) + BMI160 yaw rate (bias removed,
     scale from T5), `two_d_mode: true`, R from T5/T6, Q_vx 0.05, Q_vyaw 0.02, no gates,
     `smooth_lagged_data`, `publish_tf: false`, output `/ekf_shadow/odom`. Replay on real bags
     (`use_sim_time`), judge the mean pose/yaw vs RF2O/SLAM, not P alone (baseline C-2).
   - Nav2 AMCL in shadow on a real saved map, team laser model **A0** (`sigma_hit 0.10`, `z_rand 0.3`),
     frames map/odom/base_link/laser, initial pose on its own topic, SLAM stopped. Replay on bags first.
     Sim reference: `config/fusion_amcl_moatasem*.yaml`, `tools/localization/amcl_shadow.launch.py`.
   - Port `tools/navigation/profiled_amcl_drive_moatasem.py` + `moatasem_speed_profile.py` (both in the sim
     repo now) as a proposal-only node -> `/navigation/proposal` (never `/drive`); replace sim geometry
     (0.3302, 0.17145, 0.27145, slam_map, fusion_* frames) with `vehicle_params.yaml`; guard with the
     measured decel and delay; speed cap = mux cap.
   - Write measured noise/bias/delay/steering play to a new sim file
     `~/f1tenth_uj_sim/config/fusion_sim_sensors_realcar_<date>.yaml` (no commit there) and give the
     command to re-run sim Test 7.

## 5. Permanent stop fix (T1 done: option A chosen for the proposal to Zaid, 16 Nov)

- **A: our own converter package** replacing `ackermann_to_vesc` (speed while driving, brake current on
  stop, timed hold). Evidence-based (brake passed). Does not cover a dead Jetson (VESC timeout does),
  nor overshoot on slow-downs to nonzero speed. The mux ownership check must then also require exactly
  one publisher on `/commands/motor/brake` and `/commands/motor/current`, the same node as the speed
  publisher, one subscriber each, none on duty/position: an added check in Zaid's file, needs review.
- **B: VESC config** (speed-PID setpoint ramp `s_pid_ramp_erpms_s` is the leading hypothesis, gains,
  allow braking, timeout brake current): fixes the cause for every source; not confirmed yet, settings
  outside git (export XML), every change needs T1 again.
- Recommended: both. Read the checklist, set a timeout brake current, test the ramp hypothesis with a
  T1a speed0 repeat, keep brake-on-stop in ROS.

## 6. Known bugs and open items

| Item | State |
|---|---|
| Live rebound display | **Fixed** (`ae9a36f`, shared `ReboundTracker`: rises count only after ERPM fell to half its value at the zero). Confirmed on the current0 run: live = analysis on all 5 stops. Live peak times are ~0.06 s later than the analysis (live clock starts at the runner's zero, analysis at the converter's). |
| Brake 10 A vs `l_current_min` | Requesting 10 A did not raise the measured motor current above the 5 A runs (<= 4.2 vs <= 3.7 A). Either the ~0.1 s stop is missed by 50 Hz telemetry or the firmware brake current limit caps it: check `l_current_min` (checklist row 11). |
| VESC Tool checklist | **Pending**: `docs/real_car/T1_VESC_TOOL_CHECKLIST.md` (export motor + app XML first). |
| Brake hold release | Every brake stop (also 16 Nov at 1.0 / 1.5 m/s: 88-101 ERPM at +2.09-2.15 s) shows a 77-100 ERPM rise at +2.1-2.2 s: the 2.0 s hold ending (switch to current 0). Tune the hold / release in the permanent fix. |
| Servo horn off-centre (Zaid) | Straight = servo 0.320 = 1320 us, not 1500 us (VESC 0..1 = 1000-2000 us): left at most 0.32 of travel, right 0.68. Re-mount so straight is near 0.5 for symmetric travel; **redo T2 after that**. Zaid to decide. |
| Battery | Charged: >= 12.3 V during both 16 Nov runs (11.1 V idle on 14 Nov after a power loss). |
| Stand LiDAR | Front valid fraction ~0.83 vs the 0.75 MANUAL gate; an operator hand at 0.10-0.12 m (+10..+19 deg) latches `OBSTACLE_NEAR`. |
| F-13 | Runner input gaps never exceeded 44 ms; the keyboard GUI's latch was not reproduced (GUI not re-tested). |
| IMU TF | `base_link -> imu_link` (0.091, 0.057, 0.046, roll pi) published by the test launch, PROVISIONAL; ask Zaid how it was published on 14 Nov. |
| Team notes | Repo configs still at offset 0.361; do not use `vesc.launch.py` (`config/vesc.yaml` brake 200000 A); listed in the results doc. |
| T1b converter | Test-only (approved with conditions: one command per tick, brake/current only after a zero request, new nonzero request returns to speed at once, method logged per stop). |

## 7. Command templates

Every terminal, once:
```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Before every launch (terminal A):
```bash
~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
```

T1a stack, standard converter (terminal A); `<v>` / `<N>`: 0.5 / 4630, 1.0 / 9255, 1.5 / 13883:
```bash
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=<v> vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_test_erpm<N>.yaml
```

T1b stack (terminal A); `stop_method` = `brake` (with `brake_current_a:=5.0|10.0`) or `current0`:
```bash
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=<v> vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t1b_erpm<N>.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0
```

Preflight with the stack up (terminal B); add `t1b` for the T1b stack:
```bash
~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
```

Runner (terminal B): `n` arms, hold SPACE until it stops by itself, `q` quits:
```bash
python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t1b_stop --speed <v> --live
```

Analysis (terminal B); `<pattern>` e.g. `t1b_stop_v1.00_brake5A`:
```bash
python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*<pattern>* | head -1)
python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py --table
python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py <folder> --stop-plot 1
```

Disarmed dry run (no `--live`, `enable_real_actuation:=false`):
```bash
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=false
python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test dry_run
```

T2 (motor phases disconnected, servo on the 6 A UBEC; terminal A, then B; approved range 0.10-0.60
since 19 Nov = the launch ceiling, so one launch; `e` ends a side):
```bash
ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py servo_min:=0.10 servo_max:=0.60
python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_sweep.py
python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_sweep.py --resume <folder>
# right-side limit extension (19 Nov), linked to a finished run; launch servo_max up to 1.0, min 0.10:
ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py servo_min:=0.10 servo_max:=0.80
python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_sweep.py --extend-right <folder>
```

Self-tests (Claude may run; the runner's shutdown probe uses isolated domain 77, no publishers):
```bash
for t in run_test analyze t1b_stop_converter t2_servo_sweep; do python3 ~/f1tenth_uj/tools/real_car_tests/$t.py --self-test; done
python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py --self-test
python3 ~/f1tenth_uj/src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
```

Results: `~/f1tenth_hw_evidence/real_car_tests/<stamp>_<test>_v<speed>_<method>_erpmmax<N>_<armed|disarmed>/`
(bag, `runner_events.csv`, `keys.csv`, `publish_intervals.csv`, `vesc_speed_audit.log`, `summary.json`,
CSVs, plots, README). Not committed.
