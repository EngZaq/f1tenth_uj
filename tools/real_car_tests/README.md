# Real-car tests (`tools/real_car_tests/`)

Test tools for the physical car. Owner: Moatasem. Standard values:
`docs/real_car/REAL_CAR_BASELINE_2026-11-14.md`; numbers live only in
`config/real_car/vehicle_params.yaml`; results go to
`~/f1tenth_hw_evidence/real_car_tests/<stamp>_<test>_v<speed>_erpmmax<N>_<armed|disarmed>/` (not committed).

| File | What it does |
|---|---|
| `real_car_test.launch.py` | The one test stack: observe_only (LiDAR, RF2O, SLAM), BMI160 + `base_link -> imu_link`, `vesc_driver` (test yaml), `vesc_to_odom`, one `ackermann_to_vesc` (0.320 / -0.50), mux. **Real VESC: the servo centres as soon as it starts.** |
| `run_test.py` | Keyboard runner over SSH. Drives only through the mux inputs (`/manual/drive`, `/joy` button 5, `/control/request_mode`) at 50 Hz; records the bag; zero for >= 1 s after every stop. |
| `analyze.py` | Bag -> `summary.json`, CSVs, plots, `## Analysis` in the folder README. |
| `preflight.sh` | Read-only checks: `clean` before launching, `stack` before the runner. |
| `params.py` | Loads `vehicle_params.yaml`; checks `config/real_car/*.yaml` agree with it. |

## Rules

- The operator starts every launch and every runner. The battery disconnect / kill switch is
  the stop; the mux and the runner are not emergency stops.
- One stack at a time. `preflight.sh clean` before launching, `preflight.sh stack` before the runner.
- Nothing else may publish `/manual/drive`, `/joy`, `/control/request_mode` (the runner refuses),
  nor `/commands/motor/brake|current|duty_cycle|position`.
- Nothing here subscribes to `/drive`, `/drive_calibrated` or `/commands/*` either, not even the bag:
  the mux counts subscribers and latches `ACTUATOR_GRAPH_OWNERSHIP_FAULT` (dry run 14 Nov).
  The sent ERPM comes from the vesc_driver `VESC_SPEED_AUDIT` log (teammates' uncommitted audit
  build), which the runner copies to `vesc_speed_audit.log`; else from the mux output in `/control/status`.
- `preflight.sh stack` allows one duplicate node name: `/rf2o_laser_odometry` x2 from a single rf2o
  process (F-14, inner node inside rf2o). Any other duplicate fails.
- Caps are raised only by launch arguments (`max_speed_mps`, `max_steer_rad`, `vesc_yaml`), per approved step.
- Do not use `src/f1tenth_bringup/launch/vesc.launch.py`: it loads `config/vesc.yaml`
  (`brake_max 200000` A, `current_max 100` A) with no remapping.

## Keys (SSH terminal, no Enter)

`n` arm (SPACE released) · hold `SPACE` drive · `s` skip (idle) · `q` quit (idle) · `m` placement (ground).
**KILL** (20 Nov): while arming, armed, driving or stopping, ANY key but SPACE sends zero at once (same
path as a SPACE release: mux STOP -> converter brake), prints `*** KILL ***` and latches: nothing drives
until SPACE is released and `n` pressed. `q` there kills, then the runner ends. KILL stops never count.
Not faster than a SPACE release and not an emergency stop. SPACE is a deadman from
terminal auto-repeat: 0.7 s grace after the first press, then 0.25 s between repeats. If your
terminal's repeat delay is longer than 0.7 s the run stops at once: shorten it locally.

## Session

```bash
# terminal A
~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=false
# terminal B
~/f1tenth_uj/tools/real_car_tests/preflight.sh stack
python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test dry_run
python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py <results folder printed by the runner>
```

## T1: stop behaviour on the stand (F-01)

One run per speed step, each with its own caps: mux `max_speed_mps:=<step>` and
`vesc_yaml:=config/real_car/vesc_test_erpm<N>.yaml` (0.5 -> 4630, 1.0 -> 9255, 1.5 -> 13883 ERPM).
`run_test.py` refuses a run whose mux cap or `speed_max` does not match the step, and refuses step
N+1 until an analysed, armed run of step N has >= 5 stops, none rebounding > 300 ERPM, VESC fault 0
(`step_gate`). Run `analyze.py` on every T1 folder: the gate reads its `summary.json`.

T1b (stop-method comparison) uses `converter:=t1b stop_method:=speed0|brake|current0`
(`brake_current_a`, `brake_hold_s`) with `vesc_t1b_erpm4630.yaml` (brake_max 10 A) and
`run_test.py --test t1b_stop`. The step gate is per stop method.

Before T1b: fill in `docs/real_car/T1_VESC_TOOL_CHECKLIST.md`.

## T4: straight line on the ground (`t4_straight`, 20 Nov)

Steer 0 toward a wall, brake 5 A, 3 counted repeats (up to 2 extra attempts), step 0.5 m/s only.
Stack: `converter:=t1b stop_method:=brake brake_current_a:=5.0 max_speed_mps:=0.5`,
`vesc_t1b_erpm4630.yaml`, mux steer cap at its default 0.10 (the runner refuses another method or
steer cap). Runner: `run_test.py --test t4_straight --speed 0.5 --t1-passed-confirmed [--live]`.

- **Brake point** (LiDAR range from the laser, `vehicle_params.yaml t4`): max(mux gate 0.80 +
  guard 0.10, bumper 0.09 + v x reaction 0.20 s + v^2/(2 x 1.0) + margin 0.5) = **0.90 m at 0.5 m/s**.
  The runner sends zero there; the mux 0.80 m gate is only the backstop.
- **Brake trigger and arm window use the front median (+/-3 deg) only** (20 Nov, narrow floor): a side
  object never brakes the runner early. A side object closer than 0.80 m in the mux +/-20 deg sector
  makes the **mux** latch STOP -> zero on `/drive` -> converter brake 5 A (same brake; the repeat ends
  `MUX_LEFT_MANUAL_MODE` and does not count). Lane width: see the handoff (T4).
- **Arm** only with the front range in **2.15..3.90 m** (ramp 0.25 + steady 1.0 + brake point; max = brake
  point + 3.0 m), |ERPM| < 100 and nothing within 0.90 m in the mux sector (`MUX_SECTOR_OBJECT`).
  `m` prints the placement readout (front, window, mux nearest, side walls).
- Drives until the brake point (no fixed time); time limit = expected time + 1 s. `FRONT_RANGE_LOST`
  stops at once. Only `BRAKE_POINT` stops count; SPACE release still brakes but does not count.

**IMU guard (F-18, 20 Nov):** the runner refuses to arm on `IMU_FROZEN` (> 50 % identical consecutive
`/imu/data_raw` samples in the last 1 s) or `IMU_STALE`, and logs every IMU status change to
`runner_events.csv` (a freeze while driving does not stop the run: the IMU is not in the control loop).
`preflight.sh stack` checks 2 s with the same rule. Recovery: restart the stack. 0 publishers on
`/imu/data_raw` (21 Nov: the bmi160 node exits if the car moves during its start-up gyro calibration) ->
`IMU_STALE(IMU node not running: restart the stack and keep the car still for 10 s)`.

## T3: constant-steering circles on the ground (`t3_circle`, prepared 21 Nov)

Hall checklist, commands and decisions: `docs/real_car/T3_HALL_CHECKLIST.md`. Values: `vehicle_params.yaml t3`.
Stack: as T4 plus `max_steer_rad:=0.30`, `vesc_t3_erpm4630.yaml` (or `2780` for 0.3 m/s; servo clip 0.10-0.68)
and `converter_params_file:=config/real_car/t1b_converter_t3_t2gain.yaml` (T2 per-side gains -0.874 / -1.062,
centre 0.335; the converter now clamps its servo output to 0.10-0.68). Runner:
`run_test.py --test t3_circle --speed 0.5 --steer <+/-rad> --t1-passed-confirmed [--live]`.

- **Pre-run refusals:** mux steer cap > 0.30, converter gains not the T3 values, planned servo outside the
  converter clip (e.g. +0.30 rad -> 0.073) or the vesc_driver clip (a T4 yaml).
- **Clear disk:** circles from the same start and heading are nested, so one disk around the largest plausible
  circle (1.25 x L / tan(steer); smallest 0.8 x) + body corner + 0.30 m covers them all: +/-0.26 rad 3.31 m,
  +/-0.18 4.42 m, +/-0.15 5.14 m diameter. Arm refused if any LiDAR return (car body excluded) is inside it.
- **Mux visibility:** arm refused if, for any heading of the circle, the +/-20 deg window has fewer finite returns
  than the mux `manual_min_front_valid` + 0.05. The mux gate itself is unchanged and stays the backstop.
- **Drive stops:** `TURNS_DONE` (gyro heading = 2 turns, bias from the 1 s before the start; the only counted
  stop), `TURNS_RF2O_BACKUP`, `GEOFENCE` (RF2O base_link > 0.20 m outside the largest circle), `TURN_CHECK_FAILED`
  (after 2 s the 0.5 s mean yaw rate must have the commanded sign and >= 30 % of v tan(steer) / L),
  `CIRCLE_RF2O_STALE` (> 0.3 s), `CIRCLE_IMU_PROBLEM`, `HARD_TIME_LIMIT`, plus SPACE / KILL / mux as in T4.
  Brake 5 A (t1b converter). `m` prints the clear disk, clearance, visibility and RF2O state.

`t3_analyze.py <t3 folder> [...] [--shadow <ekf_shadow_live>] [--t2 <T2 folder>]`: per run yaw rate (gyro, gyro /
RF2O), v (C1 EKF / RF2O / VESC), R = v / yaw rate and Kasa circle fits (RF2O, EKF paths), real angle atan(L / R)
vs command and vs the T2 per-side fit at the servo actually sent, ground gain per side, slip angle / vy (RF2O
path vs yaw), IMU y sign (gravity-perpendicular +y vs v x yaw rate). Writes `t3.json`. Synthetic end-to-end
check 21 Nov (R 1.10 m, 2 deg slip, tilted IMU): R 1.100, angle 13.3 deg, slip -1.9 deg, IMU y +1.00.

`t4_yaw_plot.py <T4 folder> [...]` (20 Nov): yaw rate (BMI160 gz, bias from the 1 s before the
command, base_link = -gz; RF2O pose derivative) vs time and vs distance per counted repeat, curvature
yaw rate / VESC vx with a one / two-level / linear test (STEP, CONSTANT, OSCILLATING, GRADUAL),
turning points, motion / ramp-end / brake marks. A frozen IMU (identical samples) switches that repeat
to RF2O. Writes `t4_yaw.png` and `t4_yaw.json` into the folder; reads the bag only.

`ekf_shadow_inputs.py` (21 Nov, Phase F): live shadow EKF input node, observation only: subscribes `/sensors/core`,
`/imu/data_raw`, `/odom_rf2o`, `/t1b_stop_converter/status`; publishes only `/ekf_shadow/in/{vesc,vesc_dw,imu,rf2o}`
(base_link, no TF): VESC vx = ERPM / 8414 + vy = 0; `vesc_dw` de-weighted x100 while braking or 0 < |ERPM| < 2000;
yaw rate -(gz - bias) and gravity-free forward accel (IMU x points backward, mount tilted ~10 deg) from a start-up
still calibration (200 samples, retried with a log line until still); RF2O twist. The replay uses the same functions.
Not Zaid's `tools/shadow_covariance_adapter.py` (his `/shadow_inputs/*`, unchanged).

`ekf_shadow_replay.py [folder ...] [--variants A B C0 C1 C2] [--calib pre|startup]` (21 Nov, Phase F, offline only,
refuses if a car stack runs): builds the inputs from a T4 bag with `ekf_shadow_inputs.py`, runs `robot_localization`
`ekf_node` per repeat and variant (`config/real_car/ekf_real_shadow.yaml`) in isolated domain 88 with `use_sim_time`
+ `ros2 bag play --clock`, and compares `/ekf_shadow/odom` (laser point along the start wall normal) with the
`analyze.py --t4` LiDAR references: total, start-ramp and brake-phase error, heading vs RF2O, hold drift, ekf_node CPU,
latency-free cruise gain. `--live-check`: runs the live node on a bag and compares every message with the replay
inputs. `--rf2o-cpu`: rf2o CPU on this machine from a bag's /scan. `--live <shadow folder> <T4 folder>`: a recorded
LIVE `/ekf_shadow/odom` (`<stamp>_ekf_shadow_live/bag`) vs the T4 LiDAR, same metrics, plus RF2O max gap per repeat
(scan stamps; stale > 0.2 s = C1 had no fallback); reads bags only. The live inputs node logs RF2O freshness
(rate, max gap, latency) every 10 s and warns at once when `/odom_rf2o` is older than 0.2 s. Writes `ekf_shadow_replay[_startup].json`.

`analyze.py --t4 [folder ...]` (default: latest T4 folder; several = pooled summary as well), also
written to the folder README: per repeat stop distance (LiDAR at the converter's brake command vs at
standstill, VESC odom over the stop, and from the runner's zero), decel v^2/2d (LiDAR and VESC),
drive distance LiDAR / VESC / RF2O -> suggested `speed_to_erpm_gain` = gain x d_VESC / d_LiDAR,
start delay (converter DRIVE -> ERPM >= 150 / LiDAR -2 cm), side drift (wall fit, side walls, RF2O)
and heading change, F-01 ground rebound. LiDAR distance uses a line fit to the wall (+/-10 deg:
perpendicular distance + heading) when it fits on >= 90 % of scans, else the front median.
Flags: pre-start not still, standstill noisy, curved > 5 deg, no motion. Nothing is written to
`vehicle_params.yaml`: values change only through a results row.

## T2: servo sweep (servo-only stack)

Motor phase wires disconnected. **Servo power:** the servo is powered by the 6 A UBEC (fed from the
battery); only the signal comes from the VESC. So the battery must be connected for the servo to
move, and a binding servo draws its stall current from the UBEC (up to ~6 A), which the VESC does not
limit: answer "binding" at the first buzz. `t2_servo_only.launch.py servo_min:=.. servo_max:=..` starts only
`vesc_driver` with every motor limit 0 (`vesc_t2_servo_only.yaml`) and the servo clip just outside
the approved range. `t2_servo_sweep.py` (or `run_test.py --test t2_servo_sweep`) checks that, then
sweeps from 0.320: 0.02 steps inside the approved range (`vehicle_params.yaml` `t2`, 0.10-0.60 since
19 Nov, so one launch with `servo_min:=0.10 servo_max:=0.60` covers it), 0.01 beyond, left side (servo down) first, then
right. Each new outward point is visited for <= 0.8 s, then you answer `y` (binding / end stop /
noise: the side ends at the previous point), `n` (confirmed, held for measuring) or `e` (no binding,
but end this side at the previous point). A point outside the clip asks: Enter = pause for a wider
clip, `e` = end this side there. Both sides run first; then one relaunch command covering both is
printed (the relaunch is your approval), then `--resume <folder>`. Results: `points.csv`, `fit.json`
(gain, offset, residual, per-side fits, max angles, backlash, binding points, sides ended by the
operator).

Right-side limit extension (19 Nov): `t2_servo_sweep.py --extend-right <T2 folder>` continues the
right side of that run from its last free point in 0.02 steps (same 0.8 s visit and y / n / e prompt;
`s` at the measure prompt), up to the driver clip (launch `servo_max` up to 1.0; min stays 0.10) and
the VESC maximum 1.0. It never commands the left side. When the side ends it returns to the last free
point for its angles (unless measured), appends to that folder's `points.csv`, refits `fit.json`
(`max_right_*`, `right_last_free_servo`, `right_stop_servo`; old file kept as
`fit_before_extend_right.json`) and writes `extend_right.json` (proposed right driving clip = last
free point, one step before the stop).

Offline checks (no ROS graph, nothing published): `run_test.py --self-test` (its shutdown probe
uses an isolated domain 77, localhost only, no publishers), `analyze.py --self-test`,
`t1b_stop_converter.py --self-test`, `t2_servo_sweep.py --self-test`,
`python3 t2_servo_only.launch.py --self-test` (clip ceiling 0.10-0.60; starts no node).
Comparison of all analysed T1/T1b runs: `analyze.py --table`; one stop in detail: `analyze.py <folder> --stop-plot N`.
