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

`n` arm (SPACE released) · hold `SPACE` drive · `s` skip · `q` quit. SPACE is a deadman from
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

## T2: servo sweep (servo-only stack)

Motor phase wires disconnected. `t2_servo_only.launch.py servo_min:=.. servo_max:=..` starts only
`vesc_driver` with every motor limit 0 (`vesc_t2_servo_only.yaml`) and the servo clip just outside
the approved range. `t2_servo_sweep.py` (or `run_test.py --test t2_servo_sweep`) checks that, then
sweeps from 0.320: 0.02 steps inside 0.22-0.455, 0.01 beyond. Each new outward point is visited
for <= 0.8 s, then you answer "binding?"; only then is it held for measuring. A point outside
the clip pauses the sweep and prints the relaunch command (the relaunch is your approval), then
`--resume <folder>`. Results: `points.csv`, `fit.json` (gain, offset, residual, per-side fits,
max angles, backlash).

Offline checks (no ROS graph, nothing published): `run_test.py --self-test` (its shutdown probe
uses an isolated domain 77, localhost only, no publishers), `analyze.py --self-test`,
`t1b_stop_converter.py --self-test`, `t2_servo_sweep.py --self-test`.
Comparison of all analysed T1/T1b runs: `analyze.py --table`; one stop in detail: `analyze.py <folder> --stop-plot N`.
