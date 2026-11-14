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

Offline checks (no ROS graph, nothing published):
`run_test.py --self-test`, `analyze.py --self-test`.
