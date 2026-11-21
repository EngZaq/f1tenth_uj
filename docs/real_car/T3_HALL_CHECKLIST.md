# T3 in a hall: constant-steering circles (checklist, prepared 21 Nov)

For Moatasem (operator). Purpose: real steering angle vs command on the ground (per side), yaw-rate /
speed / radius consistency, lateral slip (judges the EKF vy = 0), IMU y-axis sign. Code:
`run_test.py --test t3_circle`, `t3_analyze.py` (README T3). Numbers: `vehicle_params.yaml` `t3`.

## 0. Decide before the session

- **Battery disconnect:** the T4 rule "hand on the battery disconnect" cannot be kept while the car circles
  (radius 1-2 m, 25-45 s per run). Software stops: SPACE release / KILL key (runner -> mux -> brake 5 A,
  ~0.14 m), mux 0.80 m front gate, runner geofence / turn check / IMU / RF2O checks. **A frozen Jetson is only
  stopped by the VESC timeout (~1 s, then coasting: 24 Oct stand test).** Either set the VESC Tool timeout
  ~250 ms + timeout brake current ~5 A first (pending backlog item, then a T1 stand check), or accept it with
  >= 1 m free beyond the clear disk and a spotter outside it. **Your decision; T3 is not run before it.**
- Written approval for the ground step (0.5 m/s, steer up to 0.26 rad, mux cap 0.30).

## 1. Commands and space

T3 converter gains = T2 per-side fit (left -0.874, right -1.062), centre 0.335: the commanded angle should be
the real angle. "90 %" is **0.26 rad**: 0.27 left would need servo 0.099, outside the 0.10 driving clip.

| Run | `--steer` | Servo planned | R nominal | Clear disk (diameter) | Disk centre from the rear axle |
|---|---|---|---|---|---|
| left 90 % | +0.26 | 0.108 | 0.98 m | 3.31 m | 1.22 m to the LEFT |
| right 90 % | -0.26 | 0.611 | 0.98 m | 3.31 m | 1.22 m to the RIGHT |
| left ~60 % | +0.18 | 0.178 | 1.43 m | 4.42 m | 1.79 m to the LEFT |
| right ~60 % | -0.18 | 0.526 | 1.43 m | 4.42 m | 1.79 m to the RIGHT |
| (left 50 %) | +0.15 | 0.204 | 1.72 m | 5.14 m | 2.15 m to the LEFT |

The clear disk covers every real radius from 0.8 to 1.25 x nominal (circles from the same start are
nested) plus the body corner and 0.30 m. **In a 5 x 5 m area 0.15 rad does not fit: use 0.18 as "~50 %"**
(the runner refuses 0.15 by itself if it does not fit). Do the 0.26 runs first (smallest space).
Optional 0.3 m/s: same commands, other stack (section 3).

## 2. Hall preparation

- [ ] Hard, flat, clean floor; nothing loose (cables, tape edges) inside the area.
- [ ] Mark the disk centre with tape at the middle of the free area; for each run place the car with its
      rear axle at the "centre offset" from the table on the side AWAY from the turn, pointing tangentially
      (perpendicular to the line to the centre). Mark each start with tape (repeat from the same mark).
- [ ] **No one inside the clear disk** while armed; operator at the laptop outside it, spotter outside it.
- [ ] **Mux visibility** (the mux stops MANUAL below 75 % finite returns in its +/-20 deg front window; the
      LiDAR reports up to 100 m, so hall walls count): the runner predicts it for every heading of the
      circle and refuses below 0.80. Problem directions are open doors, glass, black curtains. Fix: close
      the door, or put LiDAR-visible boxes (cardboard, >= 0.3 m tall: the beam is ~0.18 m above the floor)
      in that direction **outside the clear disk + 0.8 m** (2.5 m or more from the disk centre for 0.26 rad;
      for 0.18 rad this needs > 6 m, so prefer a position where the hall walls are visible).
- [ ] **Do NOT use the mux low-visibility mode** (`manual_allow_reduced_visibility`): it is a gate change
      (standing rule: gates never change), not in the test launch, and it lets "no return" pass as free
      space. The 0.80 m obstacle check stays as it is.
- [ ] `m` in the runner (after the stack is up) shows: clear disk, clearance (worst bearing), visibility
      (worst heading), RF2O. Arm only when all are OK.

## 3. Terminals (each: source ROS + workspace, `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)

1. **Stack** (stays running; car still 10 s after launch):
   ```bash
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.5 max_steer_rad:=0.30 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t3_erpm4630.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0 converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/t1b_converter_t3_t2gain.yaml
   ```
   0.3 m/s instead: `max_speed_mps:=0.3` and `vesc_t3_erpm2780.yaml` (relaunch).
2. **Checks, then runner** (one runner per command; the stack stays up):
   ```bash
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t3_circle --speed 0.5 --steer 0.26 --t1-passed-confirmed --live
   ```
   then `--steer -0.26`, `--steer 0.18`, `--steer -0.18` (q ends each runner).
3. **Shadow inputs** (stays running; car still for its first 2 s):
   `python3 ~/f1tenth_uj/tools/real_car_tests/ekf_shadow_inputs.py`
4. **C1 shadow EKF** (stays running):
   `ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_real_shadow_c1 --params-file /home/ftenth/f1tenth_uj/config/real_car/ekf_real_shadow.yaml -r odometry/filtered:=/ekf_shadow/odom`
5. **Record the shadow EKF** (stays running):
   `ros2 bag record -o ~/f1tenth_hw_evidence/real_car_tests/$(date +%Y%m%d_%H%M%S)_ekf_shadow_live/bag /ekf_shadow/odom /ekf_shadow/in/vesc /ekf_shadow/in/vesc_dw /ekf_shadow/in/imu /ekf_shadow/in/rf2o`

Per run: place at the start mark, `m`, **hands off >= 2 s** (gyro bias = the 1 s before the start), `n`,
hold SPACE. The runner brakes by itself after 2 gyro turns (`TURNS_DONE`, counted); any other stop is not
counted (one counted run per command; up to 2 extra attempts). Stops you may see: `GEOFENCE` (circle much
larger than planned), `TURN_CHECK_FAILED` (wrong direction / not turning), `CIRCLE_RF2O_STALE`,
`CIRCLE_IMU_PROBLEM`, `MUX_LEFT_MANUAL_MODE` (mux gate), `HARD_TIME_LIMIT`. Shutdown: q, Ctrl-C 5, 4, 3, 1.

## 4. Afterwards (reads bags only)

```bash
python3 ~/f1tenth_uj/tools/real_car_tests/t3_analyze.py $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*_t3_circle_*_armed | head -4)
for f in $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*_t3_circle_*_armed | head -4); do python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py "$f"; done
```
`t3_analyze.py` finds the shadow recording that covers each run. Per run: yaw rate (gyro, and gyro / RF2O),
v (C1 EKF / RF2O / VESC), R = v / yaw rate and R from circle fits (RF2O, EKF), real angle atan(0.26 / R) vs
command and vs T2, ground gain per side, slip angle and vy, IMU y sign. `analyze.py` gives the brake stops.
