# T3 in a hall: constant-steering circles (checklist, prepared 21 Nov, updated 21 Nov evening)

For Moatasem (operator). Purpose: real steering angle vs command on the ground (per side), real minimum
turning radius (full lock), yaw-rate / speed / radius consistency, lateral slip (judges the EKF vy = 0),
IMU y-axis sign. Code: `run_test.py --test t3_circle`, `t3_analyze.py`, `vesc_timeout_check.py`
(README T3). Numbers: `vehicle_params.yaml` blocks `t3` and `vesc_timeout_check`.

## 0. Before the hall: VESC firmware timeout (decided 21 Nov)

Why: the hand-on-the-battery-disconnect rule from T4 cannot be kept while the car circles for 15-45 s.
Software stops cover:
- SPACE release and the KILL key (runner -> mux -> brake 5 A);
- the mux 0.80 m front gate;
- the runner's geofence, turn, IMU and RF2O checks.

A frozen Jetson or a dead converter is stopped only by the VESC's own timeout: today ~1 s, then coasting
(24 Oct). Decision: **timeout ~250 ms + timeout brake current 5 A**, coordinated with Zaid. **Configured 22 Nov:
300 ms + 5 A** (previous: timeout unknown / default, brake current 0)
(converter proposal section 9).

1. [x] **Read and note (change nothing yet):** done 22 Nov (results doc).
   - VESC Tool (Windows): save the motor config and the app config as XML
     (`T1_VESC_TOOL_CHECKLIST.md` "First").
   - Write down App Settings > General `timeout_msec` and `timeout_brake_current` (row 14 there) in the
     results doc.
   - Tell Zaid what will change.
2. [ ] ~~**Before-check on the stand**~~ no longer possible (values already changed); the 24 Oct ~1 s release is the
   "before" reference. (Recommended: proves the method.) Do it with the current values,
   using the stand commands below. Expected result: `FAIL: timeout onset ~1.0 s ... released / coasting`.
3. [x] **Set** `timeout_msec` = 250 (set: **300**) and `timeout_brake_current` = 5.0, then Write App Configuration.
   Re-read both values and note them.
4. [x] **Stand check, after the change** (**PASSED 22 Nov** `20261122_150252_vesc_timeout_check_v0.50_brake5A_erpmmax4630_armed`: onset 0.29 s, -2.44 A,
   < 300 ERPM in 0.33 s; the T3 fallback is lifted) (car on the stand, wheels free; same commands). Expected result:
   `PASS: timeout onset ~0.30-0.34 s ..., BRAKED` (300 ms setting; the limit is 0.40 s).
   This `timeout_check.json` lifts the T3 fallback rules automatically.
5. [ ] **Optional USB check** (the frozen-Jetson case; no telemetry):
   - relaunch the stack and start the runner again;
   - during steady, pull the VESC USB cable;
   - the wheels must stop within ~0.3 s instead of running on ~1 s (a phone slow-motion video is the
     record);
   - plug the cable back in, check that `ls /dev/serial/by-id/` shows the ChibiOS port again, then relaunch.

Stand commands (each terminal: source ROS + workspace, `ROS_DOMAIN_ID=42`,
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`):
```bash
# T1 stack: T1b brake 5 A at the T1b-passed 0.5 m/s step (stays running)
~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.5 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t1b_erpm4630.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0
# T2 runner: n, hold SPACE (20 s steady)
~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test vesc_timeout_check --speed 0.5 --live
# T3, ~5 s into steady while SPACE is held: kill ONLY the converter (vesc_driver keeps reporting)
pkill -9 -f t1b_stop_converter.py
# then release SPACE, q in T2, Ctrl-C T1 (the stack has no converter any more: relaunch before anything else)
python3 ~/f1tenth_uj/tools/real_car_tests/vesc_timeout_check.py $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*_vesc_timeout_check_*_armed | head -1)
```

**Until a PASS exists, the fallback rules apply.** The runner enforces the first two:
- the clear disk is 1 m larger in radius;
- 0.5 m/s only after a counted 0.3 m/s run of the same command;
- a spotter stands outside the disk (that one is yours).

Also before the session: written approval for the ground step (0.5 m/s, steer up to 0.325 rad, mux cap 0.33).

## 1. Commands, order and space

Converter gains = the T2 per-side fit (left -0.874, right -1.062), centre 0.335. A `--steer` run should
produce that real angle. Per-side limits are 0.27 left / 0.325 right (`vehicle_params` steering note).
`--full-lock` commands exactly the limit, so the converter clamp sends the clip itself (0.10 / 0.68).

| # | Run | Runner argument | Servo | R nominal | Clear disk, verified | ... with fallback | Disk centre from the rear axle |
|---|---|---|---|---|---|---|---|
| 0 | disarmed rehearsal | (section 3) | - | - | - | - | - |
| 1 | left ~60 % | `--steer 0.18` | 0.178 | 1.43 m | 4.42 m | 6.42 m | 1.79 m LEFT |
| 2 | right ~60 % | `--steer -0.18` | 0.526 | 1.43 m | 4.42 m | 6.42 m | 1.79 m RIGHT |
| 3 | left ~90 % | `--steer 0.26` | 0.108 | 0.98 m | 3.31 m | 5.31 m | 1.22 m LEFT |
| 4 | right ~90 % | `--steer -0.31` | 0.664 | 0.81 m | 2.91 m | 4.91 m | 1.01 m RIGHT |
| 5 | full lock left | `--full-lock left` | 0.100 | 0.84 m (T2) | 2.98 m | 4.98 m | 1.05 m LEFT |
| 6 | full lock right | `--full-lock right` | 0.680 | 0.78 m (T2) | 2.84 m | 4.84 m | 0.98 m RIGHT |

The space needed is the same at 0.3 and 0.5 m/s. The clear disk covers every real radius from 0.8 to
1.25 x nominal (circles from the same start are nested), plus the outer body corner and 0.30 m. The runner
measures it with the LiDAR and refuses to arm if anything is inside.

**In a 5 x 5 m hall, ±0.18 and +0.26 fit only after the timeout PASS** (fallback 6.42 / 5.31 m). Without the
PASS, only runs 4-6 fit, and only at 0.3 m/s first.

Full lock measures the minimum turning radius, not a gain. Its geometry and turn check use the T2 angle.
If the real lock is much weaker (R > 1.25 x 0.84 m), the geofence stops it: that is a result too.

## 2. Hall preparation

- [ ] Hard, flat, clean floor; nothing loose (cables, tape edges) inside the area.
- [ ] Mark the disk centre with tape at the middle of the free area. For each run, place the car with its
      rear axle at the "disk centre" distance from the table, on the side AWAY from the turn, pointing
      tangentially (perpendicular to the line to the centre). Mark each start with tape; repeats start
      from the same mark.
- [ ] **Hall rule (22 Nov): operator, laptop, chairs, tables, bags and boxes are outside the WHOLE clear disk,
      including the part behind the car.** For a left circle the disk reaches ~2.2 m (0.18 rad) behind the start
      point and 4.4 m to the left; in the 22 Nov rehearsal real objects ~3 m behind-left (bearing 115-124 deg)
      were inside it and refused every run. Nobody inside the disk while armed; the spotter stands outside it.
- [ ] **Behind the car is not seen by the LiDAR.** The LakiBeam scans 270 deg (`lidar.fov_min/max_deg`
      -135.75 / +134.25): the rear ~90 deg is blind, so the clearance check cannot see that part of the disk.
      **Before every `n`, look behind the car**: that part of the disk is empty and nobody is in it.
- [ ] **Own-car returns:** tuck the yellow XT60 battery leads in (left front). They returned at 0.04-0.17 m,
      96-135 deg and refused every LEFT circle. `lidar.self_mask` ignores returns < 0.30 m there as a
      backstop (anything further away still counts). After tucking, re-record with a still scan:
      `python3 ~/f1tenth_uj/tools/real_car_tests/lidar_self_mask.py <rehearsal folder>` (car still, nobody
      near it); a changed mask goes into `vehicle_params.yaml` with a results row.
- [ ] **Mux visibility.** The mux stops MANUAL below 75 % finite returns in its ±20 deg front window, and
      the LiDAR reports up to 100 m, so hall walls count.
  - The runner predicts the visibility for every heading on the circle whose ±20 deg window lies inside the
    LiDAR's field of view, and refuses to arm below 0.80. Headings in the blind rear (and behind the masked
    leads) cannot be predicted: there the mux gate itself is the backstop. A mux stop is safe; the run is just
    not counted.
  - 22 Nov rehearsal: the direction ~60 deg left of the start heading had only 40-50 % returns at 12-20 m
    (an open area, doorway or glass), so the visibility there was 0.23-0.51. Close it off with boxes, or
    turn the start so that direction faces a wall.
  - Problem directions are open doors, glass and black curtains.
  - Fix: close the door, or put LiDAR-visible boxes (cardboard, >= 0.3 m tall: the beam is ~0.18 m above
    the floor) in that direction **outside the clear disk + 0.8 m**.
- [ ] **Do NOT use the mux low-visibility mode** (`manual_allow_reduced_visibility`). It is a gate change
      (standing rule: gates never change), it is not in the test launch, and it lets "no return" pass as
      free space. The 0.80 m obstacle check stays as it is.
- [ ] Press `m` in the runner (once the stack is up). It shows the clear disk (and FALLBACK if it applies),
      the clearance at the worst bearing, the visibility at the worst heading, and RF2O. Arm only when all
      are OK.

## 3. Terminals (each: source ROS + workspace, `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`)

**SSH over Wi-Fi (phone hotspot, hall Wi-Fi):** in EVERY terminal also run
`source ~/f1tenth_uj/tools/real_car_tests/local_env.sh` before any ros2 command. On 22 Nov, launching the stack
over a hotspot SSH session froze every session. The helper keeps DDS on the Jetson (no laptop RViz). Use it in
all terminals or in none. **SSH over USB needs none of it.**

1. **Stack** (stays running; keep the car still for 10 s after launch):
   ```bash
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.5 max_steer_rad:=0.33 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t3_erpm4630.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0 converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/t1b_converter_t3_t2gain.yaml
   ```
   - **0.3 m/s** (fallback first, or by choice): relaunch with `max_speed_mps:=0.3` and
     `vesc_t3_erpm2780.yaml`.
   - **Disarmed rehearsal (run 0, first):** the same launch with `enable_real_actuation:=false`.
2. **Checks, then the runner** (one runner per command; the stack stays up; **always end the runner with
   `q`**: that closes its bag. The 22 Nov `151221` bag was left unclosed (no `metadata.yaml`)):
   ```bash
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   # run 0, disarmed: no --live. Check: m readout, arm refusals, gyro / RF2O / visibility values. The car does
   # not move, so it ends with TURN_CHECK_FAILED: expected.
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t3_circle --speed 0.5 --steer 0.18 --t1-passed-confirmed
   # runs 1-6, armed, in this order:
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t3_circle --speed 0.5 --steer 0.18 --t1-passed-confirmed --live
   ```
   Then repeat it with `--steer -0.18`, `--steer 0.26`, `--steer -0.31`, `--full-lock left` and
   `--full-lock right` in place of `--steer 0.18`. Fallback: run `--speed 0.3` first on the 0.3 stack.
3. **Shadow inputs** (stays running; keep the car still for its first 2 s):
   `python3 ~/f1tenth_uj/tools/real_car_tests/ekf_shadow_inputs.py`
4. **C1 shadow EKF** (stays running):
   `ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_real_shadow_c1 --params-file /home/ftenth/f1tenth_uj/config/real_car/ekf_real_shadow.yaml -r odometry/filtered:=/ekf_shadow/odom`
5. **Record the shadow EKF** (stays running):
   `ros2 bag record -o ~/f1tenth_hw_evidence/real_car_tests/$(date +%Y%m%d_%H%M%S)_ekf_shadow_live/bag /ekf_shadow/odom /ekf_shadow/in/vesc /ekf_shadow/in/vesc_dw /ekf_shadow/in/imu /ekf_shadow/in/rf2o`

Per run:
1. Place the car at the start mark and press `m`. **Look behind the car** (blind sector: that part of the
   disk empty, nobody in it).
2. **Hands off for >= 2 s** (the gyro bias is the 1 s before the start).
3. Press `n` and hold SPACE.

The runner brakes by itself after 2 gyro turns (`TURNS_DONE`, counted). Any other stop is not counted (one
counted run per command; up to 2 extra attempts). Stops you may see:
- `GEOFENCE`: the circle is much larger than planned.
- `TURN_CHECK_FAILED`: wrong direction, or not turning.
- `CIRCLE_RF2O_STALE` or `CIRCLE_IMU_PROBLEM`: a sensor stream failed.
- `MUX_LEFT_MANUAL_MODE`: the mux gate.
- `HARD_TIME_LIMIT`.

Shutdown order: q, then Ctrl-C in 5, 4, 3, 1.

## 4. Afterwards (reads bags only)

```bash
python3 ~/f1tenth_uj/tools/real_car_tests/t3_analyze.py $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*_t3_circle_*_armed | head -6)
for f in $(ls -td ~/f1tenth_hw_evidence/real_car_tests/*_t3_circle_*_armed | head -6); do python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py "$f"; done
```

`t3_analyze.py` finds the shadow recording that covers each run.

Main table (`--steer` runs), per run:
- yaw rate (gyro, and gyro / RF2O);
- speed (C1 EKF / RF2O / VESC);
- R = v / yaw rate, and R from circle fits (RF2O, EKF);
- the real angle atan(0.26 / R) vs the command and vs T2;
- the ground gain per side;
- slip angle and vy;
- the IMU y sign.

**FULL LOCK table:** per side, the real minimum radius, the real wheel angle vs the T2 angle at that clip,
and the radius swept by the outer front corner.

`analyze.py` gives the brake stops.
