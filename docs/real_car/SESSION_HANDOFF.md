# Real-car tests: session handoff (updated 2026-11-23 evening, laptop session after the car session, no car)

For Moatasem and for the next Claude Code session. Read this, then
[REAL_CAR_BASELINE_2026-11-14.md](REAL_CAR_BASELINE_2026-11-14.md) (standard values) and
[REAL_CAR_TEST_RESULTS.md](REAL_CAR_TEST_RESULTS.md) (every measured result). Code:
`tools/real_car_tests/` (README there, sections "23 Nov" and "23 Nov eve"). Numbers: `config/real_car/vehicle_params.yaml` only.

**Status 23 Nov evening (laptop, no car; nothing actuated; ROS domain 43, localhost only, CYCLONEDDS_URI unset):**
- **Deadman stops of the 23 Nov corridor teleop = Wi-Fi SSH, not the code** (results doc row "Teleop deadman stops"):
  only run `174200` (low visibility 0.10) drove; its 2 segments ended with DEADMAN_RELEASED after 0.79 m and 0.11 m
  (VESC). Segment 2: no key for 0.327 s, then 10 `w` keys with the SAME receive time = a stalled SSH stream
  delivered in one burst (twice more later: 0.325 s + 10 keys); the runner allows 0.25 s between repeats. Segment 1:
  `a` stream ended, next key `d` 0.48 s later, alone (released `a` before pressing `d`); a 6 ms near miss earlier
  (`d` held 0.50 s then `a` 0.256 s later vs the 0.25 s window). Runs `170558`/`171342`/`171649` stopped 40-60 ms
  after DRIVE_START by the mux `LATCH_STOP_FRONT_SCAN_NOT_CLEAR` (before the 0.10 visibility launch argument).
  **Fix = PS4 controller on the Jetson** (no network in the deadman loop). The keyboard deadman stays as it is.
- **PS4 teleop implemented** (`run_test.py --test teleop_ps4`, README "23 Nov eve"): joy_node on the Jetson remapped
  to `/ps4/joy`; the runner reads it and stays the only `/joy` / `/manual/drive` publisher (the mux reads `/joy`
  button 5 + TTL 0.20 s; a second `/joy` publisher is refused by the runner, and by `preflight.sh stack` (`/joy`
  pub 0)). Laptop test with the real mux + t1b converter (table, brake 5 A) + stub VESC + synthetic `/ps4/joy`:
  R1 release, circle KILL, joy silent (Bluetooth drop: `PS4_STALE` after 0.21 s) and laptop SPACE each -> brake 5 A
  at the stub 30-70 ms later; options with R1 held refused; joy_node on `/joy` refused; steering servo 0.239-0.660.
- **Corridor map** (`hall_loop_20261123_172534/maps/`, results doc "Corridor map"): 27.5 x 6.9 m grid, 44 m^2 known,
  walls straight (rms 6-8 cm over 6-8 m, 1.3 deg from parallel), width 1.73 m between the wall fits (placement
  readout 0.86 + 0.76 m + car agrees; not 2.5-3 m), right wall doubled in ~30 % of columns. **Along-corridor scale is
  NOT observable by LiDAR here: RF2O 3.44 m, SLAM 4.21 m, VESC 9.93 m (tach = ERPM integral) for the "~7 m" push**
  -> measure the push with a tape next time; corridor maps need the wheel odometry.
- **VESC config:** no XML on the laptop (`real_car_evidence/vesc_config/` empty): `config/real_car/vesc_backup/README.md`
  holds the reported before -> after table and the slots for the XMLs. **Timeout = 250 ms** (VESC Tool, 23 Nov)
  fixed everywhere (vehicle_params, checklists, proposal, results doc, this file); 22 Nov onset 0.29 s fits 250 ms.
- **IMU TF** (`vehicle_params.yaml imu`, used by the test launch): roll 0.0497, pitch 3.2935 (pi + 8.70 deg), yaw 0
  from the 23 Nov still accel (1.54, -0.50, -10.05), tilt 9.15 deg; position unchanged (PROVISIONAL). **Shadow
  EKF yaw rate now = gyro projected on the still gravity** (`ekf_shadow_inputs.YAW_MODE 'tilt'`; -gz alone =
  0.987 x), **R_gz 1.2e-3 applied** (was 8.0e-3). C1 circle replay: tight turns EKF - RF2O **-1.21 % -> +0.02 %**, end
  position 0.144 -> 0.114 m; the -0.18 run now +1.5..+2.8 % (RF2O / run dependent).
- **EKF noise** (`ekf_r_nis.py`, results doc "EKF R / NIS"): steady cruise RF2O vx R 0.9-1.3e-2 (= the 1.0e-2 in
  use), VESC vx self-noise R 1-2e-4 but its error is scale / slip, keep 5e-3; RF2O reads 0.01 m/s below VESC. C1
  NIS on the T3 circles: VESC 0.006, gyro 0.18, RF2O 0.26 (all << 1: conservative, never overconfident); Q unchanged.
- **Route planning** (`ROUTE_PLANNING_PROPOSAL.md`): sim pipeline = extract_robust_centerline (needs a CLOSED loop,
  0.30 m clearance) -> orient -> body/steering optimizer (wheelbase 0.3302 and 24 deg hard-coded); for the car
  wheelbase 0.26, steer 0.28 rad (R_min 0.90 m, plan R >= 1.15 m), cap 0.5, decel <= 0.8, min 0.2 m/s; smallest
  way: sim step 2 as is + one new `real_route.py` (proposal, not written). The 23 Nov corridor has no loop.

**Next car session (24 Nov): PS4 teleop, then the track decision.** Every terminal on the Jetson: ROS +
`install/setup.bash` sourced, domain 42, `source ~/f1tenth_uj/tools/real_car_tests/local_env.sh` (Wi-Fi SSH).
1. **Update** (Python + yaml only, no colcon build): `cd ~/f1tenth_uj && git pull --ff-only origin
   moatasem/real-car-tests`, then `for t in run_test analyze t1b_stop_converter hall_loop_map ekf_shadow_inputs
   ekf_r_nis vesc_timeout_check; do python3 tools/real_car_tests/$t.py --self-test; done`.
2. **Controller on the Jetson, once** (USB cable = no pairing; Bluetooth for walking with the car):
   ```bash
   bluetoothctl list                      # a controller must be listed; none -> use the USB cable
   bluetoothctl                           # then inside:
     power on
     agent on
     default-agent
     scan on                              # controller: hold SHARE + PS until the light bar flashes fast
     pair <MAC of "Wireless Controller">
     trust <MAC>
     connect <MAC>
     scan off
     quit
   ls /dev/input/js*                      # /dev/input/js0 (USB or Bluetooth)
   grep -A4 -i "wireless controller" /proc/bus/input/devices
   groups | grep -w input || echo "NOT in group input: sudo usermod -aG input $USER, log out and in"
   ros2 run joy joy_enumerate_devices     # joy (SDL) sees it: device 0
   ```
   Later sessions: press PS once (it reconnects to the trusted Jetson).
3. **joy_node + mapping check** (terminal C, stays running; started AFTER `preflight.sh clean` + the launch: `clean`
   refuses any ros2 process):
   ```bash
   ros2 run joy joy_node --ros-args -r joy:=/ps4/joy -p device_id:=0 -p autorepeat_rate:=50.0 -p sticky_buttons:=false -p deadzone:=0.05
   ```
   Terminal D, quick check, then Ctrl+C: `ros2 topic hz /ps4/joy` (~50 Hz) and `ros2 topic echo /ps4/joy`: R1 ->
   `buttons[5]: 1`, options -> `buttons[9]`, circle -> `buttons[1]`, left stick left -> `axes[0]` about +1.0. A
   different R1 index: change `ps4.deadman_button` in `vehicle_params.yaml` (the runner's check names what it saw).
   **Buttons:** options = arm · HOLD R1 = drive (0.3 m/s) · left stick = steer (proportional, +/-0.28 rad) · release
   R1 = brake 5 A · cross / circle / square = KILL (latched until options) · triangle = placement readout (idle).
   Laptop: `q` ends (idle); while armed every key = KILL. Controller silent > 0.20 s = stop (`PS4_STALE`).
   **Limit:** a Bluetooth link loss with R1 held is seen only when joy_node stops publishing (the kernel may keep
   the last state until the link times out, seconds): stay within ~3 m of the car, hand on the disconnect.
4. **Stand check of the PS4 path first** (wheels off the ground, box 1.5-2 m ahead for the standard 0.75 front gate):
   ```bash
   # A
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.3 max_steer_rad:=0.28 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t3_erpm2780.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0 converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/t1b_converter_t3_table.yaml
   # C: joy_node (step 3)
   # B (tmux recommended: `tmux new -s run`; a Wi-Fi drop then does not end the run, the controller keeps control)
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test teleop_ps4 --speed 0.3 --fast
   ```
   Controller check (hands off; LEFT stick left; R1 press + release), then: options -> hold R1 2 s -> release
   (brake) · options -> R1 -> circle (KILL) · options -> R1 -> switch the controller off (hold PS 10 s) or
   `bluetoothctl disconnect <MAC>` in terminal D -> `PS4_STALE`. `q`. Check `ps4_events.csv` + `runner_events.csv`.
5. **Corridor / hall teleop with the PS4** (as step 4, launch + `manual_min_front_valid:=0.10` for the corridor;
   the runner then allows 0.3 m/s only). Car on the tape mark, still 10 s at start (IMU), back on the mark at the
   end, **measure the driven distance with a tape** (scale check: RF2O/SLAM under-read the corridor). Map on the
   laptop: `python3 tools/real_car_tests/hall_loop_map.py <run folder>`.
6. **Track decision** (`ROUTE_PLANNING_PROPOSAL.md`): the route tools need a CLOSED loop (an island / furniture
   block), lanes >= ~1.2 m, turns R >= 1.15 m; the corridor alone is out-and-back. Then T5b and the steering-table
   ground check as in the 23 Nov list below (items 4-5).

**Status 23 Nov (laptop night session 22-23 Nov, no car; nothing actuated; ROS domain 43/44, localhost only):**
- **A. Fixed: `run_test.py` refused T5b with "t1b_stop_converter parameters unreadable: {}".** Cause (reproduced
  live with the converter alone): rclpy Humble `parameter_service.py` answers a GetParameters batch with an EMPTY
  list as soon as one requested name is declared but unset; in gain mode `steer_table_left/right` are typed and
  unset, so `method` etc. were lost too (not a timeout, not the node name). Fix: per-name read after a short
  answer (unset = None = gain mode), required parameters named when missing. Live-tested: gain mode read OK
  (tables None, mode gain), table yaml read OK (limits +/-0.28), an invalid table set at runtime still refused.
- **B. Faster tests:** `--fast` (no T3 clear-disk / visibility refusal, no 0.3-first gate, no
  `--t1-passed-confirmed`, hands-off 0.5 s; kept: SPACE release = brake 5 A, converter + stop-method check, caps,
  IMU freeze, VESC status, step gate T1, T3 geofence / turn check; `"fast": true` in run_info.json); bags zstd per
  file (all analysis reads them: `analyze.read_bag`); mux armed + no `--live` -> live mode on with one line.
- **C. Analysis (bags in `~/Data/real_car_evidence`, results doc rows "T3 gyro noise in turns" and "C1 replay in
  circles"):** gyro z white noise in T3 steady turns sigma_hf 0.0065 rad/s (var 4.2e-5, ~48x T5a static R
  8.85e-7; tile T4 cruise ~4.5e-5, carpet 3.0e-4). **Proposed imu0 R_gz 1.2e-3** (`ekf_shadow_inputs.R_IMU_WZ`,
  read by imu0 of `ekf_real_shadow.yaml`; now 8.0e-3; NOT applied). C1 replay on the T3 circles (13 repeats): heading = A (gyro), EKF turns **1.2 % less than RF2O in tight turns** (~4 deg per turn), path +2.9 % vs RF2O, end position error 0.143 m mean (A 0.173); R 1.2e-3 changes nothing measurable.
- **D/E. Hall loop:** `teleop` runner profile (keyboard: hold `w`/SPACE straight, `a` left, `d` right; release =
  brake 5 A; table steering +/-0.28 rad, 0.7 rad/s; tested on the laptop through the real mux + converter with a
  stub VESC: servo 0.110..0.660, brake 5 A 2 s after every release / KILL, never speed 0); `hall_loop_record.sh`
  (hand-push fallback, sensor check + zstd record); `hall_loop_map.py` (offline slam_toolbox, loop closing off /
  on, lap closure at the tape mark; tested on T3 bag `155039`: 2.02 laps, both maps ~170 m^2, closure RF2O 0.170 m /
  +8.9 deg, SLAM off 0.182 / +10.1, on 0.237 / +12.6 (T3 stops ~720 deg after the start, not on a mark); the 4 Nov
  "manual push" bag did not move, correctly reported). The team keyboard GUI `~/continuous_keyboard_mux.py` is not
  used: not in the repo (not reviewable from here), a GUI over SSH, F-13 latch unresolved. PS4: not used (the joy node would be a second `/joy`
  publisher next to the runner; the runner refuses that).

**23 Nov at the car: corridor teleop blocked by the mux front-visibility gate** (front valid fraction 0.175 < 0.75,
`FRONT_UNKNOWN`, nearest 2.25 m: the far end is out of range). Approved by Moatasem: mux reduced-visibility floor
0.35 -> 0.10 (`0eae65f`, own commit for Zaid's review), launch argument `manual_min_front_valid` (default 0.75;
below 0.75 it turns the reduced-visibility option on), runner: a lowered threshold only for `teleop` at
`--speed <= 0.3` (banner), T3 keeps >= 0.35, every other profile refuses. Kept: 0.80 m obstacle stop (needs a finite
return), release = brake 5 A, caps, IMU / VESC checks, VESC timeout verified. Jetson: `git pull` then
`colcon build --packages-select f1tenth_bringup` (the launch runs the installed mux copy).

**23 Nov list (done: 1-3 on 23 Nov; items 4-5 still open, after the 24 Nov PS4 steps above). Next car session, in order (all Moatasem's runs; every terminal: ROS + `install/setup.bash` sourced, domain 42,
`source ~/f1tenth_uj/tools/real_car_tests/local_env.sh` over Wi-Fi):**
1. **Update:** `cd ~/f1tenth_uj && git pull --ff-only origin moatasem/real-car-tests` (Python tools only, no
   colcon build), then `for t in run_test analyze t1b_stop_converter hall_loop_map; do python3
   tools/real_car_tests/$t.py --self-test; done`.
2. **Sensor check** (with the stack of step 3 up, terminal C): `~/f1tenth_uj/tools/real_car_tests/hall_loop_record.sh
   --check-only`. Needs /scan >= 25 Hz, /imu/data_raw >= 90 Hz not frozen, /odom_rf2o >= 10 Hz; it says what to
   restart (LiDAR exit -6 / IMU not started / Errno 121 retry).
3. **Hall loop by TELEOP** (0.3 m/s: better scans, first teleop on the car; 0.5 allowed with `vesc_t3_erpm4630.yaml`
   + `max_speed_mps:=0.5` + `--speed 0.5`). Car on the START TAPE MARK, still while the stack starts (IMU
   calibration 10 s).
   ```bash
   # A
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.3 max_steer_rad:=0.28 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t3_erpm2780.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0 converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/t1b_converter_t3_table.yaml
   # B
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test teleop --speed 0.3 --fast
   ```
   Wait 3 s after the runner starts (still reference), `n`, HOLD `w` (or SPACE) to drive, roll to `a` / `d` (press
   the new key before releasing the old one), release everything = brake. 2 laps; each release ends a segment,
   `n` re-arms. Keep >= 1 m from walls ahead (mux front gate 0.80 m latches STOP: `n` again). If the mux latches
   `FRONT_UNKNOWN` again and again (hall visibility 0.44-0.65 < 0.75; low-visibility mode stays T3 only): go to
   3b. At the end put the car back ON the tape mark (pushing by hand is fine, RF2O tracks it), still 5 s, `q`.
   The runner bag is the recording (`/scan /imu/data_raw /odom_rf2o /tf /tf_static /sensors/core
   /odom_vesc_observe /control/status /manual/drive /t1b_stop_converter/status ...`; `/drive` is never recorded:
   mux subscriber count; the command side is `/control/status` + `/manual/drive` + the converter status).
   **3b. Fallback, hand push** (nothing commands the VESC):
   ```bash
   # A
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=true
   # B (car still 10 s)
   ros2 launch f1tenth_imu imu.launch.py
   # C
   ~/f1tenth_uj/tools/real_car_tests/hall_loop_record.sh
   ```
   Map (laptop, after copying the folder; stack down): `python3 tools/real_car_tests/hall_loop_map.py <folder>`
   -> `maps/map_lc_off.*`, `maps/map_lc_on.*`, lap-closure table.
4. **T5b with --fast** (stand, wheels off the ground; gain mode now accepted):
   ```bash
   # A
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.5 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t1b_erpm4630.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0
   # B
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t5b_stand_imu --speed 0.5 --fast
   python3 ~/f1tenth_uj/tools/real_car_tests/t5_imu_analyze.py stand
   ```
5. **Steering-table ground check** (hall, low-visibility mode, 0.3 m/s; expect **R ~1.43 m, pass 1.29-1.57 m**
   both sides; clear disk 4.42 m: with `--fast` the runner does not check it, look yourself, spotter outside):
   ```bash
   # A
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh clean
   ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.3 max_steer_rad:=0.33 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t3_erpm2780.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0 t3_low_visibility:=true converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/t1b_converter_t3_table.yaml
   # B
   ~/f1tenth_uj/tools/real_car_tests/preflight.sh stack t1b
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t3_circle --speed 0.3 --steer -0.18 --fast
   python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t3_circle --speed 0.3 --steer 0.18 --fast
   python3 ~/f1tenth_uj/tools/real_car_tests/t3_analyze.py
   ```
   `m` must show `converter steer_mode table (limits -0.28 / +0.28 rad)`. The geofence (R_max 1.79 m + 0.20 m) still
   stops a circle that is much wider than planned (22 Nov: two GEOFENCE stops at -0.18 with the gain converter).

**Status 22 Nov (end of day; battery dead, no more driving):**
- **Done 22 Nov:**
  - **VESC firmware timeout 250 ms** (reported as 300 that day; VESC Tool reads 250 on 23 Nov) **+ timeout brake
    current 5 A** (was default / 0 A), verified on the stand
    `20261122_150252`: brake onset 0.29 s, -2.44 A. A dead converter / frozen Jetson / lost USB now brakes
    after ~0.25-0.3 s instead of coasting after ~1 s. Still not an emergency stop.
  - **T3 COMPLETE** (hall, 0.3 m/s, low-visibility mode T3 only; results doc "T3 ground circles"): minimum radius
    **~0.87 m right / ~0.90 m left**, maximum real angle **~0.28 rad** both sides, **right understeers ~20 % near
    the centre** vs T2, left stop reached at ~0.11, slip <= 2.3 deg (supports vy = 0), IMU +y = base +y.
  - **Steering table implemented** (`cf0134f`): per-side piecewise-linear table in the t1b converter
    (`config/real_car/t1b_converter_t3_table.yaml`, limit 0.28 rad, left clip 0.11), accepted by the runner for
    `t3_circle` / `t4_straight`, `steer_mode` in the `m` line, "delta table" in `t3_analyze.py`. **NOT yet
    checked on the ground.**
  - Also: LiDAR self-mask (XT60 leads) + 270 deg field of view (blind rear unknown), `local_env.sh` for Wi-Fi
    SSH, T5 analysis tools, VESC timeout check tool.
- **Next session, in order** (all Moatasem's runs):
  1. ~~**T5a**~~ **DONE 22 Nov 17:00** (results doc "T5a"): `home_data/t5a_static_1644`, 89 939 samples at 100 Hz,
     no freeze, gz residual bias +0.003 deg/s, R 8.85e-7, ARW 0.36 deg/sqrt(h), bias instability 18.6 deg/h
     (bench 5.5), tilt 9.95 deg, |g| +4.1 %. First start hit I2C Errno 121 (node exited; retry OK).
  2. **T5b**: stand, 0.5 m/s for 90 s (`run_test.py --test t5b_stand_imu --speed 0.5 --live`, T1b stack with
     `vesc_t1b_erpm4630.yaml`), then `t5_imu_analyze.py stand`.
  3. **Steering-table ground check**: `t3_circle --steer -0.18` and `--steer +0.18` at 0.3 m/s with
     `converter_params_file:=.../t1b_converter_t3_table.yaml` (stack as `T3_HALL_CHECKLIST.md` section 3, with
     `t3_low_visibility:=true` in the hall). Expect **R ~1.43 m; pass 1.29-1.57 m** (+/-10 %) on both sides
     (`t3_analyze.py`, "delta table" column). Clear disk 4.42 m.
  4. Optional: `--steer +/-0.09` (R ~2.88 m, pass 2.59-3.17) **only in a larger space: it needs an 8.0 m clear
     disk** (does not fit a 5 x 5 m hall).
- **Open for Zaid** (section 4a, converter proposal):
  - left-wheel play and the left stop at ~0.11 (0.100 gives less angle than 0.108);
  - toe ~8 deg (difference at the first centre reading);
  - F-18 BMI160 freeze + exit on start-up motion (`IMU_DRIVER_PROPOSAL_F18.md`); **22 Nov T5a: the node also
    exits on a start-up I2C `[Errno 121]`** (0x69 answered right after; retry worked) and the accel norm reads
    **+4.1 %** (10.21 m/s^2): check the accel calibration file;
  - IMU mount: **pitch pi** (x back, y left, z down) + ~9 deg tilt, derived from T3 + T4, to be confirmed and
    the TF fixed (the test launch publishes roll pi);
  - the permanent converter proposal (`PERMANENT_STOP_CONVERTER_PROPOSAL.md`: 7 decisions, steering table
    section 8, VESC timeout section 9).

**Status 19 Nov (end of session):**
- **T1 COMPLETE** (16 Nov, stand): brake 5 A, 23 steady stops at 0.5 / 1.0 / 1.5 m/s, 0 failing.
  F-01 mitigated on the stand; **speed 0 is never a stop command**. Permanent converter: proposal to Zaid.
- **T2 COMPLETE** (19 Nov, limits): steering driving clip **0.10-0.68** (one step before the mechanical
  stops ~0.08 / ~0.70), max left 17.15 deg, right 18.38 deg (R 0.84 / 0.78 m). Fitted gain **~-0.96 NOT
  applied** (converter stays -0.50 / 0.320): the centre is uncertain because of left-wheel play.
- **20 Nov:** T4 at 0.5 m/s done (carpet + tile, brake 5 A, no F-01 rebound); **steering offset 0.330
  PROVISIONAL baseline** (tile; was 0.320; re-measure after the play fix); gain candidate ~8700 **withdrawn 21 Nov**
  (8414 kept: cruise-only 8280-8480); F-18 IMU freeze guarded in the runner and preflight.
- **21 Nov:** F-18 second cause (IMU node exits on start-up motion; guard message added, driver proposal for Zaid). Phase F: shadow EKF **C1** (VESC de-weighted while braking / below 2000 ERPM + RF2O vx fallback) recommended offline and **validated live at 0.5 m/s straight** (total +0.029 m vs LiDAR, RF2O never stale). Gain 8414 kept. T4 tile at the default 0.330: heading -8.0 deg, non-monotonic vs 0.340 / 0.358 even within one start spot -> **steering play is the top blocker**. Then T4 at 0.335 (spot B -0.9 deg/m, mean heading -0.9 deg, C1 live +0.017 m): **steering centre 0.335 PROVISIONAL** (applied in vehicle_params + both real-car node yamls; clips unchanged, right steering 0.07 rad in the 0.27-0.37 yamls). IMU x axis points backward + ~10 deg tilt (for Zaid).
- **Next (21 Nov plan, still open):** a second, fully restarted T4 session at 0.335 (3 counted runs, spot B: session-to-session variation); then section 4, step 2 (play fix, then centre) stays the top blocker. Steering findings for Zaid: section 4a.

## 1. Safety rules and standing approvals (Moatasem)

| Rule | Detail |
|---|---|
| Who actuates | **Moatasem runs every launch and every runner that can power the motor or servo**, in his own SSH terminal, with a hand on the battery disconnect. Claude never starts them (not even disarmed: the test stack centres the servo) and never sets `enable_real_actuation:=true`. Claude gives exact commands, one terminal at a time. |
| F-01 | **Mitigated on the stand (T1 passed, confirmed by Moatasem in writing 16 Nov): brake 5 A, never speed 0.** Any run that powers the motor uses the T1b converter with `stop_method:=brake brake_current_a:=5.0` until the permanent converter (section 5) replaces it; the standard `ackermann_to_vesc` (speed 0 stop) never drives the motor. Ground runs only as the T3/T4 steps below, each approved per step. Stand parts: car strapped, wheels off the ground, >= 0.8 m clear in front of the LiDAR (a box 1.5-2 m ahead keeps the front valid fraction >= 0.75), hands out of the front +/-20 deg sector. |
| Mux | Never change mux gates, thresholds or interlocks. Caps change only through the per-test launch args (`max_speed_mps`, `max_steer_rad`) and the per-step `vesc_yaml`. No new publishers or subscribers on `/drive` or `/commands/*` (the bag never records them; the mux counts subscribers). |
| One stack | `preflight.sh clean` before every launch, `preflight.sh stack` (or `stack t1b`) before every runner. |
| Speeds | 0.5 -> 1.0 -> 1.5 m/s, one step only after the previous passed (runner `step_gate`, unchanged). Ground runs: LiDAR range ahead must fit ramp + 1 m + stop + 0.5 m (runner checks). |
| Git | Claude may commit **its own files by name** without asking, author `Moatasem <motasemfaousju04@gmail.com>` passed per commit with `git -c user.name=... -c user.email=...` (the Jetson's git config is Zaid's: never change it). **Push only when Moatasem asks in that session** (22 Nov:
fast-forward to `origin/moatasem/real-car-tests`, never force). Never `git add -A`. Never stash/reset/commit teammates' files. |
| Allowed without asking | Self-tests, `analyze.py`, offline replays (only when no car stack is running). |
| Reporting | After each test: pass/fail, 3-5 numbers, the next exact commands, each terminal's command on its own. |
| Sim repo | `~/f1tenth_uj_sim` (branch `moatasem/ekf-experiments`, at `9c581af`): read-only, `git pull` allowed, no commits. |

## 2. Branch and commit state

- Branch `moatasem/real-car-tests` from `zaq/hardware-integration` @ `53dfa91`; pushed to `origin` (fast-forward only,
  when Moatasem asks in the session; 23 Nov evening: asked).
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
   **21 Nov: 0.335 adopted as the provisional centre** (results doc row "Steering centre 0.335"). Next check: a second,
   fully restarted session at 0.335 (preflight clean, new launch, 3 counted runs from spot B), no override needed now.
   **TOP BLOCKER (21 Nov):** tile T4 heading is non-monotonic in the offset (0.330 / 0.340 / 0.358) with a
   constant servo command, so play dominates and the centre cannot be resolved further before the fix. Then a
   T4 centre check: **one marked start spot** (21 Nov: start position alone shifted the heading by 2-3 deg/m),
   >= 3 counted runs per offset, heading compared in deg/m (results doc, tile offset check row).
3. **T4 on the ground, 0.5 m/s, brake 5 A: DONE 20 Nov** (results doc: tile stop ~0.137 m, decel ~0.9,
   gain candidate ~8700 (withdrawn 21 Nov, 8414 kept), provisional tile centre ~0.330, all test-only; F-18 IMU freeze found and guarded).
   Next: the left-wheel play fix (Zaid), then the centre re-measure; `decel_mps2` before 1.0 m/s.
   Original plan:
   - **T4 runner profile written 20 Nov** (`t4_straight`, `analyze.py --t4`, see
     `tools/real_car_tests/README.md` T4): brake point 0.90 m LiDAR (above the mux 0.80 m gate), arm
     window 2.15..3.90 m. Self-tests pass; **next: disarmed dry run on the floor**. Moatasem chose (20 Nov)
     to run T4 before Zaid's steering fix, at the current centre (0.320 / -0.50, steer cap 0.10, steer 0):
     side drift and heading are indicative only.
   - 20 Nov changes: brake trigger = front median only (side objects -> mux gate brakes instead, still
     5 A); KILL key (any key but SPACE while armed/driving = brake, latched until `n`). Lane width with
     the car centred, drift 0.3 m / 2 m (~17 deg): no contact (0.10 m margin) 0.88 m at a 2.2 m start,
     1.25 m at 2.8 m; mux never tripping before the brake point 1.16 m at 2.2 m, 1.61 m at 2.8 m.
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
| **Ground steering (T3, 22 Nov)** | `t3_analyze.py`, 8 runs `20261122_154013`..`155807` (results doc) | Real max angle ~0.28 rad (16-16.6 deg) both sides, below the stand T2 (17.2 / 18.4 deg); min radius ~0.90 m left / ~0.87 m right; not linear (right understeers ~20 % near the centre). Proposal: per-side steering **table** in the converter (proposal section 8). |
| **Left stop reached at ~0.11** | T3: servo 0.100 (full lock) -> +15.3 deg, LESS than 0.108 -> +16.1 deg | Left driving clip 0.11 in the table. Please check the left end stop / linkage (it may be the play or the servo saver giving way under load). |
| **IMU y axis confirmed** | T3: IMU +y = base +y in 13 / 13 circle segments | With x backward (21 Nov): the mount is **pitch pi** (x back, y left, z down) + ~9 deg tilt; the test-launch TF says roll pi, which flips y. Please confirm and fix the TF (and how it was published on 14 Nov). |
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
3a. **T3 DONE 22 Nov (results doc "T3 ground circles"):** at 0.3 m/s, min radius ~0.87 m right / ~0.90 m left,
   max real angle ~0.28 rad both sides, left stop reached at ~0.11, right understeers ~20 % near the centre vs T2,
   IMU +y = base +y. Decision: per-side steering TABLE (`t1b_converter_t3_table.yaml`, limit 0.28 rad, left clip
   0.11) instead of gains; proposal section 8 updated. The runner accepts the table yaml for `t3_circle` and
   `t4_straight` (full lock = table end; `m` shows `steer_mode`); `t3_analyze.py` then reports "delta table".
   0.5 m/s circles not run. History of the preparation:
   **T3 PREPARED 21 Nov, updated the same evening:** `t3_circle` runner profile, T3 converter / VESC
   yamls, `t3_analyze.py`, hall checklist `T3_HALL_CHECKLIST.md`. Order: disarmed rehearsal, +/-0.18, then per-side
   ~90 % (left +0.26 / servo 0.108, right -0.31 / servo 0.664), then `--full-lock left|right` (servo exactly
   0.10 / 0.68: real minimum radius, own table in `t3_analyze.py`). Per-side limits 0.27 / 0.325 rad, mux cap
   `max_steer_rad:=0.33`. **VESC timeout 250 ms + brake 5 A: configured AND VERIFIED 22 Nov (PASS 0.29 s, -2.44 A), T3 fallback lifted;** it was verified on the stand
   with `vesc_timeout_check`** (checklist section 0). Until a PASS the runner enforces the fallback (disk +1 m,
   0.3 m/s first); in a 5 x 5 m hall that excludes +/-0.18 and +0.26.
   **22 Nov rehearsal:** refusals came from the own XT60 leads (now `lidar.self_mask`; tuck them in), real objects
   ~3 m behind-left inside the disk, and the 270 deg LiDAR (blind rear now unknown, checked by eye).
   Hall rule: operator, laptop, furniture outside the whole disk incl. behind the car. Always end the runner with q.
   **22 Nov decision: T3 ONLY with the mux low-visibility mode** (`t3_low_visibility:=true`, MANUAL 0.35, AUTO /
   0.80 m gate unchanged): runner-enforced T3 only + verified timeout + 0.3 m/s first; spotter, look behind.
4. **T4** (0.5 m/s is next step 3) straight line 0.5/1.0/1.5 m/s toward a wall, 3 repeats each (runner ground check): LiDAR vs VESC
   vs RF2O distance -> `speed_to_erpm_gain` (same value in both converter and vesc_to_odom), lateral drift
   -> steering offset, start delay, stopping distance and deceleration with the chosen stop method. Profile
   not written yet.
5. **T5** IMU on the car. **(a) done 21 Nov** (results doc "T5 (a)"): still gz noise = bench (5.3e-7), driving
   ~80x (tile) to ~550x (carpet); recommended driving R_gz 1.2e-3 vs C1's 8e-3 (not applied: after T3 + T5a,
   replay first). **(b) T5b** stand 0.5 m/s 90 s: `run_test.py --test t5b_stand_imu --speed 0.5 --live`, then
   `t5_imu_analyze.py stand`. **(c) T5a** 15 min static (IMU-only launch + `ros2 bag record`), then
   `t5_imu_analyze.py static` (bias, R, Allan, drift, frozen check). Hand turns (scale) later; T3 gives the gyro
   / RF2O ratio on the ground.
6. **T6** offline on T3/T4/T5 bags: VESC vx vs RF2O vs LiDAR, gyro vs RF2O yaw rate, time offsets
   (cross-correlation), R per sensor -> `vehicle_params.yaml noise`.
7. **Phase F (shadow only, no TF authority):** **21 Nov: offline variants done, C1 recommended** (results doc rows "Phase F" + "Phase F step 2"). C1 = VESC vx (8414) + vy=0 + gyro, VESC de-weighted x100 while braking (t1b `STOP_BRAKE`) or 0 < |ERPM| < 2000, RF2O vx as fallback: total error +0.005 m (-0.010..+0.025) on 7 T4 repeats vs -0.054 (A). Live inputs: `tools/real_car_tests/ekf_shadow_inputs.py` (same functions as the replay; `--live-check` identical on all 4 topics). Not yet run live. Next: live shadow run on the stack (section 7), T3 for vy=0, IMU mount (axes + tilt) with Zaid before any accel fusion (C2).
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
| VESC Tool checklist | **Pending**: `docs/real_car/T1_VESC_TOOL_CHECKLIST.md` (export motor + app XML first). **22 Nov CONFIGURED (Moatasem):** `timeout_msec` 250 (reported as 300 on 22 Nov, read back 250 on 23 Nov; previous unknown / default), `timeout_brake_current` 5 A (was 0); note for Zaid in the converter proposal section 9. **VERIFIED 22 Nov** (`20261122_150252_vesc_timeout_check_v0.50_brake5A_erpmmax4630_armed`): PASS, onset 0.29 s, -2.44 A, < 300 ERPM in 0.33 s; T3 fallback lifted. |
| Brake hold release | Every brake stop (also 16 Nov at 1.0 / 1.5 m/s: 88-101 ERPM at +2.09-2.15 s) shows a 77-100 ERPM rise at +2.1-2.2 s: the 2.0 s hold ending (switch to current 0). Tune the hold / release in the permanent fix. |
| Servo horn off-centre (Zaid) | Straight = servo 0.320 = 1320 us, not 1500 us (VESC 0..1 = 1000-2000 us): left at most 0.32 of travel, right 0.68. Re-mount so straight is near 0.5 for symmetric travel; **redo T2 after that**. Zaid to decide. |
| **F-18 BMI160 freeze (Zaid)** | **Also (21 Nov): the node exits at start-up if the car moves during the 5 s gyro calibration** (std > 0.5 deg/s, exit code 0, "finished cleanly"); runner/preflight print `IMU node not running: restart the stack and keep the car still for 10 s`. Proposal for both causes: [IMU_DRIVER_PROPOSAL_F18.md](IMU_DRIVER_PROPOSAL_F18.md). After one `I2C read failed: [Errno 110]` the stream repeats one sample at 100 Hz until the node restarts (2 of 4 T4 sessions, 20 Nov; results doc). Guard: runner refuses to arm on `IMU_FROZEN` / `IMU_STALE`, `preflight.sh stack` checks 2 s; recovery = restart the stack. **Proposed driver fix (imu_driver, Zaid's package):** after an `OSError`, redo the soft reset + configuration (keep the gyro bias), and a watchdog that resets on N identical samples. **Diagnostic at the next freeze** (stack stopped first, before relaunching; read-only): `i2cget -y 7 0x69 0x03` (PMU_STATUS), `0x02` (ERR_REG), and `0x18`-`0x1A` (SENSORTIME) twice, a second apart: SENSORTIME not advancing = the sensor stopped converting (register addresses from the BMI160 datasheet; verify). |
| Battery | Charged: >= 12.3 V during both 16 Nov runs (11.1 V idle on 14 Nov after a power loss). |
| Stand LiDAR | Front valid fraction ~0.83 vs the 0.75 MANUAL gate; an operator hand at 0.10-0.12 m (+10..+19 deg) latches `OBSTACLE_NEAR`. |
| F-13 | Runner input gaps never exceeded 44 ms; the keyboard GUI's latch was not reproduced (GUI not re-tested). |
| IMU TF | `base_link -> imu_link` (0.091, 0.057, 0.046, roll pi) published by the test launch, PROVISIONAL; ask Zaid how it was published on 14 Nov. **21 Nov (T4 bags): IMU x points BACKWARD** (forward accel reads on -x: pitch pi or roll pi + yaw pi) **and the mount is tilted 8-10 deg**; gz sign unaffected. Not changed (Zaid). **22 Nov T3: IMU +y = base +y confirmed** -> mount = pitch pi (not roll pi) + tilt (section 4a). The shadow inputs handle it themselves (no TF). **23 Nov eve: TF changed to the real mount**: roll 0.0497, pitch 3.2935 (pi + 8.70 deg), yaw 0 (`vehicle_params.yaml`, from the 23 Nov still accel after the re-tape); position still PROVISIONAL (re-measure); tell Zaid. |
| Team notes | Repo configs still at offset 0.361; do not use `vesc.launch.py` (`config/vesc.yaml` brake 200000 A); listed in the results doc. |
| DDS over Wi-Fi SSH (22 Nov) | Launching the stack over a phone-hotspot SSH session froze all sessions. Over Wi-Fi: `source tools/real_car_tests/local_env.sh` in every terminal (ROS_LOCALHOST_ONLY=1, CYCLONEDDS_URI unset, lo multicast on); no laptop RViz then. USB SSH: not needed. |
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

T4 ground, 0.5 m/s, brake 5 A (terminal A, then B; disarmed dry run: `enable_real_actuation:=false`
and no `--live`; `m` = placement readout):
```bash
ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=true max_speed_mps:=0.5 vesc_yaml:=/home/ftenth/f1tenth_uj/config/real_car/vesc_t1b_erpm4630.yaml converter:=t1b stop_method:=brake brake_current_a:=5.0
python3 ~/f1tenth_uj/tools/real_car_tests/run_test.py --test t4_straight --speed 0.5 --t1-passed-confirmed --live
python3 ~/f1tenth_uj/tools/real_car_tests/analyze.py --t4
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

Shadow EKF, live (Phase F, C1; observation only, publishes /ekf_shadow/* only; with the stack up and the car
STILL for the first 2 s of the inputs node; terminals C, D, E stay running; the inputs node logs RF2O freshness
every 10 s and warns at once when /odom_rf2o is older than 0.2 s):
```bash
python3 ~/f1tenth_uj/tools/real_car_tests/ekf_shadow_inputs.py
ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_real_shadow_c1 --params-file /home/ftenth/f1tenth_uj/config/real_car/ekf_real_shadow.yaml -r odometry/filtered:=/ekf_shadow/odom
ros2 bag record -o ~/f1tenth_hw_evidence/real_car_tests/$(date +%Y%m%d_%H%M%S)_ekf_shadow_live/bag /ekf_shadow/odom /ekf_shadow/in/vesc /ekf_shadow/in/vesc_dw /ekf_shadow/in/imu /ekf_shadow/in/rf2o
```
Afterwards (reads bags only): `python3 ~/f1tenth_uj/tools/real_car_tests/ekf_shadow_replay.py --live <shadow folder> <T4 folder>`
(after `analyze.py --t4`). Offline (Claude may run, stack down, domain 88): `ekf_shadow_replay.py` (variants),
`--live-check`, `--rf2o-cpu`. Offline end-to-end test 21 Nov (inputs node + C1 ekf_node + record on the tile bag,
sim time): live chain = replay within 1 mm (+0.026 / +0.016 / +0.000 m).

Self-tests (Claude may run; the runner's shutdown probe uses isolated domain 77, no publishers):
```bash
for t in run_test analyze t1b_stop_converter t2_servo_sweep ekf_shadow_inputs ekf_shadow_replay t3_analyze t4_yaw_plot; do python3 ~/f1tenth_uj/tools/real_car_tests/$t.py --self-test; done
python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py --self-test
python3 ~/f1tenth_uj/src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
```

Results: `~/f1tenth_hw_evidence/real_car_tests/<stamp>_<test>_v<speed>_<method>_erpmmax<N>_<armed|disarmed>/`
(bag, `runner_events.csv`, `keys.csv`, `publish_intervals.csv`, `vesc_speed_audit.log`, `summary.json`,
CSVs, plots, README). Not committed.
