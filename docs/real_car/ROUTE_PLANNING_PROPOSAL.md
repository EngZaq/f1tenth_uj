# Route (centreline + speed profile) for the real car: what the sim pipeline does, what must change (23 Nov eve)

For Moatasem and the team. Read-only review of `~/Data/f1tenth_uj_sim` (branch `moatasem/ekf-experiments`,
`9c581af`); nothing there was changed. Proposal only: no code for this is in the real-car branch yet.

## 1. What `tools/mission/full_autonomy.py` runs (stage PLANNING)

| Step | Tool (sim repo) | Input | Output | Built-in limits |
|---|---|---|---|---|
| 1 | `nav2_map_server map_saver_cli -t /mission/slam_map` | live SLAM map | `new_track.yaml/.pgm` (trinary: 254 free, 205 unknown, 0 occupied; `negate 0`) | - |
| 2 | `tools/planning/extract_robust_centerline.py <map.yaml> --out <prefix> --step 0.10 --max-speed <cap> --expected-length <lap m>` | map; lap length from the first-lap odometry | `<prefix>_centerline.csv` (`;`-separated, `# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2`), `<prefix>_speed_profile.csv` (`index,s_m,x_m,y_m,curvature_1pm,raw_speed_mps,planned_speed_mps`), `<prefix>_preview.png` | largest observed free region; skeleton of the free space eroded by **0.30 m**; needs a **closed loop around an enclosed interior** (hole >= 0.5 m^2, loop >= 8 m, 0.55..1.55 x expected length); unknown never becomes free; speed: a_lat 0.9, accel 0.5, brake 0.8 m/s^2, cap `--max-speed` |
| 3 | `tools/mission/orient_mission_centerline.py <centerline.csv> <lap.json> <out.csv>` | step 2 + finish pose (`finish_x/y/yaw`, from `capture_map_pose.py`) | `new_track_mission_forward.csv` (same 7 columns, starts at the finish pose, travel direction = the car's heading; `ax` = 0) | finish pose <= 0.60 m from the line, heading error <= 55 deg |
| 4 | `tools/planning/prepare_vehicle_route.py <map.yaml> <oriented.csv> --out ... --report ...` -> `optimize_vehicle_route.improve` | step 3 | `new_track_body_optimized.csv` (7 columns, `#` header, slam_map frame) = what `run_profiled_saved_map.py` drives | Gaussian smoothing sigma 0..4 points, local lateral shift (differential evolution) until the **full body** keeps >= **0.15 m** from occupied / unknown cells and the peak steering <= **24 deg**; **wheelbase 0.3302** hard-coded; body from `sim_geometry` (sim laser yaml); speed: a_lat 0.75, accel 0.6, decel 0.8, v_max 0.8, narrow spots slower (0.2 + 2 x (margin - 0.15)) |
| (opt.) | `tools/navigation/moatasem_speed_profile.py <route.csv> --out ... --a-lat ... --v-max ... --accel ... --brake ...` | any 7-column route | same geometry, smoothed curvature, new `vx` / `ax` | all limits are arguments |

## 2. What must change for a real slam_toolbox map and this car

| Item | Sim value | Real car (source) | Consequence |
|---|---|---|---|
| Wheelbase | 0.3302 (hard-coded in `prepare_vehicle_route.py`, `optimize_vehicle_route.py`, `vehicle_footprint_audit.py`) | **0.260** (`vehicle_params.yaml`) | steering from curvature: delta = atan(0.26 kappa) |
| Steering limit | 24 deg (0.419 rad) | **0.28 rad = 16.0 deg** (T3 ground, both sides; steering table limit) | kappa_max = tan(0.28) / 0.26 = **1.11 1/m, R_min 0.90 m**. Plan with tracking margin: **R >= 1.15 m (0.8 x steer = 0.224 rad)**; the sim allowed R 0.74 m |
| Body | sim laser yaml (`BODY_*` in `sim_geometry`) | length 0.345, width 0.197, front bumper 0.305 m ahead of the rear axle (base_link) | body rear = -0.040 m, half width 0.0985 m |
| Frame | `slam_map` (`fusion_*` frames) | `map` (slam_toolbox in `observe_only.launch.py`) | header text only; the numbers are map coordinates |
| Map | 0.025 m sim grid, clean walls | 0.05 m, real walls straight to 6-8 cm rms, one wall partly doubled (23 Nov corridor map) | the 0.30 m erosion and the 0.15 m body margin still fit a 1.7 m corridor (1.1 m skeleton band); doubled walls only narrow it |
| Loop | closed track required | **23 Nov corridor: one way, no loop** -> step 2 refuses ("No substantial enclosed interior") | needs a **closed track** (a loop around an island / furniture block) mapped with the start mark: the track decision |
| Lap pose | `mission_first_lap.py` + `capture_map_pose.py` (live) | start tape mark pose in the map: first still pose of the `hall_loop_map.py` trajectory CSV (`map_lc_off_trajectory.csv`) | write `{finish_x, finish_y, finish_yaw}` from it |
| Speed limits | a_lat 0.75-0.9, accel 0.5-0.6, decel 0.8, v_max 0.8 | cap **0.5 m/s** (mux cap / runner, 0.3 first); accel **0.5** (runner ramp); decel **<= 0.8** (measured brake 5 A ~0.9 m/s^2 on tile; the VESC speed ramp 25000 ERPM/s = 2.97 m/s^2 is the ceiling for any speed-setpoint decel); a_lat 0.5 (never binding at 0.5 m/s: R_min gives 0.28 m/s^2) | profile is flat at the cap except narrow spots; **min planned speed 0.2 m/s** (VESC open loop below 1400 ERPM = 0.17 m/s) |
| Python deps | `scikit-image`, `scipy` | laptop: `skimage` **not installed** (23 Nov eve) | `pip install --user scikit-image` on the laptop before step 2 |

## 3. Smallest way to a centreline + speed-profile CSV for the car (proposal)

1. **Map a closed loop** (track decision first): teleop the loop with `run_test.py --test teleop_ps4` (start and
   end on the tape mark), then `hall_loop_map.py <run folder>` -> `maps/map_lc_on.yaml` (or `_off`). Check the lap
   closure there (< ~0.2 m) before using the map.
2. **Centreline (sim step 2 unchanged, run from the sim repo, read only):**
   `python3 ~/Data/f1tenth_uj_sim/tools/planning/extract_robust_centerline.py <maps>/map_lc_on.yaml --out
   <maps>/route/track --step 0.10 --max-speed 0.5 --expected-length <lap m from hall_loop_map.json>`.
   It has no vehicle constants; its 0.30 m clearance suits the car.
3. **One new small script in this repo** (`tools/real_car_tests/real_route.py`, ~120 lines, not written yet): reads
   the step-2 centreline, orients it at the tape-mark pose (same rule as `orient_mission_centerline.py`), smooths
   the curvature (same method as `moatasem_speed_profile.build`), **checks with `vehicle_params.yaml`**: peak
   delta = atan(0.26 |kappa|) <= 0.224 rad (refuse otherwise, naming the worst s), full body (0.345 x 0.197,
   bumper 0.305) >= 0.15 m from occupied / unknown cells along the whole loop (incl. between points), then the
   speed profile with the real limits (cap 0.5, accel 0.5, decel 0.8, a_lat 0.5, min 0.2). Output: the same
   7-column file (`# s_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2`, map frame) + a preview PNG. It replaces sim
   steps 3-4 instead of editing the sim repo (whose wheelbase / body / 24 deg are hard-coded).
4. Use: the speed profile is a proposal only (`/navigation/proposal`, never `/drive`) until the port of
   `profiled_amcl_drive_moatasem.py` is reviewed (handoff section 4b item 7).
