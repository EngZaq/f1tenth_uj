# F1TENTH UJ — Moatasem simulation experiment review and physical transfer
**Date:** 2026-11-08, Jordan time  
**Simulation source:** EngZaq/f1tenth_uj_sim, branch `moatasem/ekf-experiments`, head `32c3f237bbab393cf06941c9703f7fd252c807a9` (21 commits ahead of main)  
**Physical destination:** EngZaq/f1tenth_uj, branch `zaq/hardware-integration` (never automatically merge into physical main)  
**Status:** New physical source/config committed. **No new Jetson, real IMU, physical steering, braking or moving-car validation recorded**.

## 1. Exactly what the new simulation branch demonstrated

Moatasem's `docs/moatasem_ekf_experiments.md` and measurements in `docs/moatasem_results/` report:

| Experiment | Simulator finding | Physical conclusion |
|---|---|---|
| EKF C: VESC vx + IMU gyro z, no RF2O | Worked with simulated sensor imperfections; no 3-sigma rejection gates | Port as independent SHADOW estimator; retune covariance from real measurements |
| AMCL C vs AMCL B (with RF2O) | Position RMSE 0.048 ± 0.012 m vs 0.045 ± 0.011 m; no difference beyond spread; RF2O ~42% of a CPU core | Do **not** remove physical RF2O yet: it currently OWNS `odom -> base_link`; no validated physical replacement |
| Saved-map AMCL + EKF C closed-loop tracking | FIVE runs, TWO laps each, zero simulator collisions, 0.015 ± 0.000 m mean true cross-track error, speed profile capped 0.8 m/s | Strong simulator evidence for next stage, **not** proof of unknown-track end-to-end mission or real car control |
| Raw VESC with injected 1°+1° steering play | Mean true cross-track error 0.029 m, one run safety stopped | Measure actual steering play and servo calibration; don't use commanded wheel angle as actual steering |
| Map with SLAM loop closing OFF | C maps wall RMSE 0.1001 ± 0.0088 m (3 runs) | Current physical default loop closing remains OFF |
| Map with loop closing ON | Wall RMSE 0.4771 ± 0.0189 m; 1.56–1.72 m global corrections (~24°) | Do NOT enable physical loop closure merely to force closure |
| Tighter loop closing | Still erroneous closures; mean wall RMSE 0.3263 ± 0.2092 m | Tightening thresholds does not solve graph matching |
| SLAM correction audit | 0.10 s sample-gap bound missed correction during optimizer pause; 2.0 s detected it | Port an OFFLINE physical `map -> odom` TF correction diagnostic |

All RMSE/CTE values are reported *simulation* measurements. Simulator truth was used for offline evaluation, not for the controller. Raw rosbags for the teammate's multiple experiments are reported local to the teammate's machine, not all in GitHub.

**Important experiment distinction:** the 2-lap success begins from an already saved simulated map `autonomous_20261103_114924_818822/new_track.yaml`. It is not proof the mission can automatically map an unseen track, switch from SLAM to AMCL, then race without intervention.

## 2. Physical commits made from this review

### A. Non-actuating EKF Config C port
- New `src/f1tenth_bringup/config/ekf_moatasem_vesc_imu_shadow.yaml`, based directly on the teammate's `config/fusion_ekf_moatasem.yaml`.
- Physical frames: `map / odom / base_link` (rear axle); `publish_tf: false` so RF2O and SLAM own the original TF edges.
- Physical VESC forward speed input: `/shadow_inputs/odom_vesc`, provided by existing read-only `tools/shadow_covariance_adapter.py`. This adapter's variance **0.16 (m/s)^2 is provisional** and must be measured; do NOT copy the simulated R=0.0009.
- Physical IMU angular velocity z input: default `/imu/data_raw`, configurable by launch argument; check the ACTUAL live topic, units, axis and covariance before launching. This topic name is not yet confirmed.
- Exactly two fused states from sensors: VESC vx, IMU yaw rate. Process covariance is a simulation hypothesis, not a physical fit; no gating and smoothing carry over for controlled study only.
- New `src/f1tenth_bringup/launch/moatasem_ekf_shadow.launch.py` only launches EKF, no VESC/IMU/RF2O/SLAM or actuation, output should be `/odometry/filtered_moatasem_shadow`.

### B. SLAM map-correction audit
- New `tools/validation/physical_mapping_correction_audit.py` copies the source audit math with physical `map -> odom` frames.
- Defaults to a **2.0-second** comparison gap (not 0.10) to retain corrections across a SLAM optimization pause.
- Uses standard Python and existing ROS2 sqlite3 bag; does not use privileged simulator truth or issue ROS commands.
- Flags adjacent transform corrections >0.25 m or >5° for review. **Physical thresholds are provisional** and may detect legitimate global corrections. Passing does not prove map walls are accurate.
- Regression `tools/validation/test_physical_mapping_correction_audit.py` has synthetic 0.6 s global jump and a stable control.

### C. Motor-encoder sign mismatch diagnosed
- Existing `ackermann_to_vesc.yaml` uses `speed_to_erpm_gain: +8414`; existing `vesc_to_odom.yaml` uses **-8414**.
- This can invert VESC odometry under the same measured ERPM. Which sign is correct depends on motor orientation, VESC direction and measured wheel rotation. **Neither active conversion is changed**.
- `tools/diagnostics/physical_calibration_precheck.py` now reports both gains and a warning; verify physically before using VESC vx for any controller or navigation.

### D. Regression coverage
- `tools/diagnostics/test_moatasem_physical_shadow_contract.py`: asserts EKF selects only vx and yaw-rate, publishes no TF, checks process covariance shape and warns on speed sign.
- `.github/workflows/physical-offline-regression.yml` expanded to include the audit and shadow contract tests.

## 3. Commands to validate WITHOUT driving the car

Start by stopping old ROS launch instances and checking for local uncommitted changes:
```bash
cd ~/f1tenth_uj
git status --short
git fetch origin zaq/hardware-integration
git switch zaq/hardware-integration
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m compileall -q src/f1tenth_bringup tools/validation tools/diagnostics
python3 tools/validation/test_physical_mapping_correction_audit.py
python3 tools/diagnostics/test_moatasem_physical_shadow_contract.py
python3 tools/diagnostics/physical_calibration_precheck.py
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
ros2 pkg prefix robot_localization
```
If the ROS package is missing, do not fake EKF output; install/verify through the system ROS dependency process. The above tests should be run on the Jetson; GitHub commits alone are not Jetson test evidence.

## 4. Live non-actuating shadow sequence (only after input topics verified)

With propulsion independently isolated, in Terminal 1:
```bash
ros2 launch f1tenth_bringup observe_only.launch.py \
  enable_vesc:=true enable_slam_loop_closing:=false
```

Term 2 (sourced):
```bash
ros2 topic list -t | grep -Ei 'imu|gyro|odom'
ros2 topic info -v /odom_vesc_observe
# Start the actual IMU driver separately. DO NOT launch a second VESC / RF2O.
```
Confirm the real IMU topic publishes `sensor_msgs/msg/Imu`, angular velocity is in rad/s, time stamps and TF alignment are credible and `angular_velocity_covariance[8]` is **measured or at least explicitly labeled provisional**. If no IMU topic appears, the new EKF is not ready to run; do not use zeros/synthetic data.

Term 3 (from source repo):
```bash
cd ~/f1tenth_uj
python3 tools/shadow_covariance_adapter.py
```

Term 4:
```bash
ros2 launch f1tenth_bringup moatasem_ekf_shadow.launch.py \
  imu_topic:=/imu/data_raw
```
Substitute the real IMU topic if different. Then verify:
```bash
ros2 param get /ekf_moatasem_shadow publish_tf
ros2 topic hz /odometry/filtered_moatasem_shadow
ros2 topic echo /odometry/filtered_moatasem_shadow --once
```
Output must remain observational. The steering/motor path and current RF2O `odom -> base_link` owner are unchanged.

## 5. Record physical evidence and run correction audit

From a non-actuating observed/manual-push session:
```bash
OUT=~/f1tenth_hw_evidence/moatasem_port_$(date +%Y%m%d_%H%M%S)
ros2 bag record -o "$OUT" \
  /scan /tf /tf_static /map /odom_rf2o \
  /odom_vesc_observe /shadow_inputs/odom_vesc \
  /odometry/filtered_moatasem_shadow /control/status
```
Add the **verified actual** IMU topic to this bag command. After recording finishes:
```bash
find "$OUT" -name '*.db3'
python3 tools/validation/physical_mapping_correction_audit.py \
  "$OUT/<actual_filename>.db3" --max-gap-s 2.0
```
Do not interpret a 0.25 m transform translation step as necessarily the same as vehicle pose error: map->odom includes rotation about a frame origin, so a large transform translation can have a smaller effect at the actual car. Review geometry and pose, not only a single TF number.

## 6. Transfer priorities and hard boundaries

1. **NOW:** Get real wheel-speed sign and scale, IMU frame/z-rate, timestamp and actual covariance; compare RF2O and shadow VESC+IMU EKF against a measured physical straight and turn.
2. **NOW:** Keep hardware loop closing off as default; audit `map -> odom` and inspect wall geometry after a real mapping lap.
3. **NEXT:** finish steering angle/radius calibration and candidate planner shadow checks; fix actual obstacle-turn handling before powered ground autonomy.
4. **AFTER REAL MAP:** validate saved-map AMCL with an unambiguous map->odom TF owner. Do not run SLAM mapping and AMCL map localization simultaneously.
5. **THEN:** port the teammate's profile-following controller into a **non-actuating** estimated-pose observation stage, adapt physical wheelbase/body, route frames and braking checks; review measured error before the mux may accept it.
6. **ONLY AFTER:** move the validated physical fusion pipeline to own odom->base_link, retire RF2O as TF authority through an explicit controlled handoff, and permit supervised low-speed route tracking. Target multi-lap speed progression later.

**Do not auto-merge the 21 simulation commits into physical main.** Simulated truth input, `fusion_*` frames, 0.1016 m wheels, virtual VESC sensor R, saved simulated map, simulated collision topic and controller speed caps cannot be reused as real facts.
