# F1TENTH physical integration: Nov 8 steering and STOP evidence sprint

Repository: `EngZaq/f1tenth_uj`, branch `zaq/hardware-integration`.
Purpose: prepare an evidence-based fix for the actual November 6 first-motion failure, rather than raising speed or removing path guards.
Status: GitHub source changes committed; **Jetson/track execution, steering measurements and physical braking response not yet verified**.

## 1. Last physical test: proven behavior

- VESC did produce some wheel movement from the ROS AUTO drive path.
- SLAM Toolbox publishes /map every 5 s and physical mux map QoS was corrected. LiDAR + RF2O + SLAM were fresh in the last reported status.
- Final stop: `LATCH_STOP_FRONT_SCAN_NOT_CLEAR`, measured nearest front surface **0.782 m** versus conservative stop limit **0.800 m**.
- Full swept-footprint guard reported **72 blocking scan returns**, nearest swept obstacle **0.808 m**, at **+12.75 degrees**.
- Auto hold was fresh; this was **not** an auto-hold timeout or map QoS failure.
- Steering demand was at the prior nominal +/-0.10 rad cap; ideal kinematic turning radius with 0.260 m wheelbase: **2.59 m**. Actual front road-wheel steering range and track radius remain unknown.
- A real autonomous first mapping lap, map handoff and physical Pure Pursuit have **not** passed.

## 2. New Nov 8 code changes

1. `src/f1tenth_bringup/scripts/physical_stop_evidence.py` captures a self-contained snapshot of command, obstacle geometry, freshness and command-chain ownership at STOP.
2. `physical_drive_mux.py` stores `last_stop_snapshot` in `/control/status` and writes `~/f1tenth_hw_evidence/last_physical_stop.json` **after** issuing a zero-speed command. Does not relax any stop gates.
3. `tools/diagnostics/fit_steering_measurements.py` estimates equivalent center steering angle from measured left/right road-wheel angles; fits `servo_command = gain*steering_angle_rad + center`; estimates nominal minimum turn radius and identifies sign mismatch/high fit residual. **Does not modify active configs.**
4. `tools/exploration/physical_candidate_planner.py` reports candidate evaluation time and fails closed when a candidate search exceeds its 65 ms CPU budget. The experimental planner is still opt-in.
5. `.github/workflows/physical-offline-regression.yml` provides offline tests without ROS or hardware. CI outcome must be checked; installation on the Jetson is not yet verified.

Existing earlier work: longer-lookahead candidate steering evaluation (1.05 m vs mux 0.80 m), early approach slowdown, `/exploration/diagnostics`, adjustable physical first-lap thresholds. **These are experimental until tested against real scans.**

## 3. Jetson software checkout and non-actuating tests

Stop all old launches first. Do not overwrite local changes: inspect `git status --short` before switching branches.

```bash
cd ~/f1tenth_uj
git status --short
git fetch origin zaq/hardware-integration
git switch zaq/hardware-integration
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 -m compileall -q src/f1tenth_bringup tools/exploration tools/diagnostics
python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
python3 src/f1tenth_bringup/scripts/physical_stop_evidence.py
python3 tools/exploration/physical_candidate_planner.py
python3 tools/diagnostics/test_physical_candidate_geometry.py
python3 tools/diagnostics/fit_steering_measurements.py --self-test
python3 tools/diagnostics/physical_calibration_precheck.py
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
```

If any test fails, capture its stdout/stderr and DO NOT proceed to powered autonomous motion.

## 4. Real steering measurement protocol

Mechanical steering measurements come first. Fully disconnect or isolate propulsion, secure wheels off ground, keep hands clear of the moving linkage, and ensure the servo can't unexpectedly move the chassis. Do not run a competing VESC driver or auto-mission. Measure **wheel plane** angles, not servo-horn rotation.

Nominal VESC servo mapping (unverified):
- Neutral servo command `0.361`
- Small tests `0.341` and `0.381`
- Existing software commanded extremes `0.311` and `0.411`, only if linkage is clear and end stops permit it
- Existing configured VESC servo position clamp: `0.30 ... 0.42`
- Current commanded virtual steering limit: `+/-0.10 rad` (5.73 degrees), but **real wheel angles may differ**.

For each commanded position record left and right front wheel angles relative to vehicle longitudinal centerline, positive when pointing left (CCW top view), negative when pointing right. Recenter between sweeps, note backlash/hysteresis. Record both sweep directions and any mechanical interference.

Create `~/f1tenth_steering_measurements.csv` (manually fill numeric values):
```csv
servo,left_deg,right_deg
0.361,0,0
0.341,,
0.381,,
0.311,,
0.411,,
```
The zeros above for neutral are EXAMPLES, not measured facts; replace with actual values. You can generate an empty template with:
```bash
python3 tools/diagnostics/fit_steering_measurements.py --template
```

After recording real measurements:
```bash
python3 tools/diagnostics/fit_steering_measurements.py ~/f1tenth_steering_measurements.csv
```
The fitter rejects incomplete numeric CSV rows; omit any servo command not safely tested. It requires >=3 distinct positions and both-turn-direction coverage for a useful result. **Do not copy fitted gains into YAML automatically.** Independently check a holdout servo command, wheel response and tire clearance.

Measure the real track's smallest CENTERLINE radius (m), narrowest lane width (m) and approximate total loop length (m):
```bash
python3 tools/diagnostics/steering_radius_audit.py --radius YOUR_MEASURED_RADIUS_METERS
```

For an ideal Ackermann bicycle, `delta_required = atan(wheelbase / radius)`. Example: radius 1.0 m requires ~14.6 deg virtual steering; the current +/-5.73 deg limit is insufficient. A hardware centerline or inside/outside wall measurement is necessary before deciding how much steering range the vehicle needs.

## 5. Shadow verification without motor drive

Once offline tests pass:
```bash
ros2 launch f1tenth_bringup real_integrated.launch.py \
  enable_real_actuation:=false \
  enable_autonomous_actuation:=true \
  enable_slam_loop_closing:=false \
  experimental_candidate_planner:=true
```
Separate sourced terminal:
```bash
ros2 topic echo /exploration/diagnostics --once --field data
ros2 topic echo /control/status --once --field data
python3 tools/diagnostics/physical_autonomy_preflight.py
```

Check: (a) measured-scan candidate paths respond to track walls; (b) `steering_saturated` and `ideal_min_radius_m` are consistent with track measurements; (c) planner `compute_ms` remains below 65 ms with no compute-budget fault; (d) `path_ready=false` when a real obstacle occupies the intended footprint; (e) zero extra /drive subscribers and no duplicate actual VESC drivers.

No motor movement occurs in this shadow launch from the command mux (`enable_real_actuation=false`), but the VESC is still electrically connected; this is not an independent motor isolation.

## 6. Physical stopping and mission gate

Do not attempt powered ground autonomy until a dependable independent propulsion cutoff, VESC firmware current/timeout settings, servo limits, motor direction, stop behavior and stopping distance are physically checked. A fresh `/control/status` snapshot is not a braking test.

If a controlled test faults later:
```bash
cat ~/f1tenth_hw_evidence/last_physical_stop.json
ros2 topic echo /control/status --once --field data
```
A snapshot captures *one* stop, not a full trajectory. Also record the following topics before any approved moving test:
```bash
ros2 bag record -o ~/f1tenth_hw_evidence/drive_$(date +%Y%m%d_%H%M%S) \
  /scan /odom_rf2o /map /exploration/drive \
  /exploration/diagnostics /control/status \
  /mission/physical_lap_progress /mission/first_lap_complete
```
**Do not subscribe to `/drive`** with `ros2 topic echo` or bag recorder: the mux requires exactly one intended subscriber and detects extra graph endpoints as a fault.

## 7. Definition of done for this iteration

- [ ] Python compile and all offline test results PASS on Jetson
- [ ] Left/right wheel angle vs 3–5 servo commands measured with repeatability
- [ ] Actual minimal track turning radius and lane width measured
- [ ] Steering fit sign, neutral, extreme positions and residual reviewed
- [ ] Experimental candidate planner in shadow mode responds correctly to real track scans without CPU budget faults
- [ ] Independent stop and real braking response verified on controlled stand before powered ground motion
- [ ] Evidence retained: Git commit, calibration CSV, track measurements, status and ROS bag

Next after these gates: first controlled complete physical mapping lap, map/route acceptance, physical saved-map localization, then Pure Pursuit. Simulation remains a separate verified mapping/planning baseline; do not transplant simulator truth, geometry, or its loop-close-on settings to the car.
