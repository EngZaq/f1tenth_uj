# Physical F1TENTH — 2026-11-06 evidence and 2026-11-07 test plan

Scope: EngZaq/f1tenth_uj branch `zaq/hardware-integration`. This is an engineering handoff, **not** a claim that hardware racing is operational or safe.

## 1. What the real Jetson test established

- Real actuator graph ownership: exactly one `/drive` publisher/subscriber and one `/commands/motor/speed` publisher/subscriber, plus one RF2O publisher.
- Motor physically moved under the AUTO controller. The Jetson–ROS–VESC–motor path works at least intermittently; motor direction, actual wheel speed calibration, and stopping response are **not verified**.
- LiDAR scans ~30 Hz (prior evidence); RF2O /odometry and SLAM map fresh. The SLAM `/map` publisher uses RELIABLE, TRANSIENT_LOCAL QoS and 5-second map update interval; physical mux was corrected to match.
- Earlier AUTO activation was rejected because `/control/auto_hold` was held TRUE *before* the AUTO request. To request mode, release hold, request AUTO, then actively hold.
- A short powered run latched STOP with **`LATCH_STOP_FRONT_SCAN_NOT_CLEAR`**. Front obstacle at **0.782 m**, below the 0.80 m stop threshold; swept footprint reported **72 colliding LiDAR returns**, nearest path hit **0.808 m** at **12.75 degrees**. This is a geometry problem, not a QoS or battery C-rating failure. The obstacle guard must not be bypassed.
- Latest stopped-state status: front scan / RF2O / map fresh, `first_lap_completed=false`, map age ~0.34 s, no duplicate actuator endpoints. STOP persists intentionally after a fault.
- Current initial exploration speed cap 0.35 m/s; physical VESC ROS speed clamp ±3500 ERPM; nominal converter 8414 ERPM/(m/s). Neither motor current nor wheel-speed calibration has been verified.
- Current steering limit ±0.10 rad with wheelbase 0.26 m implies an **ideal minimum turn radius ~2.59 m**. Actual steering angle, direction and steering-to-servo relation must be measured. A tighter real track may be impossible with the current limits.

## 2. Changes made to address the observed failure

- Physical explorer now publishes `/exploration/diagnostics`: requested gap angle, raw target steering angle, steering saturation, nominal minimum radius, and clearance.
- Optional `experimental_candidate_planner:=true`: evaluates steering candidates within the *existing* ±0.10 rad bound using the same swept-body geometry helper as the independent mux, with a **1.05 m** lookahead versus mux's **0.80 m**. It chooses a feasible proposal or outputs zero when all candidates are blocked. The physical mux still independently checks every proposed command. This is **experimental and not Jetson-validated yet**. Default is FALSE.
- With the optional planner enabled, proposal searches run at ~10 Hz and publish at 20 Hz. No obstacle limit, unknown-ray rule, physical mux hold, or motor-current limit was relaxed.
- Early slowdown added when front LiDAR clearance is <1.50 m; stronger slowdown for <1.10 m. Hard STOP threshold at 0.80 m unchanged.
- `real_integrated.launch.py` now exposes `minimum_travel_m`, `minimum_departure_m`, and `minimum_duration_s` to configure first-lap completion to the actual track. Defaults remain 22 m, 2.5 m, 40 s; **do not assume the actual perimeter is 22 m**. Closure still requires observed map topology and return to starting pose.
- Added ROS-free `tools/exploration/physical_candidate_planner.py` self-test and read-only `tools/diagnostics/steering_radius_audit.py`.
- Earlier changes remain: map QoS correction; STOP history in `last_stop_reason`, `last_stop_age_s`; SLAM loop-closing launch arg.

## 3. First task on Jetson: integrate and syntax/unit tests

Stop all previous launches (duplicate VESC/SLAM/drive nodes are a known blocker); close VESC Tool (serial conflict). Check `git status --short` before pulling so no uncommitted files are lost.

```bash
cd ~/f1tenth_uj
git status --short
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 -m py_compile \
  tools/exploration/physical_candidate_planner.py \
  tools/exploration/exploration_controller_v2.py \
  tools/diagnostics/steering_radius_audit.py \
  src/f1tenth_bringup/launch/real_integrated.launch.py
python3 tools/exploration/physical_candidate_planner.py
python3 tools/diagnostics/steering_radius_audit.py
python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test

colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
```

If any test fails, do not run powered autonomy. Record the specific error. The new code was committed through GitHub but **has not been executed on the Jetson by this assistant**.

## 4. Test candidate steering in non-actuating shadow mode

Run with motor drive disabled, while LiDAR faces the real track:

```bash
ros2 launch f1tenth_bringup real_integrated.launch.py \
  enable_real_actuation:=false \
  enable_autonomous_actuation:=true \
  enable_slam_loop_closing:=true \
  experimental_candidate_planner:=true
```

In a second sourced terminal:

```bash
ros2 topic echo /exploration/diagnostics --once --field data
ros2 topic echo /control/status --once --field data
ros2 topic echo /exploration/drive --once
```

Verify that the candidate planner offers a nonzero steering proposal where a valid path exists, and that its `candidate_planner.candidates` agree with actual observed clear paths. If all candidates are blocked, the report gives `NO_FEASIBLE_STEER_WITHIN_LIMITS`. **Do not broaden the steering or collision rules to force a pass.**

The launch physically connects the VESC but does not allow motor commands when `enable_real_actuation=false`. A connected, powered controller is not the same as an independent propulsion isolation circuit.

## 5. Physical steering and stop validation gate

Before any unsupervised or ground autonomous motion:
- Verify servo center and direction with front wheels off the ground, and measure the actual wheel angles in both directions. Current nominal mapping: `servo = 0.361 - 0.50 * steer_rad`; at steer +0.10 -> servo ~0.311; at 0 -> 0.361; at -0.10 -> ~0.411. Mechanical output is unverified.
- Measure real minimum corner/centerline radius and compare with the 2.59 m **nominal** turning radius. Run `python3 tools/diagnostics/steering_radius_audit.py --radius <measured_m>`. If the radius is too small, actual steering calibration and feasible software/hardware limits must be established; higher software values alone are not an engineering fix.
- Verify motor spins forward for a positive request and stops when the hold is released and on sensor loss, during supervised wheels-off-ground testing. The motor must not continue accelerating on UART or ROS disconnect.
- Establish and verify an independently effective way to stop/isolate propulsion before a ground run. A ROS topic publisher or firmware ADC kill input alone is not an independent emergency-stop guarantee. Regen behavior, LiPo charge rating and actual braking distance are not verified.
- Track wall should be visible at LiDAR height, with adequate width/turning radius. No persons should stand inside a powered test area.

These are acceptance gates, not reasons to weaken the path guard.

## 6. First complete mapping lap (only after gate 5)

Use the same real-integrated launch with `enable_slam_loop_closing:=true`, `experimental_candidate_planner:=true` *after* non-actuating validation. Set first-lap travel minimum based on the measured track length. For an ~18-20 m closed loop, 12 m is a possible conservative minimum, but it is NOT a measurement of your present track. Other closure criteria remain active. Do not claim finished mapping from distance alone.

The existing AUTO mode/hold sequence must be observed: HOLD FALSE -> request AUTO -> verify AUTO_READY -> supervised HOLD TRUE. A hold already TRUE at mode selection produces `REJECT_AUTO_HOLD_HELD`; any obstacle/hold fault latches STOP until explicitly re-armed.

Record a rosbag before testing so a stop is diagnosable without reproducing motion:

```bash
OUT=~/f1tenth_hw_evidence/test_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
ros2 bag record -o "$OUT/rosbag" \
  /scan /odom_rf2o /map \
  /exploration/drive /exploration/diagnostics \
  /control/status /mission/physical_lap_progress \
  /mission/first_lap_complete
```

Never subscribe to `/drive` merely for logging: the physical mux expects exactly ONE subscriber and treats extra subscribers as an ownership fault. Record `/control/status` and `/exploration/drive` instead.

After a STOP, inspect `last_stop_reason` in `/control/status`; it preserves the original fault. A fresh obstacle scan after the stop does not erase that history.

## 7. Mapping pass criteria and outputs

- A complete physical lap measured by RF2O with a meaningful excursion from the starting pose and a return to the start.
- `/mission/physical_lap_progress` reports `observed_corridor_closed=true`, enough actual travel, map age <=8s, return distance <=0.85m and heading error <=48° (plus configured travel/duration).
- The mux stops on `/mission/first_lap_complete=true`.
- The finalizer produces `~/f1tenth_hw_evidence/physical_mapping_YYYYMMDD_HHMMSS/` with:
  `new_track.yaml`, `new_track.pgm`, `new_track_centerline.csv`, `new_track_body_optimized.csv`, `route_preparation.json`, `mission_status.json`.
- Final mission status is `COMPLETED_MAPPING_AND_PLANNING`; route audit `accepted=true`. Anything less is partial progress, not a success claim.

If the car stops on a wall again, inspect `/exploration/diagnostics`. If `raw_target_steer_rad` is frequently above 0.10 rad or all candidate swept paths are blocked, verify measured steering geometry and track radius first. If the route is viable but code selects the wrong side, capture the bag and correct the planner from real scan evidence.

## 8. Saved-map localization and racing are separate engineering deliverables

The physical system presently **does not** automatically switch from SLAM to AMCL or a Pure Pursuit controller. The simulator has these components, but transferring them requires compatible physical pose sources, a single TF authority, calibration, stage gating, and collision/braking verification. `robot_localization` was also reported missing on the Jetson. Therefore a successful mapping lap is not yet evidence of multi-lap physical racing.

Do not merge this branch into main before these gates have passed on real hardware.
