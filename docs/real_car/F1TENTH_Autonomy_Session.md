# F1TENTH | First physical autonomy evaluation
**Zaid • 25 November 2026 • one power cycle, up to 90 minutes**

**Decision for this session.** Evaluate whether the physical explorer gives repeatable, sensible steering and stop proposals on a measured course. Use existing recordings first. Spend car time only on missing scenes, physical references, or a supervised closed-loop segment if a working, brake-integrated autonomous configuration is independently confirmed. On the reviewed branch, the immediately available experiment is proposal evaluation. A recorded proposal measures what the planner wanted. A closed-loop run measures what the vehicle actually did.

## 1. Prepare with the car off

- Ask for the laptop copy of the most recent real closed-course run, if one exists. A useful bundle includes the complete bag directory (metadata and data files), map YAML **and** image if saved, course sketch or photo, and any start/finish marks, route, launch configuration and run notes. For obstacle-avoidance tests, also obtain Ahmad’s exact revision, entry point, parameters and output topic. The existing physical explorer is a separate baseline until his implementation is identified.
- Use `hall_loop_20261123_172534` for sparse corridor scans; use `20261123_174200_teleop…` for manual requests, mux and converter events; use the T4 tile runs for stops near a wall and T3 circle runs for turn response. An existing complete course bag with physical reference measurements replaces a new recording of the same condition. The November 23 corridor run ended before the return and has no measured endpoint.
- On the laptop, open the repository at the intended revision, source ROS Humble and the built workspace, and verify `ros2 bag info <bag-directory>` can read an existing recording. Required local ROS packages include the project messages, rosbag storage/compression plugins and stamped Ackermann messages. Check disk space on the laptop and stage a USB drive or known transfer destination. Do any dependency setup before the session.
- Pick a room and lay out the course once. Tape a start line and forward arrow at the rear axle, mark the obstacle positions and record the actual clear widths. Use a fixed camera with a scale in view where geometry will be measured; record an identifiable event visible in the video and trace so they can be aligned. A sketch with tape measurements is sufficient for endpoint and width checks. Whole-lap true tracking error needs an independently calibrated trajectory reference.
- With power off, inspect the known wheel play and IMU attachment. Photograph the condition; record any change since T3/T4. Mechanical retention matters for powered driving. A rigidly mounted tilted IMU can be characterized using the existing software and data. A loose wheel or shifting IMU mount invalidates different measurements. If wheel retention is questionable, keep the chassis stationary until secured.
- Write a row for each **missing** scene in the table below. Assign an operator and observer for any moving trial. Reserve the last 20 minutes for verification, copying and shutdown. The 90 minutes is an upper bound; finish earlier when the required evidence is captured.

### Run cards

| ID | Question | Scene and physical reference | Evidence and interpretation |
|---|---|---|---|
| A | Does the baseline permit forward progress on a clear section? | Clear straight, then one measured corner in the same layout. Record car heading and nearest wall distances. | Scan + proposal + reason. A repeated hold or steering saturation at the marked corner identifies a concrete planner/geometry issue. |
| B | Does the decision respond correctly to obstacle side? | Same marked car pose and heading. Record a clear baseline, then place a stationary obstacle left and right at measured offsets. Keep the opposite passage physically wide enough for the car and a turning arc. | Photo, obstacle dimensions, both usable gap widths, steering sign, status and variation while stationary. A correct side choice alone does not establish clearance through the maneuver. |
| C | Does the planner withhold a forward command when the path is blocked? | At the marked pose, block the usable passage. Keep the car stationary; put the obstacle in position before recording the scored window. | Measure front-bumper distance, record scan and status. Score a positive-speed proposal during an unambiguously blocked steady window as a failure. This is a planner decision test, not a brake-distance test. |
| D, only if missing | Can saved-map localization be evaluated on an independent traversal? | On a closed course, make one reference traversal and a separately recorded repeat with the same layout. Mark start and finish at the same position and heading. Hand-guided travel is sufficient for localization evidence if the wheel assembly is secure. | Build or reuse the map from the first dataset. Evaluate localization on the second. Measure lane widths and at least two landmark dimensions. Keep both runs distinguishable from driven laps. |
| E, conditional | Does autonomous control produce the intended real motion? | Short clear segment first, supervised, in a controlled area. Use only a reviewed deployed autonomous command path with a verified brake converter, independent stop, secure steering/wheels, and working intervention. For saved-route driving also require a matching map, route and localization. | Log command selection, converter outputs, measured vehicle motion, body clearance, intervention and stop. Record the actual tested configuration. Attempt a longer segment only after the first is reviewed on site. |

**Known integration gap.** At physical branch `b3af767`, `real_integrated.launch.py` includes the physical explorer and original `ackermann_to_vesc_node`. The T1/T4 brake evidence belongs to the T1b brake converter configured for those tests. The working manual setup alone does not verify an autonomous command path through that converter. If there is a newer local solution, capture the source revision, effective launch, exclusive mux output and verified stop evidence for that exact path. With the reviewed state, run A–D as applicable. Do not enable powered AUTO on the old integrated launch or loosen the visibility gate to make a case pass.

## 2. Use the one power cycle

| Elapsed | Action | Exit condition |
|---|---|---|
| 0–10 min | Snapshot deployment, evidence folders and disk space; check for runs after 23 Nov. | Known checkout and existing-data inventory. |
| 10–20 min | Start the observation stack if new data is needed. Capture one short bag and inspect it immediately. | Usable scan/TF/IMU data, plus wheel telemetry if the experiment needs it. |
| 20–50 min | Collect only missing A–C scenes. If D is genuinely missing and the course is ready, record the marked traversals. | Each selected case has a bag, reference and time window. |
| 50–70 min | Conditional E only if the integration above was resolved and verified beforehand. Otherwise repeat the one ambiguous scene or capture the independent traversal. | Result, intervention or documented blocker. |
| By 70 min | End acquisition. Check bags, export new evidence, verify the copy and shut down. | Laptop copy opens; if transfer overruns, close the bags and preserve the originals before power runs low. |

**Jetson inventory** (read only)
```bash
cd ~/f1tenth_uj
git status --short --branch
git log -1 --format='%h %ci %s'
git submodule status
sha256sum src/f1tenth_bringup/scripts/physical_drive_mux.py install/f1tenth_bringup/share/f1tenth_bringup/python_nodes/physical_drive_mux.py
df -h "$HOME"
find ~/f1tenth_hw_evidence -name metadata.yaml -printf '%T+ %p\n' | sort -r | head -40
```

**New sensor capture only if the old evidence is missing it.** On the Jetson, in each terminal, source the same project environment. Use the capture configuration separately from any other bringup. Check for an existing stack before starting another.
```bash
cd ~/f1tenth_uj
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source tools/real_car_tests/local_env.sh
ros2 node list
```

With the prior stack down, use separate terminals:
```bash
# A: observation and read-only VESC telemetry
ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=true
```
```bash
# B: IMU, keep the car still through startup
ros2 launch f1tenth_imu imu.launch.py
```
```bash
# C: checks sensors, then starts a fresh timestamped bag
bash tools/real_car_tests/hall_loop_record.sh --check-only
bash tools/real_car_tests/hall_loop_record.sh
```
For A–C, keep the car in the planned pose and log steady scene intervals in the run notes. For D, follow the recorder’s marked-start instructions, keep still for 10 s at the start and 5 s at the finish. Stop terminal C with Ctrl+C and wait for “Recording stopped”; use the printed folder for the next checks. Repeat recording as needed without power cycling. The recorder’s default motion instructions apply to a lap; its bagging also works for the stationary scenes.

**Verify immediately.** Run `ros2 bag info <printed-run-folder>/bag`. Inspect topic counts, duration, and the recorder’s printed optional topics. Keep `/scan`, `/imu/data_raw`, `/odom_rf2o`, `/tf` and `/tf_static`. For wheel/fusion comparisons also confirm `/sensors/core` and `/odom_vesc_observe`. Record the actual IMU mount orientation and effective TF/config separately if the observation graph does not publish that transform. The scan-only planner replay below does not need it. The acquisition helper records raw inputs; planner proposals and diagnostics can be generated afterwards. For a genuinely driven run, retain the runner’s original bag, configuration, events, mux and converter status, servo command, feedback and any planner output. Avoid broad `ros2 bag record -a` on the car: extra subscribers on actuator topics interfere with the project’s ownership check.

**Write beside each bag:** run ID, mode (stationary / hand-guided / manual / autonomous), date, commit and local changes, configuration, start pose, obstacle dimensions and offsets, tape measurements, start/end times for each scene, command source, observer, video filename, and what caused every stop. Attach the route and map used for a saved-route run. Record interventions separately from planner stops. Copy each **whole** run folder, including bag metadata, compressed files, recorder info, events and notes. On the laptop run `ros2 bag info` again and open the video. Keep the Jetson originals.

## 3. Run the autonomy analysis on the laptop, with the car off

**Existing explorer baseline** at `b3af767`. Use a laptop with a matching ROS Humble workspace built, all three terminals sourced, and the same isolated local domain. The installed explorer needs its sibling guard modules.
```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=87
unset CYCLONEDDS_URI
```
In terminal A start the physical-mode proposal node. The speed, steering and visibility values below match the integrated launch’s baseline profile. The separately calibrated ground steering limit is different. Keep the chosen profile with the results.
```bash
python3 "$(ros2 pkg prefix --share f1tenth_bringup)/python_nodes/exploration_controller_v2.py" --ros-args \
  -p use_sim_time:=true -p platform:=physical -p enable_drive:=false \
  -p wheelbase:=0.260 -p speed:=0.35 -p max_steer:=0.10 \
  -p stop_distance:=0.80 -p scan_timeout:=0.22 -p min_front_valid_fraction:=0.90 \
  -p enable_candidate_planning:=false
```
In terminal B record the proposals and reasons in a **new** output directory for each replay:
```bash
ros2 bag record -o baseline_proposals_A /exploration/drive /exploration/diagnostics /clock
```
In terminal C play only the original scan input (replace the path):
```bash
ros2 bag play /path/to/run/bag --clock 100 --rate 1.0 --topics /scan
```
Stop the recorder and node after playback. Confirm both output topics contain messages and that diagnostics respond to scans. If they only report waiting or stale scans, check `ros2 topic info /scan -v`, clock and QoS compatibility before scoring a run. Align proposal windows to the original scan time using simulation clock and message stamps. The node tracks scan arrival time; replay alone cannot establish the original physical decision latency. Treat Ahmad’s planner as a separate comparison using its actual interface once supplied.

| Offline result | Calculation and reference | Decision |
|---|---|---|
| Obstacle decisions | For each labeled steady scene count move/hold proposals and status reasons; report wrong-side steering and forward proposals in blocked windows. Compute the finite-scan fraction using the same angular sector and range rules as the node. | Identify which geometry or return pattern causes an incorrect decision or unnecessary hold. |
| Steering feasibility | Compare requested angle and saturation time with the active software cap and measured T3 curvature. The kinematic radius is wheelbase / tan(abs(steer)); 0.260 m and 0.10 rad imply about 2.59 m, versus about 0.90 m at 0.28 rad. Wheel play and slip affect actual response. | Distinguish an impossible requested corner from a sensing error. |
| Mapping and localization | Only for a closed course with reference measurements: use the existing `hall_loop_map.py <run-folder>` on a copy. Compare loop-closing variants, endpoint position/heading at the tape mark and measured landmark widths. Then test localization on a held-out traversal. | Decide whether this map can support saved-route testing. A scan-to-map match alone cannot establish true position accuracy. |
| Motion and stopping | For a driven trial align mux selection, stop request, converter brake, ERPM, RF2O, and video or measured endpoint. Report latency and distance with the reference and uncertainty. No physical stopping-distance result can be inferred from stationary proposals. | Decide whether the tested command chain behaves as intended on the car. |
| Closed-loop driving, if collected | Report completed segment/laps, interventions, worst clearance, and estimated cross-track error. Label external measurements separately. Plot scan/command at the first failure and compare it with the video. | Identify the single change needed before extending the run. |

**Scope of verification.** Commands and filenames were checked against the physical repository. This sheet has not been executed on your Jetson or against your raw bags. The attached handoff covers operator evidence through 24 Nov at 00:23. Newer local files or runs may supersede it.

**Project sources:** [Integrated launch](https://github.com/EngZaq/f1tenth_uj/blob/b3af767/src/f1tenth_bringup/launch/real_integrated.launch.py) · [Existing explorer](https://github.com/EngZaq/f1tenth_uj/blob/b3af767/tools/exploration/exploration_controller_v2.py) · [Recorder](https://github.com/EngZaq/f1tenth_uj/blob/b3af767/tools/real_car_tests/hall_loop_record.sh) · [Measured results](https://github.com/EngZaq/f1tenth_uj/blob/b3af767/docs/real_car/REAL_CAR_TEST_RESULTS.md) · [Route proposal](https://github.com/EngZaq/f1tenth_uj/blob/b3af767/docs/real_car/ROUTE_PLANNING_PROPOSAL.md)
