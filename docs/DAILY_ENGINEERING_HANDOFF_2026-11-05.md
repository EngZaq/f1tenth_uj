# F1TENTH UJ — Physical Engineering Daily Handoff
**Date:** 2026-11-05 (Thursday, Asia/Amman)  
**Repo:** https://github.com/EngZaq/f1tenth_uj  
**Development branch:** zaq/hardware-integration — **not merged to main**  
**Simulation reference:** https://github.com/EngZaq/f1tenth_uj_sim

## Executive handoff

Today we advanced from isolated live sensors to a connected physical ROS 2 autonomy command pipeline, verified the remote-free AUTO state handshake with propulsion disarmed, repaired the car-size-aware LiDAR path gate and its runtime performance, and implemented a no-VESC real-track mapping rehearsal that can save a real map and produce a candidate body-audited centerline after a manually pushed full lap.

**Verified on Jetson:** LiDAR / scan and RF2O / odometry + SLAM live; nonzero autonomous proposal; exact hardware command-graph ownership; AUTO_READY to AUTO_ACTIVE transition with real output disabled; swept-path software precondition PASS in one later real scan; guard runtime 4.17 ms; new mapping progress topic operating while stationary.

**Not achieved:** Real powered motion, steering calibration, physical stop test, independent propulsion cutoff, final hardware VESC limit verification, a closed physical track, a complete physical lap, saving a real closed-track map/route, or >5 m/s driving. Do not claim that a functioning proposal means a moving robot.

**User decisions:** Leave every VESC Tool firmware parameter unchanged for now. Battery C rating unknown. Long-term intended speed **greater than 5 m/s**, not capped at 5 m/s. Prioritize meaningful new project milestones rather than repeated tests.

## 1. Physical setup

| Part | Known values |
| --- | --- |
| Computer | Jetson Orin Nano 8 GB, Ubuntu 22.04, ROS 2 Humble |
| ROS | ROS_DOMAIN_ID=42, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp |
| LiDAR | LakiBeam L1, 1440 beams per scan, approximately 30.2 Hz, firmware driver scan 45–315 degrees |
| Odometry / mapping | RF2O /odom_rf2o about 20 Hz; SLAM Toolbox /map; RF2O sole odom-to-base_link TF |
| Car | Wheelbase 0.260 m, body length 0.345 m, width 0.197 m |
| LiDAR position | +0.215 m x from rear-axle base_link; height 0.150 m estimated |
| BLDC / controller | DYS D3542 1450KV, VESC MINI 6.7 Pro, firmware connection reported version 7.0 |
| Command scaling | 8414 ERPM per m/s, steering gain -0.50 and offset 0.361; NOT physically calibrated |
| VESC ROS clamps | ±1400 ERPM, servo commands [0.30, 0.42], distinct from firmware limits |
| Physical safety | enable_real_actuation=false by default; no independently validated physical E-stop |

The LiDAR driver represents no-return raw zero readings as positive infinity, which remains **unknown**, not evidence of clear 25 m space. Prior white-board central ±5 degree evaluation at 1.0 and 1.95 m gave approximately 2 cm error and 100% finite returns in the selected core.

## 2. Work done — physical autonomy command path

The integrated real hardware launch connects LiDAR, RF2O, SLAM, real VESC ROS driver, Ackermann-to-VESC conversion, physical_drive_mux, experimental LiDAR explorer, first-lap monitor, and physical map finalizer. The explorer only publishes speed/steering *proposals* to /exploration/drive; the mux alone owns /drive.

On real hardware the explorer published speed 0.075 m/s with steering -0.100 rad, and later ~0.146 m/s with steering -0.0053 rad; **no actual motor movement** was observed or claimed. Verified ROS topic ownership in the final reports: /drive had one publisher and one subscriber; /commands/motor/speed one publisher and one subscriber; /odom_rf2o one publisher. The /control/status output remained speed_mps=0 and steer_rad=0 due to REAL_ACTUATION_DISABLED.

Remote-free AUTO mode logic: user sends an AUTO selection request with hold released, then a live /control/auto_hold Bool heartbeat. Mode must pass STOP -> AUTO_READY -> AUTO_ACTIVE. The initial one-shot request did not reliably transition and diagnostics lacked acknowledgment fields; added last_mode_request, mode_request_result, auto_hold raw/fresh state and request age. A test script originally introduced a second /drive subscriber and triggered ACTUATOR_GRAPH_OWNERSHIP_FAULT. Fixed the script to avoid subscribing to the protected actuator topic.

**Confirmed Jetson handshake output:**
- PASS AUTO hold visibly released.
- PASS AUTO selection acknowledged: AUTO_READY_ACCEPTED.
- PASS AUTO state entered without actuation: AUTO_ACTIVE.
- PASS AUTO_ACTIVE stable for about 0.8 s; mux reported zero real drive output.
- Cleanup requested STOP and released hold; an independent subsequent confirmation of STOP after cleanup was not separately captured.

This proves software mode transitions under disarmed conditions only, not safe vehicle actuation.

## 3. Work done — LiDAR clearance and performance

The old blanket >=90% finite return demand within ±20 degrees blocked exploration despite good central range evidence. A **proposal-only** guard now demands reliable narrow center evidence and rejects measured nearby obstacles across a wider sector. The actuator mux has its own more complete car-footprint-aware path check.

The physical swept-path guard models the rear-axle bicycle trajectory over 0.8 m, body bounds rear=-0.040 m, front=0.305 m, half-width=0.0985 m, wheelbase=0.260 m and extra margin 0.080 m. It vetoes measured LiDAR points in the inflated swept rectangle. Unknown/no-return rays are not fabricated into maximum-range obstacle-free rays. An experimental software heuristic tolerates limited unknown-strip widths along the path; this is NOT a safety certification for thin/dark obstacles.

The first version saw 19–25 close reflection points at about 0.099–0.117 m and +71 to +74 degrees, positioned near x=0.25 m, y=+0.10–0.11 m in the rear-axle frame. Those likely came from the left body, wheel, mount or bumper, but were not physically identified with certainty. Subsequent real scans showed zero such measured blocking returns; exact physical change was not recorded.

The next problem was consecutive UNKNOWN LiDAR rays. Angular gaps were 1.75 to 2.75 degrees and included near-side +70.75..+73 degrees at ~0.18 m, where the estimated blind width was under 1 cm. Added diagnostics for start/end angles and approximate unknown projected widths at their intersection with the swept path; did NOT simply mark +infinity free space.

The nested-loop Python geometry regressed to **78.2 ms per guard** on Jetson, longer than the 50 ms mux cycle. Replaced it with NumPy-vectorized ray/rectangle intersections and added per-call compute_ms reporting plus fail-closed handling if a call reaches 40 ms.

**Final real Jetson snapshot today:**
- swept_path.ready=true; reason SWEEP_PRECONDITION_PASS_NOT_SAFETY_CERTIFIED.
- blocking_return_count=0.
- swept_known_fraction=0.8051 (19.49% unknown).
- max_unknown_path_width_m=0.0113 vs experimental threshold 0.020 m.
- compute_ms=4.17 on Jetson.
- scan_fresh=true, odom_fresh=true, map_fresh=true.
- physical actuation disabled; command output zero; selector mode STOP.

This is one useful live integration precondition, NOT proof of collision-free motion, braking, safe stop or reliable detection of low-reflectivity obstacles. Guard regression tests (unknown, obstacles, thin missing gaps, partial scans) passed before and after optimization.

## 4. Read-only calibration and VESC observations

The calibration script calculates expected conversion, not physical measurements:
- 0.075 m/s -> ~631 ERPM.
- 0.15 m/s -> ~1262 ERPM.
- steering -0.10 rad -> servo ~0.411; neutral 0.361; +0.10 rad -> ~0.311.
- ±0.10 rad steering at 0.260 m wheelbase -> theoretical turn radius ~2.59 m.

Actual motor polarity, wheel-speed scale, servo centering and angle/sign remain unmeasured.

User provided VESC Tool screenshots showing roughly +42.85 A motor current max, -42.85 A brake current, 64.28 A absolute maximum; +99 A battery current and -60 A battery regen; 1000 ms command timeout, timeout brake current 0; app kill switch disabled; minimum ERPM around 900. Screenshots may include connected and disconnected views: actual saved device firmware must ultimately be verified; these numbers cannot be treated as approved limits. Previous motor start transient exceeded ~7.93 A before an earlier external software abort.

**No VESC Tool values were altered.** User does not know battery C rating and wants firmware values left unchanged for now. Since motor/battery current limits, timeout, braking, reverse, and independent cutoff have not been validated, **do not enable powered autonomous motion** or infer safety from the ROS YAML clamps. The final speed objective exceeds 5 m/s, but actual capability requires physical calibration, rated hardware and a larger course.

## 5. New physical mapping and route-generation workflow

New file physical_map_rehearsal.launch.py launches:
1. Actual LakiBeam /scan and static LiDAR TF.
2. RF2O odometry.
3. SLAM Toolbox mapping **with loop closing explicitly enabled**.
4. Physical first-lap monitor publishing /mission/physical_lap_progress.
5. Physical map finalizer, which listens for a verified completion and attempts to save and audit the actual map and route.

The mapping rehearsal intentionally launches **NO VESC driver, Ackermann converter, physical motor mux or drive publisher**. Propulsion must be physically isolated for a manual push lap. observe_only.launch.py has an optional enable_slam_loop_closing arg (default false) to avoid changing established observation behavior.

The first-lap watcher now records ~1 Hz progress fields: travel_m, departure/return distance, heading return error, elapsed time, observed corridor closure, map free/safe cells, map age, eligibility for completion and thresholds. It writes ~/f1tenth_hw_evidence/physical_first_lap_progress.json.

Rehearsal-only default gates: departure >1.5 m, travel >=12.0 m, duration >=20 s, return distance <=0.85 m, yaw error <=48 degrees, AND a genuinely observed closed and clearance-eroded corridor. No map topology closure can be faked using UNKNOWN cells. The integrated actuation-intent watcher retains original longer mission thresholds.

Map finalizer uses the existing physical planning tools after real complete-lap evidence: it saves new_track.yaml and new_track.pgm, extracts new_track_centerline.csv, prepares new_track_body_optimized.csv with vehicle geometry audits, and records route_preparation.json, logs, mission_status.json. Added a stale prior-lap evidence check. The actual finalizer has **not** completed for a real track today.

**Actual stationary Jetson mapping progress received:**
- phase AWAITING_DEPARTURE.
- travel_m 0.08–0.11 m (stationary RF2O accumulated noise/drift, not real travel).
- return_distance_m 0.0, heading error about 0.3–0.5 degrees.
- known_free_cells 3228, safe_cells 203.
- observed_corridor_closed=false; largest interior area 0.
- map_age_s cycled around 0–5 s.
This confirms the topic, computation and local map are active, but **NO complete closed-track mapping lap has occurred**.

The user does not have a physical course yet. One possible mapping/low-speed first course was proposed: circular walls, outer diameter 7 m, inner diameter 5 m, 1 m-wide lane, about 18.85 m centerline circumference. With theoretical turn radius 3 m, needed ideal steering is about 0.0865 rad; actual steering still uncalibrated. Walls must intersect the LiDAR beam plane (about 15 cm high). This course was suggested, **not built**, and is not suitable to establish safe >5 m/s racing.

## 6. Source and documentation paths changed today

- src/f1tenth_bringup/launch/real_integrated.launch.py
- src/f1tenth_bringup/launch/observe_only.launch.py
- src/f1tenth_bringup/launch/physical_map_rehearsal.launch.py
- src/f1tenth_bringup/scripts/physical_drive_mux.py
- src/f1tenth_bringup/scripts/physical_path_guard.py
- src/f1tenth_bringup/scripts/test_physical_path_guard.py
- src/f1tenth_bringup/scripts/physical_first_lap.py
- src/f1tenth_bringup/scripts/physical_map_finalizer.py
- src/f1tenth_bringup/CMakeLists.txt
- tools/exploration/exploration_controller_v2.py and front_scan_guard.py
- tools/diagnostics/disarmed_auto_handshake.py
- tools/diagnostics/physical_near_return_probe.py
- tools/diagnostics/physical_calibration_precheck.py
- tools/diagnostics/lakibeam_reliability_audit.py and reanalyze_lidar_wall.py
- docs/PHYSICAL_FIRST_MOTION_READINESS_2026-11-05.md
- docs/PHYSICAL_TRACK_MAPPING_REHEARSAL_2026-11-05.md
- docs/REAL_INTEGRATED_ROS_HARDWARE_2026-11-05.md
- docs/LAKIBEAM_L1_RELIABILITY_2026-11-05.md
- This handoff and README.md.

These represent relevant work tracked on GitHub branch today; not all paths were modified during the final end-of-day commit. Jetson evidence files and ROS bags under ~/f1tenth_hw_evidence/ are not automatically GitHub uploads.

## 7. End-of-day engineering truth table

| Subsystem | State | Remaining proof |
| --- | --- | --- |
| Real LiDAR, RF2O, SLAM connectivity | Verified stationary | Dynamic localization/mapping on real course |
| Remote-free disarmed AUTO state machine | PASSED | Physically safe control after power commissioning |
| Physical LiDAR exploration proposals | PASSED | Actual ground motion and reliable detection |
| Exclusive ROS VESC command chain | Verified in graph snapshot | Fault injection; hardware cutoff |
| Optimized swept-footprint software gate | PASS once on Jetson; 4.17 ms | Worst-case response, dark/thin objects, real stopping margin |
| Physical mapping-only rehearsal launch | Operational stationary | Push a real complete lap |
| Real closed map and route extraction | Implemented, NOT DONE | Build track, push loop, receive accepted finalizer outputs |
| Real steering / motor speed / braking | NOT VALIDATED | Controlled attended physical commissioning |
| Firmware electrical protection and physical cutoff | NOT VALIDATED | Battery rating, actual firmware, independent E-stop |
| Greater than 5 m/s racing | Long-term target only | Hardware, track and control upgrades |

## 8. Tomorrow — shortest path to real progress

1. **Build physical closed track** with actual LiDAR-visible walls and clear geometry. Preferred suggested first course if room exists: 7 m outside diameter / 5 m inside diameter. Map and low-speed mechanical geometry only, not high-speed racing.
2. With propulsion independently isolated, run the **no-VESC** physical_map_rehearsal.launch.py and manually push the car around one complete circuit. Watch only /mission/physical_lap_progress rather than repeating LiDAR audits.
3. Inspect ~/f1tenth_hw_evidence/physical_mapping_YYYYMMDD_HHMMSS/mission_status.json; if COMPLETE, inspect actual new_track.yaml, new_track.pgm, new_track_centerline.csv, new_track_body_optimized.csv and clearance report. If it fails, use its specific stage/error instead of redoing unrelated tests.
4. Implement shadow physical localization handoff and Pure Pursuit using real map/estimated pose (no motor commands). Only after electrical protection, steering calibration, independent stop, motor response and braking are checked should the team consider supervised powered driving.
5. Keep VESC Tool firmware values unchanged as explicitly requested; do not claim they are safe. No ROS limit raised and no unverified powered speed changes.

### Resume command for next mapping session
    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch f1tenth_bringup physical_map_rehearsal.launch.py

Separate sourced terminal:
    ros2 topic echo /mission/physical_lap_progress --field data

### Shutdown and sync
- Stop the ROS launch using Ctrl+C and place hardware into an electrically safe state. A ROS STOP or serial disconnect is NOT a physical E-stop.
- GitHub connector changes are on **zaq/hardware-integration**, not main.
- Jetson-only uncommitted work, raw ROS bags, sensor logs, large mapping artifacts and private configs cannot be inferred to be backed up by remote commits. Inspect git status before staging and pushing. Avoid committing secrets or oversized bags.
