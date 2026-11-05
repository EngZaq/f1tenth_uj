# Physical F1TENTH — Real VESC integrated ROS2 (2026-11-05)

This branch connects real VESC driver, original Ackermann converter, joystick, physical explorer, LiDAR, RF2O and SLAM through one command selector. The system is **not** a shadow-only VESC demonstration. It is also **not** approved for ground/autonomous operation until independent stop, motor/battery/regen firmware current limits, timeout and steering direction are verified.

## Actual command flow

    LiDAR -> /scan -> RF2O -> /odom_rf2o -> SLAM -> /map
                  |
                  v
           exploration_controller_v2 -> /exploration/drive
                                             |
    Joystick -> joy_teleop -> /manual/drive    |
          \                    /               |
          /joy -----------> physical_drive_mux <-
                                   |
                       ONLY publisher of /drive
                                   |
                         ackermann_to_vesc_node
                            /              \
               /commands/motor/speed     /commands/servo/position
                            \              /
                              real vesc_driver
                                    |
                           REAL MOTOR + SERVO

The real VESC is always connected in this launch. When disabled or disarmed, the mux emits zero motor-speed requests; it is not publishing nonzero physical motion. Software zero-speed, joystick release and a firmware timeout are **not** an emergency-stop solution.

The launch reuses observe_only.launch.py only for sensors, RF2O and SLAM, with its extra VESC instance OFF. A single real VESC driver and converter are created separately. Do not run the old teleop.launch.py, separate VESC Tool connection, or other VESC drivers simultaneously.

## Modes

- STOP is the default, latched after any deadman release or missed required inputs.
- MANUAL_READY is selected only with joystick deadman button 5 released. Hold that button to enter MANUAL_ACTIVE. The joystick axes are the same as the previously working teleop, with reduced scales: 0.15 m/s and 0.10 rad.
- AUTO_READY requires a separate launch option enable_autonomous_actuation true, plus the same held deadman. AUTO additionally needs valid forward LiDAR measurements, fresh RF2O and SLAM occupancy map; unknown forward beams remain unknown. Recorded LiDAR validity ~88%, below the autonomous gate of 90%, so automatic motion in that scene may correctly remain blocked.
- Manual forward scanning requires >=75% finite rays +/-20 degrees and no measured obstacle below 0.80 m; this is only a supervised commissioning guard, not collision avoidance. No reverse motion.
- The software selector clamps speed and steering to 0.15 m/s and +/-0.10 rad. At 8414 ERPM per m/s, 0.15 m/s is about 1262 ERPM, above the firmware 900 minimum. These gains remain physically uncalibrated.
- Releasing the joystick deadman latches STOP. To resume, release button 5, select MANUAL again, then hold the button. A lost joystick, stale command, stale scan or competing drive publisher also stops this selector.

## Build and verify — hardware remains DISARMED

Stop all previous ROS nodes and VESC Tool. On Jetson:

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
    python3 -m py_compile src/f1tenth_bringup/launch/real_integrated.launch.py
    ros2 launch f1tenth_bringup real_integrated.launch.py enable_real_actuation:=false enable_autonomous_actuation:=false

This launch **does start the real VESC driver** and converter, with the mux disarmed and its nonzero output locked. Run with motor power isolated on first graph check if possible.

Second sourced terminal:

    ros2 topic echo /control/status --once
    ros2 topic info /drive -v
    ros2 topic info /commands/motor/speed -v
    ros2 param get /vesc_driver speed_max
    ros2 param get /physical_drive_mux enable_real_actuation
    ros2 topic hz /scan

Expected: exactly 1 publisher on /drive (physical_drive_mux), exactly 1 subscriber (ackermann_to_vesc_node); exactly 1 publisher on /commands/motor/speed (Ackermann converter) and 1 subscriber (VESC driver); speed_max=1400.0; mode STOP, output 0. Multiple publishers are a critical error; STOP and inspect.

## Real, wheels-off-ground commissioning — not yet authorized

A *different* launch can enable real motor and servo actuation. **Do not perform powered testing until a properly rated independent cutoff is installed and tested, and the actual firmware motor/battery/regen current and command timeout limits are read back and validated.** User has declined firmware changes at this stage, so do not represent the current setup as meeting those prerequisites. Battery unplugging under load may arc.

Once all safeguards are independently satisfied, secure the car raised with a spotter and run exactly ONE launch:

    ros2 launch f1tenth_bringup real_integrated.launch.py enable_real_actuation:=true enable_autonomous_actuation:=false

Then select MANUAL with joystick button 5 released:

    ros2 topic pub --once /control/request_mode std_msgs/msg/String "{data: MANUAL}"

Hold button 5 while providing a small joystick speed/steering input, observe on-stand response, release button to stop and latch. The real VESC receives the converted output. Do not publish directly to /drive or /commands/motor/speed. Do not enable autonomous actuation at this stage.

## Engineering limitations

- VESC firmware current screenshots showed 42.85A phase current, 99A battery current, -60A regen current before controller-confirmed updates; these are unverified and too permissive for initial commissioning. The ROS driver's current_max=2A only clamps explicit current *commands* and cannot cap speed-mode phase current. A previous pulse reached 7.93A before the software detected it.
- 300 ms timeout was stated but has not been independently read back after the change. Timeout Brake Current was 0A, meaning no active braking on timeout.
- Motor speed sign, steering end stops, mechanical backlash, acceleration transients, physical E-stop and Jetson brownout behavior remain unvalidated.
- Physical LiDAR has persistent missing rays, and a planar laser alone is not sufficient for low obstacles or safe autonomous ground operation.
- If the computer crashes, the mux cannot publish STOP; physical cutoff and independently verified VESC behavior are required.

## Git changes

- launch/real_integrated.launch.py
- scripts/physical_drive_mux.py
- config/joy_teleop_integrated.yaml
- config/vesc_real_integration.yaml
- CMakeLists.txt installs mux plus existing exploration_controller_v2.py and front_scan_guard.py as ROS executables.

Status: committed but Jetson build and live integrated graph verification pending.


## 2026-11-05 follow-up: launch packaging fix and real simulator-mission port

The first physical combined launch **failed before starting** because `exploration_controller_v2.py` was a GitHub mode-100644 file. ROS `launch_ros Node` could not locate it as an executable under the package `libexec` directory, even after `colcon build --symlink-install`. No successful integrated launch was demonstrated by that run.

**Fix now committed:** CMake installs the Python files as package `share/f1tenth_bringup/python_nodes/` **data**, and `real_integrated.launch.py` starts the explorer and mux with `python3 -u /path/to/script.py` via `ExecuteProcess`, so no Git file chmod is needed.

The physical launch now also includes:
- `physical_first_lap.py`: real RF2O pose continuity, traveled distance, start/return distance and heading gates; observes `/map` and uses the same `map_coverage.corridor_coverage()` source ported from simulator `main` to demand a real observed free-space closed corridor. Latches `/mission/first_lap_complete` only when *all* checks hold.
- `physical_drive_mux.py`: subscribes to the first-lap completion latch, forces STOP and refuses rearming until a new mission session. AUTO no longer requires a joystick; it instead requires a **continuously refreshed supervised /control/auto_hold Bool**. A ROS Bool heartbeat is NOT an emergency stop. The launch defaults to `enable_joystick:=false` because the user does not currently have a remote.
- `physical_map_finalizer.py`: after completion, invokes `ros2 run nav2_map_server map_saver_cli -t /map -f ...`, then the existing **physical geometry** `tools/physical_planning/extract_robust_centerline.py` and `prepare_vehicle_route.py`; records `mission_status.json`, map, centerline, audited optimized route and logs in `~/f1tenth_hw_evidence/physical_mapping_YYYYMMDD_HHMMSS/`. It does not command propulsion. The route's direction is NOT yet verified against the actual robot heading; it must NOT be handed straight to the race controller.
- The existing physical SLAM Toolbox uses `map → odom → base_link`, unlike simulator's `slam_map → fusion_odom → fusion_base`. The direct simulator launcher **must not** run on the physical Jetson. This is an actual source-based adapter, not a copy of simulator truth / model sensors / Gym.

### One command to rebuild and inspect complete physical stack

In a clean terminal, with old VESC, ROS launch and VESC Tool stopped:

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select f1tenth_bringup
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

python3 -m py_compile \
  src/f1tenth_bringup/scripts/physical_drive_mux.py \
  src/f1tenth_bringup/scripts/physical_first_lap.py \
  src/f1tenth_bringup/scripts/physical_map_finalizer.py \
  src/f1tenth_bringup/launch/real_integrated.launch.py
python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
ls -l install/f1tenth_bringup/share/f1tenth_bringup/python_nodes/

# Real VESC + sensors + RF2O + SLAM + physical mission, NON-ACTUATING initially:
ros2 launch f1tenth_bringup real_integrated.launch.py \
  enable_real_actuation:=false \
  enable_autonomous_actuation:=true \
  enable_joystick:=false
```

In another sourced terminal:

```bash
ros2 topic echo /control/status --once
ros2 topic info /drive -v
ros2 topic info /commands/motor/speed -v
ros2 topic hz /scan
ros2 topic hz /odom_rf2o
ros2 topic echo /map --once --field info.resolution
ros2 pkg prefix nav2_map_server
```

The VESC driver is **real and connected**; disabled here means the command selector sends zero, not that VESC hardware has been removed. This is a bench integration test, not a ground-drive command. Verify zero drive output and no conflicting motor publishers.

### Remote-free AUTO mode evaluation (real VESC connected, output still zero)

Only after launch with `enable_real_actuation:=false`, inspect `/control/status` and then:

```bash
ros2 topic pub --once /control/request_mode std_msgs/msg/String "{data: AUTO}"
ros2 topic pub -r 20 /control/auto_hold std_msgs/msg/Bool "{data: true}"
```

AUTO should advance READY → ACTIVE, but `real_actuation_enabled=false` MUST keep `speed_mps=0`. Cancel the continuous hold with Ctrl+C; the selector should latch STOP. This verifies that **a remote is not architecturally required** for autonomous requests. It does not validate ground movement or rescue hardware faults.

### What still limits a powered autonomous drive

**Do not enable powered AUTO just because the combined launch passes**. User's VESC configuration was last seen with approximately 42.85 A motor phase, 99 A battery, -60 A regen limits, zero timeout braking, and an unverified 300ms timeout; prior raised bench test reached 7.93 A before reactive software abort. Battery plug removal is not a tested independent E-stop. The real front scan in the relocated environment has median 88% finite rays across ±20°, below the current 90% AUTO acceptance. No confirmed dynamic body-clearance or stopping distance yet.

**Also:** simulator `main` accepts mapping + nominal planning only. The physical route's orientation/AMCL/Pure Pursuit racing handoff has NOT been ported or validated. Focus next on (1) first integrated graph PASS; (2) independent power/stop commissioning; (3) real map progress and first lap; (4) save/plan; (5) localization and tracking.


## Hardware integration graph fault observed: 2026-11-05

Actual Jetson output confirmed `/scan` about **29.93–29.95 Hz**, `/odom_rf2o` about **20 Hz**, `nav2_map_server` installed. The launch package and main sensors are now running.

But the ROS graph had **three separate `ackermann_to_vesc_node` instances**, three subscriptions to `/drive`, and three publishers to `/commands/motor/speed`. It also had duplicate `laser_static_tf`, `rf2o_laser_odometry`, `joy` and `joy_teleop` names, even though the new launch configured `enable_joystick:=false`. This strongly indicates multiple launch/process instances remained running. A single VESC driver may accept commands from three converters and cannot distinguish ownership.

The `physical_drive_mux` correctly remained STOP with `UNEXPECTED_ACTUATOR_PUBLISHER_COUNTS`. Commit `993c13c` expands its check to all FOUR publisher/subscriber counts and changes the latching reason to `ACTUATOR_GRAPH_OWNERSHIP_FAULT`; it reports all four counts. This is a software precaution, **not** protection against an independently running rogue node or process after a computer crash.

### Recovery without touching firmware or commanding motion

1. **Physically isolate drive power / keep the car raised.** Stop **every** former launch session using Ctrl+C, including observe-only, integrated, stand-alone RF2O/SLAM and any joystick teleop; stop VESC Tool.
2. In a separate terminal, inspect living processes:
   ```bash
   ps -eo pid,ppid,args | grep -E '(ackermann_to_vesc|rf2o_laser_odometry|static_transform_publisher|joy_teleop|joy_node|real_integrated.launch|observe_only.launch|teleop.launch)' | grep -v grep
   ```
   If any process remains after Ctrl+C, inspect its PID and parent launch, then terminate that **specific** stale process using `kill -INT PID`. Do not blindly kill unrelated ROS processes.
3. Reinitialize graph discovery if needed with `ros2 daemon stop; ros2 daemon start`.
4. Pull `zaq/hardware-integration`, build `f1tenth_bringup` and start **one** real integrated launch with `enable_real_actuation:=false enable_autonomous_actuation:=true enable_joystick:=false`.
5. Check ownership:
   ```bash
   ros2 topic info /drive
   ros2 topic info /commands/motor/speed
   ros2 node list | sort | uniq -c | grep -E 'ackermann_to_vesc|rf2o_laser_odometry|laser_static_tf|joy'
   ros2 topic echo /control/status --once --field data
   ```
   Expect **exactly 1 publisher + 1 subscriber** at each actuator link. Exactly one Ackermann converter, one RF2O and one LiDAR static TF; zero joystick nodes when disabled. If still duplicated, capture `ps` plus complete launch-terminal logs before making any actuation change.

### Readiness

Passing ownership + ROS transport is a real integration milestone, not physical motion clearance. The vehicle still lacks a verified independent motor stop, confirmed safe firmware motor/battery/regen limits and tested stopping distance; VESC startup drew 7.93 A on the earlier bench pulse. The live physical explorer also rejects most scans with ~88% finite forward measurements against its 90% acceptance floor. These issues must be solved to run powered autonomous exploration reliably without merely disabling the checks.

## 2026-11-05 AUTO software-state handshake fix

Hardware evidence:
- /exploration/drive published steering -0.10 rad, speed 0.075m/s.
- /drive stayed at zero with enable_real_actuation=false.
- /control/status showed mode STOP, real disabled, hold_held_and_fresh=true.
- Repeating a one-shot hold false followed immediately by one-shot AUTO then continuous hold true did NOT enter AUTO_READY/AUTO_ACTIVE.

Exact cause visible in physical_drive_mux.py: mode_cb explicitly rejects AUTO while self.auto_hold is true. The release is sent as a transient ROS one-shot with no acknowledgement, and AUTO is a second one-shot with no acknowledgement. A missed or unordered message makes AUTO stay STOP. The state reported REAL_ACTUATION_DISABLED regardless of the actual mode request outcome, hiding this defect. The actuator chain is functioning normally.

Fix: physical_drive_mux now exports last_mode_request, mode_request_result, auto_hold_raw_value, mode_request_age_s in /control/status. It labels successful AUTO_READY_ACCEPTED and rejection REJECT_AUTO_HOLD_HELD / REJECT_AUTO_DISABLED_AT_LAUNCH. It still refuses selecting a mode with the hold asserted. **No actuation gate changed.**

Added tools/diagnostics/disarmed_auto_handshake.py:
- Checks a live /control/status publisher and requires real_actuation_enabled=false, and verifies /drive zero.
- Refuses to run if another hold publisher still exists or another request source is active.
- Streams hold false and waits until reported false.
- Streams AUTO requests while false until receives READY acknowledgment.
- Streams true and waits until reports AUTO_ACTIVE, with drive zero.
- Reports PASS per state; sends STOP and hold false on cleanup.

To run: STOP any old continuous ROS hold publisher (Ctrl+C); STOP old integrated launch (Ctrl+C). Then:

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ros2 launch f1tenth_bringup real_integrated.launch.py enable_real_actuation:=false enable_autonomous_actuation:=true enable_joystick:=false

In another sourced terminal (without any old hold publisher):

    cd ~/f1tenth_uj
    python3 tools/diagnostics/disarmed_auto_handshake.py

Expected: PASS hold released, PASS AUTO_READY, PASS AUTO_ACTIVE, PASS stabilized and cleanup STOP. All with real disabled. If test aborts because stale publisher exists, do not forcibly enable real power; kill the identified old /control/auto_hold CLI before retesting. If it times out, print full last /control/status and mode_request_result. This is a ROS mode-state test only, not evidence that real driving is safe.

Physical motor commissioning (firmware limits + independent power cutoff) remains unverified.
