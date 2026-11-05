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

