# F1TENTH UJ — Path to Real Autonomous Map Exploration

**Date:** 2026-11-05 (Jordan)  
**Working branch:** `zaq/hardware-integration`  
**Decision:** Advance autonomous planning using *real LiDAR* today, in observation-only mode, and make physical motion an explicit separate commissioning gate. An RC remote is not required for future autonomous operation, but it is **not** a substitute for an independent stopping means.

## Video baseline
The user-supplied ~6.4-second remote-control video shows the car moving and changing direction on an indoor tile floor near a rug. It does **not** establish full steering travel, repeatability, traction polarity, electronic stop reliability, ROS actuation, or sustained autonomous obstacle avoidance. The user reports looseness/high clearance in steering ball joints and limited left/right travel. Keep steering calibration and hardware inspection in TODO, not PASS.

## Already demonstrated on the physical platform
- LakiBeam LiDAR about 29.7 Hz.
- RF2O odometry 20 Hz; physical map built with SLAM Toolbox at 0.2 Hz.
- Isolated VESC firmware 7.0 handshake / VESC core telemetry about 50 Hz.
- VESC wheel odometry 50 Hz.
- Shadow `robot_localization` EKF 30 Hz, 14/14 transport/integration tests passed, **provisional** covariance and no authoritative TF.
- `tools/exploration/exploration_controller_v2.py` produces local LiDAR reactive commands on `/exploration/drive` while `platform:=physical` and `enable_drive:=false`. It **explicitly refuses** physical drive mode. Its reactive largest-gap policy is **not a coverage planner** and is not guaranteed to complete a closed map.

## Critical blockers to powered physical exploration
1. **Unexplained Jetson poweroff on 2026-11-04.** Check supply stability, reboot evidence and temperature; a control-computer poweroff must not leave a motor driving.
2. **No tested independent stop / no RC remote currently available.** The user explicitly deferred hardware E-stop verification. Without a remote, a verified independent stop is even more essential.
3. VESC Tool readback (2026-11-04): firmware timeout **1000 ms**, timeout brake current **0 A**, kill switch mode **Disabled**; high motor/current/regen limits still unvalidated. A timeout releasing torque is **not** a tested emergency stop.
4. Actual steering neutral, left/right angle, backlash and physical safe bounds **not calibrated**; `servo=0.361-0.5*steering_angle` is only numerically tested.
5. Forward/reverse motor polarity, ratio and true m/s to ERPM scale not physically verified.
6. No real closed corridor map yet. Simulator route checks cannot be used as driving clearance proof.
7. Bench-only `tools/safety/physical_drive_gate.py` is not a production safety system; it cannot control VESC firmware behavior after process crash.

Do not switch the v2 explorer to `enable_drive:=true`, remove its physical guard, start `teleop.launch.py`, or remap `/exploration/drive` to actuator topics until these are resolved and the independent stop is verified.

## Today's concrete non-actuating test (real sensors)

After checking boot/power health, use three sourced terminals.

**T1 — Real LiDAR + RF2O + SLAM, no VESC actuation:**

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch f1tenth_bringup observe_only.launch.py enable_vesc:=false
```

**T2 — Physical exploration algorithm, real scans, shadow proposals:**

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 ~/f1tenth_uj/tools/exploration/exploration_controller_v2.py --ros-args \
 -p platform:=physical -p enable_drive:=false \
 -p scan_topic:=/scan -p proposal_topic:=/exploration/drive \
 -p wheelbase:=0.26 -p speed:=0.15 -p max_steer:=0.15 \
 -p lookahead:=0.9 -p stop_distance:=0.8 \
 -p scan_timeout:=0.25 -p front_fov_deg:=120.0 \
 -p safety_bubble_m:=0.35
```

**T3 — Quantitative independent acceptance auditor:**

```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
python3 tools/exploration/physical_exploration_audit.py --self-test
OUT="$HOME/f1tenth_hw_evidence/exploration_shadow_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
python3 tools/exploration/physical_exploration_audit.py \
 --duration 60 --output "$OUT/decision_audit.json" | tee "$OUT/decision_audit.log"
ros2 run nav2_map_server map_saver_cli -t /map -f "$OUT/initial_map"
```

The auditor subscribes to `/scan`, `/exploration/drive`, `/odom_rf2o` and `/map`; records proposed speed, steering, real forward clearance, measured known-map area, stop fraction and unsafe proposals. It has **no publishers**. It independently treats missing/invalid front beams as blocked, flags proposals exceeding 0.15 m/s or 0.15 rad, and flags motion proposals with front measured obstacle nearer than 0.8 m. Audit failure is a **controller discrepancy to debug**, not a reason to weaken checks. A completely static robot cannot explore a whole map.

## First real powered movement: acceptance gates (not yet done)
- Independent stop electrically cuts/inhibits propulsion and is tested under appropriate supervision, with safe reset; no remote is required if an equivalent independent stop/deadman is available.
- Stable Jetson power and recovery after abrupt restart.
- Motor firmware timeout/limits verified on a secured stand; safe reaction to loss of USB/ROS/heartbeat tested.
- Steering checked for binding/backlash and permitted angles; motor sign/speed/current calibrated with wheels safely clear.
- One exclusive actuator command authority, stale-scan fail-closed, overspeed/current/electrical faults stop, startup disarmed, physical cut independent of software.
- Clear test area with restraints, no people in path, spotter at stop, measured low-speed first motion; record telemetry/scan/odom and actual distance. Only then enable autonomous small-step motion. Stop immediately for unexpected steering, power, current or map/TF faults.

## Physical autonomous mapping architecture after gates
LiDAR `/scan` → unknown-space-aware local obstacle avoidance + progress/coverage logic → bounded Ackermann trajectory proposal → independent validated safety controller → Ackermann-to-VESC converter → VESC, with separate hardware cutoff. RF2O + SLAM provide pose and accumulated map, optional shadow EKF only until calibrated. Unknown cells remain blocked, no simulated/free route is assumed. Use slow supervised small movements first; later detect returned-start / loop completion and save a complete map; only then centerline extraction and racing.

**Status:** The first activity is software evaluation on real sensor data, not physical autonomous driving. Real autonomous exploration is pending the safety and calibration gates.
