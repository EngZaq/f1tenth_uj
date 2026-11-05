# F1TENTH physical: fast first-motion readiness | 2026-11-05

Status: hardware branch zaq/hardware-integration, NOT pushed to main. This plan reduces repeated tests, does not bypass safety gates. Software has not been run on the Jetson with the new swept-path guard yet.

## Completed with real hardware

- Physical LiDAR live ~30.2Hz and 1440 samples/frame, calibrated white-board central +/-5deg 100% finite at 1.0 and 1.95m, approximately +/-2cm median.
- RF2O /odom_rf2o live ~20Hz, map and scan fresh.
- /drive one mux publisher and one Ackermann converter subscriber; /commands/motor/speed one converter publisher and one real VESC driver subscriber.
- Real LiDAR explorer published nonzero speed 0.075m/s and steer -0.100rad.
- Disarmed AUTO handshake STOP -> AUTO_READY -> AUTO_ACTIVE passed; STOP cleanup requested and /drive remained zero with enable_real_actuation false.
- LakiBeam upstream maps raw zero returns to +inf; VESC firmware current limits and physical independent stop remain UNVERIFIED.

## Code changed

- src/f1tenth_bringup/scripts/physical_path_guard.py adds physically sized Ackermann swept rectangle using rear-axle reference and LiDAR x=0.215m. Car front=0.305m, rear=-0.040m, half width=0.0985m, wheelbase=0.260m. It inflates by 0.08m and checks a 0.8m future rear-axle travel horizon.
- It treats measured obstacles inside swept path as blocking, no echo +inf as UNKNOWN (never fabricated 25m distance), and rejects wide/contiguous unknown angular gaps. Sparse no-return allowance is a limited *software commissioning heuristic*, NOT a verified guarantee against dark or narrow obstacles.
- physical_drive_mux.py independently computes the new swept-path result against the most recent /exploration/drive proposal and publishes /control/status.swept_path. AUTO uses it instead of the blanket 90% finite return quota. MANUAL still uses old conservative gate.
- It leaves default enable_real_actuation=false and all existing graph/hold/lap checks intact; no motor commands enabled by this commit.
- CMakeLists installs the helper alongside the mux Python script.

## Single disarmed integration check on Jetson

Stop old integrated launch (Ctrl+C), keep drive power isolated, and execute:

    cd ~/f1tenth_uj
    git pull --ff-only origin zaq/hardware-integration
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select f1tenth_bringup
    source install/local_setup.bash
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    python3 src/f1tenth_bringup/scripts/test_physical_path_guard.py
    python3 src/f1tenth_bringup/scripts/physical_drive_mux.py --self-test
    python3 tools/diagnostics/physical_calibration_precheck.py

    ros2 launch f1tenth_bringup real_integrated.launch.py \
        enable_real_actuation:=false \
        enable_autonomous_actuation:=true \
        enable_joystick:=false

Then from a separate sourced terminal:

    ros2 topic echo /control/status --once --field data
    ros2 topic echo /exploration/drive --once --field drive

Look for swept_path.ready=true and swept_path.reason=SWEEP_PRECONDITION_PASS_NOT_SAFETY_CERTIFIED in an obstacle-free environment. If false, inspect the exact reason and measured/unknown indices. Verify output speed=0 while real disabled. Does NOT require another manual AUTO handshake, already passed. If guard compute mean >=40ms or scan/mux deadlines missed, DO NOT enable real actuation.

## Read-only calibration from current config

    python3 tools/diagnostics/physical_calibration_precheck.py

Current configuration EXPECTED (NOT physically verified):
- Ackermann speed_to_erpm_gain=8414.0; 0.075m/s -> ~631 ERPM; 0.15m/s -> ~1262 ERPM; VESC ROS speed clamps +/-1400 ERPM.
- steering_angle_to_servo_gain=-0.5 and offset=0.361, servo clamp [0.30,0.42]. Software +/-0.1rad road-wheel angle maps to 0.411/0.311 servo command. Kinematic radius at 0.1 rad for 0.260m wheelbase ~2.59m.
- The actual servo centering, steering sign, car wheelbase and ERPM-to-ground-speed relationship all still need physical validation.

## Fast physical commissioning sequence (operator on site)

1. Hardware preparation: secure all 4 wheels clear of floor on a suitable stable stand, isolate propulsion power if electrically possible while inspecting steering, install accessible independent hardware drive cutoff/E-stop, verify disconnect actually de-energizes motor, and keep hands clear of rotating hardware. Do not depend on a ROS hold or a computer SSH session to stop the vehicle.
2. With VESC Tool connected and motor idle, READ AND RECORD configured motor phase current max/min, battery current max/min (including regenerative braking), duty/RPM protection, motor thermal limits, fault history and command timeout + its failsafe behavior. Compare with motor, ESC, battery and mechanical ratings; safe numeric thresholds cannot be inferred merely from ROS YAML. Earlier test measured ~7.93A transient before an attempted 3A software limit; ROS current_max=2 is NOT a speed-mode phase current limit. The firmware settings must be verified/corrected before motion; stop if settings are unverified.
3. Inspect actual steering servo position (wheels off ground) at neutral and small left/right commands using controlled single-sourced VESC Tool servo diagnostics with motor drive isolated; verify steering sign relative to ROS conventions and mechanical endpoint before changing ackermann_to_vesc.yaml gain/offset. A software +/-0.10rad target is not proof of actual wheel angle.
4. After verified independent physical cutoff and safe firmware limits, conduct only an attended, brief wheels-off-ground motor response test; measure motor electrical RPM, wheel direction and commanded speed conversion. Abort on unintended direction, current spike/fault, poor stop, wiring/servo interference. Do not execute uncontrolled direct topic pulses to the VESC.
5. With driver verified and cutoff operator present, perform a *controlled low-speed ground stop test* in a clear enclosed area, observing coasting/stopping distance, drive watchdog behavior after command dropouts, and steering sign/response. Set a speed-dependent stop margin based on these real measurements before autonomous movement.
6. Only then consider enabling live autonomy under strict supervision; map/first-lap and route planning can run already in disarmed mode, but real laps have not been demonstrated.

**Hard NO-GO conditions:** firmware current/regen/timeout settings unknown or unsafe, no physical independent cutoff, steering direction or motor direction unknown, poor measured stopping behavior, a false swept_path result, high CPU latency, or dark-object obstacles missing from laser path. A new software status PASS is not permission to skip any of these.

## Needed feedback to progress quickly

One consolidated output:
- ROS-free path guard tests including guard runtime,
- physical_calibration_precheck output,
- a single /control/status JSON with swept_path details after the new launch.
Separately note whether an independent motor cutoff exists and provide VESC Tool screenshot showing actual firmware current limits/timeout (not stale disconnected screenshot).
