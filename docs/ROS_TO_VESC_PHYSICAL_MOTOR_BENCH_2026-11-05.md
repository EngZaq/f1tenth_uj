# F1TENTH UJ — Original RC command pipeline and controlled bench motor commissioning

## Original pipeline as implemented in this physical repository

`src/f1tenth_bringup/launch/teleop.launch.py` starts `joy_node`, `joy_teleop`, `ackermann_to_vesc_node`, the `vesc_driver_node` and `vesc_to_odom_node`.

`config/joy_teleop.yaml`: joystick deadman button index **5**; steering axis **2** scaled 0.62 radians; speed axis **1** scaled 1.18 m/s; output `/drive` as `ackermann_msgs/msg/AckermannDriveStamped`.

`config/ackermann_to_vesc.yaml`: `speed_to_erpm_gain = 8414`, `speed_to_erpm_offset=0`, `steering_angle_to_servo_gain=-0.5`, `steering_angle_to_servo_offset=0.361`.

The `ackermann_to_vesc_node` consumes `/drive` and generates VESC motor ERPM and servo position topics. VESC driver then forwards commands to hardware. Hardware user confirms this pipeline previously powered the motor with a remote connected to Jetson.

The most recent *isolated steering bench launch* had intentionally remapped motor speed input to `/bench_unused/motor_speed` and only published `/bench/servo_position`. No motor motion was expected. Actual physical servo movement and VESC telemetry **are confirmed**.

Recent real stationary telemetry: 11.9V input, 0A motor/input current, speed 0 ERPM, VESC fault_code 0, controller_id 13. Motor speed bench-topic subscriber confirmed.

**Important:** the existing `config/vesc.yaml` contains broad provisional ROS-side limits such as `current_max=100.0`, `speed_max=10000.0`. It must not be mistaken for VESC firmware current limiting, or treated as ground-drive readiness. The user previously had VESC motor current ~42.85A and battery current ~99A configured with a 1s timeout and 0A timeout-brake setting, **still unverified**. Do not use `teleop.launch.py` on the ground until this is resolved.

## Dedicated lower-bound bench software

- `src/f1tenth_bringup/config/vesc_motor_bench.yaml`: isolated ROS driver-side ERPM bounds ±600; direct current commands ±2A; other motor command modes disabled in configuration. **This does not change firmware-set current limits for speed control.**
- `tools/commissioning/vesc_motor_bench_pulse.py`: default dry-run, `--self-test` no ROS; real test requires `--run` and typed interactive confirmation. Publishes only to `/bench_test/motor_erpm` for a fixed **500 ERPM / 0.5 second** pulse and repeated zero speed (2 seconds after). Rejects missing bench subscriber, missing/stale VESC telemetry, VESC fault, abnormally high current/ERPM, low battery and moving wheels before arming. ROS communication itself cannot guarantee motor stop after a host crash; the script is *not* an E-stop.

### Hardware prerequisites (HOLD if any absent)

1. Car fully supported on rigid stand, driven wheels freely rotating and fixed against movement; spotter and no people or loose objects close to drivetrain.
2. Independent readily accessible and suitably rated stop, not relying on Ctrl+C, power unplug under motor load or Jetson. Battery unplugging can arc; a convenient battery connector is not an independent validated stop.
3. Inspect VESC Tool **Motor Settings → General → Current** for real motor and battery limits, and **App Settings/General → Timeout** values; set according to measured motor, ESC, battery and wiring ratings, **verify readback**. ROS YAML `current_max` is insufficient.
4. Before energizing verify motor rotation direction and first-stop procedure; only one VESC driver; do not launch joy, teleop, Ackermann converter or other motor publisher concurrently.
5. First run should remain a short wheel-off-ground test, **not a ground motion**. Do not use ROS topic-pub --once as a deadman.

### ROS commands after hardware prerequisites (two terminals)

Terminal 1 (driver):
```bash
cd ~/f1tenth_uj
git pull --ff-only origin zaq/hardware-integration
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run vesc_driver vesc_driver_node --ros-args \
  -r __node:=vesc_driver \
  --params-file "$HOME/f1tenth_uj/src/f1tenth_bringup/config/vesc_motor_bench.yaml" \
  -r commands/motor/speed:=/bench_test/motor_erpm \
  -r commands/motor/current:=/bench_unused/motor_current \
  -r commands/motor/brake:=/bench_unused/motor_brake \
  -r commands/motor/duty_cycle:=/bench_unused/motor_duty \
  -r commands/motor/position:=/bench_unused/motor_position \
  -r commands/servo/position:=/bench_unused/servo
```

Terminal 2 (verify only):
```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_uj/install/local_setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 node list
ros2 param get /vesc_driver speed_max
ros2 param get /vesc_driver current_max
ros2 topic info /bench_test/motor_erpm -v
ros2 topic echo /sensors/core --once
python3 ~/f1tenth_uj/tools/commissioning/vesc_motor_bench_pulse.py --self-test
```

Then, **only if all hardware prerequisites have been separately checked**, run the on-stand actual test:
```bash
python3 ~/f1tenth_uj/tools/commissioning/vesc_motor_bench_pulse.py --run
```
The on-stand pulse test is not yet run/validated on Jetson. If test results diverge, do not repeat at higher speed—inspect controller mode, wiring, recorded telemetry, and ROS graph first.

### Status

Steering servo moved. VESC telemetry working. **Motor actuation via ROS not yet demonstrated in this current commissioning session**, despite user-confirmed prior RC operation through Jetson.
