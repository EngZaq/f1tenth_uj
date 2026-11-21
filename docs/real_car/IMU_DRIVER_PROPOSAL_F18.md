# F-18 proposal: bmi160 driver changes (for Zaid, NOT applied)

`src/hardware/imu_driver/f1tenth_imu/bmi160_node.py` is Zaid's package. Nothing below is changed in it;
the test tools (runner, preflight) only detect the problems. Moatasem, 2026-11-21.

## Evidence

| Case | Session | What happened |
|---|---|---|
| A: freeze after I2C timeout | `20261120_200901`, `20261120_211256` | One `I2C read failed: [Errno 110]`, then one sample repeated at 100 Hz until restart (results doc, F-18). |
| B: node exits at start-up | launch `2026-11-20-21-35-28` (bmi160 pid 9510, ROS log `python3_9510_*.log`), runner folder `20261120_213603` | Gyro calibration std **X 3.04 / Y 1.67 / Z 1.43 deg/s** > `gyro_motion_std_limit_dps` 0.5 (car touched during the 5 s window; healthy launches: 0.04-0.10). `RuntimeError` in `__init__` -> node exits 8.6 s after start. Runner: `IMU_STALE(no /imu/data_raw for > 0.3 s)`, 4 x `ARM_REFUSED`. Relaunch at 21:37: std 0.05, healthy. |

Case B was hard to see: the exception is raised inside `BMI160Node.__init__`, so in `main()` `node` is
still `None`; the message goes to a bare `print()` (not the ROS log file), the exception is swallowed and the
process exits **0**. Launch prints `process has finished cleanly`, the rest of the stack keeps running.

## Proposed changes

1. **Retry the gyro calibration instead of exiting** (case B). In `__init__`, around `calibrate_gyro()`:
   ```python
   attempt = 0
   while True:
       attempt += 1
       bias, std = self.calibrate_gyro()          # return std too; no raise inside
       if max(std) <= self.gyro_motion_limit:
           break
       self.get_logger().warn(
           f'Gyro calibration attempt {attempt}: MOTION (std X={std[0]:.2f} Y={std[1]:.2f} '
           f'Z={std[2]:.2f} deg/s > {self.gyro_motion_limit:.2f}). Keep the car still; retrying in 1 s.')
       if 0 < self.max_attempts <= attempt:      # new param gyro_calibration_max_attempts, 0 = forever
           raise RuntimeError('Gyro calibration failed: car not still')
       time.sleep(1.0)
   self.gyro_bias = bias
   ```
   Same 5 s window and 0.5 deg/s limit (unchanged); only the reaction changes. Each attempt is 5 s.
2. **Fail loudly when it does exit**: in `main()`, log through `rclpy.logging.get_logger('bmi160')` when
   `node is None`, and `sys.exit(1)` after any exception, so launch shows `process has died ... exit code 1`.
3. **Recover from the I2C freeze** (case A, already in the handoff): after an `OSError`, redo the soft reset
   + `configure_sensor()` keeping the gyro bias; a watchdog that does the same after N identical samples.
4. Optional: create the publisher **before** the calibration (publish nothing until calibrated), so tools can
   tell "calibrating" (1 publisher, no samples) from "not running" (0 publishers).

## Guard already in the test tools (21 Nov, our files)

`run_test.py` `ImuWatch.status(now, publishers)` and `preflight.sh stack`: 0 publishers on `/imu/data_raw` ->
`IMU_STALE(IMU node not running: restart the stack and keep the car still for 10 s)` (10 s = 2 s warm-up +
5 s calibration + margin); arm refused as before. If change 1 is applied without change 4, the same message
also appears while the node is retrying: then keep the car still and wait instead of restarting.
