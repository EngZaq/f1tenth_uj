#!/usr/bin/env python3
"""Supervised, time-bounded, wheels-OFF-GROUND VESC motor commissioning pulse.

DEFAULT: no ROS publisher or motor command. --self-test uses no ROS.
--run is OPT-IN, requires a typed confirmation and VESC firmware current /
timeout/stop preflight. NEVER use on the floor or for autonomous driving.
Uses a dedicated bench-only std_msgs/Float64 ERPM topic, NOT /drive.
A ROS publisher is NOT a safety interlock and cannot replace a physical E-stop.
"""

import argparse
import math
import sys
import time

BENCH_TOPIC = '/bench_test/motor_erpm'
ERPM = 1000.0  # above confirmed firmware minimum of 900 ERPM
PULSE_SECONDS = 0.50
STOP_SECONDS = 2.0
ABS_MOTOR_CURRENT_ABORT_A = 3.0
ABS_SPEED_ABORT_ERPM = 1500.0
MIN_INPUT_VOLTAGE = 10.0
STALE_TELEMETRY_S = 0.30


def self_test():
    assert BENCH_TOPIC != '/drive'
    assert BENCH_TOPIC != '/commands/motor/speed'
    assert BENCH_TOPIC != '/bench_unused/motor_speed'
    assert 900 < ERPM <= 1100
    assert 0 < PULSE_SECONDS <= .5
    assert STOP_SECONDS >= 2.
    print('PASS: fixed ERPM and duration bounds, isolated bench topic')
    print('No ROS imports/publishers or motor commands were made.')


def run():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64
    from vesc_msgs.msg import VescStateStamped
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterType

    class Bench(Node):
        def __init__(self):
            super().__init__('f1tenth_single_motor_bench_pulse')
            self.pub = self.create_publisher(Float64, BENCH_TOPIC, 10)
            self.last_telem = None
            self.last_recv = None
            self.max_erpm = 0.0
            self.max_motor_current = 0.0
            self.sent_pulse_count = 0
            self.abort = None
            self.sub = self.create_subscription(
                VescStateStamped, '/sensors/core', self.telem_cb, 10
            )

        def verify_driver_speed_limits(self):
            """Query the ACTIVE node, not the on-disk YAML file. Fail closed."""
            client = self.create_client(GetParameters, '/vesc_driver/get_parameters')
            if not client.wait_for_service(timeout_sec=2.0):
                raise RuntimeError('VESC parameter service unavailable: cannot verify active limits')
            req = GetParameters.Request()
            req.names = ['speed_min', 'speed_max']
            future = client.call_async(req)
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if not future.done() or future.exception() is not None:
                raise RuntimeError('Unable to read active /vesc_driver speed bounds')
            values = future.result().values
            if len(values) != 2 or any(v.type != ParameterType.PARAMETER_DOUBLE for v in values):
                raise RuntimeError('Invalid driver speed parameter types')
            lo, hi = values[0].double_value, values[1].double_value
            print('ACTIVE /vesc_driver speed limits: [%.1f, %.1f] ERPM' % (lo, hi))
            if (not all(map(math.isfinite, (lo, hi)))
                    or lo > -ERPM or hi < ERPM or hi > 1100.0):
                raise RuntimeError(
                    'ACTIVE SPEED LIMIT MISCONFIGURATION: %.1f ERPM pulse would be '
                    'clipped or bounds exceed dedicated bench config. '
                    'Stop the old driver, restart using vesc_motor_bench.yaml, '
                    'and verify /vesc_driver speed_max = 1100.0' % ERPM)
            return lo, hi

        def telem_cb(self, msg):
            s = msg.state
            self.last_telem = s
            self.last_recv = time.monotonic()
            if math.isfinite(s.speed):
                self.max_erpm = max(self.max_erpm, abs(s.speed))
            if math.isfinite(s.current_motor):
                self.max_motor_current = max(
                    self.max_motor_current, abs(s.current_motor)
                )
            if s.fault_code != 0:
                self.abort = 'VESC FAULT code=%s' % s.fault_code
            elif not (math.isfinite(s.voltage_input)
                      and s.voltage_input >= MIN_INPUT_VOLTAGE):
                self.abort = 'low/invalid input voltage'
            elif not (math.isfinite(s.current_motor) and
                      abs(s.current_motor) <= ABS_MOTOR_CURRENT_ABORT_A):
                self.abort = 'high/invalid motor current'
            elif not (math.isfinite(s.speed) and
                      abs(s.speed) <= ABS_SPEED_ABORT_ERPM):
                self.abort = 'high/invalid ERPM'

        def publish(self, speed):
            self.pub.publish(Float64(data=float(speed)))

        def telem_is_fresh(self):
            return (self.last_recv is not None and
                    0 <= time.monotonic()-self.last_recv < STALE_TELEMETRY_S)

        def poll(self, seconds, speed):
            stop_time = time.monotonic() + seconds
            while time.monotonic() < stop_time:
                rclpy.spin_once(self, timeout_sec=.01)
                if speed != 0.0 and self.abort:
                    print('ABORT:', self.abort, flush=True)
                    break
                if speed != 0.0 and not self.telem_is_fresh():
                    self.abort = 'telemetry timed out during pulse'
                    print('ABORT:', self.abort, flush=True)
                    break
                self.publish(speed)
                if speed:
                    self.sent_pulse_count += 1
                time.sleep(.03)
            self.publish(0.0)

    rclpy.init()
    node = Bench()
    try:
        print('Preflight telemetry and subscriber check; publishing STOP only...')
        node.poll(.75, 0.0)
        endpoints = node.pub.get_subscription_count()
        print('Bench motor command subscribers:', endpoints)
        if endpoints != 1:
            raise RuntimeError('Need exactly one dedicated bench VESC subscriber')
        node.verify_driver_speed_limits()
        if node.abort or not node.telem_is_fresh():
            raise RuntimeError('VESC telemetry fault or stale preflight')
        st = node.last_telem
        print('VESC before pulse: voltage=%.2fV motor_current=%.2fA ERPM=%.1f fault=%s' %
              (st.voltage_input, st.current_motor, st.speed, st.fault_code))
        if abs(st.speed) > 100.0:
            raise RuntimeError('motor not near stationary; refuse test')
        print('DANGER: firmware motor/battery/regen current limits must already be reduced to verified bench-safe values.\nThe ROS current_max setting does NOT constrain speed-mode motor current.')
        print('Car MUST be firmly secured on a wheel-off-ground stand; no people near drivetrain.')
        print('Independent accessible motor power cutout/stop and spotter REQUIRED.')
        print('Will send fixed %.0f ERPM for %.2f seconds, then publish zero for %.1fs.' %
              (ERPM, PULSE_SECONDS, STOP_SECONDS))
        user = input('Type "BENCH WHEELS UP LIMITS VERIFIED" to execute: ')
        if user.strip() != 'BENCH WHEELS UP LIMITS VERIFIED':
            print('Not confirmed. No motion requested.')
            return
        # Recheck immediately after user delay.
        node.poll(.15, 0.0)
        if node.abort or not node.telem_is_fresh() or node.pub.get_subscription_count() != 1:
            raise RuntimeError('preflight became invalid before pulse')
        node.verify_driver_speed_limits()
        if node.abort or not node.telem_is_fresh():
            raise RuntimeError('telemetry became invalid during active parameter check')
        print('EXECUTING SHORT MOTOR PULSE...')
        node.poll(PULSE_SECONDS, ERPM)
        node.poll(STOP_SECONDS, 0.0)
        print('Pulse publications:', node.sent_pulse_count)
        print('Max observed |ERPM|:', node.max_erpm)
        print('Max observed |motor A|:', node.max_motor_current)
        print('Fault:', node.abort or 'none reported')
        if node.max_erpm < 50 and node.max_motor_current < 0.1:
            print('RESULT: MOTOR DID NOT RESPOND TO SPEED COMMAND.')
            print('DIAGNOSIS: check VESC Tool > Motor Settings > PID Controllers >')
            print('Speed Controller > Minimum ERPM (s_pid_min_erpm).')
            print('Firmware speed control may release the motor when the')
            print('requested |ERPM| is below that parameter. Do not blindly')
            print('increase the test speed or change current limits.')
        elif node.abort:
            print('RESULT: PULSE ABORTED ON TELEMETRY FAULT.')
        else:
            print('RESULT: Measurable motor response observed; movement direction')
            print('and safe motor stop still require independent verification.')
        print('Motor pulse sequence ended. Remain raised and monitor until zero motion.')
    except KeyboardInterrupt:
        print('Interrupted: publishing STOP...', flush=True)
    finally:
        # Best-effort stop on graceful exit, NOT a substitute for independent cutoff.
        try:
            node.poll(STOP_SECONDS, 0.0)
        except Exception as exc:
            print('Could not continue ROS stop publication:', exc)
        node.destroy_node()
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test', action='store_true')
    p.add_argument('--run', action='store_true',
                   help='Explicitly request real motor pulse on raised test stand')
    args = p.parse_args()
    if args.self_test:
        self_test()
    elif args.run:
        run()
    else:
        print(__doc__)
        print('Dry-run default: NO motor commands. Use --self-test first.')
        print('Use --run ONLY after controller firmware and hardware stop preflight.')


if __name__ == '__main__':
    main()
