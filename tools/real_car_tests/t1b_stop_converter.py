#!/usr/bin/env python3
"""T1b TEST-ONLY converter: replaces ackermann_to_vesc_node for the stop-method comparison (F-01).

Launched INSTEAD of ackermann_to_vesc (real_car_test.launch.py converter:=t1b), never next to
it, so the mux still sees exactly one /drive subscriber and one /commands/motor/speed publisher.
While /drive asks for speed it does what ackermann_to_vesc does (gain/offset from
config/real_car/vehicle_params.yaml). The only difference is what it sends once the
request is zero or /drive is stale (> 0.2 s):
  speed0    /commands/motor/speed 0              (= ackermann_to_vesc; control run)
  brake     /commands/motor/brake brake_current_a for brake_hold_s, then /commands/motor/current 0
  current0  /commands/motor/current 0            (motor released, coasts)
Brake units are AMPS (vesc_packet.cpp: brake x 1000 -> mA); vesc_driver clips at brake_max
(10.0 in config/real_car/vesc_t1b_erpm4630.yaml). Output at 50 Hz so the VESC never times out
mid-brake. Diagnostics: /t1b_stop_converter/status (JSON). Not an emergency stop.

  t1b_stop_converter.py --self-test
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

METHODS = ('speed0', 'brake', 'current0')
BRAKE_MAX_A = 10.0      # never above the T1b yaml brake_max
HOLD_MAX_S = 5.0
STALE_S = 0.2           # same order as the mux COMMAND_TTL (0.22 s)
RATE_HZ = 50.0


def validate(method, brake_a, hold_s):
    if method not in METHODS:
        raise ValueError('method must be one of %s' % (METHODS,))
    if not (math.isfinite(brake_a) and 0.0 < brake_a <= BRAKE_MAX_A):
        raise ValueError('brake_current_a must be in (0, %.1f] A' % BRAKE_MAX_A)
    if not (math.isfinite(hold_s) and 0.0 < hold_s <= HOLD_MAX_S):
        raise ValueError('brake_hold_s must be in (0, %.1f] s' % HOLD_MAX_S)


class StopLogic:
    """Pure output selection. step() -> ('speed'|'brake'|'current', value, phase)."""

    def __init__(self, method, brake_a, hold_s, erpm_gain, erpm_offset):
        validate(method, brake_a, hold_s)
        self.method, self.brake_a, self.hold_s = method, brake_a, hold_s
        self.gain, self.offset = erpm_gain, erpm_offset
        self.moving = False
        self.t_zero = None

    def step(self, now, speed, drive_age):
        fresh = drive_age is not None and drive_age < STALE_S
        if fresh and math.isfinite(speed) and speed > 1e-6:
            self.moving, self.t_zero = True, None
            return 'speed', self.gain * speed + self.offset, 'DRIVE'
        if self.moving:  # falling edge (zero request or stale /drive)
            self.moving, self.t_zero = False, now
        if self.method == 'speed0':
            return 'speed', 0.0, 'STOP_SPEED0'
        if self.method == 'brake' and self.t_zero is not None and now - self.t_zero < self.hold_s:
            return 'brake', self.brake_a, 'STOP_BRAKE'
        return 'current', 0.0, 'RELEASED'  # brake after hold, current0, or idle before any motion


def self_test():
    p = vp.load()
    g = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    for bad in (('coast', 5, 2), ('brake', 0, 2), ('brake', 10.5, 2), ('brake', 5, 0), ('brake', 5, 6),
                ('brake', float('nan'), 2)):
        try:
            validate(*bad)
        except ValueError:
            continue
        raise AssertionError('accepted %r' % (bad,))
    s = StopLogic('brake', 5.0, 2.0, g, 0.0)
    assert s.step(0.0, 0.0, 0.01) == ('current', 0.0, 'RELEASED')        # idle: no brake at start
    assert s.step(0.1, 0.5, 0.01) == ('speed', 0.5 * g, 'DRIVE')
    assert s.step(1.0, 0.0, 0.01) == ('brake', 5.0, 'STOP_BRAKE')        # falling edge
    assert s.step(2.9, 0.0, 0.01) == ('brake', 5.0, 'STOP_BRAKE')
    assert s.step(3.0, 0.0, 0.01) == ('current', 0.0, 'RELEASED')        # hold over
    assert s.step(3.1, 0.5, 0.01)[0] == 'speed'                          # next repeat
    assert s.step(3.2, 0.5, 0.25) == ('brake', 5.0, 'STOP_BRAKE')        # stale /drive = stop
    assert s.step(3.3, float('nan'), 0.01)[0] == 'brake'                 # non-finite = stop
    c = StopLogic('current0', 5.0, 2.0, g, 0.0)
    c.step(0.0, 0.5, 0.01)
    assert c.step(0.1, 0.0, 0.01) == ('current', 0.0, 'RELEASED')
    z = StopLogic('speed0', 5.0, 2.0, g, 0.0)
    z.step(0.0, 0.5, 0.01)
    assert z.step(0.1, 0.0, 0.01) == ('speed', 0.0, 'STOP_SPEED0')
    assert z.step(0.2, 0.5, None) == ('speed', 0.0, 'STOP_SPEED0')       # never received = stop
    print('PASS: method/limit validation, drive pass-through, brake hold then release,')
    print('      current0, speed0, stale and non-finite /drive stop. No ROS imported.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--self-test', action='store_true')
    args, _ros_args = ap.parse_known_args()
    if args.self_test:
        self_test()
        return

    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Float64, String

    p = vp.load()
    gain = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    offset = float(vp.get(p, 'drivetrain.speed_to_erpm_offset'))
    s_gain = float(vp.get(p, 'steering.servo_gain'))
    s_offset = float(vp.get(p, 'steering.servo_offset'))

    class Converter(Node):
        def __init__(self):
            super().__init__('t1b_stop_converter')
            self.declare_parameter('method', 'speed0')
            self.declare_parameter('brake_current_a', 5.0)
            self.declare_parameter('brake_hold_s', 2.0)
            method = str(self.get_parameter('method').value)
            brake_a = float(self.get_parameter('brake_current_a').value)
            hold_s = float(self.get_parameter('brake_hold_s').value)
            self.logic = StopLogic(method, brake_a, hold_s, gain, offset)  # raises -> node exits
            self.speed_pub = self.create_publisher(Float64, '/commands/motor/speed', 10)
            self.brake_pub = self.create_publisher(Float64, '/commands/motor/brake', 10)
            self.current_pub = self.create_publisher(Float64, '/commands/motor/current', 10)
            self.servo_pub = self.create_publisher(Float64, '/commands/servo/position', 10)
            self.status_pub = self.create_publisher(String, '/t1b_stop_converter/status', 10)
            self.create_subscription(AckermannDriveStamped, '/drive', self.on_drive, 10)
            self.speed, self.steer, self.drive_at = 0.0, 0.0, None
            self.create_timer(1.0 / RATE_HZ, self.tick)
            self.get_logger().warn('T1b TEST CONVERTER: method=%s brake=%.1f A hold=%.1f s. '
                                   'Replaces ackermann_to_vesc. Not an emergency stop.' % (method, brake_a, hold_s))

        def on_drive(self, msg):
            self.speed = float(msg.drive.speed)
            self.steer = float(msg.drive.steering_angle)
            self.drive_at = time.monotonic()

        def tick(self):
            now = time.monotonic()
            age = None if self.drive_at is None else now - self.drive_at
            kind, value, phase = self.logic.step(now, self.speed, age)
            pub = {'speed': self.speed_pub, 'brake': self.brake_pub, 'current': self.current_pub}[kind]
            pub.publish(Float64(data=float(value)))
            steer = self.steer if (age is not None and age < STALE_S and math.isfinite(self.steer)) else 0.0
            servo = s_offset + s_gain * steer
            self.servo_pub.publish(Float64(data=servo))
            self.status_pub.publish(String(data=json.dumps(dict(
                method=self.logic.method, phase=phase, out=kind, value=round(value, 3),
                speed_in=round(self.speed, 4), drive_age_s=None if age is None else round(age, 3),
                servo=round(servo, 4)), separators=(',', ':'))))

    rclpy.init()
    node = Converter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Best effort, like ackermann_to_vesc: release the motor on exit. Not a safe stop.
        try:
            for _ in range(5):
                node.current_pub.publish(Float64(data=0.0))
                time.sleep(0.01)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
