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
Parameter servo_offset (default = vehicle_params steering.servo_offset) is a TEST-ONLY override of
the straight-ahead servo value (T4 20 Nov: --ros-args --params-file
config/real_car/t1b_converter_t4_offset0358.yaml); validated against the baseline and T2 driving clip.
Parameters servo_gain_left / servo_gain_right (default = vehicle_params steering.servo_gain) are TEST-ONLY
per-side steering gains (T3 21 Nov: config/real_car/t1b_converter_t3_t2gain.yaml, T2 per-side fit):
servo = offset + (left gain if steer > 0 else right gain) x steer. Validated in SERVO_GAIN_RANGE. The servo
output is clamped to the T2 driving clip 0.10-0.68 (the per-test VESC yaml clips again, usually tighter).
Optional parameters steer_table_left / steer_table_right (T3 ground, 22 Nov: config/real_car/t1b_converter_t3_table.yaml)
are TEST-ONLY per-side piecewise-linear steering tables, flat [delta_rad, servo, delta_rad, servo, ...] starting
with (0, servo_offset); left angles >= 0 rising, right angles <= 0 falling, servo monotonic outward and inside
the driving clip. When given (both or neither), they replace the per-side gains; the command is clamped to
the table's end angles. Empty (default) = per-side gains, as before.

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
SERVO_OFFSET_MAX_DELTA = 0.05     # test override at most this far from the baseline offset
SERVO_DRIVE_CLIP = (0.10, 0.68)   # T2 19 Nov driving clip (REAL_CAR_TEST_RESULTS.md)
SERVO_GAIN_RANGE = (-1.2, -0.4)  # test gain override limits (baseline -0.50, T2 fit -0.87 / -1.06)
STEER_TABLE_MAX_RAD = 0.40       # table angles never beyond the mux steer ceiling
STALE_S = 0.2           # same order as the mux COMMAND_TTL (0.22 s)
RATE_HZ = 50.0


def validate(method, brake_a, hold_s):
    if method not in METHODS:
        raise ValueError('method must be one of %s' % (METHODS,))
    if not (math.isfinite(brake_a) and 0.0 < brake_a <= BRAKE_MAX_A):
        raise ValueError('brake_current_a must be in (0, %.1f] A' % BRAKE_MAX_A)
    if not (math.isfinite(hold_s) and 0.0 < hold_s <= HOLD_MAX_S):
        raise ValueError('brake_hold_s must be in (0, %.1f] s' % HOLD_MAX_S)


def validate_servo_offset(offset, baseline):
    if not (math.isfinite(offset) and abs(offset - baseline) <= SERVO_OFFSET_MAX_DELTA + 1e-9
            and SERVO_DRIVE_CLIP[0] <= offset <= SERVO_DRIVE_CLIP[1]):
        raise ValueError('servo_offset %r must be within %.2f of the baseline %.3f and inside %.2f..%.2f'
                         % (offset, SERVO_OFFSET_MAX_DELTA, baseline, SERVO_DRIVE_CLIP[0], SERVO_DRIVE_CLIP[1]))


def validate_servo_gain(g):
    if not (math.isfinite(g) and SERVO_GAIN_RANGE[0] <= g <= SERVO_GAIN_RANGE[1]):
        raise ValueError('servo gain %r must be in %.2f..%.2f' % (g, SERVO_GAIN_RANGE[0], SERVO_GAIN_RANGE[1]))


def parse_steer_table(flat, side, offset):
    """Flat [delta, servo, ...] -> [(delta, servo)] for side 'left' (delta >= 0, rising; servo falling) or
    'right' (delta <= 0, falling; servo rising). First pair must be (0, offset). Raises ValueError."""
    vals = [float(v) for v in flat]
    if len(vals) < 4 or len(vals) % 2 or not all(math.isfinite(v) for v in vals):
        raise ValueError('steer_table_%s needs >= 2 finite [delta_rad, servo] pairs, got %r' % (side, flat))
    pts = list(zip(vals[0::2], vals[1::2]))
    if abs(pts[0][0]) > 1e-9 or abs(pts[0][1] - offset) > 1e-6:
        raise ValueError('steer_table_%s must start with (0, servo_offset %.3f), got %r' % (side, offset, pts[0]))
    sgn = 1.0 if side == 'left' else -1.0
    for (d0, s0), (d1, s1) in zip(pts, pts[1:]):
        if not sgn * d1 > sgn * d0:
            raise ValueError('steer_table_%s angles must move outward (%s): %r -> %r' % (side, side, d0, d1))
        if not -sgn * s1 > -sgn * s0:          # left: servo falls; right: servo rises
            raise ValueError('steer_table_%s servo must move outward: %r -> %r' % (side, s0, s1))
    for d, sv in pts:
        if abs(d) > STEER_TABLE_MAX_RAD + 1e-9 or not SERVO_DRIVE_CLIP[0] <= sv <= SERVO_DRIVE_CLIP[1]:
            raise ValueError('steer_table_%s point (%r, %r) outside |delta| <= %.2f / servo %s'
                             % (side, d, sv, STEER_TABLE_MAX_RAD, SERVO_DRIVE_CLIP))
    return pts


def table_servo(table, steer):
    """Piecewise-linear servo for steer on one side's table; steer clamped to the table's end angle."""
    d_end = table[-1][0]
    steer = min(steer, d_end) if d_end > 0 else max(steer, d_end)
    for (d0, s0), (d1, s1) in zip(table, table[1:]):
        if min(d0, d1) <= steer <= max(d0, d1):
            return s0 + (s1 - s0) * (steer - d0) / (d1 - d0)
    return table[-1][1]


def servo_command(offset, gain_left, gain_right, steer, tables=None):
    """Servo position for a steering angle (+ = left): per-side table (tables = (left, right)) or per-side
    linear gain, always clamped to the T2 driving clip."""
    if tables:
        servo = table_servo(tables[0], steer) if steer > 0 else table_servo(tables[1], steer)
    else:
        servo = offset + (gain_left if steer > 0 else gain_right) * steer
    return min(SERVO_DRIVE_CLIP[1], max(SERVO_DRIVE_CLIP[0], servo))


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
    base = float(vp.get(p, 'steering.servo_offset'))
    for ok in (base, 0.358, base + 0.05, base - 0.05):
        validate_servo_offset(ok, base)
    for bad in (base + 0.051, base - 0.051, float('nan'), float('inf')):
        try:
            validate_servo_offset(bad, base)
        except ValueError:
            continue
        raise AssertionError('servo_offset %r accepted' % bad)
    try:
        validate_servo_offset(0.69, 0.66)       # inside the delta but outside the T2 clip
        raise AssertionError('servo_offset outside the T2 clip accepted')
    except ValueError:
        pass
    for gl in (-0.4, -0.874, -1.2):
        validate_servo_gain(gl)
    for bad in (-0.39, -1.21, 0.5, float('nan')):
        try:
            validate_servo_gain(bad)
        except ValueError:
            continue
        raise AssertionError('servo gain %r accepted' % bad)
    sg = float(vp.get(p, 'steering.servo_gain'))
    assert abs(servo_command(base, sg, sg, 0.1) - (base + sg * 0.1)) < 1e-12       # baseline: unchanged
    assert servo_command(base, sg, sg, 0.0) == base
    assert abs(servo_command(0.335, -0.874, -1.062, 0.26) - 0.10776) < 1e-9        # T3 left 0.26 rad
    assert abs(servo_command(0.335, -0.874, -1.062, -0.26) - 0.61112) < 1e-9       # T3 right 0.26 rad
    assert servo_command(0.335, -0.874, -1.062, 0.40) == SERVO_DRIVE_CLIP[0]        # clamped, never past
    assert servo_command(0.335, -0.874, -1.062, -0.40) == SERVO_DRIVE_CLIP[1]
    # T3 ground table (22 Nov): config/real_car/t1b_converter_t3_table.yaml
    import yaml
    tp = yaml.safe_load((vp.CONFIG / 't1b_converter_t3_table.yaml').read_text())['t1b_stop_converter']['ros__parameters']
    tl = parse_steer_table(tp['steer_table_left'], 'left', 0.335)
    tr = parse_steer_table(tp['steer_table_right'], 'right', 0.335)
    tab = (tl, tr)
    assert servo_command(0.335, -0.5, -0.5, 0.0, tab) == 0.335
    assert abs(servo_command(0.335, -0.5, -0.5, 0.2042, tab) - 0.178) < 1e-9          # left 11.7 deg point
    assert abs(servo_command(0.335, -0.5, -0.5, -0.1414, tab) - 0.526) < 1e-9         # right 8.1 deg point
    assert abs(servo_command(0.335, -0.5, -0.5, 0.1021, tab) - (0.335 + (0.178 - 0.335) * 0.5)) < 1e-9  # halfway
    assert abs(servo_command(0.335, -0.5, -0.5, 0.40, tab) - 0.11) < 1e-9             # clamped: left clip 0.11
    assert abs(servo_command(0.335, -0.5, -0.5, -0.40, tab) - tr[-1][1]) < 1e-9        # clamped at -0.28
    assert tl[-1][0] == 0.28 and tr[-1][0] == -0.28
    prev = 1.0
    for k in range(1, 300):                                      # monotonic, continuous across the centre
        sv = servo_command(0.335, -0.5, -0.5, 0.30 - k * 0.002, tab)
        assert sv >= prev - 1e-12 or prev == 1.0, (k, sv, prev)
        prev = sv
    for bad, side in (([0.0, 0.335], 'left'),                                 # one pair only
                      ([0.01, 0.335, 0.2, 0.18], 'left'),                     # does not start at 0
                      ([0.0, 0.330, 0.2, 0.18], 'left'),                      # centre != servo_offset
                      ([0.0, 0.335, 0.2, 0.40], 'left'),                      # left servo must fall
                      ([0.0, 0.335, -0.2, 0.18], 'left'),                     # left angle must be > 0
                      ([0.0, 0.335, 0.2, 0.05], 'left'),                      # below the 0.10 clip
                      ([0.0, 0.335, -0.2, 0.30], 'right'),                    # right servo must rise
                      ([0.0, 0.335, -0.45, 0.60], 'right'),                   # beyond the 0.40 ceiling
                      ([0.0, 0.335, 0.2], 'left'), ([0.0, 0.335, float('nan'), 0.2], 'left')):
        try:
            parse_steer_table(bad, side, 0.335)
        except ValueError:
            continue
        raise AssertionError('steer table %r (%s) accepted' % (bad, side))
    assert servo_command(0.335, -0.874, -1.062, 0.26, None) == servo_command(0.335, -0.874, -1.062, 0.26)  # no table
    print('PASS: method/limit validation, drive pass-through, brake hold then release,')
    print('      current0, speed0, stale and non-finite /drive stop, servo_offset override limits,')
    print('      per-side servo gains (limits, T3 values, baseline unchanged), output clamp 0.10-0.68;')
    print('      steering table (T3 yaml points, interpolation, clamp to +/-0.28, monotonic, 10 bad tables refused).')
    print('      No ROS imported.')


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
    from rclpy.parameter import Parameter
    from rclpy.exceptions import ParameterUninitializedException

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
            self.declare_parameter('servo_offset', s_offset)
            self.declare_parameter('servo_gain_left', s_gain)
            self.declare_parameter('servo_gain_right', s_gain)
            self.declare_parameter('steer_table_left', Parameter.Type.DOUBLE_ARRAY)    # unset = per-side gains
            self.declare_parameter('steer_table_right', Parameter.Type.DOUBLE_ARRAY)
            method = str(self.get_parameter('method').value)
            brake_a = float(self.get_parameter('brake_current_a').value)
            hold_s = float(self.get_parameter('brake_hold_s').value)
            self.logic = StopLogic(method, brake_a, hold_s, gain, offset)  # raises -> node exits
            self.servo_offset = float(self.get_parameter('servo_offset').value)
            validate_servo_offset(self.servo_offset, s_offset)              # raises -> node exits
            if abs(self.servo_offset - s_offset) > 1e-9:
                self.get_logger().warn('TEST servo_offset override %.3f (baseline %.3f in vehicle_params.yaml)'
                                       % (self.servo_offset, s_offset))
            self.gain_left = float(self.get_parameter('servo_gain_left').value)
            self.gain_right = float(self.get_parameter('servo_gain_right').value)
            for g in (self.gain_left, self.gain_right):
                validate_servo_gain(g)                                       # raises -> node exits
            if abs(self.gain_left - s_gain) > 1e-9 or abs(self.gain_right - s_gain) > 1e-9:
                self.get_logger().warn('TEST servo gains left %.3f / right %.3f (baseline %.3f in vehicle_params.yaml)'
                                       % (self.gain_left, self.gain_right, s_gain))
            raw = {}
            for side in ('left', 'right'):
                try:
                    raw[side] = list(self.get_parameter('steer_table_%s' % side).value or [])
                except ParameterUninitializedException:
                    raw[side] = []
            if bool(raw['left']) != bool(raw['right']):
                raise ValueError('steer_table_left and steer_table_right must be given together')
            self.tables = None
            if raw['left']:
                self.tables = (parse_steer_table(raw['left'], 'left', self.servo_offset),     # raises -> node exits
                               parse_steer_table(raw['right'], 'right', self.servo_offset))
                self.get_logger().warn('TEST steering TABLE (gains unused): left %s | right %s' % (
                    self.tables[0], self.tables[1]))
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
            servo = servo_command(self.servo_offset, self.gain_left, self.gain_right, steer, self.tables)
            self.servo_pub.publish(Float64(data=servo))
            self.status_pub.publish(String(data=json.dumps(dict(
                method=self.logic.method, phase=phase, out=kind, value=round(value, 3),
                speed_in=round(self.speed, 4), drive_age_s=None if age is None else round(age, 3),
                servo=round(servo, 4), servo_offset=self.servo_offset, servo_gain_left=self.gain_left,
                servo_gain_right=self.gain_right, steer_mode='table' if self.tables else 'gain',
                steer_limits=[self.tables[1][-1][0], self.tables[0][-1][0]] if self.tables else None),
                separators=(',', ':'))))

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
