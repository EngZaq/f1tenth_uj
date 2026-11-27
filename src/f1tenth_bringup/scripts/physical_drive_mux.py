#!/usr/bin/env python3
"""F1TENTH physical vehicle command mux (ROS 2).

This program CAN publish to the REAL /drive actuator chain. It is not an E-stop,
not a replacement for independent hardware motor cutoff or VESC firmware limits.

Start DISARMED. MANUAL uses joystick button 5. AUTO uses a separately
supervised /control/auto_hold Bool heartbeat (no RC remote required); loss of
hold latches STOP. A ROS hold is NOT a physical motor cutoff or emergency stop.
AUTO remains separately disabled unless explicitly configured at launch.

Only this node should publish to /drive. Other publishers to /drive and the VESC
motor topic are detected and inhibit *this* node, but cannot be physically
prevented without an external safety-rated command arbiter.
"""
import argparse
import json
import math
import time
from physical_path_guard import swept_path_guard
from physical_stop_evidence import make_snapshot, save_snapshot

MAX_SPEED = 0.35
MAX_STEER = 0.10
# Per-test caps (parameters max_speed_mps / max_steer_rad) can never exceed these.
MAX_SPEED_CEILING = 1.5
MAX_STEER_CEILING = 0.40
COMMAND_TTL = 0.22
JOY_TTL = 0.20
SCAN_TTL = 0.22
ODOM_TTL = 0.30
MAP_TTL = 8.0
AUTO_HOLD_TTL = 0.20  # supervised input, NOT a safety-rated physical switch
MANUAL_LOW_VISIBILITY_FLOOR = 0.10  # manual_allow_reduced_visibility only (was 0.35); standard MANUAL stays 0.75
# safety_mode (start-up parameter; approved by Moatasem 28 Nov for PS4 manual driving by line of sight):
#   'autonomy' (DEFAULT): every gate exactly as before.
#   'teleop': the MANUAL front-visibility gate and the 0.80 m front-obstacle gate do NOT stop or latch; they are
#     still computed and published as WARN fields (teleop_warn*). Kept: arming handshake, /drive and
#     /commands/motor/speed ownership check, speed / steering caps, /joy deadman + JOY_TTL, plus a mux-side PS4
#     check (stale-input stop): /ps4/joy arrival and header stamp younger than TELEOP_JOY_TTL, raw R1 held,
#     exactly one /ps4/joy publisher. Arms only on the PS4 runner's TELEOP_MANUAL_REQUEST (a plain MANUAL, e.g.
#     the keyboard runner, is refused); AUTO is refused; refuses to start with enable_autonomous_actuation.
SAFETY_MODES = ('autonomy', 'teleop')
DEFAULT_SAFETY_MODE = 'autonomy'
TELEOP_MANUAL_REQUEST = 'MANUAL_PS4'
TELEOP_JOY_TTL = 0.25


def evaluate_request(mode, allow_real, allow_auto, deadman, deadman_fresh,
                     cmd, cmd_fresh, scan_ok, odom_fresh, map_fresh,
                     max_speed=MAX_SPEED, max_steer=MAX_STEER):
    """Pure deterministic gate; returns speed, angle, explanation."""
    if not allow_real:
        return 0.0, 0.0, 'REAL_ACTUATION_DISABLED'
    if mode not in ('MANUAL_ACTIVE', 'AUTO_ACTIVE'):
        return 0.0, 0.0, 'DISARMED_OR_READY'
    if mode == 'AUTO_ACTIVE' and not allow_auto:
        return 0.0, 0.0, 'AUTONOMY_LOCKED'
    if not deadman or not deadman_fresh:
        return 0.0, 0.0, 'JOYSTICK_DEADMAN_RELEASED_OR_STALE'
    if cmd is None or not cmd_fresh:
        return 0.0, 0.0, 'DRIVE_REQUEST_STALE'
    speed, steer = cmd
    if not (math.isfinite(speed) and math.isfinite(steer)):
        return 0.0, 0.0, 'NONFINITE_DRIVE_REQUEST'
    if speed < -1e-6:  # no reverse in first physical implementation
        return 0.0, 0.0, 'REVERSE_UNCALIBRATED'
    if not scan_ok:
        return 0.0, 0.0, 'FRONT_SCAN_NOT_CLEAR'
    if mode == 'AUTO_ACTIVE' and (not odom_fresh or not map_fresh):
        return 0.0, 0.0, 'AUTO_LOCALIZATION_OR_MAP_STALE'
    if speed <= 1e-5:
        return 0.0, 0.0, 'ZERO_REQUEST'
    # Clamp all requests to the commissioning speed and steering range.
    return min(speed, max_speed), max(-max_steer, min(max_steer, steer)), 'PASS'


def validate_caps(max_speed, max_steer):
    """Per-test caps: finite, > 0 and never above the hard ceilings."""
    if not (math.isfinite(max_speed) and 0. < max_speed <= MAX_SPEED_CEILING):
        raise ValueError('max_speed_mps must be in (0, %.2f]' % MAX_SPEED_CEILING)
    if not (math.isfinite(max_steer) and 0. < max_steer <= MAX_STEER_CEILING):
        raise ValueError('max_steer_rad must be in (0, %.2f]' % MAX_STEER_CEILING)
    return max_speed, max_steer


def front_evidence(scan, min_fraction=0.75, stop_m=0.80):
    """Finite-only front measurements; no-return never means empty space."""
    if scan is None or len(scan.ranges) < 100:
        return False, 'NO_SCAN', 0.0, None
    inc = float(scan.angle_increment)
    if not math.isfinite(inc) or inc <= 0:
        return False, 'INVALID_SCAN_GEOMETRY', 0.0, None
    samples = []
    for i, raw in enumerate(scan.ranges):
        if abs(float(scan.angle_min) + i * inc) <= math.radians(20.0):
            samples.append(float(raw))
    if len(samples) < 40:
        return False, 'NO_FRONT_SECTOR', 0.0, None
    finite = [
        d for d in samples if (
            math.isfinite(d) and
            float(scan.range_min) < d <= float(scan.range_max)
        )
    ]
    fraction = len(finite)/len(samples)
    nearest = min(finite) if finite else None
    if fraction < min_fraction:
        return False, 'FRONT_UNKNOWN', fraction, nearest
    if nearest is None or nearest < stop_m:
        return False, 'OBSTACLE_NEAR', fraction, nearest
    return True, 'FRONT_MEASURED', fraction, nearest


def validate_safety_mode(mode, allow_auto):
    """Start-up only. 'autonomy' (default, every gate) or 'teleop' (PS4 manual, see SAFETY_MODES above)."""
    m = str(mode).strip().lower()
    if m not in SAFETY_MODES:
        raise ValueError('safety_mode must be one of %s, got %r' % (SAFETY_MODES, mode))
    if m == 'teleop' and allow_auto:
        raise ValueError('safety_mode teleop refuses enable_autonomous_actuation=true')
    return m


def manual_scan_gate(safety_mode, scan_ok):
    """MANUAL front gate fed to evaluate_request: autonomy = the LiDAR verdict (unchanged); teleop = never a stop
    (the verdict is published as a warning by front_warnings instead)."""
    return True if safety_mode == 'teleop' else scan_ok


def front_warnings(scan_reason, fraction, nearest, min_fraction, stop_m=.80):
    """teleop: both MANUAL front gates evaluated independently, as warnings only."""
    unknown = (scan_reason in ('NO_SCAN', 'INVALID_SCAN_GEOMETRY', 'NO_FRONT_SECTOR')
               or fraction < min_fraction)
    near = nearest is not None and nearest < stop_m
    warn = '+'.join(w for w, on in (('FRONT_UNKNOWN', unknown), ('OBSTACLE_NEAR', near)) if on)
    return {'front_unknown': unknown, 'obstacle_near': near, 'warn': warn or None}


def ps4_joy_problem(now, arrival, stamp_age, buttons, deadman_index, ttl=TELEOP_JOY_TTL):
    """teleop stale-input check on /ps4/joy: None = fresh. stamp_age = ROS now - header.stamp (None = no stamp)."""
    if arrival is None:
        return 'PS4_JOY_MISSING'
    if not 0 <= now - arrival < ttl:
        return 'PS4_JOY_STALE(%.2f s)' % (now - arrival)
    if stamp_age is None:
        return 'PS4_JOY_NO_STAMP'
    if not -ttl < stamp_age < ttl:
        return 'PS4_JOY_STAMP_OLD(%.2f s)' % stamp_age
    if buttons is None or len(buttons) <= deadman_index:
        return 'PS4_JOY_TOO_FEW_BUTTONS'
    return None


def teleop_mode_request(m, safety_mode, ps4_problem, ps4_deadman_held, ps4_publishers):
    """Mode request pre-check -> (mode to process, rejection or None). autonomy: no change at all
    (TELEOP_MANUAL_REQUEST then falls through to REJECT_INVALID_MODE). teleop: only the PS4 runner's request arms."""
    if safety_mode != 'teleop' or m == 'STOP':
        return m, None
    if m == 'AUTO':
        return m, 'REJECT_AUTO_IN_TELEOP_SAFETY_MODE'
    if m == 'MANUAL':
        return m, 'REJECT_TELEOP_SAFETY_MODE_NEEDS_PS4_RUNNER'
    if m != TELEOP_MANUAL_REQUEST:
        return m, None
    if ps4_publishers != 1:
        return m, 'REJECT_PS4_JOY_PUBLISHERS(%s)' % ps4_publishers
    if ps4_problem:
        return m, 'REJECT_' + ps4_problem
    if ps4_deadman_held:
        return m, 'REJECT_PS4_DEADMAN_HELD'
    return 'MANUAL', None


def self_test_safety_mode():
    assert DEFAULT_SAFETY_MODE == 'autonomy'
    assert validate_safety_mode(DEFAULT_SAFETY_MODE, False) == 'autonomy'
    assert validate_safety_mode(' Teleop ', False) == 'teleop'
    for bad, auto in (('manual', False), ('', False), ('teleop', True)):
        try:
            validate_safety_mode(bad, auto)
        except ValueError:
            continue
        raise AssertionError('safety_mode %r (auto %s) accepted' % (bad, auto))

    class Scan:
        angle_min = -math.pi
        angle_increment = 2*math.pi/1440
        range_min = 0.0
        range_max = 100.0
        ranges = [2.0]*1440
    corridor, near = Scan(), Scan()
    corridor.ranges = [float('inf')]*1440          # far end out of range: FRONT_UNKNOWN
    near.ranges = [0.5]*1440                         # obstacle at 0.5 m: OBSTACLE_NEAR
    for s, why in ((corridor, 'FRONT_UNKNOWN'), (near, 'OBSTACLE_NEAR')):
        ok, reason, frac, nearest = front_evidence(s, .75, .80)
        assert not ok and reason == why
        # autonomy: unchanged gate -> FRONT_SCAN_NOT_CLEAR, which the tick latches (LATCH_STOP_...)
        r = evaluate_request('MANUAL_ACTIVE', True, False, True, True, (.3, 0), True,
                             manual_scan_gate('autonomy', ok), True, True, max_speed=.3, max_steer=.28)
        assert r == (0.0, 0.0, 'FRONT_SCAN_NOT_CLEAR'), r
        # teleop: no stop, the same verdict as a warning
        r = evaluate_request('MANUAL_ACTIVE', True, False, True, True, (.3, .1), True,
                             manual_scan_gate('teleop', ok), True, True, max_speed=.3, max_steer=.28)
        assert r == (.3, .1, 'PASS'), r
        w = front_warnings(reason, frac, nearest, .75)
        assert w['warn'] == why and w['front_unknown'] == (why == 'FRONT_UNKNOWN'), w
    w = front_warnings('FRONT_UNKNOWN', .17, .55, .75)         # both at once (front_evidence reports only one)
    assert w == {'front_unknown': True, 'obstacle_near': True, 'warn': 'FRONT_UNKNOWN+OBSTACLE_NEAR'}, w
    assert front_warnings('NO_SCAN', 0., None, .75)['front_unknown']
    assert front_warnings('FRONT_MEASURED', .9, 2.0, .75)['warn'] is None
    # teleop keeps the deadman, the command TTL and the caps
    assert evaluate_request('MANUAL_ACTIVE', True, False, False, True, (.3, 0), True,
                            manual_scan_gate('teleop', False), True, True)[2] == 'JOYSTICK_DEADMAN_RELEASED_OR_STALE'
    assert evaluate_request('MANUAL_ACTIVE', True, False, True, True, (.3, 0), False,
                            manual_scan_gate('teleop', False), True, True)[2] == 'DRIVE_REQUEST_STALE'
    assert evaluate_request('MANUAL_ACTIVE', True, False, True, True, (2.0, .5), True,
                            manual_scan_gate('teleop', False), True, True,
                            max_speed=1.5, max_steer=.28) == (1.5, .28, 'PASS')
    # stale-input stop on /ps4/joy: no new message, old stamp, no stamp
    b = [0]*5 + [1] + [0]*7
    assert ps4_joy_problem(10.0, 9.80, 0.01, b, 5) is None
    assert ps4_joy_problem(10.0, None, None, None, 5) == 'PS4_JOY_MISSING'
    assert ps4_joy_problem(10.0, 9.75, 0.01, b, 5).startswith('PS4_JOY_STALE')
    assert ps4_joy_problem(10.0, 9.99, 0.25, b, 5).startswith('PS4_JOY_STAMP_OLD')
    assert ps4_joy_problem(10.0, 9.99, -0.30, b, 5).startswith('PS4_JOY_STAMP_OLD')
    assert ps4_joy_problem(10.0, 9.99, None, b, 5) == 'PS4_JOY_NO_STAMP'
    assert ps4_joy_problem(10.0, 9.99, 0.01, [1]*4, 5) == 'PS4_JOY_TOO_FEW_BUTTONS'
    # arming: keyboard runner (plain MANUAL) refused in teleop; autonomy unchanged
    assert teleop_mode_request('MANUAL', 'teleop', None, False, 1) == (
        'MANUAL', 'REJECT_TELEOP_SAFETY_MODE_NEEDS_PS4_RUNNER')
    assert teleop_mode_request('MANUAL_PS4', 'teleop', None, False, 1) == ('MANUAL', None)
    assert teleop_mode_request('MANUAL_PS4', 'teleop', 'PS4_JOY_STALE(0.30 s)', False, 1)[1].startswith(
        'REJECT_PS4_JOY_STALE')
    assert teleop_mode_request('MANUAL_PS4', 'teleop', None, True, 1)[1] == 'REJECT_PS4_DEADMAN_HELD'
    assert teleop_mode_request('MANUAL_PS4', 'teleop', None, False, 2)[1] == 'REJECT_PS4_JOY_PUBLISHERS(2)'
    assert teleop_mode_request('AUTO', 'teleop', None, False, 1)[1] == 'REJECT_AUTO_IN_TELEOP_SAFETY_MODE'
    assert teleop_mode_request('STOP', 'teleop', 'PS4_JOY_MISSING', True, 0) == ('STOP', None)
    for m in ('MANUAL', 'AUTO', 'STOP', 'MANUAL_PS4'):
        assert teleop_mode_request(m, 'autonomy', 'PS4_JOY_MISSING', True, 0) == (m, None)
    print('PASS: safety_mode default autonomy (bad value / teleop + AUTO refused); autonomy still stops on')
    print('      FRONT_UNKNOWN and OBSTACLE_NEAR; teleop does not, publishes both as warnings; teleop keeps')
    print('      deadman, command TTL, caps; /ps4/joy stale / old stamp / no stamp; keyboard MANUAL refused in')
    print('      teleop, PS4 request accepted only fresh + R1 released + 1 publisher; AUTO refused in teleop.')


def self_test():
    ok = evaluate_request('MANUAL_ACTIVE', True, False,
                          True, True, (.12, -.07), True, True, True, True)
    assert ok == (.12, -.07, 'PASS'), ok
    assert evaluate_request('MANUAL_ACTIVE', False, False,
                            True, True, (.12, 0), True, True, True, True)[0] == 0
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            False, True, (.12, 0), True, True, True, True)[0] == 0
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (.12, 0), False, True, True, True)[0] == 0
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (.12, 0), True, False, True, True)[0] == 0
    assert evaluate_request('AUTO_ACTIVE', True, False,
                            True, True, (.12, 0), True, True, True, True)[0] == 0
    # AUTO may use a fresh supervisor hold rather than a joystick: the pure
    # gate accepts any independently validated held input, not button 5.
    assert evaluate_request('AUTO_ACTIVE', True, True,
                            True, True, (.12, .04), True, True, True, True) == (
                                .12, .04, 'PASS')
    assert evaluate_request('AUTO_ACTIVE', True, True,
                            True, True, (.12, 0), True, True, False, True)[0] == 0
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (.5, .4), True, True, True, True) == (
                                .35, .10, 'PASS')
    # Per-test caps clamp the same way; defaults above are unchanged.
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (2.0, -.5), True, True, True, True,
                            max_speed=1.0, max_steer=.30) == (1.0, -.30, 'PASS')
    assert evaluate_request('MANUAL_ACTIVE', False, False,
                            True, True, (1.0, 0), True, True, True, True,
                            max_speed=1.5, max_steer=.40)[0] == 0
    assert validate_caps(MAX_SPEED, MAX_STEER) == (MAX_SPEED, MAX_STEER)
    for bad in ((1.6, .1), (.35, .41), (0., .1), (.35, 0.), (float('nan'), .1)):
        try:
            validate_caps(*bad)
        except ValueError:
            continue
        raise AssertionError('cap accepted: %r' % (bad,))
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (-.1, 0), True, True, True, True)[0] == 0
    assert evaluate_request('MANUAL_ACTIVE', True, False,
                            True, True, (float('nan'), 0), True, True, True, True)[0] == 0

    class Scan:
        angle_min = -math.pi
        angle_increment = 2*math.pi/1440
        range_min = 0.0
        range_max = 100.0
        ranges = [2.0]*1440

    s = Scan()
    assert front_evidence(s)[0]
    s.ranges = [float('inf')]*1440
    assert not front_evidence(s)[0]
    s.ranges = [0.4]*1440
    assert not front_evidence(s)[0]
    print('PASS: disarm, deadman, command TTL, LiDAR gate, auto lock,')
    print('      missing map/odom, reverse denial, bounded manual request.')
    self_test_safety_mode()
    print('No ROS imported and no real actuator commands emitted.')


def run_ros():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Joy, LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String, Bool

    class PhysicalMux(Node):
        def __init__(self):
            super().__init__('physical_drive_mux')
            self.declare_parameter('enable_real_actuation', False)
            self.declare_parameter('enable_autonomous_actuation', False)
            self.declare_parameter('manual_min_front_valid', .75)
            # Explicit, supervised manual-only low-visibility mode. Does not
            # affect AUTO, 0.80m obstacle stop, actuator ownership or deadman.
            self.declare_parameter('manual_allow_reduced_visibility', False)
            self.declare_parameter('auto_min_front_valid', .90)
            # Per-test caps, set only by the real-car test launch; defaults unchanged.
            self.declare_parameter('max_speed_mps', MAX_SPEED)
            self.declare_parameter('max_steer_rad', MAX_STEER)
            # Start-up only (no parameter callback): which safety set is active. See SAFETY_MODES.
            self.declare_parameter('safety_mode', DEFAULT_SAFETY_MODE)
            self.declare_parameter('teleop_ps4_topic', '/ps4/joy')
            self.declare_parameter('teleop_ps4_deadman_button', 5)
            self.allow_real = bool(self.get_parameter('enable_real_actuation').value)
            self.allow_auto = bool(self.get_parameter('enable_autonomous_actuation').value)
            self.safety_mode = validate_safety_mode(
                self.get_parameter('safety_mode').value, self.allow_auto)
            self.ps4_topic = str(self.get_parameter('teleop_ps4_topic').value)
            self.ps4_deadman = int(self.get_parameter('teleop_ps4_deadman_button').value)
            if self.safety_mode == 'teleop' and (self.ps4_topic in ('/joy', '') or self.ps4_deadman < 0):
                raise ValueError('safety_mode teleop needs teleop_ps4_topic != /joy and a deadman index >= 0')
            self.manual_fraction = float(self.get_parameter('manual_min_front_valid').value)
            self.manual_low_visibility = bool(self.get_parameter('manual_allow_reduced_visibility').value)
            self.auto_fraction = float(self.get_parameter('auto_min_front_valid').value)
            self.max_speed, self.max_steer = validate_caps(
                float(self.get_parameter('max_speed_mps').value),
                float(self.get_parameter('max_steer_rad').value))
            if (self.max_speed, self.max_steer) != (MAX_SPEED, MAX_STEER):
                self.get_logger().warn('TEST CAPS: max_speed_mps=%.2f max_steer_rad=%.2f (defaults %.2f / %.2f)' % (
                    self.max_speed, self.max_steer, MAX_SPEED, MAX_STEER))
            # Reduced-visibility floor 0.10 (was 0.35; approved by Moatasem 23 Nov for corridor teleop at <= 0.3 m/s,
            # where the far end is out of range: front finite fraction 0.175). Only with
            # manual_allow_reduced_visibility:=true; the parameter defaults (0.75, reduced mode off), the standard
            # MANUAL floor 0.75, AUTO 0.90 and the 0.80 m obstacle stop (needs a finite return) are unchanged.
            manual_floor = MANUAL_LOW_VISIBILITY_FLOOR if self.manual_low_visibility else .75
            if not manual_floor <= self.manual_fraction <= 1.0:
                raise ValueError('manual_min_front_valid must be >=%.2f' % manual_floor)
            if self.manual_low_visibility:
                self.get_logger().warn('EXPERIMENTAL MANUAL LINE-OF-SIGHT LOW-VISIBILITY MODE: front finite fraction >= %.2f; independent cutoff required; AUTO unchanged' % self.manual_fraction)
            if not .90 <= self.auto_fraction <= 1.0:
                raise ValueError('auto_min_front_valid must be >=0.90')
            self.mode = 'STOP'
            self.manual_cmd = None
            self.manual_at = None
            self.auto_cmd = None
            self.auto_at = None
            self.joy_at = None
            self.deadman = False
            self.auto_hold = False
            self.auto_hold_at = None
            self.first_lap_completed = False
            self.scan = None
            self.scan_at = None
            self.odom_at = None
            self.map_at = None
            self.last_reason = None
            self.last_stop_reason = 'NOT_YET_ACTIVE'
            self.last_stop_at = None
            self.last_stop_snapshot = None
            self.last_stop_persisted_at = None
            self.last_mode_request = None
            self.last_mode_request_at = None
            self.mode_request_result = 'NO_MODE_REQUEST_RECEIVED'
            self.ps4_at = None
            self.ps4_stamp_ns = None
            self.ps4_buttons = None
            self.last_teleop_warn = None
            self.drive_pub = self.create_publisher(
                AckermannDriveStamped, '/drive', 10)
            self.status_pub = self.create_publisher(
                String, '/control/status', 10)
            self.subs = [
                self.create_subscription(
                    AckermannDriveStamped, '/manual/drive',
                    self.manual_cb, 10),
                self.create_subscription(
                    AckermannDriveStamped, '/exploration/drive',
                    self.auto_cb, 10),
                self.create_subscription(
                    Joy, '/joy', self.joy_cb, qos_profile_sensor_data),
                self.create_subscription(
                    String, '/control/request_mode', self.mode_cb, 10),
                self.create_subscription(
                    Bool, '/control/auto_hold', self.auto_hold_cb, 10),
                self.create_subscription(
                    Bool, '/mission/first_lap_complete',
                    self.lap_cb, QoSProfile(depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)),
                self.create_subscription(
                    LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data),
                self.create_subscription(
                    Odometry, '/odom_rf2o', self.odom_cb, qos_profile_sensor_data),
                self.create_subscription(
                    OccupancyGrid, '/map', self.map_cb,
                    QoSProfile(depth=1,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)),
            ]
            if self.safety_mode == 'teleop':
                # Only in teleop: the raw controller stream, for the mux-side stale-input check.
                self.subs.append(self.create_subscription(
                    Joy, self.ps4_topic, self.ps4_cb, qos_profile_sensor_data))
            self.create_timer(.05, self.tick)
            self.get_logger().warn(
                'REAL /drive connected. Starts STOP. '
                'This ROS mux is NOT a physical emergency stop. '
                'enable_real_actuation=%s enable_autonomous_actuation=%s safety_mode=%s' %
                (self.allow_real, self.allow_auto, self.safety_mode)
            )
            if self.safety_mode == 'teleop':
                self.get_logger().warn(
                    'SAFETY_MODE TELEOP: MANUAL front-visibility and 0.80 m obstacle gates only WARN (no stop). '
                    'Kept: arming, ownership check, caps, /joy deadman, %s fresh < %.2f s (arrival + stamp) with '
                    'R1 (button %d) held and 1 publisher. Arms only on %s; AUTO refused.' % (
                        self.ps4_topic, TELEOP_JOY_TTL, self.ps4_deadman, TELEOP_MANUAL_REQUEST))

        def manual_cb(self, msg):
            self.manual_cmd = (float(msg.drive.speed),
                               float(msg.drive.steering_angle))
            self.manual_at = time.monotonic()

        def auto_cb(self, msg):
            self.auto_cmd = (float(msg.drive.speed),
                             float(msg.drive.steering_angle))
            self.auto_at = time.monotonic()

        def joy_cb(self, msg):
            self.joy_at = time.monotonic()
            self.deadman = len(msg.buttons) > 5 and msg.buttons[5] == 1

        def ps4_cb(self, msg):
            self.ps4_at = time.monotonic()
            stamp = msg.header.stamp
            ns = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
            self.ps4_stamp_ns = ns if ns > 0 else None
            self.ps4_buttons = list(msg.buttons)

        def ps4_state(self, now):
            """teleop only: (problem or None, raw R1 held, publisher count, arrival age, stamp age)."""
            pubs = len(self.get_publishers_info_by_topic(self.ps4_topic))
            stamp_age = (None if self.ps4_stamp_ns is None else
                         (self.get_clock().now().nanoseconds - self.ps4_stamp_ns) / 1e9)
            problem = ps4_joy_problem(now, self.ps4_at, stamp_age, self.ps4_buttons, self.ps4_deadman)
            if problem is None and pubs != 1:
                problem = 'PS4_JOY_PUBLISHERS(%d)' % pubs
            held = problem is None and self.ps4_buttons[self.ps4_deadman] == 1
            age = None if self.ps4_at is None else now - self.ps4_at
            return problem, held, pubs, age, stamp_age

        def auto_hold_cb(self, msg):
            self.auto_hold = bool(msg.data)
            self.auto_hold_at = time.monotonic()

        def lap_cb(self, msg):
            if msg.data:
                self.first_lap_completed = True
                self.mode = 'STOP'
                self.last_stop_reason = 'FIRST_LAP_COMPLETE'
                self.last_stop_at = time.monotonic()
                self.get_logger().warn(
                    'Physical first lap completed: STOP latched for map saving.')

        def scan_cb(self, msg):
            self.scan = msg
            self.scan_at = time.monotonic()

        def odom_cb(self, msg):
            self.odom_at = time.monotonic()

        def map_cb(self, msg):
            if msg.info.width and msg.info.height and msg.data:
                self.map_at = time.monotonic()

        def mode_cb(self, msg):
            m = msg.data.strip().upper()
            self.last_mode_request = m
            self.last_mode_request_at = time.monotonic()
            if self.safety_mode == 'teleop':
                problem, held, pubs, _, _ = self.ps4_state(time.monotonic())
                m, reject = teleop_mode_request(m, self.safety_mode, problem, held, pubs)
                if reject:
                    self.mode_request_result = reject
                    self.get_logger().warn('safety_mode teleop: %s' % reject)
                    return
            if m == 'STOP':
                if self.mode != 'STOP':
                    self.last_stop_reason = 'OPERATOR_STOP_REQUESTED'
                    self.last_stop_at = time.monotonic()
                self.mode = 'STOP'
                self.mode_request_result = 'STOP_ACCEPTED'
                self.get_logger().warn('Operator STOP requested; reselect mode to re-arm')
                return
            if m not in ('MANUAL', 'AUTO'):
                self.mode_request_result = 'REJECT_INVALID_MODE'
                self.get_logger().warn('Invalid mode; use STOP, MANUAL or AUTO')
                return
            if self.first_lap_completed:
                self.mode_request_result = 'REJECT_FIRST_LAP_COMPLETED'
                self.get_logger().warn('Mission complete: restart after map handoff')
                return
            # Never allow selecting while the respective hold is asserted;
            # an explicit received release event is required. A short-lived
            # "ros2 topic pub --once false" may be lost to ROS discovery.
            if m == 'MANUAL' and self.deadman:
                self.mode_request_result = 'REJECT_JOYSTICK_DEADMAN_HELD'
                self.get_logger().warn('Release joystick deadman before MANUAL selection')
                return
            if m == 'AUTO' and self.auto_hold:
                self.mode_request_result = 'REJECT_AUTO_HOLD_HELD'
                self.get_logger().warn('Release /control/auto_hold before AUTO selection')
                return
            if m == 'AUTO' and not self.allow_auto:
                self.mode_request_result = 'REJECT_AUTO_DISABLED_AT_LAUNCH'
                self.get_logger().warn('AUTO is locked at launch; no automatic actuation')
                return
            self.mode = m + '_READY'
            self.manual_cmd = None
            self.auto_cmd = None
            self.manual_at = None
            self.auto_at = None
            self.mode_request_result = m + '_READY_ACCEPTED'
            if m == 'AUTO':
                self.get_logger().warn('AUTO_READY: request accepted; assert fresh auto hold to enter AUTO_ACTIVE')
            else:
                self.get_logger().warn('MANUAL_READY: request accepted; hold joystick button 5 to engage')

        def fresh(self, timestamp, seconds, now):
            return timestamp is not None and 0 <= now-timestamp < seconds

        def tick(self):
            now = time.monotonic()
            mode_before_tick = self.mode
            joy_fresh = self.fresh(self.joy_at, JOY_TTL, now)
            joy_held = joy_fresh and self.deadman
            ps4_problem = ps4_held = ps4_pubs = ps4_age = ps4_stamp_age = None
            if self.safety_mode == 'teleop':
                # Stale-input stop kept in teleop: the /joy deadman AND a fresh /ps4/joy with R1 held.
                ps4_problem, ps4_held, ps4_pubs, ps4_age, ps4_stamp_age = self.ps4_state(now)
                joy_held = joy_held and ps4_held
            auto_hold_fresh = self.fresh(self.auto_hold_at, AUTO_HOLD_TTL, now)
            auto_hold_held = auto_hold_fresh and self.auto_hold
            active_hold = (joy_held if self.mode.startswith('MANUAL')
                           else auto_hold_held)
            # AUTO uses a live supervised ROS hold signal, NOT a joystick.
            if self.mode in ('MANUAL_READY', 'AUTO_READY') and active_hold:
                self.mode = self.mode.replace('READY', 'ACTIVE')
            if self.mode in ('MANUAL_ACTIVE', 'AUTO_ACTIVE') and not active_hold:
                self.last_stop_reason = (
                    'AUTO_HOLD_RELEASED_OR_STALE' if self.mode == 'AUTO_ACTIVE'
                    else 'JOYSTICK_DEADMAN_RELEASED_OR_STALE')
                if ps4_problem and self.mode == 'MANUAL_ACTIVE':
                    self.last_stop_reason += '(TELEOP_%s)' % ps4_problem
                self.last_stop_at = now
                self.get_logger().warn('STOP LATCHED: %s' % self.last_stop_reason)
                self.mode = 'STOP'

            if self.mode.startswith('MANUAL'):
                cmd, received = self.manual_cmd, self.manual_at
                fraction_req = self.manual_fraction
            else:
                cmd, received = self.auto_cmd, self.auto_at
                fraction_req = self.auto_fraction

            scan_fresh = self.fresh(self.scan_at, SCAN_TTL, now)
            scan_ok, scan_reason, fraction, nearest = front_evidence(
                self.scan if scan_fresh else None,
                min_fraction=fraction_req, stop_m=.80)
            # Independently evaluate the requested AUTO trajectory against
            # the full physical car footprint, even while the motor is
            # disarmed, so commissioning gets actionable path evidence.
            # Never infer free distance from +inf.
            if self.auto_cmd is not None and scan_fresh:
                guard_started = time.perf_counter()
                path = swept_path_guard(
                    self.scan, max(0.0, self.auto_cmd[0]), self.auto_cmd[1])
                guard_ms = (time.perf_counter()-guard_started)*1000.
                path['compute_ms'] = round(guard_ms,2)
                if guard_ms >= 40.:
                    # No stale positive clearance on a starved control loop.
                    path['ready'] = False
                    path['reason'] = 'SWEEP_COMPUTE_DEADLINE_EXCEEDED'
            else:
                path = {
                    'ready': False,
                    'reason': 'NO_FRESH_SCAN_OR_AUTO_PROPOSAL',
                    'nearest_obstacle_m': None,
                    'swept_known_fraction': 0.0,
                    'compute_ms': None,
                }
            # MANUAL retains the original conservative front LiDAR gate.
            # AUTO uses the swept-vehicle geometry and bounded unknown-run
            # policy, NOT the blanket 90% of the +/-20deg sector rule.
            # safety_mode teleop: the MANUAL front gates never stop; computed anyway, published as WARN fields.
            request_scan_ok = (
                manual_scan_gate(self.safety_mode, scan_ok) if self.mode.startswith('MANUAL')
                else bool(path['ready']))
            teleop_warn = (front_warnings(scan_reason, fraction, nearest, self.manual_fraction)
                           if self.safety_mode == 'teleop' else None)
            if teleop_warn is not None and teleop_warn['warn'] != self.last_teleop_warn:
                self.last_teleop_warn = teleop_warn['warn']
                if teleop_warn['warn']:
                    self.get_logger().warn('TELEOP WARN (no stop): %s; front valid %.2f (< %.2f warns), nearest %s'
                                           % (teleop_warn['warn'], fraction, self.manual_fraction,
                                              '%.2f m' % nearest if nearest is not None else 'none'))
            odom_fresh = self.fresh(self.odom_at, ODOM_TTL, now)
            map_fresh = self.fresh(self.map_at, MAP_TTL, now)

            speed, steer, reason = evaluate_request(
                self.mode, self.allow_real, self.allow_auto,
                active_hold, (joy_fresh if self.mode.startswith('MANUAL')
                              else auto_hold_fresh), cmd,
                self.fresh(received, COMMAND_TTL, now),
                request_scan_ok, odom_fresh, map_fresh,
                max_speed=self.max_speed, max_steer=self.max_steer)

            if self.first_lap_completed:
                self.mode = 'STOP'
                speed, steer = 0.0, 0.0
                reason = 'FIRST_LAP_COMPLETE'

            # If an active mode loses a required input, latch STOP rather
            # than resuming motion uncommanded when the data reappears.
            if self.mode.endswith('ACTIVE') and reason not in (
                    'PASS', 'ZERO_REQUEST', 'REAL_ACTUATION_DISABLED'):
                self.mode = 'STOP'
                speed, steer = 0.0, 0.0
                reason = 'LATCH_STOP_' + reason
                self.last_stop_reason = reason
                self.last_stop_at = now
                self.get_logger().warn('STOP LATCHED: %s' % reason)

            # Check *both* ends of the real hardware chain. On 2026-11-05
            # three stale Ackermann converter nodes each subscribed to /drive
            # and published /commands/motor/speed. A single /drive publisher
            # is insufficient to establish a single converter or command path.
            # This software only suppresses its own output; rogue publishers
            # must be removed and a physical independent cutoff remains vital.
            drive_publishers = len(self.get_publishers_info_by_topic('/drive'))
            drive_subscribers = len(self.get_subscriptions_info_by_topic('/drive'))
            motor_publishers = len(
                self.get_publishers_info_by_topic('/commands/motor/speed'))
            motor_subscribers = len(
                self.get_subscriptions_info_by_topic('/commands/motor/speed'))
            rf2o_publishers = len(
                self.get_publishers_info_by_topic('/odom_rf2o'))
            if self.mode == 'AUTO_ACTIVE' and rf2o_publishers != 1:
                speed, steer = 0.0, 0.0
                self.mode = 'STOP'
                reason = 'AUTO_DUPLICATE_OR_MISSING_RF2O_ODOMETRY'
                self.last_stop_reason = reason
                self.last_stop_at = now
                self.get_logger().warn('STOP LATCHED: %s' % reason)
            if (drive_publishers != 1 or drive_subscribers != 1
                    or motor_publishers != 1 or motor_subscribers != 1):
                speed, steer = 0.0, 0.0
                self.mode = 'STOP'
                reason = 'ACTUATOR_GRAPH_OWNERSHIP_FAULT'
                self.last_stop_reason = reason
                self.last_stop_at = now
                self.get_logger().warn('STOP LATCHED: %s' % reason)

            cmd_out = AckermannDriveStamped()
            cmd_out.header.stamp = self.get_clock().now().to_msg()
            cmd_out.drive.speed = float(speed)
            cmd_out.drive.steering_angle = float(steer)
            self.drive_pub.publish(cmd_out)

            # Snapshot a stop exactly once at its first STOP control tick,
            # not from a later scan after the obstacle may have disappeared.
            # Always publish the zero-speed command BEFORE any file I/O.
            if (self.mode == 'STOP' and self.last_stop_at is not None
                    and self.last_stop_at != self.last_stop_persisted_at):
                self.last_stop_persisted_at = self.last_stop_at
                self.last_stop_snapshot = make_snapshot(
                    self.last_stop_reason, mode_before_tick, cmd, path,
                    scan_reason=scan_reason, nearest_front_m=nearest,
                    scan_fresh=scan_fresh, odom_fresh=odom_fresh,
                    map_fresh=map_fresh,
                    hold_age_s=(now-self.auto_hold_at if self.auto_hold_at is not None else None),
                    drive_publishers=drive_publishers, drive_subscribers=drive_subscribers,
                    motor_publishers=motor_publishers, motor_subscribers=motor_subscribers,
                    rf2o_publishers=rf2o_publishers)
                try:
                    filename = save_snapshot(self.last_stop_snapshot)
                    self.get_logger().warn('STOP EVIDENCE: %s' % filename)
                except OSError as exc:
                    self.get_logger().error('Could not persist STOP snapshot: %s' % exc)

            status = {
                'mode': self.mode,
                'reason': reason,
                'last_stop_reason': self.last_stop_reason,
                'last_stop_snapshot': self.last_stop_snapshot,
                'last_stop_age_s': (round(now-self.last_stop_at, 3)
                                    if self.last_stop_at is not None else None),
                'auto_hold_age_s': (round(now-self.auto_hold_at, 3)
                                    if self.auto_hold_at is not None else None),
                'real_actuation_enabled': self.allow_real,
                'autonomous_actuation_enabled': self.allow_auto,
                'max_speed_mps': self.max_speed,
                'max_steer_rad': self.max_steer,
                'manual_min_front_valid_fraction': self.manual_fraction,
                'manual_low_visibility_enabled': self.manual_low_visibility,
                'deadman_button_5_held': joy_held,
                'auto_hold_held_and_fresh': auto_hold_held,
                'auto_hold_raw_value': self.auto_hold,
                'last_mode_request': self.last_mode_request,
                'mode_request_result': self.mode_request_result,
                'mode_request_age_s': (
                    round(now-self.last_mode_request_at, 3)
                    if self.last_mode_request_at is not None else None),
                'first_lap_completed': self.first_lap_completed,
                'scan_fresh': scan_fresh,
                'scan_reason': scan_reason,
                'front_valid_fraction': round(fraction, 4),
                'nearest_front_m': round(nearest, 3) if nearest is not None else None,
                'swept_path': path,
                'odom_fresh': odom_fresh,
                'map_fresh': map_fresh,
                'map_age_s': (round(now-self.map_at, 3)
                              if self.map_at is not None else None),
                'speed_mps': round(speed, 4),
                'steer_rad': round(steer, 4),
                'drive_publisher_count': drive_publishers,
                'drive_subscriber_count': drive_subscribers,
                'motor_publisher_count': motor_publishers,
                'motor_subscriber_count': motor_subscribers,
                'rf2o_publisher_count': rf2o_publishers,
                'safety_mode': self.safety_mode,
                'teleop_warn': teleop_warn['warn'] if teleop_warn else None,
                'teleop_warn_front_unknown': teleop_warn['front_unknown'] if teleop_warn else None,
                'teleop_warn_obstacle_near': teleop_warn['obstacle_near'] if teleop_warn else None,
                'ps4_joy_problem': ps4_problem,
                'ps4_deadman_held': ps4_held,
                'ps4_joy_publisher_count': ps4_pubs,
                'ps4_joy_age_s': round(ps4_age, 3) if ps4_age is not None else None,
                'ps4_joy_stamp_age_s': round(ps4_stamp_age, 3) if ps4_stamp_age is not None else None,
            }
            self.status_pub.publish(String(data=json.dumps(status, separators=(',', ':'))))
            if reason != self.last_reason:
                self.get_logger().info('%s: %s; speed=%.3f, steer=%.3f' %
                                       (self.mode, reason, speed, steer))
                self.last_reason = reason

    rclpy.init()
    node = PhysicalMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            # Best-effort only; NOT a safe stop after a process crash.
            stop = AckermannDriveStamped()
            for _ in range(5):
                node.drive_pub.publish(stop)
                rclpy.spin_once(node, timeout_sec=.01)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    args, unknown = parser.parse_known_args()
    if args.self_test:
        self_test()
    else:
        # ROS 2 remapping/parameter arguments pass through to rclpy.init.
        run_ros()


if __name__ == '__main__':
    main()
