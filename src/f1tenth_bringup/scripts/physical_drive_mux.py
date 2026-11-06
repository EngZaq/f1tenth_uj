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

MAX_SPEED = 0.35
MAX_STEER = 0.10
COMMAND_TTL = 0.22
JOY_TTL = 0.20
SCAN_TTL = 0.22
ODOM_TTL = 0.30
MAP_TTL = 8.0
AUTO_HOLD_TTL = 0.20  # supervised input, NOT a safety-rated physical switch


def evaluate_request(mode, allow_real, allow_auto, deadman, deadman_fresh,
                     cmd, cmd_fresh, scan_ok, odom_fresh, map_fresh):
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
    return min(speed, MAX_SPEED), max(-MAX_STEER, min(MAX_STEER, steer)), 'PASS'


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
            self.declare_parameter('auto_min_front_valid', .90)
            self.allow_real = bool(self.get_parameter('enable_real_actuation').value)
            self.allow_auto = bool(self.get_parameter('enable_autonomous_actuation').value)
            self.manual_fraction = float(self.get_parameter('manual_min_front_valid').value)
            self.auto_fraction = float(self.get_parameter('auto_min_front_valid').value)
            if not .75 <= self.manual_fraction <= 1.0:
                raise ValueError('manual_min_front_valid must be >=0.75')
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
            self.last_mode_request = None
            self.last_mode_request_at = None
            self.mode_request_result = 'NO_MODE_REQUEST_RECEIVED'
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
            self.create_timer(.05, self.tick)
            self.get_logger().warn(
                'REAL /drive connected. Starts STOP. '
                'This ROS mux is NOT a physical emergency stop. '
                'enable_real_actuation=%s enable_autonomous_actuation=%s' %
                (self.allow_real, self.allow_auto)
            )

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
            if m == 'STOP':
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
            joy_fresh = self.fresh(self.joy_at, JOY_TTL, now)
            joy_held = joy_fresh and self.deadman
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
            request_scan_ok = (
                scan_ok if self.mode.startswith('MANUAL')
                else bool(path['ready']))
            odom_fresh = self.fresh(self.odom_at, ODOM_TTL, now)
            map_fresh = self.fresh(self.map_at, MAP_TTL, now)

            speed, steer, reason = evaluate_request(
                self.mode, self.allow_real, self.allow_auto,
                active_hold, (joy_fresh if self.mode.startswith('MANUAL')
                              else auto_hold_fresh), cmd,
                self.fresh(received, COMMAND_TTL, now),
                request_scan_ok, odom_fresh, map_fresh)

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

            status = {
                'mode': self.mode,
                'reason': reason,
                'last_stop_reason': self.last_stop_reason,
                'last_stop_age_s': (round(now-self.last_stop_at, 3)
                                    if self.last_stop_at is not None else None),
                'auto_hold_age_s': (round(now-self.auto_hold_at, 3)
                                    if self.auto_hold_at is not None else None),
                'real_actuation_enabled': self.allow_real,
                'autonomous_actuation_enabled': self.allow_auto,
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
