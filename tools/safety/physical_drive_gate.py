#!/usr/bin/env python3
"""F1TENTH offline safety-gate prototype. BENCH TOPICS ONLY; NOT A SAFETY SYSTEM.

Never connect this prototype to a physical VESC, steering servo, or teleop bridge.
A verified physical cutoff and an independent motor controller timeout are required.
"""
import math
import sys
import time

MAX_TEST_SPEED = 0.25
MAX_TEST_STEER = 0.10
COMMAND_TIMEOUT = 0.25
HEARTBEAT_TIMEOUT = 0.25


class CommandGate:
    def __init__(self):
        self.armed = False
        self.arm_low_seen = False
        self.last_command = None
        self.last_command_at = None
        self.last_heartbeat_at = None
        self.status = 'DISARMED'

    def _latch(self, cause):
        self.armed = False
        self.arm_low_seen = False
        self.status = cause

    def command(self, speed, steer, now):
        if (not math.isfinite(speed) or not math.isfinite(steer)
                or abs(speed) > MAX_TEST_SPEED
                or abs(steer) > MAX_TEST_STEER):
            self._latch('FAULT_LIMIT')
            self.last_command = None
            self.last_command_at = None
            return
        self.last_command = (speed, steer)
        self.last_command_at = now

    def heartbeat(self, is_ok, now):
        if not is_ok:
            self.last_heartbeat_at = None
            self._latch('FAULT_HEARTBEAT_FALSE')
            return
        self.last_heartbeat_at = now

    def arm(self, value, now):
        if not value:
            self.armed = False
            self.arm_low_seen = True
            self.status = 'DISARMED'
            return
        if not self.arm_low_seen or self.armed:
            return
        self.arm_low_seen = False  # one explicit rising edge per attempt
        cmd_fresh = (self.last_command_at is not None and
                     0 <= now - self.last_command_at < COMMAND_TIMEOUT)
        hb_fresh = (self.last_heartbeat_at is not None and
                    0 <= now - self.last_heartbeat_at < HEARTBEAT_TIMEOUT)
        if cmd_fresh and hb_fresh:
            self.armed = True
            self.status = 'ARMED'
        else:
            self.status = 'ARM_DENIED_STALE_INPUT'

    def output(self, now):
        if not self.armed:
            return 0.0, 0.0
        if (self.last_heartbeat_at is None or
                not 0 <= now - self.last_heartbeat_at < HEARTBEAT_TIMEOUT):
            self._latch('FAULT_HEARTBEAT_TIMEOUT')
            return 0.0, 0.0
        if (self.last_command_at is None or
                not 0 <= now - self.last_command_at < COMMAND_TIMEOUT):
            self._latch('FAULT_COMMAND_TIMEOUT')
            return 0.0, 0.0
        return self.last_command


def self_test():
    g = CommandGate()
    t = 10.0
    assert g.output(t) == (0.0, 0.0)
    g.command(0.2, 0.05, t)
    g.heartbeat(True, t)
    g.arm(True, t)
    assert g.output(t) == (0.0, 0.0), 'Must reject arming without low->high'
    g.arm(False, t)
    g.arm(True, t)
    assert g.output(t) == (0.2, 0.05), 'Arming with fresh inputs failed'
    g.heartbeat(True, t + 0.4)
    assert g.output(t + 0.4) == (0.0, 0.0), 'Stale command did not stop output'
    assert not g.armed
    g.command(0.2, 0.05, t + 0.41)
    g.heartbeat(True, t + 0.41)
    assert g.output(t + 0.41) == (0.0, 0.0), 'Unexpected auto-rearm'
    g.arm(True, t + 0.41)
    assert not g.armed, 'A second high must not rearm'
    g.arm(False, t + 0.41)
    g.arm(True, t + 0.41)
    assert g.output(t + 0.41) == (0.2, 0.05)
    g.command(0.9, 0.05, t + 0.42)
    assert g.output(t + 0.42) == (0.0, 0.0), 'Out-of-range command was not latched'
    g.arm(False, t + 0.43)
    g.heartbeat(True, t + 0.43)
    g.command(-0.1, 0.02, t + 0.43)
    g.arm(True, t + 0.43)
    assert g.output(t + 0.43) == (-0.1, 0.02)
    g.heartbeat(False, t + 0.44)
    assert g.output(t + 0.44) == (0.0, 0.0), 'Heartbeat-false failed'
    g.arm(False, t + 0.45)
    g.heartbeat(True, t + 0.45)
    g.command(0.1, 0, t + 0.45)
    g.arm(True, t + 0.45)
    assert g.output(t + 0.45) == (0.1, 0)
    assert g.output(t + 0.71) == (0.0, 0.0), 'Heartbeat timeout failed'
    print('PASS: default-disarmed, explicit arming, fresh inputs,')
    print('      stale-command latch, no auto-rearm, limit rejection,')
    print('      heartbeat-false and heartbeat-timeout latch.')
    print('Hardware is not involved.')


def run_ros():
    import rclpy
    from rclpy.node import Node
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import Bool, String

    class GateNode(Node):
        def __init__(self):
            super().__init__('offline_physical_drive_gate')
            self.gate = CommandGate()
            self.last_status = self.gate.status
            self.pub = self.create_publisher(AckermannDriveStamped, '/bench/gated_drive', 10)
            self.state_pub = self.create_publisher(String, '/bench/gate_state', 10)
            self.create_subscription(AckermannDriveStamped, '/bench/raw_drive', self.command, 10)
            self.create_subscription(Bool, '/bench/heartbeat', self.heartbeat, 10)
            self.create_subscription(Bool, '/bench/arm', self.arm, 10)
            self.create_timer(0.02, self.tick)
            self.get_logger().warning('OFFLINE BENCH ONLY. NEVER CONNECT TO VESC ACTUATORS.')

        def command(self, msg):
            self.gate.command(float(msg.drive.speed), float(msg.drive.steering_angle), time.monotonic())

        def heartbeat(self, msg):
            self.gate.heartbeat(bool(msg.data), time.monotonic())

        def arm(self, msg):
            self.gate.arm(bool(msg.data), time.monotonic())

        def tick(self):
            speed, steer = self.gate.output(time.monotonic())
            m = AckermannDriveStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.drive.speed = speed
            m.drive.steering_angle = steer
            self.pub.publish(m)
            self.state_pub.publish(String(data=self.gate.status))
            if self.gate.status != self.last_status:
                self.get_logger().info('Gate: %s' % self.gate.status)
                self.last_status = self.gate.status

    rclpy.init()
    node = GateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) == 2 and sys.argv[1] == '--self-test':
        self_test()
    elif len(sys.argv) == 1:
        run_ros()
    else:
        raise SystemExit('Usage: physical_drive_gate.py [--self-test]')

