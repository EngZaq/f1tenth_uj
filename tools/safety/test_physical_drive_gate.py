#!/usr/bin/env python3
"""ROS2 bench-only integration test for physical_drive_gate.py.
Does not publish to hardware actuator topics.
"""
import time
import sys
import math
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String

rclpy.init()
node = rclpy.create_node('offline_gate_test_runner')

raw_pub = node.create_publisher(AckermannDriveStamped, '/bench/raw_drive', 10)
hb_pub = node.create_publisher(Bool, '/bench/heartbeat', 10)
arm_pub = node.create_publisher(Bool, '/bench/arm', 10)
received = []
states = []
sub1 = node.create_subscription(
    AckermannDriveStamped, '/bench/gated_drive',
    lambda m: received.append((time.monotonic(), float(m.drive.speed), float(m.drive.steering_angle))), 10)
sub2 = node.create_subscription(
    String, '/bench/gate_state',
    lambda m: states.append((time.monotonic(), m.data)), 10)


def pump(seconds, speed=None, steer=0.0, heartbeat=None, arm=None):
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        if speed is not None:
            m = AckermannDriveStamped()
            m.header.stamp = node.get_clock().now().to_msg()
            m.drive.speed = float(speed)
            m.drive.steering_angle = float(steer)
            raw_pub.publish(m)
        if heartbeat is not None:
            hb_pub.publish(Bool(data=heartbeat))
        if arm is not None:
            arm_pub.publish(Bool(data=arm))
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.005)


def check(label, want_armed, want_speed):
    now = time.monotonic()
    last = [(v,s) for t,v,s in received if t >= now - 0.30]
    ss = [s for t,s in states if t >= now - 0.30]
    if len(last) < 4 or len(ss) < 4:
        raise AssertionError(f'{label}: insufficient new output samples')
    is_armed = ss[-1] == 'ARMED'
    actual = [v for v,s in last]
    passed = is_armed == want_armed and all(abs(v-want_speed) < 1e-5 for v in actual)
    print(f'{label}: {"PASS" if passed else "FAIL"} | state={ss[-1]} speed={actual[-1]:+.3f} m/s')
    if not passed:
        raise AssertionError(f'{label} check failed')

try:
    print('=== ROS2 OFFLINE GATE INTEGRATION ===')
    deadline = time.monotonic() + 7
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if (raw_pub.get_subscription_count() == 1
            and hb_pub.get_subscription_count() == 1
            and arm_pub.get_subscription_count() == 1
            and node.count_publishers('/bench/gated_drive') == 1
            and node.count_publishers('/bench/gate_state') == 1):
            break
    else:
        raise RuntimeError('Expected exactly one offline gate node; check that it is running')
    pump(0.4)
    check('Starts disarmed', False, 0)
    pump(0.15, arm=False)
    pump(0.4, speed=.2, steer=.05, heartbeat=True)
    pump(0.6, speed=.2, steer=.05, heartbeat=True, arm=True)
    check('Explicitly armed with fresh inputs', True, .2)
    pump(0.6)
    check('Timeout stops and latches', False, 0)
    pump(0.4, speed=.2, steer=.05, heartbeat=True)
    check('No automatic re-arming', False, 0)
    pump(0.15, speed=.2, steer=.05, heartbeat=True, arm=False)
    pump(0.55, speed=.2, steer=.05, heartbeat=True, arm=True)
    check('Explicit re-arm', True, .2)
    pump(0.4, speed=.8, steer=.05, heartbeat=True)
    check('Overspeed rejected and latched', False, 0)
    print('FINAL RESULT: PASS — bench-only, no physical actuation')
except Exception as e:
    print(f'FINAL RESULT: FAIL — {e}')
    sys.exit(1)
finally:
    node.destroy_node()
    rclpy.shutdown()

