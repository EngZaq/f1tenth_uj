#!/usr/bin/env python3
"""Read-only 30-second audit of the non-authoritative F1TENTH shadow EKF.

Checks only ROS topic plumbing, stamps, finite values and injected variances.
This does not validate state-estimation accuracy or physical driving safety.
"""

import argparse
import json
import math
import statistics
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

TOPICS = (
    '/shadow_inputs/odom_rf2o',
    '/shadow_inputs/odom_vesc',
    '/odometry/filtered_shadow',
)


class Audit(Node):
    def __init__(self):
        super().__init__('f1tenth_shadow_ekf_audit')
        self.data = defaultdict(list)
        self.subscriptions = []
        for topic in TOPICS:
            self.subscriptions.append(self.create_subscription(
                Odometry, topic,
                lambda msg, t=topic: self.receive(t, msg),
                qos_profile_sensor_data,
            ))

    def receive(self, topic, msg):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.data[topic].append({
            'arrival': time.monotonic(),
            'stamp_ns': stamp_ns,
            'header_age_s': (now_ns - stamp_ns) * 1e-9 if stamp_ns else None,
            'frame': msg.header.frame_id,
            'child': msg.child_frame_id,
            'x': float(p.x),
            'y': float(p.y),
            'yaw': math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            ),
            'vx': float(msg.twist.twist.linear.x),
            'pose_x_var': float(msg.pose.covariance[0]),
            'pose_y_var': float(msg.pose.covariance[7]),
            'pose_yaw_var': float(msg.pose.covariance[35]),
            'twist_vx_var': float(msg.twist.covariance[0]),
        })

    def report(self):
        out = {}
        passes = {}
        for topic in TOPICS:
            records = self.data[topic]
            count = len(records)
            ages = [
                x['header_age_s'] for x in records
                if x['header_age_s'] is not None
                and math.isfinite(x['header_age_s'])
            ]
            last = records[-1] if records else None
            rate = (
                (count - 1) / (records[-1]['arrival'] - records[0]['arrival'])
                if count > 1 and records[-1]['arrival'] > records[0]['arrival']
                else None
            )
            out[topic] = {
                'count': count,
                'rate_hz': round(rate, 3) if rate is not None else None,
                'median_stamp_age_s': (
                    round(statistics.median(ages), 5) if ages else None
                ),
                'frame_pairs': sorted(set(
                    (x['frame'], x['child']) for x in records
                )),
                'header_stamps_monotonic': all(
                    b['stamp_ns'] >= a['stamp_ns']
                    for a, b in zip(records[:-1], records[1:])
                ),
                'last': {
                    k: round(v, 6) if isinstance(v, float) else v
                    for k, v in (last or {}).items() if k != 'arrival'
                } if last else None,
                'net_xy_m': round(math.hypot(
                    records[-1]['x'] - records[0]['x'],
                    records[-1]['y'] - records[0]['y'],
                ), 6) if count > 1 else None,
                'all_pose_finite': all(
                    math.isfinite(x[k]) for x in records
                    for k in ('x', 'y', 'yaw')
                ),
            }
            passes[topic + '_has_messages'] = count >= 10
            passes[topic + '_monotonic_stamps'] = (
                count >= 10 and out[topic]['header_stamps_monotonic']
            )
            passes[topic + '_expected_frames'] = (
                count >= 10 and
                out[topic]['frame_pairs'] == [('odom', 'base_link')]
            )
            passes[topic + '_finite_pose'] = (
                count >= 10 and out[topic]['all_pose_finite']
            )

        rf = out['/shadow_inputs/odom_rf2o']['last']
        ve = out['/shadow_inputs/odom_vesc']['last']
        passes['rf2o_pose_nonzero_test_variance'] = bool(
            rf and all(rf[k] > 0 for k in
                       ('pose_x_var', 'pose_y_var', 'pose_yaw_var'))
        )
        passes['vesc_vx_nonzero_test_variance'] = bool(
            ve and ve['twist_vx_var'] > 0
        )
        return {
            'test': 'F1TENTH shadow EKF observation',
            'no_motor_or_servo_commands': True,
            'publish_tf_disabled_must_be_checked_separately': True,
            'sensor_uncertainty_calibrated': False,
            'topics': out,
            'checks': passes,
            'pass': all(passes.values()),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=35.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 5.0 <= args.duration <= 300.0:
        parser.error('duration must be in [5, 300] seconds')

    rclpy.init()
    node = Audit()
    try:
        print('SHADOW EKF OBSERVATION — no actuator commands.')
        until = time.monotonic() + args.duration
        while time.monotonic() < until and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n=== SHADOW EKF CHECKS ===')
    for key, passed in result['checks'].items():
        print('%-54s %s' % (key, 'PASS' if passed else 'FAIL'))
    print('\n=== TOPICS ===')
    for topic, r in result['topics'].items():
        print(
            '%s: %d samples, %s Hz, median age %s s, net XY %s m'
            % (topic, r['count'], r['rate_hz'],
               r['median_stamp_age_s'], r['net_xy_m'])
        )
        print('  latest:', r['last'])
    print('\nRESULT:', 'PASS' if result['pass'] else 'FAIL')
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print('Saved:', output)
    if not result['pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
