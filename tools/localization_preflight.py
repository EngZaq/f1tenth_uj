#!/usr/bin/env python3
"""F1TENTH ROS 2 read-only localization input preflight.

Collects odometry timing, covariance and frame evidence. No publishers, no
motor/servo commands, no EKF, no TF authority changes. Zero/invalid covariances
are reported rather than silently "corrected" by invented noise estimates.
"""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

POSE_NAMES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw')
TWIST_NAMES = ('vx', 'vy', 'vz', 'vroll', 'vpitch', 'vyaw')
DIAGONAL = (0, 7, 14, 21, 28, 35)


def compact_stats(values):
    if not values:
        return None
    values = sorted(float(x) for x in values)
    return {
        'min': round(values[0], 6),
        'median': round(statistics.median(values), 6),
        'p95': round(values[min(len(values) - 1, math.ceil(len(values) * .95) - 1)], 6),
        'max': round(values[-1], 6),
    }


def diagonal(cov, names):
    return {name: float(cov[idx]) for name, idx in zip(names, DIAGONAL)}


class Monitor(Node):
    def __init__(self):
        super().__init__('f1tenth_localization_preflight')
        self.data = {
            '/odom_rf2o': [],
            '/odom_vesc_observe': [],
        }
        self.subs = []
        for topic in self.data:
            self.subs.append(self.create_subscription(
                Odometry, topic, lambda m, t=topic: self.record(t, m),
                qos_profile_sensor_data,
            ))

    def record(self, topic, m):
        arrival = time.monotonic()
        stamp_ns = int(m.header.stamp.sec) * 1_000_000_000 + int(m.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - stamp_ns) / 1e9 if stamp_ns else None
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z),
        )
        self.data[topic].append({
            'arrival': arrival,
            'stamp_ns': stamp_ns,
            'stamp_age_s': age,
            'parent': str(m.header.frame_id),
            'child': str(m.child_frame_id),
            'pose_diag': diagonal(m.pose.covariance, POSE_NAMES),
            'twist_diag': diagonal(m.twist.covariance, TWIST_NAMES),
            'x': float(p.x),
            'y': float(p.y),
            'yaw': yaw,
            'vx': float(m.twist.twist.linear.x),
            'vy': float(m.twist.twist.linear.y),
            'wz': float(m.twist.twist.angular.z),
        })

    def report(self, duration):
        summaries = {}
        alerts = []
        for topic, records in self.data.items():
            count = len(records)
            age = [r['stamp_age_s'] for r in records if r['stamp_age_s'] is not None
                   and math.isfinite(r['stamp_age_s'])]
            arrivals = [r['arrival'] for r in records]
            intervals = [b - a for a, b in zip(arrivals[:-1], arrivals[1:]) if b > a]
            stamps = [r['stamp_ns'] for r in records if r['stamp_ns'] > 0]
            last = records[-1] if records else None
            summary = {
                'count': count,
                'rate_hz': round((count-1)/(arrivals[-1]-arrivals[0]), 3)
                           if count > 1 and arrivals[-1] > arrivals[0] else None,
                'arrival_gap_s': compact_stats(intervals),
                'header_age_s': compact_stats(age),
                'frame_pairs': sorted(set((r['parent'], r['child']) for r in records)),
                'header_stamps_monotonic': all(b >= a for a, b in zip(stamps[:-1], stamps[1:])),
                'zero_header_stamps': count-len(stamps),
                'last_pose_covariance_diagonal': last['pose_diag'] if last else None,
                'last_twist_covariance_diagonal': last['twist_diag'] if last else None,
                'last_linear_x_mps': round(last['vx'], 6) if last else None,
                'last_yaw_rate_radps': round(last['wz'], 6) if last else None,
                'net_xy_m': (
                    round(math.hypot(records[-1]['x']-records[0]['x'],
                                     records[-1]['y']-records[0]['y']), 6)
                    if count > 1 else None
                ),
                'net_yaw_deg': (
                    round(math.degrees(math.atan2(
                        math.sin(records[-1]['yaw']-records[0]['yaw']),
                        math.cos(records[-1]['yaw']-records[0]['yaw']))), 5)
                    if count > 1 else None
                ),
            }
            summaries[topic] = summary
            if count < 10:
                alerts.append('%s: fewer than 10 samples received' % topic)
            if summary['frame_pairs'] and summary['frame_pairs'] != [('odom', 'base_link')]:
                alerts.append('%s: unexpected frame pair(s) %s' % (
                    topic, summary['frame_pairs']))
            if not summary['header_stamps_monotonic']:
                alerts.append('%s: header timestamps went backwards' % topic)
            if summary['zero_header_stamps']:
                alerts.append('%s: missing header timestamps' % topic)
            if summary['header_age_s'] and summary['header_age_s']['p95'] > 0.5:
                alerts.append('%s: p95 message timestamp age > 0.5 s' % topic)
            if last:
                # We intend to use RF2O XY+heading as a pose observation and
                # VESC forward speed as a *future* velocity observation.
                intended = (('pose_diag', ('x', 'y', 'yaw'))
                            if topic == '/odom_rf2o'
                            else ('twist_diag', ('vx',)))
                field, keys = intended
                for name in keys:
                    value = last[field][name]
                    if not math.isfinite(value) or value <= 0:
                        alerts.append(
                            '%s: %s %s variance is %s; NOT ready for EKF fusion'
                            % (topic, field, name, value)
                        )

        return {
            'test': 'F1TENTH localization preflight (read-only)',
            'duration_requested_s': duration,
            'no_commands_published': True,
            'odometry': summaries,
            'alerts': alerts,
            'transport_observed': all(s['count'] >= 10 for s in summaries.values()),
            'covariance_ready_for_fusion': all('variance' not in a for a in alerts),
            'notes': [
                'Covariance readiness here is only a basic nonzero check, NOT a calibration.',
                'RF2O and wheel odometry can be correlated; EKF gains must be justified.',
                'Stationary pose changes do not prove moving-scale accuracy.',
                'Do not replace the working RF2O TF or let a shadow EKF publish TF.',
                'Emergency stop, current limits and actuator calibration remain pending.',
            ],
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if not 5 <= args.duration <= 300:
        parser.error('--duration must be between 5 and 300 seconds')

    rclpy.init()
    node = Monitor()
    print('Read-only localization preflight; no publishers or actuator commands.')
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        result = node.report(args.duration)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n=== F1TENTH LOCALIZATION PREFLIGHT ===')
    for topic, record in result['odometry'].items():
        print('\n', topic)
        for k in ('count', 'rate_hz', 'frame_pairs', 'header_age_s',
                  'header_stamps_monotonic', 'last_pose_covariance_diagonal',
                  'last_twist_covariance_diagonal', 'net_xy_m', 'net_yaw_deg'):
            print('  %s: %s' % (k, record[k]))
    print('\nBasic odometry transport:', 'PASS' if result['transport_observed'] else 'FAIL')
    print('Basic covariance prerequisite:', 'PASS' if result['covariance_ready_for_fusion'] else 'NOT READY')
    print('Alerts:')
    if result['alerts']:
        for a in result['alerts']:
            print(' -', a)
    else:
        print(' - none')
    if args.output:
        target = Path(args.output).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print('Saved:', target)
    if not result['transport_observed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
