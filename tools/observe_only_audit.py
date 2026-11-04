#!/usr/bin/env python3
"""Read-only ROS 2 audit for the F1TENTH observe_only launch.

Subscribes to LiDAR, RF2O, SLAM, VESC telemetry/odometry and TF. Publishes
NOTHING. Never actuates a motor or servo. This is an observation test, not a
functional safety certification or a physical odometry calibration.
"""

import argparse
import json
import math
import statistics
import time
from collections import defaultdict
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from vesc_msgs.msg import VescStateStamped


def number_or_none(v):
    return round(float(v), 6) if math.isfinite(float(v)) else None


class ObserveAudit(Node):
    def __init__(self, include_vesc):
        super().__init__('f1tenth_observe_only_audit')
        self.include_vesc = include_vesc
        self.arrival = defaultdict(list)
        self.tf_edges = defaultdict(int)
        self.scan_stats = []
        self.rf2o_poses = []
        self.vesc_speeds = []
        self.vesc_values = []
        self.core = []
        self.maps = []
        self.subs = []

        self.subs.append(self.create_subscription(
            LaserScan, '/scan', self.on_scan, qos_profile_sensor_data
        ))
        self.subs.append(self.create_subscription(
            Odometry, '/odom_rf2o', self.on_rf2o, qos_profile_sensor_data
        ))
        self.subs.append(self.create_subscription(
            OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data
        ))
        self.subs.append(self.create_subscription(
            TFMessage, '/tf', self.on_tf, qos_profile_sensor_data
        ))

        if include_vesc:
            self.subs.append(self.create_subscription(
                VescStateStamped, '/sensors/core',
                self.on_core, qos_profile_sensor_data
            ))
            self.subs.append(self.create_subscription(
                Odometry, '/odom_vesc_observe',
                self.on_vesc_odom, qos_profile_sensor_data
            ))

    def tick(self, topic):
        self.arrival[topic].append(time.monotonic())

    def on_scan(self, msg):
        self.tick('/scan')
        valid = sum(
            1 for x in msg.ranges
            if math.isfinite(x) and msg.range_min <= x <= msg.range_max
        )
        self.scan_stats.append({
            'beams': len(msg.ranges),
            'valid': valid,
            'frame': msg.header.frame_id,
        })

    def on_rf2o(self, msg):
        self.tick('/odom_rf2o')
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.rf2o_poses.append((float(p.x), float(p.y), float(yaw)))

    def on_map(self, msg):
        self.tick('/map')
        width = int(msg.info.width)
        height = int(msg.info.height)
        unknown = sum(1 for v in msg.data if v < 0)
        self.maps.append({
            'width': width, 'height': height,
            'resolution_m': number_or_none(msg.info.resolution),
            'unknown_fraction': (
                round(unknown / (width * height), 5)
                if width and height else None
            ),
            'frame': msg.header.frame_id,
        })

    def on_tf(self, msg):
        self.tick('/tf')
        for trans in msg.transforms:
            key = '%s->%s' % (
                trans.header.frame_id.lstrip('/'),
                trans.child_frame_id.lstrip('/'),
            )
            self.tf_edges[key] += 1

    def on_core(self, msg):
        self.tick('/sensors/core')
        s = msg.state
        self.core.append({
            'voltage_v': float(s.voltage_input),
            'erpm': float(s.speed),
            'input_current_a': float(s.current_input),
            'motor_current_a': float(s.current_motor),
            'fault_code': int(s.fault_code),
        })

    def on_vesc_odom(self, msg):
        self.tick('/odom_vesc_observe')
        self.vesc_speeds.append(float(msg.twist.twist.linear.x))
        self.vesc_values.append({
            'frame': msg.header.frame_id,
            'child_frame': msg.child_frame_id,
        })

    def report(self, duration):
        expected = ['/scan', '/odom_rf2o', '/map']
        if self.include_vesc:
            expected += ['/sensors/core', '/odom_vesc_observe']

        counts = {t: len(self.arrival[t]) for t in expected}
        rates = {}
        for t in expected:
            stamps = self.arrival[t]
            rates[t] = (
                round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 3)
                if len(stamps) > 1 and stamps[-1] > stamps[0]
                else None
            )

        checks = {
            'laser_scan_received': counts['/scan'] > 10,
            'rf2o_odom_received': counts['/odom_rf2o'] > 5,
            'slam_map_received': counts['/map'] >= 1,
            'rf2o_tf_odom_to_base_link': (
                self.tf_edges['odom->base_link'] > 2
            ),
            'slam_tf_map_to_odom': self.tf_edges['map->odom'] > 2,
        }
        if self.include_vesc:
            checks['vesc_core_received'] = counts['/sensors/core'] > 10
            checks['vesc_odom_received'] = (
                counts['/odom_vesc_observe'] > 10
            )

        rf2o_drift = None
        if len(self.rf2o_poses) > 1:
            x0, y0, yaw0 = self.rf2o_poses[0]
            x1, y1, yaw1 = self.rf2o_poses[-1]
            rf2o_drift = {
                'net_displacement_m': round(math.hypot(x1 - x0, y1 - y0), 6),
                'net_yaw_deg': round(
                    math.degrees(math.atan2(
                        math.sin(yaw1 - yaw0),
                        math.cos(yaw1 - yaw0),
                    )), 5
                ),
                'note': 'Net change only; not ground-truth accuracy.',
            }

        latest_scan = self.scan_stats[-1] if self.scan_stats else None
        latest_map = self.maps[-1] if self.maps else None
        latest_core = self.core[-1] if self.core else None
        report = {
            'test': 'F1TENTH physical observe-only integration',
            'duration_requested_s': duration,
            'commands_published': False,
            'hardware_actuation_validated': False,
            'message_counts': counts,
            'measured_rates_hz': rates,
            'tf_edges_observed': dict(self.tf_edges),
            'latest_scan': latest_scan,
            'latest_map': latest_map,
            'rf2o_net_motion': rf2o_drift,
            'latest_vesc_core': latest_core,
            'max_abs_vesc_odom_mps': (
                round(max(abs(v) for v in self.vesc_speeds), 6)
                if self.vesc_speeds else None
            ),
            'all_vesc_fault_codes': (
                sorted(set(x['fault_code'] for x in self.core))
                if self.core else []
            ),
            'checks': checks,
            'overall_observation_pass': all(checks.values()),
            'caveats': [
                'Passing a topic check does not prove odometry accuracy.',
                'Motor/steering commands were not sent by this audit.',
                'TF topic observation alone does not prove there are no duplicate publishers.',
                'Physical emergency stop and motor-controller timeout are not validated.',
            ],
        }
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--duration', type=float, default=30.0,
        help='Time to observe in seconds (default: 30)',
    )
    parser.add_argument(
        '--with-vesc', action='store_true',
        help='Also require isolated VESC telemetry + odometry topics',
    )
    parser.add_argument(
        '--output', default='',
        help='Optional JSON result file path',
    )
    args = parser.parse_args()
    if not (5.0 <= args.duration <= 300.0):
        parser.error('--duration must be between 5 and 300 seconds')

    rclpy.init()
    node = ObserveAudit(args.with_vesc)
    print('OBSERVE ONLY — 0 actuator commands. Collecting ROS messages.')
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        result = node.report(args.duration)
    except KeyboardInterrupt:
        result = node.report(args.duration)
        result['interrupted'] = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print('\n=== F1TENTH READ-ONLY INTEGRATION AUDIT ===')
    for name, ok in result['checks'].items():
        print('%-38s %s' % (name, 'PASS' if ok else 'FAIL'))
    for topic, rate in result['measured_rates_hz'].items():
        print('%-38s %s Hz' % (
            topic,
            ('%.2f' % rate) if rate is not None else 'no rate',
        ))
    print('RF2O net change:', result['rf2o_net_motion'])
    print('Latest VESC:', result['latest_vesc_core'])
    print('TF edges:', result['tf_edges_observed'])
    print('RESULT:', 'PASS' if result['overall_observation_pass'] else 'FAIL')

    if args.output:
        target = Path(args.output).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(
            json.dumps(result, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        print('Saved:', str(target))

    if not result['overall_observation_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
