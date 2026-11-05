#!/usr/bin/env python3
"""Observe only: audit LiDAR-based autonomous exploration proposals on hardware.

Subscribes to LaserScan, Ackermann exploration proposals, RF2O odom and SLAM map.
NEVER publishes commands, arm/heartbeat or TF. DOES NOT authorize real driving.
Independent evaluator intentionally treats invalid/missing forward beams as
blocked, and reports violations rather than modifying the controller output.

Start in a real ROS2 environment, with the physical v2 explorer in shadow mode.
"""
import argparse
import json
import math
import statistics
import time
from pathlib import Path


def evaluate_front(angle_min, increment, ranges, range_min, range_max,
                   half_angle_deg=20.0, required_valid_fraction=0.90):
    """Return conservative forward-sector clearance and sensor validity."""
    if not ranges or not math.isfinite(increment) or increment == 0:
        return {'safe': False, 'reason': 'invalid_geometry', 'valid_fraction': 0.0,
                'nearest_m': None}
    half = math.radians(half_angle_deg)
    selected = [
        r for i, r in enumerate(ranges)
        if abs(angle_min + i * increment) <= half
    ]
    if len(selected) < 8:
        return {'safe': False, 'reason': 'no_front_sector', 'valid_fraction': 0.0,
                'nearest_m': None}
    valid = [
        float(r) for r in selected
        if math.isfinite(r) and range_min <= r <= range_max
    ]
    fraction = len(valid) / len(selected)
    if not valid or fraction < required_valid_fraction:
        return {'safe': False, 'reason': 'front_invalid',
                'valid_fraction': round(fraction, 4),
                'nearest_m': min(valid) if valid else None}
    return {'safe': True, 'reason': 'valid',
            'valid_fraction': round(fraction, 4),
            'nearest_m': min(valid)}


def check_proposal(speed, steer, front, clearance_threshold=0.80,
                   max_shadow_speed=0.15, max_shadow_steer=0.15):
    """Independent conservative *shadow* acceptance; no actuation."""
    if not math.isfinite(speed) or not math.isfinite(steer):
        return 'non_finite_command'
    if speed < -1e-5 or speed > max_shadow_speed + 1e-5:
        return 'outside_shadow_speed_limit'
    if abs(steer) > max_shadow_steer + 1e-5:
        return 'outside_shadow_steering_limit'
    if speed > 1e-5:
        if not front or not front['safe']:
            return 'motion_with_invalid_front_scan'
        if front['nearest_m'] is None or front['nearest_m'] < clearance_threshold:
            return 'motion_with_near_obstacle'
    return 'accepted'


def self_test():
    good = evaluate_front(-0.3, 0.01, [2.0] * 61, .05, 25.0)
    bad = evaluate_front(-0.3, 0.01, [float('inf')] * 61, .05, 25.0)
    near = evaluate_front(-0.3, 0.01, [.4] * 61, .05, 25.0)
    assert good['safe'] and good['nearest_m'] == 2.0
    assert not bad['safe']
    assert check_proposal(.10, .08, good) == 'accepted'
    assert check_proposal(.10, .08, bad) == 'motion_with_invalid_front_scan'
    assert check_proposal(.10, .08, near) == 'motion_with_near_obstacle'
    assert check_proposal(.22, .08, good) == 'outside_shadow_speed_limit'
    assert check_proposal(.10, .25, good) == 'outside_shadow_steering_limit'
    assert check_proposal(0.0, 0.0, bad) == 'accepted'
    print('PASS: forward scan validity, close obstacle, speed, steering, fail-closed checks')
    print('This self-test publishes no ROS or actuator messages.')


def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid, Odometry
    from ackermann_msgs.msg import AckermannDriveStamped

    class AuditNode(Node):
        def __init__(self):
            super().__init__('physical_exploration_shadow_audit')
            self.scans = 0
            self.proposals = 0
            self.odom = 0
            self.maps = 0
            self.last_scan = None
            self.last_scan_at = None
            self.issues = {}
            self.speeds = []
            self.steering = []
            self.nearest = []
            self.front_valid_fraction_samples = []
            self.front_scan_reasons = {}
            self.forward_beam_tallies = {'valid': 0, 'pos_inf': 0, 'neg_inf': 0,
                                         'nan': 0, 'below_min': 0, 'above_max': 0}
            self.map_known_start = None
            self.map_known_end = None
            self.map_resolution = None
            self.last_reason = None
            self.odom_first = None
            self.odom_last = None
            self.keepalive = []
            self.keepalive.append(self.create_subscription(
                LaserScan, '/scan', self.on_scan, qos_profile_sensor_data))
            self.keepalive.append(self.create_subscription(
                AckermannDriveStamped, '/exploration/drive',
                self.on_proposal, 10))
            self.keepalive.append(self.create_subscription(
                OccupancyGrid, '/map', self.on_map, qos_profile_sensor_data))
            self.keepalive.append(self.create_subscription(
                Odometry, '/odom_rf2o', self.on_odom, qos_profile_sensor_data))

        def on_scan(self, msg):
            self.scans += 1
            self.last_scan_at = time.monotonic()
            self.last_scan = evaluate_front(
                float(msg.angle_min), float(msg.angle_increment),
                msg.ranges, float(msg.range_min), float(msg.range_max))
            if self.last_scan['nearest_m'] is not None:
                self.nearest.append(self.last_scan['nearest_m'])
            self.front_valid_fraction_samples.append(
                self.last_scan['valid_fraction'])
            reason = self.last_scan['reason']
            self.front_scan_reasons[reason] = self.front_scan_reasons.get(reason, 0) + 1
            half = math.radians(20.0)
            for i, raw in enumerate(msg.ranges):
                if abs(float(msg.angle_min) + i * float(msg.angle_increment)) > half:
                    continue
                value = float(raw)
                if math.isnan(value):
                    k = 'nan'
                elif math.isinf(value):
                    k = 'pos_inf' if value > 0 else 'neg_inf'
                elif value < msg.range_min:
                    k = 'below_min'
                elif value > msg.range_max:
                    k = 'above_max'
                else:
                    k = 'valid'
                self.forward_beam_tallies[k] += 1

        def on_proposal(self, msg):
            self.proposals += 1
            speed = float(msg.drive.speed)
            steer = float(msg.drive.steering_angle)
            self.speeds.append(speed)
            self.steering.append(steer)
            scan = self.last_scan
            if (self.last_scan_at is None or
                    time.monotonic() - self.last_scan_at > 0.25):
                scan = None
            reason = check_proposal(speed, steer, scan)
            self.last_reason = reason
            if reason != 'accepted':
                self.issues[reason] = self.issues.get(reason, 0) + 1

        def on_map(self, msg):
            self.maps += 1
            known = sum(v >= 0 for v in msg.data)
            self.map_resolution = float(msg.info.resolution)
            if self.map_known_start is None:
                self.map_known_start = known
            self.map_known_end = known

        def on_odom(self, msg):
            self.odom += 1
            p = msg.pose.pose.position
            pair = (float(p.x), float(p.y))
            if self.odom_first is None:
                self.odom_first = pair
            self.odom_last = pair

        def report(self):
            moved = (math.hypot(
                self.odom_last[0]-self.odom_first[0],
                self.odom_last[1]-self.odom_first[1])
                if self.odom_first and self.odom_last else None)
            area = self.map_resolution**2 if self.map_resolution else None
            fr = sorted(self.front_valid_fraction_samples)
            front_fraction_stats = ({
                'p10': fr[int((len(fr) - 1) * 0.10)],
                'median': statistics.median(fr),
                'p90': fr[int((len(fr) - 1) * 0.90)],
                'min': fr[0], 'max': fr[-1],
                'fraction_below_90pct': sum(x < .90 for x in fr) / len(fr),
            } if fr else None)
            pass_transport = (self.scans > 10 and self.proposals > 10
                              and self.odom > 10 and self.maps >= 1)
            return {
                'mode': 'physical exploration proposal audit — NO actuation',
                'scans_received': self.scans,
                'proposals_received': self.proposals,
                'rf2o_messages': self.odom,
                'maps_received': self.maps,
                'max_abs_proposed_speed_mps': max(map(abs, self.speeds))
                                             if self.speeds else None,
                'max_abs_proposed_steering_rad': max(map(abs, self.steering))
                                                 if self.steering else None,
                'stopped_proposal_fraction': round(
                    sum(abs(s)<1e-5 for s in self.speeds)/len(self.speeds), 4)
                    if self.speeds else None,
                'median_observed_front_nearest_m': statistics.median(self.nearest)
                                                   if self.nearest else None,
                'proposal_violations': self.issues,
                'front_scan_validity': front_fraction_stats,
                'front_scan_reason_counts': self.front_scan_reasons,
                'forward_sector_beam_tallies': self.forward_beam_tallies,
                'last_proposal_decision': self.last_reason,
                'slam_known_area_start_m2': self.map_known_start * area
                                            if area is not None and self.map_known_start is not None else None,
                'slam_known_area_end_m2': self.map_known_end * area
                                          if area is not None and self.map_known_end is not None else None,
                'rf2o_net_xy_m': moved,
                'transport_pass': pass_transport,
                'strict_shadow_acceptance_pass': pass_transport and not self.issues,
                'caveat': 'No physical motion, emergency stop, or real obstacle avoidance certified.',
            }

    rclpy.init()
    node = AuditNode()
    print('PHYSICAL EXPLORATION SHADOW AUDIT: observing proposals only')
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        result = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2)+'\n')
        print('Saved:', output)
    if not result['strict_shadow_acceptance_pass']:
        raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not 5 <= args.duration <= 300:
        parser.error('duration must be between 5 and 300 seconds')
    run_ros(args)


if __name__ == '__main__':
    main()
