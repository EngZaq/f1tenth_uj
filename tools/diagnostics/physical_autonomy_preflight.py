#!/usr/bin/env python3
"""Read-only physical ROS graph and forward LiDAR preflight.

No command publishers, no ROS service writes, no motor/servo actuation.
Reports duplicate graph endpoints and forward-sector measured/unknown returns.
Includes a 5-second measurement window to distinguish intermittent problems.
A passing report does NOT certify safe physical motion.
"""
import json
import math
import time


def check_scan(scan):
    results = {}
    if not scan or not scan.ranges:
        return {'error': 'No /scan received'}
    amin = float(scan.angle_min)
    inc = float(scan.angle_increment)
    minimum = float(scan.range_min)
    maximum = float(scan.range_max)
    if not all(map(math.isfinite, (amin, inc, minimum, maximum))) or inc == 0.0:
        return {'error': 'Invalid scan geometry'}
    for half in (5, 10, 20):
        chosen = []
        for i, value in enumerate(scan.ranges):
            angle = math.degrees(amin + i * inc)
            if abs(angle) <= half:
                chosen.append((angle, float(value)))
        valid = [(angle, distance) for angle, distance in chosen
                 if math.isfinite(distance) and minimum <= distance <= maximum]
        values = sorted(distance for _, distance in valid)
        nearest = sorted(valid, key=lambda pair: pair[1])[:6]
        results[str(half)+'deg'] = {
            'number_of_beams': len(chosen),
            'valid_beams': len(valid),
            'finite_valid_fraction': round(len(valid)/len(chosen), 4) if chosen else 0.0,
            'nearest_m': round(values[0], 3) if values else None,
            'median_m': round(values[len(values)//2], 3) if values else None,
            'nearest_beams_angle_deg_distance_m': [
                [round(angle, 2), round(distance, 3)] for angle, distance in nearest
            ],
            'within_0_80m': sum(distance < .80 for _, distance in valid),
            'unknown_beams': len(chosen)-len(valid),
        }
    return results


def scan_front_classification(scan, half_deg=20.0):
    """Classify all forward beams without interpreting missing rays as free."""
    counts = {'valid': 0, 'positive_infinity': 0, 'negative_infinity': 0,
              'nan': 0, 'below_range_min': 0, 'above_range_max': 0}
    near_angles = []
    valid_values = []
    if scan is None or not scan.ranges:
        return {'error': 'No /scan'}
    amin, increment = float(scan.angle_min), float(scan.angle_increment)
    min_range, max_range = float(scan.range_min), float(scan.range_max)
    if not all(math.isfinite(v) for v in (amin, increment, min_range, max_range)) or increment == 0:
        return {'error': 'Invalid scan geometry'}
    for i, raw in enumerate(scan.ranges):
        angle = math.degrees(amin+i*increment)
        if abs(angle) > half_deg:
            continue
        d = float(raw)
        if math.isnan(d):
            counts['nan'] += 1
        elif math.isinf(d):
            counts['positive_infinity' if d > 0 else 'negative_infinity'] += 1
        elif d < min_range:
            counts['below_range_min'] += 1
        elif d > max_range:
            counts['above_range_max'] += 1
        else:
            counts['valid'] += 1
            valid_values.append(d)
            if d < .80:
                near_angles.append(angle)
    size = sum(counts.values())
    return {
        'counts': counts, 'beam_count': size,
        'valid_fraction': round(counts['valid']/size, 4) if size else 0.,
        'nearest_m': round(min(valid_values), 3) if valid_values else None,
        'near_0_80m_count': len(near_angles),
        'near_0_80m_angle_span_deg': (
            [round(min(near_angles), 1), round(max(near_angles), 1)]
            if near_angles else None),
        'range_min_m': min_range, 'range_max_m': max_range,
        'frame_id': scan.header.frame_id,
    }


def run():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry, OccupancyGrid
    from ackermann_msgs.msg import AckermannDriveStamped
    from std_msgs.msg import String

    class Probe(Node):
        def __init__(self):
            super().__init__('physical_autonomy_readonly_preflight')
            self.scan = None
            self.scan_window = []
            self.received = {}
            self.status = ''
            self.proposal = None
            self.subs = [
                self.create_subscription(LaserScan, '/scan', self.on_scan,
                                         qos_profile_sensor_data),
                self.create_subscription(Odometry, '/odom_rf2o',
                                         lambda msg: self.mark('odom'),
                                         qos_profile_sensor_data),
                self.create_subscription(OccupancyGrid, '/map',
                                         lambda msg: self.mark('map'),
                                         qos_profile_sensor_data),
                self.create_subscription(AckermannDriveStamped,
                                         '/exploration/drive', self.on_proposal, 10),
                self.create_subscription(String, '/control/status',
                                         self.on_status, 10),
            ]

        def mark(self, key):
            self.received[key] = time.monotonic()

        def on_scan(self, msg):
            self.scan = msg
            self.mark('scan')
            if len(self.scan_window) < 250:
                self.scan_window.append(scan_front_classification(msg))

        def on_proposal(self, msg):
            self.proposal = {
                'speed_mps': round(float(msg.drive.speed), 4),
                'steer_rad': round(float(msg.drive.steering_angle), 4),
            }
            self.mark('proposal')

        def on_status(self, msg):
            self.status = msg.data
            self.mark('status')

        def counts(self, topic):
            return {
                'publishers': len(self.get_publishers_info_by_topic(topic)),
                'subscribers': len(self.get_subscriptions_info_by_topic(topic)),
            }

    rclpy.init()
    n = Probe()
    try:
        until = time.monotonic()+5.0
        while time.monotonic() < until:
            rclpy.spin_once(n, timeout_sec=.1)
        names = [name for name, namespace in n.get_node_names_and_namespaces()]
        duplicates = {name: names.count(name) for name in sorted(set(names))
                      if names.count(name) > 1}
        now = time.monotonic()
        try:
            status = json.loads(n.status) if n.status else None
        except json.JSONDecodeError:
            status = {'raw': n.status}
        series = [v for v in n.scan_window if 'counts' in v]
        fractions = [v['valid_fraction'] for v in series]
        clear_count = sum(v['valid_fraction'] >= .90 and
                          v['near_0_80m_count'] == 0 for v in series)
        near_count = sum(v['near_0_80m_count'] > 0 for v in series)
        unknown_count = sum(v['valid_fraction'] < .90 for v in series)
        invalid_totals = {
            key: sum(x['counts'][key] for x in series)
            for key in ('positive_infinity', 'negative_infinity', 'nan',
                        'below_range_min', 'above_range_max')}
        result = {
            'mode': 'READ_ONLY_DIAGNOSTIC_NO_ACTUATION',
            'endpoints': {topic: n.counts(topic) for topic in
                          ['/drive', '/commands/motor/speed', '/odom_rf2o',
                           '/scan', '/map', '/exploration/drive']},
            'duplicate_node_names': duplicates,
            'input_age_sec': {key: round(now-t, 3) for key, t in n.received.items()},
            'forward_scan': check_scan(n.scan),
            'forward_scan_raw_categories_latest': scan_front_classification(n.scan),
            'forward_scan_window': {
                'sample_count': len(series),
                'fraction_with_near_obstacle': round(near_count/len(series), 3) if series else None,
                'fraction_with_unknown_10pct_or_more': round(unknown_count/len(series), 3) if series else None,
                'fraction_meeting_both_clearance_and_90pct_valid': round(clear_count/len(series), 3) if series else None,
                'valid_fraction_min': min(fractions) if fractions else None,
                'valid_fraction_max': max(fractions) if fractions else None,
                'invalid_beam_total_by_category': invalid_totals,
            },
            'explorer_proposal': n.proposal,
            'physical_control_status': status,
        }
        print(json.dumps(result, indent=2, allow_nan=False))
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    run()
