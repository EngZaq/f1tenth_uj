#!/usr/bin/env python3
"""Offline, read-only physical LiDAR footprint experiment on a ROS 2 bag.

Compares the previous global forward valid-fraction heuristic against
candidate bicycle-arc *leading-front-vehicle-edge* sampling. The experiment
assesses measured/unknown/near-obstacle proportions only. It is NOT a collision
checker, a deployable local planner, or clearance to actuate the vehicle.
It publishes NO ROS messages and makes NO VESC/steering commands.

A 2D LiDAR cannot see low obstacles, height changes or the rear blind region;
the front-edge samples are not a complete swept-footprint/3D proof.
"""

import argparse
import json
import math
import statistics
from pathlib import Path


WHEELBASE_M = 0.260
BODY_FRONT_X_M = 0.305
LIDAR_X_M = 0.215
HALF_WIDTH_WITH_TEST_MARGIN_M = 0.0985 + 0.050
STEERING_CANDIDATES_RAD = (-0.15, -0.075, 0.0, 0.075, 0.15)


def finite_return(value, range_min, range_max):
    return math.isfinite(value) and range_min <= value <= range_max and value > 0.0


def scan_value_at_bearing(msg, bearing):
    inc = float(msg.angle_increment)
    if not math.isfinite(inc) or abs(inc) < 1e-12 or not msg.ranges:
        return None
    idx = round((bearing - msg.angle_min) / inc)
    if idx < 0 or idx >= len(msg.ranges):
        return None
    angle = float(msg.angle_min) + idx * inc
    # Do not use a distant angular cell to certify this ray.
    if abs(angle - bearing) > abs(inc) * 0.51:
        return None
    return float(msg.ranges[idx])


def bicycle_pose_at_arc_length(distance_m, steering_rad):
    curvature = math.tan(steering_rad) / WHEELBASE_M
    if abs(curvature) < 1e-10:
        return distance_m, 0.0, 0.0
    heading = curvature * distance_m
    return math.sin(heading) / curvature, (1.0 - math.cos(heading)) / curvature, heading


def classify_sample(msg, px, py, obstacle_margin_m=0.030):
    """Ray-state at one hypothetical front-edge point; not occupancy proof."""
    x_laser = px - LIDAR_X_M
    y_laser = py
    radial = math.hypot(x_laser, y_laser)
    bearing = math.atan2(y_laser, x_laser)
    value = scan_value_at_bearing(msg, bearing)
    if value is None or not finite_return(value, msg.range_min, msg.range_max):
        return 'unknown'
    if radial + obstacle_margin_m >= value:
        return 'hit_or_occluded'
    return 'measured_to_sample'


def footprint_probe(msg, steering_rad, preview_m=0.50, step_m=0.05):
    """Sample *front bumper line* along hypothetical forward bicycle arc.

    This omits full vehicle swept polygon, dynamics and scan beam footprint.
    All candidate outputs are diagnostic only.
    """
    counts = {'measured_to_sample': 0, 'unknown': 0, 'hit_or_occluded': 0}
    n = max(1, math.ceil(preview_m / step_m))
    # Test five lateral positions across front vehicle width with extra margin.
    for i in range(n + 1):
        s = preview_m * i / n
        x, y, heading = bicycle_pose_at_arc_length(s, steering_rad)
        c, sn = math.cos(heading), math.sin(heading)
        for lateral in (-1.0, -0.5, 0.0, 0.5, 1.0):
            ly = lateral * HALF_WIDTH_WITH_TEST_MARGIN_M
            px = x + c * BODY_FRONT_X_M - sn * ly
            py = y + sn * BODY_FRONT_X_M + c * ly
            counts[classify_sample(msg, px, py)] += 1
    total = sum(counts.values())
    return {
        **counts,
        'n_probe_points': total,
        'measured_fraction': round(counts['measured_to_sample'] / total, 4),
        'unknown_fraction': round(counts['unknown'] / total, 4),
        'hit_fraction': round(counts['hit_or_occluded'] / total, 4),
        'experimental_status': (
            'measured_on_sampled_front_edge_only'
            if counts['unknown'] == 0 and counts['hit_or_occluded'] == 0
            else 'measured_obstacle_in_probe' if counts['hit_or_occluded'] > 0
            else 'insufficient_visibility'
        ),
    }


def global_forward_gate(msg, fraction=0.90, half_angle_deg=20.0, stop_m=0.80):
    front = []
    for i, value in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        if abs(angle) <= math.radians(half_angle_deg):
            front.append(float(value))
    if len(front) < 8:
        return False, 0.0, None
    measured = [v for v in front if finite_return(v, msg.range_min, msg.range_max)]
    ratio = len(measured) / len(front)
    nearest = min(measured) if measured else None
    return ratio >= fraction and nearest is not None and nearest >= stop_m, ratio, nearest


def self_test():
    class Synthetic:
        angle_min = -math.pi
        angle_increment = 2 * math.pi / 1440
        range_min = 0.0
        range_max = 100.0
        ranges = [3.0] * 1440

    s = Synthetic()
    ok, ratio, dist = global_forward_gate(s)
    assert ok and ratio == 1.0 and dist == 3.0
    probe = footprint_probe(s, 0.0, preview_m=.5)
    assert probe['unknown'] == 0 and probe['hit_or_occluded'] == 0, probe

    s.ranges = [float('inf')] * 1440
    ok, _, _ = global_forward_gate(s)
    assert not ok
    probe = footprint_probe(s, 0.0, preview_m=.5)
    assert probe['unknown'] > 0, probe

    s.ranges = [0.3] * 1440
    probe = footprint_probe(s, 0.0, preview_m=.5)
    assert probe['hit_or_occluded'] > 0, probe

    assert bicycle_pose_at_arc_length(.5, 0.0) == (.5, 0.0, 0.0)
    print('PASS: finite scan, all +Inf, close object and straight bicycle path')
    print('No command publishers; offline diagnostic does not authorize motion.')


def summarize(values):
    if not values:
        return None
    values = sorted(values)
    return {
        'median': round(statistics.median(values), 4),
        'p10': round(values[int(0.10 * (len(values)-1))], 4),
        'p90': round(values[int(0.90 * (len(values)-1))], 4),
    }


def replay(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan

    bag = Path(args.bag).expanduser().resolve()
    meta = bag / 'metadata.yaml'
    if not meta.exists():
        raise FileNotFoundError('No ROS bag metadata.yaml: ' + str(meta))

    # Avoid requiring PyYAML; ROS2 installs may vary.
    from rosbag2_py import StorageOptions, ConverterOptions, SequentialReader
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr',
                         output_serialization_format='cdr')
    )
    topic_types = {x.name: x.type for x in reader.get_all_topics_and_types()}
    if topic_types.get('/scan') != 'sensor_msgs/msg/LaserScan':
        raise RuntimeError('Bag has no /scan LaserScan topic: ' + str(topic_types))

    candidates = {f'{v:+.3f}': {
        'probe_measured_fraction': [], 'probe_unknown_fraction': [],
        'probe_hit_fraction': [], 'sampled_front_edge_all_measured': 0,
        'any_measured_obstacle_in_probe': 0,
    } for v in STEERING_CANDIDATES_RAD}

    n_total = 0
    n_evaluated = 0
    front_valid = []
    nearest = []
    global_ok = 0
    timestamps_first = None
    timestamps_last = None
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic != '/scan':
            continue
        n_total += 1
        if (n_total - 1) % args.every != 0:
            continue
        if args.max_scans and n_evaluated >= args.max_scans:
            break
        msg = deserialize_message(raw, LaserScan)
        n_evaluated += 1
        if timestamps_first is None:
            timestamps_first = timestamp_ns
        timestamps_last = timestamp_ns
        accepted, fraction, front_nearest = global_forward_gate(msg)
        global_ok += int(accepted)
        front_valid.append(fraction)
        if front_nearest is not None:
            nearest.append(front_nearest)
        for steering_rad, summary in zip(STEERING_CANDIDATES_RAD, candidates.values()):
            probe = footprint_probe(msg, steering_rad, preview_m=args.preview_m)
            summary['probe_measured_fraction'].append(probe['measured_fraction'])
            summary['probe_unknown_fraction'].append(probe['unknown_fraction'])
            summary['probe_hit_fraction'].append(probe['hit_fraction'])
            summary['sampled_front_edge_all_measured'] += int(
                probe['experimental_status'] == 'measured_on_sampled_front_edge_only')
            summary['any_measured_obstacle_in_probe'] += int(
                probe['hit_or_occluded'] > 0)

    if not n_evaluated:
        raise RuntimeError('No /scan messages successfully replayed')
    table = {}
    for steering, values in candidates.items():
        table[steering] = {
            'median_measured_probe_fraction': summarize(values['probe_measured_fraction']),
            'median_unknown_probe_fraction': summarize(values['probe_unknown_fraction']),
            'median_hit_probe_fraction': summarize(values['probe_hit_fraction']),
            'all_sampled_edge_points_measured_fraction': round(
                values['sampled_front_edge_all_measured'] / n_evaluated, 4),
            'any_obstacle_in_front_edge_probe_fraction': round(
                values['any_measured_obstacle_in_probe'] / n_evaluated, 4),
        }
    report = {
        'purpose': 'OFFLINE proposal research — no actuation/steering authorization',
        'bag': str(bag),
        'total_scan_messages_read': n_total,
        'evaluated_scan_count': n_evaluated,
        'evaluated_duration_s': round(
            (timestamps_last - timestamps_first)/1e9, 3),
        'old_90_percent_global_gate_accepted_fraction': round(global_ok/n_evaluated, 4),
        'global_forward_valid_fraction': summarize(front_valid),
        'global_front_nearest_finite_m': summarize(nearest),
        'preview_m': args.preview_m,
        'candidate_front_edge_probes': table,
        'geometry_assumptions': {
            'wheelbase_m': WHEELBASE_M,
            'front_body_x_from_rear_axle_m': BODY_FRONT_X_M,
            'half_width_including_provisional_margin_m': HALF_WIDTH_WITH_TEST_MARGIN_M,
            'laser_x_from_rear_axle_m': LIDAR_X_M,
            'laser_y_m': 0.0,
            'laser_yaw_rad': 0.0,
        },
        'limitations': [
            'Reports front-edge sample ray coverage, not a full swept polygon or collision-free path.',
            'No measured physical ground truth, turning radius, steering backlash or motor stopping distance.',
            'Does not test dynamic obstacles, low obstacles or 3D terrain.',
            'A ray marked finite supports measurement only along that single laser ray.',
            'No-return +Inf is UNKNOWN, never assumed open.',
            'Positive findings are exploratory diagnostics, not permission to power the car.',
        ],
    }
    print('=== OFFLINE FOOTPRINT EXPERIMENT ===')
    print('Scans:', n_evaluated, 'recording span:', report['evaluated_duration_s'], 'sec')
    print('Old ±20° rule accepted:', report['old_90_percent_global_gate_accepted_fraction'])
    print('Raw forward validity:', report['global_forward_valid_fraction'])
    print('Candidate  median measured  median unknown   hit sample rate  all sampled measured')
    for steer, data in table.items():
        print('%9s  %15s  %13s  %15.4f  %20.4f' % (
            steer,
            data['median_measured_probe_fraction']['median'],
            data['median_unknown_probe_fraction']['median'],
            data['any_obstacle_in_front_edge_probe_fraction'],
            data['all_sampled_edge_points_measured_fraction'],
        ))
    print('WARNING: This is only coverage of sampled front-edge rays, not safe driving.')
    if args.output:
        out = Path(args.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print('Saved:', out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?', help='Existing ROS 2 bag directory')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--preview-m', type=float, default=.5,
                        help='Lookahead sweep of front edge in meters (0.15-0.7)')
    parser.add_argument('--every', type=int, default=5,
                        help='Evaluate one of every N scans')
    parser.add_argument('--max-scans', type=int, default=0,
                        help='0 = process entire bag')
    parser.add_argument('--output', default='', help='Save report JSON')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.bag:
        parser.error('Provide a ROS bag folder or use --self-test')
    if not 0.15 <= args.preview_m <= 0.7:
        parser.error('--preview-m must be 0.15..0.7')
    if args.every < 1 or args.max_scans < 0:
        parser.error('--every must be >=1 and --max-scans >=0')
    replay(args)


if __name__ == '__main__':
    main()
