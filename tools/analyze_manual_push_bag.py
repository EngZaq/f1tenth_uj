#!/usr/bin/env python3
"""Read-only analysis of an existing F1TENTH manual-push ROS 2 bag.

Reports *maximum excursion* as well as final displacement. A return to the
start can otherwise look like no motion. Pose measurements are smoothed in
0.5-second bins for the excursion metric. VESC electrical RPM statistics are
examined separately. This is not ground-truth accuracy calibration.
"""
import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ODOM_TOPICS = {
    '/odom_rf2o',
    '/odom_vesc_observe',
    '/odometry/filtered_shadow',
}
CORE_TOPIC = '/sensors/core'
INTEREST = ODOM_TOPICS | {CORE_TOPIC}


def median(items):
    return float(statistics.median(items))


def yaw_of(quaternion):
    q = quaternion
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(x):
    return math.atan2(math.sin(x), math.cos(x))


def smoothed_series(rows, bin_width=0.5):
    if not rows:
        return []
    start = rows[0]['t']
    groups = defaultdict(list)
    for r in rows:
        groups[int((r['t'] - start) / bin_width)].append(r)
    result = []
    for group in sorted(groups):
        items = groups[group]
        result.append({
            't': median([r['t'] for r in items]),
            'x': median([r['x'] for r in items]),
            'y': median([r['y'] for r in items]),
            'yaw': math.atan2(
                median([math.sin(r['yaw']) for r in items]),
                median([math.cos(r['yaw']) for r in items]),
            ),
        })
    return result


def summarize_odometry(rows):
    if len(rows) < 2:
        return {'samples': len(rows), 'error': 'Not enough data'}
    t0, t1 = rows[0]['t'], rows[-1]['t']
    first = [r for r in rows if r['t'] <= t0 + 3.0] or rows[:1]
    last = [r for r in rows if r['t'] >= t1 - 3.0] or rows[-1:]
    base_x = median([r['x'] for r in first])
    base_y = median([r['y'] for r in first])
    base_yaw = math.atan2(
        median([math.sin(r['yaw']) for r in first]),
        median([math.cos(r['yaw']) for r in first]),
    )
    end_x = median([r['x'] for r in last])
    end_y = median([r['y'] for r in last])
    end_yaw = math.atan2(
        median([math.sin(r['yaw']) for r in last]),
        median([math.cos(r['yaw']) for r in last]),
    )
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    dx, dy = end_x - base_x, end_y - base_y
    bins = smoothed_series(rows)
    excursions = []
    longitudinals = []
    laterals = []
    for b in bins:
        ex, ey = b['x'] - base_x, b['y'] - base_y
        excursions.append(math.hypot(ex, ey))
        longitudinals.append(c * ex + s * ey)
        laterals.append(-s * ex + c * ey)
    speeds = [abs(r['vx']) for r in rows if math.isfinite(r['vx'])]
    return {
        'samples': len(rows),
        'duration_s': round(t1 - t0, 3),
        'first_3s_reference_xy_m': [round(base_x, 5), round(base_y, 5)],
        'last_3s_xy_m': [round(end_x, 5), round(end_y, 5)],
        'net_xy_m': round(math.hypot(dx, dy), 5),
        'net_forward_m': round(c * dx + s * dy, 5),
        'net_lateral_m': round(-s * dx + c * dy, 5),
        'net_yaw_deg': round(math.degrees(wrap(end_yaw - base_yaw)), 4),
        'peak_excursion_from_start_m_smoothed': round(max(excursions), 5),
        'smoothed_forward_range_m': [
            round(min(longitudinals), 5),
            round(max(longitudinals), 5),
        ],
        'smoothed_lateral_range_m': [
            round(min(laterals), 5),
            round(max(laterals), 5),
        ],
        'max_abs_reported_linear_x_mps': round(max(speeds), 5) if speeds else None,
        'note': 'Smooth max excursion is NOT traveled distance or ground truth.',
    }


def summarize_core(rows):
    if not rows:
        return {'samples': 0, 'error': 'No VESC core messages'}
    erpm = [r['erpm'] for r in rows if math.isfinite(r['erpm'])]
    return {
        'samples': len(rows),
        'erpm_min': round(min(erpm), 2) if erpm else None,
        'erpm_max': round(max(erpm), 2) if erpm else None,
        'peak_abs_erpm': round(max(abs(v) for v in erpm), 2) if erpm else None,
        'samples_abs_erpm_ge_50': sum(abs(v) >= 50 for v in erpm),
        'samples_abs_erpm_ge_400': sum(abs(v) >= 400 for v in erpm),
        'input_voltage_range_v': [
            round(min(r['voltage'] for r in rows), 2),
            round(max(r['voltage'] for r in rows), 2),
        ],
        'fault_codes': sorted(set(r['fault'] for r in rows)),
        'note': 'A manually pushed unpowered or low-speed motor may have near-zero ERPM. This does not validate encoder scale.',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='ROS 2 bag directory containing metadata.yaml')
    parser.add_argument('--output', default='', help='Optional JSON report path')
    args = parser.parse_args()
    bag = Path(args.bag).expanduser().resolve()
    metadata_path = bag / 'metadata.yaml'
    if not metadata_path.is_file():
        parser.error('metadata.yaml not found in: ' + str(bag))

    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    storage_id = metadata['rosbag2_bagfile_information'].get(
        'storage_identifier', 'sqlite3'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    available = sorted(types)
    msg_classes = {
        t: get_message(types[t]) for t in INTEREST if t in types
    }
    observations = {t: [] for t in INTEREST}

    while reader.has_next():
        topic, raw, time_ns = reader.read_next()
        if topic not in msg_classes:
            continue
        msg = deserialize_message(raw, msg_classes[topic])
        t = time_ns / 1e9
        if topic == CORE_TOPIC:
            observations[topic].append({
                't': t,
                'erpm': float(msg.state.speed),
                'voltage': float(msg.state.voltage_input),
                'fault': int(msg.state.fault_code),
            })
        else:
            p = msg.pose.pose.position
            observations[topic].append({
                't': t,
                'x': float(p.x), 'y': float(p.y),
                'yaw': yaw_of(msg.pose.pose.orientation),
                'vx': float(msg.twist.twist.linear.x),
            })

    report = {
        'bag': str(bag),
        'storage_id': storage_id,
        'available_topics': available,
        'odometry': {
            t: summarize_odometry(observations[t])
            for t in sorted(ODOM_TOPICS)
        },
        'vesc_core': summarize_core(observations[CORE_TOPIC]),
        'limitations': [
            'No independent tape-measured physical ground truth is in this bag.',
            'First and last 3-second windows are treated as stationary reference periods.',
            'Peak excursion uses 0.5-second median bins to reduce scan-matching noise.',
            'Fused EKF and RF2O are correlated; their agreement does not prove accuracy.',
            'Without reliable moving ERPM, VESC odometry cannot yet be calibrated.',
            'No commands are published by this script.',
        ],
    }
    print('=== F1TENTH MANUAL PUSH BAG ANALYSIS ===')
    print('Bag:', bag)
    print('Storage:', storage_id)
    for topic in sorted(ODOM_TOPICS):
        print('\n', topic)
        for key, value in report['odometry'][topic].items():
            if key not in ('note',):
                print('  %s: %s' % (key, value))
    print('\n /sensors/core')
    for key, value in report['vesc_core'].items():
        if key not in ('note',):
            print('  %s: %s' % (key, value))
    print('\nInterpretation: max excursion != actual travel distance.')
    print('Compare the measured rear-axle displacement with net_forward_m.')
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print('Saved:', output)


if __name__ == '__main__':
    main()
