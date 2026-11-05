#!/usr/bin/env python3
"""Evaluate sampled 2D swept vehicle footprints from a recorded LaserScan bag.

OFFLINE RESEARCH ONLY: no ROS publishers, motor, servo, arm, or VESC commands.
Unlike front-bumper probes, this samples the *interior and boundary* of the car
body at multiple bicycle-model future poses. No-return is always UNKNOWN.
A sample with finite range beyond it is a *measured radial line*, NOT proof
that the continuous vehicle footprint or the 3D world is collision-free.

An initial physical footprint is excluded from forward-motion statistics, as
its region is occupied by our own car. This is an explicit assumption, not
proof it is or remains safe. This script DOES NOT select a motion command or
authorize physical driving. Real E-stop, braking, electrical and mechanical
calibration remain open.
"""
import argparse
import json
import math
import statistics
from pathlib import Path

# Recorded/measured vehicle configuration. Some external values unverified.
WHEELBASE_M = 0.260
BASE_REAR_M = -0.040
BASE_FRONT_M = 0.305
BODY_HALF_WIDTH_M = 0.0985
GEOMETRY_MARGIN_M = 0.050  # provisional diagnostic, not calibrated clearance
LIDAR_BASE_X_M = 0.215
LIDAR_BASE_Y_M = 0.0

CANDIDATES = (-0.150, -0.075, 0.0, 0.075, 0.150)
CLASSIFICATIONS = ('observed_to_point', 'unknown_ray',
                   'near_or_occluded_return', 'already_under_body')


def bicycle_pose(s, steering):
    k = math.tan(steering) / WHEELBASE_M
    if abs(k) < 1e-10:
        return s, 0.0, 0.0
    th = k * s
    return math.sin(th) / k, (1.0 - math.cos(th)) / k, th


def float_ray(msg, angle):
    if not msg.ranges or not math.isfinite(msg.angle_increment) or msg.angle_increment == 0:
        return None
    idx = round((angle - float(msg.angle_min)) / float(msg.angle_increment))
    if not 0 <= idx < len(msg.ranges):
        return None
    actual = float(msg.angle_min) + float(msg.angle_increment) * idx
    if abs(actual - angle) > abs(float(msg.angle_increment)) * .51:
        return None
    value = float(msg.ranges[idx])
    if not (math.isfinite(value) and
            max(0.0, float(msg.range_min)) < value <= float(msg.range_max)):
        return None
    return value


def label_sample(msg, bx, by, margin_m=0.03):
    # This region was physically occupied by our own car at time t=0.
    if (BASE_REAR_M <= bx <= BASE_FRONT_M and
            abs(by) <= BODY_HALF_WIDTH_M + GEOMETRY_MARGIN_M):
        return 'already_under_body'
    lx = bx - LIDAR_BASE_X_M
    ly = by - LIDAR_BASE_Y_M
    d = math.hypot(lx, ly)
    if d <= 0.02:
        return 'unknown_ray'
    measured = float_ray(msg, math.atan2(ly, lx))
    if measured is None:
        return 'unknown_ray'
    if measured <= d + margin_m:
        return 'near_or_occluded_return'
    return 'observed_to_point'


def spaced(lo, hi, approx_step):
    n = max(1, math.ceil((hi-lo)/approx_step))
    return [lo + (hi-lo)*i/n for i in range(n+1)]


def samples_for_path(steering, preview_m, step_m, xy_step_m):
    """Plan sampled future *body surface* and corridor points in base frame."""
    half = BODY_HALF_WIDTH_M + GEOMETRY_MARGIN_M
    # Here the footprint is the inflated body rectangle rather than just the bumper.
    body_x = spaced(BASE_REAR_M-GEOMETRY_MARGIN_M,
                    BASE_FRONT_M+GEOMETRY_MARGIN_M, xy_step_m)
    body_y = spaced(-half, half, xy_step_m)
    forwards = spaced(0.0, preview_m, step_m)
    for s in forwards:
        px, py, yaw = bicycle_pose(s, steering)
        ct, st = math.cos(yaw), math.sin(yaw)
        for local_x in body_x:
            for local_y in body_y:
                yield (px + ct*local_x - st*local_y,
                       py + st*local_x + ct*local_y)


def analyze_candidate(msg, test_samples):
    counts = {k: 0 for k in CLASSIFICATIONS}
    for bx, by in test_samples:
        counts[label_sample(msg, bx, by)] += 1
    effective = sum(v for k, v in counts.items() if k != 'already_under_body')
    return {
        'counts': counts,
        'newly_sampled_points': effective,
        'fraction_observed': counts['observed_to_point']/effective if effective else None,
        'fraction_unknown': counts['unknown_ray']/effective if effective else None,
        'fraction_hit_or_occluded': counts['near_or_occluded_return']/effective if effective else None,
        'no_unmeasured_sampled_points': (
            effective > 0 and counts['unknown_ray'] == 0),
        'no_observed_close_samples': (
            effective > 0 and counts['near_or_occluded_return'] == 0),
    }


class Dummy:
    range_min = 0.0
    range_max = 100.0
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / 1440
    ranges = [3.0] * 1440


def self_test():
    pts = list(samples_for_path(0.0, .35, .05, .05))
    assert len(pts) > 100
    assert bicycle_pose(.3, 0) == (.3, 0, 0)
    assert abs(bicycle_pose(.3, .15)[1]) > .005
    m = Dummy()
    good = analyze_candidate(m, pts)
    # Even a perfect synthetic 360-degree 3 m scan cannot resolve a sample
    # located within the 2 cm LiDAR-origin exclusion disk. Such points must
    # remain UNKNOWN rather than being incorrectly counted as clear.
    assert good['fraction_observed'] > 0.99, good
    assert good['counts']['unknown_ray'] >= 1, good
    assert not good['no_unmeasured_sampled_points'], good
    assert good['no_observed_close_samples'], good
    assert good['counts']['already_under_body'] > 0, good
    m.ranges = [float('inf')] * 1440
    bad = analyze_candidate(m, pts)
    assert bad['fraction_unknown'] > 0.9, bad
    assert not bad['no_unmeasured_sampled_points'], bad
    m.ranges = [.15] * 1440
    obstacle = analyze_candidate(m, pts)
    assert obstacle['fraction_hit_or_occluded'] > 0.8, obstacle
    assert not obstacle['no_observed_close_samples'], obstacle
    print('PASS: 2D bicycle geometry, full-body samples, unknown rays, near returns')
    print('No ROS publishers, no actuation, no collision-free-path certification.')


def stats(vals):
    if not vals:
        return None
    values = sorted(vals)
    return {
        'median': round(statistics.median(values), 4),
        'p10': round(values[int(.1*(len(values)-1))], 4),
        'p90': round(values[int(.9*(len(values)-1))], 4),
    }


def run(args):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    bag = Path(args.bag).expanduser().resolve()
    if not (bag/'metadata.yaml').is_file():
        raise FileNotFoundError('Missing finalized metadata.yaml in '+str(bag))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                   output_serialization_format='cdr'))
    tt = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if tt.get('/scan') != 'sensor_msgs/msg/LaserScan':
        raise ValueError('Input bag has no /scan LaserScan')
    samples = {str(v): list(samples_for_path(
        v, args.preview_m, args.path_step_m, args.xy_step_m))
        for v in CANDIDATES}
    results = {str(v): {'known': [], 'unknown': [], 'near': [],
                        'no_unknown_count': 0, 'no_near_count': 0,
                        'all_probe_clear_count': 0}
               for v in CANDIDATES}
    read = examined = 0
    ts0 = ts1 = None
    while reader.has_next():
        topic, blob, timestamp = reader.read_next()
        if topic != '/scan':
            continue
        read += 1
        if (read-1) % args.every:
            continue
        if args.max_scans and examined >= args.max_scans:
            break
        scan = deserialize_message(blob, LaserScan)
        examined += 1
        ts0 = timestamp if ts0 is None else ts0
        ts1 = timestamp
        for key, points in samples.items():
            res = analyze_candidate(scan, points)
            data = results[key]
            if res['fraction_observed'] is not None:
                data['known'].append(res['fraction_observed'])
                data['unknown'].append(res['fraction_unknown'])
                data['near'].append(res['fraction_hit_or_occluded'])
            data['no_unknown_count'] += int(res['no_unmeasured_sampled_points'])
            data['no_near_count'] += int(res['no_observed_close_samples'])
            data['all_probe_clear_count'] += int(
                res['no_unmeasured_sampled_points'] and
                res['no_observed_close_samples'])
    if examined == 0:
        raise RuntimeError('No recorded /scan frames')
    output = {}
    for key, value in results.items():
        output[key] = {
            'median_observed_fraction': stats(value['known']),
            'median_unknown_fraction': stats(value['unknown']),
            'median_near_or_occluded_fraction': stats(value['near']),
            'fraction_snapshots_all_sampled_points_measured': round(
                value['no_unknown_count']/examined, 4),
            'fraction_snapshots_no_near_returns_at_samples': round(
                value['no_near_count']/examined, 4),
            'fraction_snapshots_all_probe_points_measured_and_clear': round(
                value['all_probe_clear_count']/examined, 4),
            'sample_points_per_candidate_per_scan': len(samples[key]),
        }
    report = {
        'purpose': 'OFFLINE sweep measurement coverage; NOT motion authorization',
        'bag': str(bag),
        'scans_evaluated': examined,
        'recorded_scan_span_s': round((ts1-ts0)/1e9, 3),
        'preview_distance_m': args.preview_m,
        'path_step_m': args.path_step_m,
        'xy_grid_step_m': args.xy_step_m,
        'candidate_steering_rad': list(CANDIDATES),
        'results': output,
        'caveats': [
            'Sampled grid is an approximation of the continuous swept footprint, not proof of free space.',
            'Unmeasured LaserScan rays always remain UNKNOWN, never treated as free.',
            'The region initially under the robot is excluded from new-sweep statistics, not proven safe.',
            'A point with a farther finite LiDAR return is only measured along one radial beam.',
            'No low-obstacle, camera, pedestrians, moving objects, curb or vertical profile validation.',
            'Steering, stopping, electronic failsafe, power and E-stop are unverified.',
            'No executable command or ROS publisher; actuation must remain disabled.',
        ],
    }
    print('=== OFFLINE SAMPLED VEHICLE-SWEEP COMPARISON ===')
    print('scans', examined, '| span', report['recorded_scan_span_s'],
          's | preview', args.preview_m, 'm')
    print('steer(rad) | median observed | median unknown | median near | all sampled clear')
    for key, v in output.items():
        print('%+8.3f | %15.4f | %14.4f | %11.4f | %s' % (
            float(key),
            v['median_observed_fraction']['median'],
            v['median_unknown_fraction']['median'],
            v['median_near_or_occluded_fraction']['median'],
            v['fraction_snapshots_all_probe_points_measured_and_clear'],
        ))
    print('WARNING: All values are diagnostic; no physical motion clearance.')
    if args.output:
        p = Path(args.output).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print('Saved:', p)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', nargs='?')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--preview-m', type=float, default=.5)
    parser.add_argument('--every', type=int, default=5)
    parser.add_argument('--max-scans', type=int, default=0)
    parser.add_argument('--path-step-m', type=float, default=.05)
    parser.add_argument('--xy-step-m', type=float, default=.05)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.bag:
        parser.error('Specify existing ROS bag directory')
    if not 0.15 <= args.preview_m <= .70:
        parser.error('preview distance must be between .15m and .70m')
    if args.every < 1 or args.max_scans < 0:
        parser.error('invalid scan subsampling')
    if not .025 <= args.path_step_m <= .1 or not .025 <= args.xy_step_m <= .1:
        parser.error('step sizes must be .025m to .10m')
    run(args)


if __name__ == '__main__':
    main()
