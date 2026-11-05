#!/usr/bin/env python3
"""Measure intermittent versus persistent LiDAR no-returns, without actuation.

STATIONARY DIAGNOSTIC ONLY. Temporal aggregation without compensating robot
motion and occlusions is NOT an approved collision checker. Any-finite data
across multiple frames is reported as *measurement coverage* ONLY, not free
space or safe autonomous-driving permission.
"""
import argparse
import collections
import json
import math
import statistics
import time
from pathlib import Path


def fraction_valid_in_window(frames):
    """ROS-free: per-angle fraction with >=1 finite measurement in window."""
    if not frames:
        return None
    n = len(frames[0])
    if n == 0 or any(len(f) != n for f in frames):
        raise ValueError('inconsistent scan length')
    coverage = [any(f[i] for f in frames) for i in range(n)]
    missing = [not x for x in coverage]
    run = longest_true_run(missing)
    return len(coverage) and sum(coverage) / len(coverage), run


def longest_true_run(flags):
    best = cur = 0
    for f in flags:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    return best


def test():
    a = [True, False, False, True]
    b = [True, True, False, True]
    c = [True, True, True, True]
    assert fraction_valid_in_window([a]) == (0.5, 2)
    assert fraction_valid_in_window([a, b]) == (0.75, 1)
    assert fraction_valid_in_window([a, b, c]) == (1.0, 0)
    assert longest_true_run([True, True, False, True]) == 2
    print('PASS: window measurement coverage and contiguous unknown-arc tests')
    print('No ROS topics published; this is not a collision-avoidance system.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    if args.self_test:
        test()
        return
    if not 5 <= args.duration <= 120:
        parser.error('duration must be 5–120 s')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class Monitor(Node):
        def __init__(self):
            super().__init__('f1tenth_temporal_coverage_monitor')
            self.windows = (1, 2, 3, 5, 10)
            self.history = collections.deque(maxlen=max(self.windows))
            self.metrics = {n: {'cover': [], 'missing_run_deg': []}
                            for n in self.windows}
            self.frame_counts = 0
            self.angles = None
            self.valid_geometry = True
            self.last_header = None
            self.gaps_0 = []
            self.last_stamp_ns = None
            self.header_nonmonotonic = 0
            self.sub = self.create_subscription(
                LaserScan, '/scan', self.callback, qos_profile_sensor_data)

        def callback(self, msg):
            stamp_ns = int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            if self.last_stamp_ns is not None and stamp_ns < self.last_stamp_ns:
                self.header_nonmonotonic += 1
            self.last_stamp_ns = stamp_ns
            positions = [i for i in range(len(msg.ranges))
                         if abs(msg.angle_min + i*msg.angle_increment)
                            <= math.radians(20)]
            if not positions:
                self.valid_geometry = False
                return
            if self.angles is None:
                self.angles = positions
            elif positions != self.angles:
                self.valid_geometry = False
                self.history.clear()
                self.angles = positions
            mask = [math.isfinite(float(msg.ranges[i])) and
                    msg.range_min <= float(msg.ranges[i]) <= msg.range_max
                    for i in positions]
            self.history.append(mask)
            self.frame_counts += 1
            self.last_header = (float(msg.range_min), float(msg.range_max),
                                float(msg.angle_increment))
            resolution = abs(math.degrees(float(msg.angle_increment)))
            for n in self.windows:
                if len(self.history) < n:
                    continue
                coverage, gap = fraction_valid_in_window(list(self.history)[-n:])
                self.metrics[n]['cover'].append(coverage)
                self.metrics[n]['missing_run_deg'].append(gap*resolution)

        def summary(self):
            out = {}
            for n in self.windows:
                m = self.metrics[n]
                coverage = m['cover']
                gaps = m['missing_run_deg']
                out[str(n)] = {
                    'evaluated_frames': len(coverage),
                    'measurement_coverage_median': (
                        round(statistics.median(coverage), 4) if coverage else None),
                    'measurement_coverage_p10': (
                        round(sorted(coverage)[int(.1*(len(coverage)-1))],4)
                        if coverage else None),
                    'fraction_windows_at_least_90pct_covered': (
                        round(sum(x >= .9 for x in coverage)/len(coverage), 4)
                        if coverage else None),
                    'unmeasured_contiguous_arc_p95_deg': (
                        round(sorted(gaps)[math.ceil(.95*len(gaps))-1], 3)
                        if gaps else None),
                }
            return {
                'test': 'Stationary angular measurement coverage across scan windows',
                'frames': self.frame_counts,
                'geometry_consistent': self.valid_geometry,
                'header_timestamps_nonmonotonic': self.header_nonmonotonic,
                'last_range_min_max_and_angle_increment': self.last_header,
                'windows': out,
                'interpretation': ('If coverage rises with window length, many '
                                   'no-returns are intermittent at fixed bearings. '
                                   'Persistent unknowns remain even after multiple scans.'),
                'warning': ('Temporal coverage is NOT safe driving clearance. '
                            'Vehicle movement and dynamic obstacles invalidate '
                            'uncompensated historical measurements.'),
            }

    rclpy.init()
    node = Monitor()
    print('STATIONARY ONLY, READ-ONLY: collecting LiDAR no-return persistence')
    try:
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
        result = node.summary()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print('\n=== TEMPORAL LIDAR MEASUREMENT COVERAGE (NOT CLEARANCE) ===')
    print('frames:', result['frames'], 'angle geometry consistent:',
          result['geometry_consistent'])
    print('window | median coverage | >=90% windows | p95 longest missing arc')
    for window, summary in result['windows'].items():
        print('%6s | %14s | %12s | %s deg' %
              (window, summary['measurement_coverage_median'],
               summary['fraction_windows_at_least_90pct_covered'],
               summary['unmeasured_contiguous_arc_p95_deg']))
    print('WARNING:', result['warning'])
    if args.output:
        path = Path(args.output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        print('Saved:', path)
    if result['frames'] < 10 or not result['geometry_consistent']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
