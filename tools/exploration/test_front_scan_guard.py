#!/usr/bin/env python3
"""No ROS, no actuators: regression tests for physical front-scan safety."""
import math
from front_scan_guard import front_scan_guard


def check(ranges, expected, note):
    got = front_scan_guard(-0.4, 0.01, ranges, 0.05, 25.0, 0.8, 0.9)
    assert got['ready'] is expected, (note, got)
    print('PASS:', note, '|', got['reason'], '| valid=%.3f' % got['valid_fraction'])


def main():
    scan = [2.0] * 81
    check(scan, True, 'full valid forward sector permits shadow proposal')
    check([float('inf')] * 81, False, 'all infinite rays reject')
    check([0.0] * 81, False, 'all zero rays reject')
    check([float('nan')] * 81, False, 'all NaN rays reject')
    scan_missing = scan.copy()
    for i in range(20, 40):
        scan_missing[i] = float('inf')
    check(scan_missing, False, 'many missing forward returns reject')
    scan_near = scan.copy()
    scan_near[40] = 0.45
    check(scan_near, False, 'single measured front obstacle rejects')
    scan_invalid = scan.copy()
    scan_invalid[40] = 30.0
    check(scan_invalid, True, 'one invalid beam tolerated when >90% valid')
    scan_margin = scan.copy()
    scan_margin[40] = 0.8
    check(scan_margin, True, 'measured stop-distance equality accepted')
    scan_outside = scan.copy()
    scan_outside[0] = 0.1
    check(scan_outside, True, 'outside ±20deg does not affect forward gate')
    bad = front_scan_guard(-0.4, 0.0, scan, .05, 25.0)
    assert not bad['ready'], bad
    print('PASS: invalid angular geometry rejects')
    empty = front_scan_guard(0.0, .01, [], .05, 25.0)
    assert not empty['ready'], empty
    print('PASS: empty scan rejects')
    print('PASS: 11 physical front-sector gate cases; ROS / motors not involved')


if __name__ == '__main__':
    main()
