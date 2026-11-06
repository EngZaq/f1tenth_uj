#!/usr/bin/env python3
"""No-ROS synthetic integration checks for physical steering candidate guard.

Only tests deterministic simulated range arrays. Does not certify that the
LiDAR, steering or brakes work on the actual car.
"""
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'/'exploration'))
sys.path.insert(0, str(ROOT/'src'/'f1tenth_bringup'/'scripts'))
from physical_candidate_planner import select_steering
from physical_path_guard import swept_path_guard


class Scan:
    angle_min = -math.pi
    angle_increment = 2*math.pi/1440
    range_min = 0.
    range_max = 25.

    def __init__(self, value):
        self.ranges = [float(value)]*1440


def main():
    clear = Scan(5.)
    steer, report = select_steering(
        clear, .08, 0., .1, .2, swept_path_guard)
    assert report['ready'] and abs(steer-.08)<1.e-4, report
    blocked = Scan(.35)
    steer, report = select_steering(
        blocked, -.08, 0., .1, .2, swept_path_guard)
    assert not report['ready'] and steer == 0., report
    print('PASS: real swept-geometry guard accepts synthetic clear path,')
    print('      refuses synthetic wall, and stays within +/-0.10 rad.')
    print('NOT proof of physical steering, LiDAR, or braking behavior.')


if __name__ == '__main__':
    main()
