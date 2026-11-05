#!/usr/bin/env python3
"""Conservative forward-sector health gate for physical SHADOW exploration.

This is only a software precondition, not a physical collision-avoidance or
braking guarantee. Non-finite/missing/out-of-range readings are *unknown*,
never evidence of empty space. ROS-free to allow independent self-tests.
"""
import math


def front_scan_guard(angle_min, angle_increment, ranges, range_min, range_max,
                     stop_distance=0.8, valid_fraction_min=0.90,
                     front_half_angle_deg=20.0):
    if (not all(math.isfinite(v) for v in
                (angle_min, angle_increment, range_min, range_max,
                 stop_distance, valid_fraction_min, front_half_angle_deg))
            or angle_increment == 0 or range_min < 0 or range_max <= range_min
            or stop_distance <= 0 or not 0.0 < valid_fraction_min <= 1.0
            or front_half_angle_deg <= 0 or not ranges):
        return {'ready': False, 'reason': 'front_invalid_geometry',
                'valid_fraction': 0.0, 'nearest_m': None}
    half_rad = math.radians(front_half_angle_deg)
    selected = [float(r) for i, r in enumerate(ranges)
                if abs(angle_min + angle_increment*i) <= half_rad]
    if len(selected) < 8:
        return {'ready': False, 'reason': 'front_sector_missing',
                'valid_fraction': 0.0, 'nearest_m': None}
    valid = [r for r in selected if
             math.isfinite(r) and range_min <= r <= range_max]
    fraction = len(valid)/len(selected)
    nearest = min(valid) if valid else None
    if fraction < valid_fraction_min:
        return {'ready': False, 'reason': 'front_insufficient_valid_returns',
                'valid_fraction': fraction, 'nearest_m': nearest}
    if nearest is None or nearest < stop_distance:
        return {'ready': False, 'reason': 'front_obstacle',
                'valid_fraction': fraction, 'nearest_m': nearest}
    return {'ready': True, 'reason': 'ok',
            'valid_fraction': fraction, 'nearest_m': nearest}
