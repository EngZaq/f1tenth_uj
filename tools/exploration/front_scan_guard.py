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


def proposal_front_guard(angle_min, angle_increment, ranges, range_min, range_max,
                         stop_distance=0.8, core_half_angle_deg=5.0,
                         outer_half_angle_deg=20.0,
                         core_valid_fraction_min=0.90):
    """Directional *proposal-only* front test; NOT a motion safety certificate.

    Reject any measured obstacle in +/-outer_half_angle_deg, while avoiding
    the erroneous requirement that all rays of the outer sector hit objects.
    Require good finite return evidence only in a narrow central target band.
    Missing returns elsewhere remain UNKNOWN and must NEVER be converted into
    max-range free measurements. This proposal gate is not sufficient to
    authorize a vehicle-width swept path or stop when obstacles have no echo.

    The independent /drive actuator mux retains its wider conservative test.
    """
    if (not all(math.isfinite(v) for v in (
            angle_min, angle_increment, range_min, range_max,
            stop_distance, core_half_angle_deg, outer_half_angle_deg,
            core_valid_fraction_min)) or not ranges or
            angle_increment <= 0 or
            not 0 < core_half_angle_deg <= outer_half_angle_deg <= 90 or
            range_min < 0 or range_max <= range_min or stop_distance <= 0 or
            not .5 <= core_valid_fraction_min <= 1.0):
        return {'ready': False, 'reason': 'invalid_proposal_geometry',
                'valid_fraction': 0.0, 'nearest_m': None}

    outer = [float(r) for i, r in enumerate(ranges)
             if abs(angle_min + angle_increment*i) <= math.radians(outer_half_angle_deg)]
    if len(outer) < 20:
        return {'ready': False, 'reason': 'outer_forward_sector_missing',
                'valid_fraction': 0.0, 'nearest_m': None}
    measured = [r for r in outer if math.isfinite(r) and range_min < r <= range_max]
    # Any real reported obstacle in the outer sector is still respected.
    if measured and min(measured) < stop_distance:
        return {'ready': False, 'reason': 'measured_forward_obstacle',
                'valid_fraction': len(measured)/len(outer),
                'nearest_m': min(measured)}

    core = front_scan_guard(
        angle_min, angle_increment, ranges, range_min, range_max,
        stop_distance=stop_distance, valid_fraction_min=core_valid_fraction_min,
        front_half_angle_deg=core_half_angle_deg)
    if not core['ready']:
        return {
            'ready': False, 'reason': 'central_' + core['reason'],
            'valid_fraction': core['valid_fraction'],
            'nearest_m': min(measured) if measured else core['nearest_m']
        }
    return {
        'ready': True, 'reason': 'propose_only_center_observed',
        'valid_fraction': core['valid_fraction'],
        'outer_valid_fraction': len(measured)/len(outer),
        'nearest_m': min(measured) if measured else core['nearest_m']
    }
