#!/usr/bin/env python3
"""Capture the scene that triggered a physical command-mux STOP.

Diagnostics only. No ROS imports and no actuator actions.
The snapshot is evidence about one scan and proposed command, not proof
that the obstacle was correctly modeled or the physical car stopped.
"""
import json
import math
import os
import time
from pathlib import Path


def _number(x, digits=4):
    if x is None:
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return round(v, digits) if math.isfinite(v) else None


def make_snapshot(reason, mode_before, proposal, path, *,
                  scan_reason, nearest_front_m, scan_fresh,
                  odom_fresh, map_fresh, hold_age_s,
                  drive_publishers, drive_subscribers,
                  motor_publishers, motor_subscribers,
                  rf2o_publishers):
    """Immutable human-readable evidence from the stop *tick*."""
    if proposal is None:
        speed = steer = None
    else:
        speed, steer = proposal
    return {
        'recorded_unix_s': round(time.time(), 3),
        'stop_reason': str(reason),
        'mode_before_tick': str(mode_before),
        'proposed_speed_mps': _number(speed),
        'proposed_steer_rad': _number(steer),
        'scan_fresh': bool(scan_fresh),
        'scan_reason': str(scan_reason),
        'nearest_front_m': _number(nearest_front_m),
        'odom_fresh': bool(odom_fresh),
        'map_fresh': bool(map_fresh),
        'auto_hold_age_s': _number(hold_age_s, 3),
        'path_reason': str(path.get('reason', 'MISSING')),
        'path_ready': bool(path.get('ready', False)),
        'nearest_path_obstacle_m': _number(path.get('nearest_obstacle_m')),
        'blocking_return_count': path.get('blocking_return_count'),
        'path_steer_rad': _number(path.get('steer_rad')),
        'path_lookahead_m': _number(path.get('path_lookahead_m')),
        'path_compute_ms': _number(path.get('compute_ms'), 2),
        'closest_blocking_returns': path.get('closest_blocking_returns', [])[:5],
        'swept_known_fraction': _number(path.get('swept_known_fraction')),
        'max_unknown_path_width_m': _number(path.get('max_unknown_path_width_m')),
        'graph': {
            'drive_publishers': int(drive_publishers),
            'drive_subscribers': int(drive_subscribers),
            'motor_publishers': int(motor_publishers),
            'motor_subscribers': int(motor_subscribers),
            'rf2o_publishers': int(rf2o_publishers),
        },
        'note': 'ROS software snapshot only; not a braking measurement.'
    }


def save_snapshot(snapshot, path='~/f1tenth_hw_evidence/last_physical_stop.json'):
    """Write AFTER zero-speed command publication, once per stop transition."""
    dest = Path(path).expanduser()
    dest.parent.mkdir(parents=True, exist_ok=True)
    temporary = dest.with_name(dest.name + '.tmp')
    temporary.write_text(json.dumps(snapshot, indent=2) + '\n')
    os.replace(temporary, dest)
    return str(dest)


def self_test():
    x = make_snapshot(
        'LATCH_STOP_FRONT_SCAN_NOT_CLEAR', 'AUTO_ACTIVE',
        (.175, 0.0),
        {'ready': False, 'reason': 'MEASURED_OBSTACLE_IN_SWEPT_PATH',
         'nearest_obstacle_m': .808, 'blocking_return_count': 72,
         'closest_blocking_returns': [{'distance_m': .808, 'angle_deg': 12.75}]},
        scan_reason='OBSTACLE_NEAR', nearest_front_m=.782,
        scan_fresh=True, odom_fresh=True, map_fresh=True, hold_age_s=.01,
        drive_publishers=1, drive_subscribers=1, motor_publishers=1,
        motor_subscribers=1, rf2o_publishers=1)
    assert x['stop_reason'] == 'LATCH_STOP_FRONT_SCAN_NOT_CLEAR', x
    assert x['nearest_front_m'] == .782 and x['blocking_return_count'] == 72, x
    assert x['closest_blocking_returns'][0]['angle_deg'] == 12.75, x
    print('PASS: November 6 stop scene retained without ROS or motor commands')


if __name__ == '__main__':
    self_test()
