#!/usr/bin/env python3
"""ROS-free selection of feasible physical explorer steering proposals.

The supplied guard checks actual swept car geometry for each candidate.
A proposal that passes is NOT certified safe. The independent physical mux
continues to evaluate its own path gate, freshness, hold and ownership checks.
This selector never publishes an actuator command.
"""
import math
import time


def select_steering(scan, target_steer, previous_steer, steering_limit,
                    requested_speed, guard_fn, lookahead_m=1.05,
                    max_compute_ms=65.0):
    """Return (steer, details). No feasible candidates -> zero steering/speed.

    A longer lookahead than the mux's 0.8 m looks ahead for corners. This is
    not a braking-distance calculation and does not validate actuator response.
    """
    started = time.perf_counter()
    values = (target_steer, previous_steer, steering_limit,
              requested_speed, lookahead_m, max_compute_ms)
    if (not all(math.isfinite(float(x)) for x in values)
            or not .01 <= steering_limit <= .42
            or not 0 <= requested_speed <= 1.5
            or not .8 <= lookahead_m <= 1.5
            or not 10. <= max_compute_ms <= 500.):
        return 0., {'ready': False, 'reason': 'INVALID_PLANNER_PARAMETERS',
                    'candidates': []}

    target = max(-steering_limit, min(steering_limit, target_steer))
    probes = [target, previous_steer,
              -steering_limit, -.5*steering_limit, 0.,
              .5*steering_limit, steering_limit]
    candidates = []
    seen = set()
    for probe in probes:
        steer = max(-steering_limit, min(steering_limit, probe))
        key = round(steer, 4)
        if key in seen:
            continue
        seen.add(key)
        reply = guard_fn(
            scan, requested_speed, steer,
            lookahead_travel_m=lookahead_m)
        candidates.append({
            'steer_rad': round(steer, 4),
            'ready': bool(reply.get('ready', False)),
            'reason': reply.get('reason', 'NO_REASON'),
            'blocking_return_count': reply.get('blocking_return_count'),
            'max_unknown_path_width_m': reply.get('max_unknown_path_width_m'),
        })
        elapsed_ms = 1000.*(time.perf_counter()-started)
        if elapsed_ms >= max_compute_ms:
            return 0., {
                'ready': False, 'reason': 'PLANNER_COMPUTE_BUDGET_EXCEEDED',
                'compute_ms': round(elapsed_ms, 2),
                'compute_budget_ms': max_compute_ms,
                'candidate_count': len(candidates),
                'candidates': candidates,
            }
    feasible = [q for q in candidates if q['ready']]
    if not feasible:
        return 0., {
            'ready': False, 'reason': 'NO_FEASIBLE_STEER_WITHIN_LIMITS',
            'lookahead_m': lookahead_m,
            'compute_ms': round(1000.*(time.perf_counter()-started),2),
            'candidate_count': len(candidates),
            'target_steer_rad': round(target_steer, 4),
            'steering_limit_rad': steering_limit,
            'candidates': candidates,
        }
    best = min(feasible, key=lambda q: (
        abs(q['steer_rad']-target)*3 +
        abs(q['steer_rad']-previous_steer)*.15,
        abs(q['steer_rad'])))
    return best['steer_rad'], {
        'ready': True, 'reason': 'PATH_CANDIDATE_AVAILABLE_NOT_CERTIFIED',
        'lookahead_m': lookahead_m,
        'compute_ms': round(1000.*(time.perf_counter()-started),2),
        'candidate_count': len(candidates),
        'target_steer_rad': round(target_steer, 4),
        'steering_limit_rad': steering_limit,
        'selected_steer_rad': best['steer_rad'],
        'candidates': candidates,
    }


def self_test():
    class Scan:
        pass
    scan = Scan()
    def open_guard(_scan, _speed, steer, **_):
        return {'ready': True, 'reason': 'TEST'}
    s, data = select_steering(scan, .08, 0., .1, .2, open_guard)
    assert data['ready'] and abs(s-.08)<1e-4
    def blocked_positive(_scan, _speed, steer, **_):
        return {'ready': steer < -.01, 'reason': 'TEST_GEOMETRY'}
    s, data = select_steering(scan, .07, 0., .1, .2, blocked_positive)
    assert data['ready'] and s < 0.
    def blocked_all(_scan, _speed, steer, **_):
        return {'ready': False, 'reason': 'TEST_BLOCK'}
    s, data = select_steering(scan, .08, 0., .1, .2, blocked_all)
    assert s == 0. and not data['ready'] and len(data['candidates']) >= 5
    s, data = select_steering(scan, float('nan'), 0., .1, .2, open_guard)
    assert s == 0. and data['reason'] == 'INVALID_PLANNER_PARAMETERS'
    def slow_guard(_scan,_speed,steer,**_):
        time.sleep(.015)
        return {'ready': True, 'reason': 'TEST'}
    s, data = select_steering(scan, .08, 0., .1, .2, slow_guard,
                              max_compute_ms=10.0)
    assert s == 0. and data['reason'] == 'PLANNER_COMPUTE_BUDGET_EXCEEDED', data
    print('PASS: reachable target, alternate feasible steer, all-blocked stop, invalid-input stop')


if __name__ == '__main__':
    self_test()
