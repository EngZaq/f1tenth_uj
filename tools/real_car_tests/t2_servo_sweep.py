#!/usr/bin/env python3
"""T2 servo sweep (F-02, F-06, F-17): servo value -> wheel angles, range, backlash, linear fit.

Stack: t2_servo_only.launch.py (vesc_driver alone). Motor phases DISCONNECTED; the tool also
reads back that every vesc_driver motor limit is 0 and that no mux / converter is running.
The tool is the only /commands/servo/position publisher. Line input (both hands are free to
measure); servo commands are slew-limited at 50 Hz.

Rules (Moatasem, 14 Nov; two servos already burned on this car), values in vehicle_params.yaml t2:
- Start inside the approved range (vehicle_params.yaml t2, 0.10-0.60 since 19 Nov) in 0.02 steps
  from the centre (vehicle_params steering.servo_offset: 0.320 on 19 Nov, 0.330 since 20 Nov), then 0.01.
- Every new outward point is first visited for at most pulse_s (< 1 s) and the operator answers
  "binding / noise?". Yes -> that side ends at the previous point and the tool never goes back.
  No -> the point is confirmed and may be held for measuring (back to centre after max_hold_s).
  e -> no binding seen, but that side ends at the previous point (recorded as ended, not binding).
- A point outside the vesc_driver hard clip needs a relaunch with a clip just outside it
  (that relaunch is the approval). The operator can instead end that side there (e). Both sides
  run before any pause; the tool then saves and prints one relaunch command for both, then --resume.
- Any exit (q, Ctrl+C, SSH loss, error) slews back to the centre first.
- --extend-right DIR (19 Nov): continues the RIGHT side of an earlier run beyond its last free point
  in extend_step (0.02) steps, same 0.8 s visit + y/n/e prompt at every point, up to the driver clip
  (relaunch to go further) and the VESC maximum 1.0. Rows are appended to DIR/points.csv (note
  "extend_right"), fit.json is refitted (old one kept as fit_before_extend_right.json), and
  DIR/extend_right.json records the last free point, the stop and the proposed right driving clip
  (= last free point, one step before the stop). The left side is never commanded.

Wheel angles in degrees, + = wheel turned LEFT (servo below the centre steers left: gain -0.50).

  t2_servo_sweep.py                  new sweep
  t2_servo_sweep.py --resume DIR     continue after relaunching with a wider clip
  t2_servo_sweep.py --extend-right DIR   right-side limit extension, linked to that run
  t2_servo_sweep.py --fit DIR        fit only (offline)
  t2_servo_sweep.py --self-test
"""
import argparse
import csv
import json
import math
import os
import select
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

FIELDS = ['wall', 'kind', 'side', 'servo', 'servo_seen', 'left_deg', 'right_deg', 'binding', 'note']
FORBIDDEN_NODES = {'physical_drive_mux', 'ackermann_to_vesc_node', 't1b_stop_converter',
                   'real_car_test_runner'}
ZERO_LIMITS = ['speed_min', 'speed_max', 'current_min', 'current_max', 'brake_min', 'brake_max',
               'duty_cycle_min', 'duty_cycle_max', 'position_min', 'position_max']
RATE_HZ = 50.0
EXT_TAG = 'extend_right'
VESC_SERVO_MAX = 1.0    # VESC firmware truncates the servo output to 0..1 (pwm_servo.c)


# ---------------------------------------------------------------- pure logic

def next_point(side, edge, rules):
    """Next outward servo value after the confirmed edge. side 'left' = decreasing servo."""
    lo, hi = rules['approved_servo_min'], rules['approved_servo_max']
    inside, outside = rules['inside_step'], rules['outside_step']
    if side == 'left':
        if edge - inside >= lo - 1e-9:
            return round(edge - inside, 3)
        return round(lo, 3) if edge > lo + 1e-9 else round(edge - outside, 3)
    if edge + inside <= hi + 1e-9:
        return round(edge + inside, 3)
    return round(hi, 3) if edge < hi - 1e-9 else round(edge + outside, 3)


def clip_needed(target, clip_lo, clip_hi, margin):
    """None if the driver clip already allows target, else the clip pair to relaunch with."""
    if clip_lo - 1e-9 <= target <= clip_hi + 1e-9:
        return None
    if target < clip_lo:
        return round(target - margin, 3), clip_hi
    return clip_lo, round(target + margin, 3)


def merge_clip(current, need):
    """Widest clip covering both pending relaunch requests (None = no request yet)."""
    if current is None:
        return need
    return round(min(current[0], need[0]), 3), round(max(current[1], need[1]), 3)


def equivalent_angle(left_deg, right_deg):
    """Bicycle-model angle (rad) from the two front wheels: cot d = (cot dL + cot dR) / 2."""
    a, b = math.radians(left_deg), math.radians(right_deg)
    if a * b > 0 and abs(a) > 1e-3 and abs(b) > 1e-3:
        return math.atan(2.0 / (1.0 / math.tan(a) + 1.0 / math.tan(b)))
    return 0.5 * (a + b)


def linear_fit(xs, ys):
    """ys = offset + gain * xs; returns (offset, gain, rms residual in ys units)."""
    n = len(xs)
    if n < 2:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    gain = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    offset = my - gain * mx
    rms = math.sqrt(sum((y - offset - gain * x) ** 2 for x, y in zip(xs, ys)) / n)
    return offset, gain, rms


def fit_rows(rows, centre):
    """Servo -> angle results from measured rows (binding rows excluded)."""
    meas = [r for r in rows if r['kind'] == 'measure' and r['left_deg'] not in ('', None)]
    pts = [(float(r['servo']), equivalent_angle(float(r['left_deg']), float(r['right_deg'])), r)
           for r in meas]
    out = {'points': len(pts)}
    if len(pts) < 3:
        out['error'] = 'need >= 3 measured points'
        return out
    full = linear_fit([d for _, d, _ in pts], [s for s, _, _ in pts])
    if full:
        off, g, rms = full
        out.update(offset=round(off, 4), gain=round(g, 4), rms_servo=round(rms, 5),
                   rms_deg=round(math.degrees(rms / abs(g)), 3) if g else None,
                   centre=centre, offset_disagrees_with_centre=abs(off - centre) > 0.005)
    for name, sel in (('left', lambda d: d >= -1e-3), ('right', lambda d: d <= 1e-3)):
        sub = [(s, d) for s, d, _ in pts if sel(d)]
        f = linear_fit([d for _, d in sub], [s for s, _ in sub])
        if f:
            out['%s_offset' % name], out['%s_gain' % name] = round(f[0], 4), round(f[1], 4)
            out['%s_rms_servo' % name] = round(f[2], 5)
    angles = [d for _, d, _ in pts]
    out['max_left_rad'] = round(max(angles), 4)
    out['max_right_rad'] = round(min(angles), 4)
    out['max_left_servo'] = min((s for s, d, _ in pts if d == max(angles)), default=None)
    out['max_right_servo'] = max((s for s, d, _ in pts if d == min(angles)), default=None)
    c = {r['note']: equivalent_angle(float(r['left_deg']), float(r['right_deg']))
         for r in meas if r['note'] in ('centre_from_left', 'centre_from_right')}
    if len(c) == 2:
        dd = c['centre_from_left'] - c['centre_from_right']
        out['backlash_deg'] = round(math.degrees(dd), 3)
        if full and full[1]:
            out['backlash_servo'] = round(abs(dd * full[1]), 4)
    out['binding'] = [dict(servo=float(r['servo']), side=r['side'], note=r['note'])
                      for r in rows if r['kind'] == 'pulse' and r['binding'] == 'yes']
    out['ended_by_operator'] = [dict(servo=float(r['servo']), side=r['side'], note=r['note'])
                                for r in rows if r['binding'] == 'end']
    for side, pick in (('left', min), ('right', max)):
        pul = [r for r in rows if r['side'] == side and r['kind'] == 'pulse']
        free = [float(r['servo']) for r in pul if r['binding'] == 'no']
        stops = [float(r['servo']) for r in pul if r['binding'] == 'yes']
        if free:
            out['%s_last_free_servo' % side] = pick(free)
        if stops:
            out['%s_stop_servo' % side] = stops[0]
    return out


def extend_state(rows):
    """Right-side extension: (last free right servo or None, extension done, stop servo or None)."""
    edge, done, stop = None, False, None
    for r in rows:
        if r['side'] != 'right':
            continue
        s = float(r['servo'])
        if r['kind'] == 'pulse' and r['binding'] == 'no':
            edge = s if edge is None else max(edge, s)
        if r['note'].startswith(EXT_TAG) and r['binding'] in ('yes', 'end'):
            done = True
            if r['binding'] == 'yes' and stop is None:
                stop = s
    return edge, done, stop


def extend_next(edge, step):
    """Next extension point, or None past the VESC maximum."""
    t = round(edge + step, 3)
    return t if t <= VESC_SERVO_MAX + 1e-9 else None


def measured_at(rows, side, servo):
    return any(r['kind'] == 'measure' and r['side'] == side and r['left_deg'] not in ('', None)
               and abs(float(r['servo']) - servo) < 1e-6 for r in rows)


def extend_right(node, rules, rows, record, measure, clip_hi):
    """Right-side limit extension (see module doc). Returns the target that needs a wider clip, or None."""
    edge, done, _ = extend_state(rows)
    if edge is None:
        raise SystemExit('No confirmed right point in this run: run the normal sweep first.')
    if done:
        print('Right extension already ended in this run (last free point %.3f).' % edge)
        return None
    step = float(rules['extend_step'])
    print('Right extension from %.3f in %.3f steps; driver clip max %.3f.' % (edge, step, clip_hi))
    node.go(edge)  # slews through confirmed points only
    pending = None
    while True:
        target = extend_next(edge, step)
        if target is None:
            record(kind='end', side='right', servo=edge, binding='end',
                   note='%s: VESC servo maximum %.1f reached' % (EXT_TAG, VESC_SERVO_MAX))
            break
        if target > clip_hi + 1e-9:
            ans = (ask('Next point %.3f is above the driver clip %.3f. Enter = pause for a wider clip, '
                       'e = end the right side here:' % (target, clip_hi), 300) or '').lower()
            if ans == 'e':
                record(kind='end', side='right', servo=edge, binding='end',
                       note='%s: ended by operator at the clip, next %.3f' % (EXT_TAG, target))
            else:
                pending = target
            break
        node.go(target)
        time.sleep(float(rules['pulse_s']))
        node.go(edge)
        ans = None
        while ans not in ('y', 'n', 'e'):
            ans = (ask('Binding, end stop or noise at %.3f? [y/n, e = no binding, end side here]:'
                       % target, 300) or '').lower()
        if ans == 'y':
            record(kind='pulse', side='right', servo=target, binding='yes', note='%s: stop' % EXT_TAG)
            print('Right side: stop at %.3f, last free point %.3f.' % (target, edge))
            break
        if ans == 'e':
            record(kind='pulse', side='right', servo=target, binding='end',
                   note='%s: no binding; ended by operator at %.3f' % (EXT_TAG, edge))
            print('Right side ends at %.3f (operator).' % edge)
            break
        record(kind='pulse', side='right', servo=target, binding='no', note=EXT_TAG)
        edge = target
        measure(edge, 'right', EXT_TAG)
    edge, done, _ = extend_state(rows)
    if done and not measured_at(rows, 'right', edge):
        print('Last free point %.3f: measure it now (this sets max_right).' % edge)
        measure(edge, 'right', '%s_last_free' % EXT_TAG)
    return pending


def extend_summary(rows, linked, clip_hi):
    edge, done, stop = extend_state(rows)
    ext = [dict(servo=float(r['servo']), binding=r['binding'], kind=r['kind'], note=r['note'])
           for r in rows if r['side'] == 'right' and r['note'].startswith(EXT_TAG)]
    return dict(linked_run=str(linked), driver_clip_max=clip_hi, step_points=ext,
                last_free_servo=edge, stop_servo=stop, ended=done,
                last_free_measured=measured_at(rows, 'right', edge) if edge is not None else False,
                proposed_right_driving_clip=edge if done else None,
                rule='driving clip = last free point, one step before the stop (left: 0.10, stop ~0.08)')


def edges_from_rows(rows, centre):
    """Resume: confirmed outward edge and done flag per side from earlier rows."""
    state = {'left': [centre, False], 'right': [centre, False]}
    for r in rows:
        side = r['side']
        if side not in state:
            continue
        s = float(r['servo'])
        if r['binding'] in ('yes', 'end'):  # binding pulse, or side ended by the operator
            state[side][1] = True
        elif r['kind'] == 'pulse' and r['binding'] == 'no':
            if (side == 'left' and s < state[side][0]) or (side == 'right' and s > state[side][0]):
                state[side][0] = s
    return state


def self_test():
    p = vp.load()
    rules = vp.get(p, 't2')
    centre = 0.320   # algorithm / fit fixture (the 19 Nov T2 grid), independent of the yaml
    live_centre = float(vp.get(p, 'steering.servo_offset'))

    def walk(side, n, r, c=centre):
        out, e = [], c
        for _ in range(n):
            e = next_point(side, e, r)
            out.append(e)
        return out
    # Step algorithm on a fixed range (independent of the yaml): 0.02 inside, edge, then 0.01
    fixed = dict(rules, approved_servo_min=0.22, approved_servo_max=0.455)
    left = walk('left', 7, fixed)
    assert left == [0.30, 0.28, 0.26, 0.24, 0.22, 0.21, 0.20], left
    right = walk('right', 9, fixed)
    assert right == [0.34, 0.36, 0.38, 0.40, 0.42, 0.44, 0.455, 0.465, 0.475], right
    # 19 Nov grid (centre 0.320, range 0.10-0.60): the measured points 0.28 0.20 0.10 / 0.36 0.46 0.60
    # lie on it, and the next point after each edge is outside a 0.10-0.60 clip (operator answered e).
    r19 = dict(rules, approved_servo_min=0.10, approved_servo_max=0.60)
    left = walk('left', 12, r19)
    assert left[:11] == [0.30, 0.28, 0.26, 0.24, 0.22, 0.20, 0.18, 0.16, 0.14, 0.12, 0.10], left
    assert left[11] == 0.09 and clip_needed(0.09, 0.10, 0.60, rules['clip_margin']) is not None
    right = walk('right', 15, r19)
    assert right[:14] == [0.34, 0.36, 0.38, 0.40, 0.42, 0.44, 0.46, 0.48, 0.50, 0.52, 0.54, 0.56,
                          0.58, 0.60], right
    assert right[14] == 0.61 and clip_needed(0.61, 0.10, 0.60, rules['clip_margin']) is not None
    # Live yaml (centre and approved range): 0.02 steps from the centre, ending exactly on each edge,
    # then 0.01 beyond it, which needs a wider clip.
    lo, hi = float(rules['approved_servo_min']), float(rules['approved_servo_max'])
    assert lo < live_centre < hi
    for side, edge, sign in (('left', lo, -1), ('right', hi, 1)):
        pts = walk(side, 60, rules, live_centre)
        k = pts.index(round(edge, 3))
        steps = [round(b - a, 3) for a, b in zip([live_centre] + pts[:k], pts[:k + 1])]
        assert all(abs(x) == 0.02 for x in steps[:-1]) and 0 < abs(steps[-1]) <= 0.02 + 1e-9, (side, steps)
        assert pts[k + 1] == round(edge + sign * 0.01, 3)
        assert clip_needed(pts[k + 1], lo, hi, rules['clip_margin']) is not None
    assert clip_needed(0.10, 0.10, 0.60, rules['clip_margin']) is None
    assert clip_needed(0.60, 0.10, 0.60, rules['clip_margin']) is None
    assert clip_needed(0.22, 0.215, 0.46, 0.005) is None
    assert clip_needed(0.21, 0.215, 0.46, 0.005) == (0.205, 0.46)
    assert clip_needed(0.465, 0.215, 0.46, 0.005) == (0.215, 0.47)
    assert abs(equivalent_angle(10, 10) - math.radians(10)) < 1e-9
    assert abs(equivalent_angle(-5, -5) + math.radians(5)) < 1e-9
    assert equivalent_angle(0.0, 0.0) == 0.0
    d = equivalent_angle(22.0, 18.0)   # Ackermann: inner wheel turns more
    assert math.radians(18) < d < math.radians(22)
    # Fit recovers servo = 0.320 - 0.50 d on the right and -0.45 d on the left
    rows = []
    for s in (0.30, 0.28, 0.26, 0.34, 0.36, 0.38):
        g = -0.45 if s < centre else -0.50
        deg = math.degrees((s - centre) / g)
        rows.append(dict(kind='measure', side='left' if s < centre else 'right', servo=s,
                         left_deg=deg, right_deg=deg, binding='', note=''))
    rows.append(dict(kind='measure', side='centre', servo=centre, left_deg=0.3, right_deg=0.3,
                     binding='', note='centre_from_left'))
    rows.append(dict(kind='measure', side='centre', servo=centre, left_deg=-0.2, right_deg=-0.2,
                     binding='', note='centre_from_right'))
    rows.append(dict(kind='pulse', side='left', servo=0.24, left_deg='', right_deg='',
                     binding='yes', note=''))
    f = fit_rows(rows, centre)
    assert abs(f['left_gain'] + 0.45) < 0.02 and abs(f['right_gain'] + 0.50) < 0.02, f
    assert abs(f['backlash_deg'] - 0.5) < 1e-6 and f['binding'][0]['servo'] == 0.24
    assert f['max_left_servo'] == 0.26 and f['max_right_servo'] == 0.38
    pulses = [dict(kind='pulse', side='left', servo=0.30, binding='no'),
              dict(kind='pulse', side='left', servo=0.28, binding='no'),
              dict(kind='pulse', side='right', servo=0.34, binding='no'),
              dict(kind='pulse', side='right', servo=0.36, binding='yes')]
    st = edges_from_rows(pulses, centre)
    assert st == {'left': [0.28, False], 'right': [0.34, True]}, st
    # 'e' after a pulse: side done, edge stays at the last confirmed point; 'e' at the clip likewise
    ended = pulses[:3] + [dict(kind='pulse', side='right', servo=0.36, binding='end', note=''),
                          dict(kind='end', side='left', servo=0.28, binding='end', note='')]
    st = edges_from_rows(ended, centre)
    assert st == {'left': [0.28, True], 'right': [0.34, True]}, st
    f = fit_rows(rows + ended, centre)
    assert [e['side'] for e in f['ended_by_operator']] == ['right', 'left'] and len(f['binding']) == 1
    # Both sides paused at the clip: one relaunch covering both
    assert merge_clip(None, (0.205, 0.46)) == (0.205, 0.46)
    assert merge_clip((0.205, 0.46), (0.215, 0.47)) == (0.205, 0.47)
    # --extend-right: state from a finished main run (right ended by e at the 0.60 clip)
    main = [dict(kind='pulse', side='right', servo=v, binding='no', note='') for v in (0.34, 0.46, 0.60)]
    main += [dict(kind='measure', side='right', servo=0.60, left_deg=-19.0, right_deg=-11.0,
                  binding='', note=''),
             dict(kind='end', side='right', servo=0.60, binding='end', note='ended by operator at the clip')]
    assert extend_state(main) == (0.60, False, None)       # main-run clip end is not an extension end
    assert extend_next(0.60, 0.02) == 0.62 and extend_next(0.98, 0.02) == 1.0
    assert extend_next(1.0, 0.02) is None

    class FakeNode:
        def __init__(self):
            self.went = []

        def go(self, v):
            self.went.append(round(v, 3))

    def scripted(answers):
        it = iter(answers)
        return lambda prompt, timeout: next(it)

    def run_ext(answers, measures, clip_hi):
        rows = [dict(r) for r in main]
        node, meas = FakeNode(), iter(measures)

        def record(**kw):
            rows.append({k: kw.get(k, '') for k in FIELDS})

        def measure(servo, side, note=''):
            m = next(meas)
            if m is not None:
                record(kind='measure', side=side, servo=servo, left_deg=m[0], right_deg=m[1], note=note)
        saved = globals()['ask']
        globals()['ask'] = scripted(answers)
        try:
            pending = extend_right(node, dict(rules, pulse_s=0.0), rows, record, measure, clip_hi)
        finally:
            globals()['ask'] = saved
        return rows, node, pending
    # n, n, y with 's' at 0.62/0.64, then the last free point 0.64 is measured at the end
    rows_x, node, pending = run_ext(['n', 'n', 'y'], [None, None, (-24.0, -15.0)], 1.0)
    assert pending is None and node.went[:4] == [0.60, 0.62, 0.60, 0.64], node.went
    assert max(node.went) == 0.66 and 0.68 not in node.went     # stop point visited, never passed
    assert extend_state(rows_x) == (0.64, True, 0.66)
    f = fit_rows(rows + rows_x, centre)      # with the fit fixture: >= 3 measured points
    assert f['max_right_servo'] == 0.64 and f['right_last_free_servo'] == 0.64, f
    assert f['right_stop_servo'] == 0.66 and 0.66 in [b['servo'] for b in f['binding']]
    assert f['left_stop_servo'] == 0.24
    sm = extend_summary(rows_x, '/x', 1.0)
    assert sm['proposed_right_driving_clip'] == 0.64 and sm['last_free_measured'] and sm['stop_servo'] == 0.66
    # e after one free point: ended, no stop, last free 0.62 already measured -> no extra prompt
    rows_e, _, _ = run_ext(['n', 'e'], [(-21.0, -13.0)], 1.0)
    assert extend_state(rows_e) == (0.62, True, None)
    assert sum(r['kind'] == 'measure' for r in rows_e) == 2      # main 0.60 + 0.62, no extra prompt
    # clip 0.64: two free points, then the clip prompt: Enter -> pause at 0.66 (not commanded)
    rows_p, node, pending = run_ext(['n', 'n', ''], [None, None], 0.64)
    assert pending == 0.66 and 0.66 not in node.went and extend_state(rows_p) == (0.64, False, None)
    # an extension that already ended commands nothing and asks nothing
    idle = FakeNode()
    assert extend_right(idle, rules, rows_x, None, None, 1.0) is None and idle.went == []
    print('PASS: sweep order (0.02 inside the range, then 0.01), live range 0.10-0.60 grid and')
    print('      edges, clip relaunch values,')
    print('      equivalent Ackermann angle, linear + per-side fit, backlash, binding record,')
    print('      resume edges, side ended by operator (e), one relaunch for both sides,')
    print('      --extend-right (y / e / clip pause, stop never passed, last free point measured,')
    print('      fit max_right + last free / stop, summary driving clip).')
    print('      No ROS imported; nothing published.')


# ---------------------------------------------------------------- operator I/O

class Abort(Exception):
    pass


def ask(prompt, timeout):
    """Line input with a timeout; None on timeout. Abort on EOF (SSH lost) or 'q'."""
    print(prompt, end=' ', flush=True)
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        print('(timeout)')
        return None
    line = sys.stdin.readline()
    if not line:
        raise Abort('terminal closed')
    line = line.strip()
    if line.lower() == 'q':
        raise Abort('operator quit')
    return line


# ---------------------------------------------------------------- ROS

def run(args):
    from run_test import RosSession, get_node_params
    from rclpy.node import Node
    from std_msgs.msg import Float64

    p = vp.load()
    rules = vp.get(p, 't2')
    centre = float(vp.get(p, 'steering.servo_offset'))
    slew = float(rules['slew_per_s']) / RATE_HZ

    class Servo(Node):
        def __init__(self):
            super().__init__('t2_servo_sweep')
            self.pub = self.create_publisher(Float64, '/commands/servo/position', 10)
            self.create_subscription(Float64, '/sensors/servo_position_command', self.on_seen, 10)
            self.lock = threading.Lock()
            self.cmd = None          # nothing published until the checks pass
            self.target = centre
            self.seen = None
            self.running = True
            threading.Thread(target=self.loop, daemon=True).start()

        def on_seen(self, msg):
            self.seen = msg.data

        def loop(self):
            while self.running:
                with self.lock:
                    if self.cmd is not None:
                        step = max(-slew, min(slew, self.target - self.cmd))
                        self.cmd += step
                        self.pub.publish(Float64(data=self.cmd))
                time.sleep(1.0 / RATE_HZ)

        def go(self, value, timeout=5.0):
            """Slew to value and wait until the command is there."""
            with self.lock:
                self.target = value
                if self.cmd is None:
                    self.cmd = centre
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                with self.lock:
                    if abs(self.cmd - value) < 1e-6:
                        return
                time.sleep(0.01)
            raise Abort('servo command did not reach %.3f' % value)

    with RosSession(Servo) as node:
        try:
            sweep(args, node, p, rules, centre, get_node_params)
        finally:
            try:
                node.go(centre)
                print('Servo back at centre %.3f.' % centre)
            except Abort as exc:
                print('WARNING: %s' % exc)
            node.running = False
            time.sleep(0.1)


def sweep(args, node, p, rules, centre, get_node_params):
    time.sleep(1.0)  # discovery
    names = set(node.get_node_names())
    if 'vesc_driver' not in names:
        raise SystemExit('No /vesc_driver: start t2_servo_only.launch.py first.')
    if names & FORBIDDEN_NODES:
        raise SystemExit('T2 needs the servo-only stack; found %s.' % sorted(names & FORBIDDEN_NODES))
    lim = get_node_params(node, 'vesc_driver', ZERO_LIMITS + ['servo_min', 'servo_max'])
    nonzero = {k: lim.get(k) for k in ZERO_LIMITS if lim.get(k) != 0.0}
    if nonzero:
        raise SystemExit('vesc_driver motor limits must all be 0 for T2: %s' % nonzero)
    clip_lo, clip_hi = float(lim['servo_min']), float(lim['servo_max'])
    servo_pubs = len(node.get_publishers_info_by_topic('/commands/servo/position'))
    motor_pubs = {t: len(node.get_publishers_info_by_topic(t)) for t in (
        '/commands/motor/speed', '/commands/motor/current', '/commands/motor/brake',
        '/commands/motor/duty_cycle', '/commands/motor/position')}
    if servo_pubs != 1 or any(motor_pubs.values()):
        raise SystemExit('Other command sources: servo publishers %d, motor %s' % (servo_pubs, motor_pubs))
    print('vesc_driver: motor limits 0, servo clip %.3f..%.3f. Approved range %.3f..%.3f.' % (
        clip_lo, clip_hi, rules['approved_servo_min'], rules['approved_servo_max']))
    if ask('Motor phase wires disconnected? Type "disconnected":', 120) != 'disconnected':
        raise SystemExit('Not confirmed.')

    if args.resume or args.extend_right:
        out = Path(args.resume or args.extend_right).expanduser()
        with open(out / 'points.csv') as fp:
            rows = list(csv.DictReader(fp))
    else:
        out = vp.evidence_root() / ('%s_t2_servo_sweep' % datetime.now().strftime('%Y%m%d_%H%M%S'))
        out.mkdir(parents=True)
        rows = []
        with open(out / 'points.csv', 'w', newline='') as fp:
            csv.DictWriter(fp, FIELDS).writeheader()
    print('Results: %s' % out)

    def record(**kw):
        row = dict(wall=time.time(), servo_seen=node.seen, **kw)
        row = {k: row.get(k, '') for k in FIELDS}
        rows.append(row)
        with open(out / 'points.csv', 'a', newline='') as fp:
            csv.DictWriter(fp, FIELDS).writerow(row)

    def measure(servo, side, note=''):
        while True:
            node.go(servo)
            ans = ask('servo %.3f: left_deg right_deg [binding? b] (+ = left; s = skip):' % servo,
                      float(rules['max_hold_s']))
            if ans is None:
                node.go(centre)
                ask('Hold time over, servo at centre. Enter to go back to %.3f:' % servo, 3600)
                continue
            if ans.lower() == 's':
                return
            parts = ans.split()
            try:
                left, right = float(parts[0]), float(parts[1])
            except (IndexError, ValueError):
                print('Type two numbers, e.g. "12.5 10.0", or s / q.')
                continue
            record(kind='measure', side=side, servo=servo, left_deg=left, right_deg=right,
                   binding='yes' if 'b' in parts[2:] else '', note=note)
            return

    if args.extend_right:
        pending = extend_right(node, rules, rows, record, measure, clip_hi)
        summary = extend_summary(rows, out, clip_hi)
        (out / 'extend_right.json').write_text(json.dumps(summary, indent=2))
        if (out / 'fit.json').exists() and not (out / 'fit_before_extend_right.json').exists():
            (out / 'fit_before_extend_right.json').write_text((out / 'fit.json').read_text())
        fit = fit_rows(rows, centre)
        (out / 'fit.json').write_text(json.dumps(fit, indent=2))
        with open(out / 'README.md', 'a') as fp:
            fp.write('\n## Right-side extension (%s)\n\n```\n%s\n```\n' % (
                datetime.now().strftime('%Y-%m-%d %H:%M'), json.dumps(summary, indent=2)))
        print(json.dumps(summary, indent=2))
        print('fit.json: max_right_servo %s, max_right_rad %s' % (
            fit.get('max_right_servo'), fit.get('max_right_rad')))
        if pending is not None:
            hi = min(VESC_SERVO_MAX, round(pending + float(rules['clip_margin']), 3))
            print('\nTo approve %.3f: stop the launch, relaunch with\n'
                  '  ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py '
                  'servo_min:=%.3f servo_max:=%.3f\nand continue with\n'
                  '  python3 %s --extend-right %s' % (pending, clip_lo, hi, os.path.realpath(__file__), out))
        return

    node.go(centre)
    if not args.resume:
        measure(centre, 'centre', 'centre_start')
    edges = edges_from_rows(rows, centre)
    relaunch = None
    for side in ('left', 'right'):
        edge, done = edges[side]
        start_edge = edge
        while not done:
            target = next_point(side, edge, rules)
            need = clip_needed(target, clip_lo, clip_hi, float(rules['clip_margin']))
            if need:
                ans = (ask('%s side: next point %.3f is outside the driver clip %.3f..%.3f. '
                           'Enter = pause for a wider clip, e = end this side here:' % (
                               side, target, clip_lo, clip_hi), 300) or '').lower()
                if ans == 'e':
                    record(kind='end', side=side, servo=edge, binding='end',
                           note='ended by operator at the clip, next %.3f' % target)
                    print('%s side ends at %.3f (operator).' % (side, edge))
                else:
                    relaunch = merge_clip(relaunch, need)
                break
            # First visit: at most pulse_s at the new point, then back to the confirmed edge.
            node.go(target)
            time.sleep(float(rules['pulse_s']))
            node.go(edge)
            ans = None
            while ans not in ('y', 'n', 'e'):
                ans = (ask('Binding, end stop or noise at %.3f? [y/n, e = no binding, end side here]:'
                           % target, 300) or '').lower()
            if ans == 'e':
                record(kind='pulse', side=side, servo=target, binding='end',
                       note='no binding; side ended by operator at %.3f' % edge)
                print('%s side ends at %.3f (operator).' % (side, edge))
                break
            record(kind='pulse', side=side, servo=target, binding='yes' if ans == 'y' else 'no')
            if ans == 'y':
                done = True
                print('%s side ends at %.3f (binding at %.3f).' % (side, edge, target))
                break
            edge = target
            measure(edge, side)
        # Approach the centre from this side for the backlash pair (only if the side moved now,
        # so a resume does not ask again for a side finished earlier).
        if edge != start_edge:
            node.go(centre)
            measure(centre, 'centre', 'centre_from_%s' % side)

    fit = fit_rows(rows, centre)
    (out / 'fit.json').write_text(json.dumps(fit, indent=2))
    lines = ['# T2 servo sweep', '', 'Driver clip %.3f..%.3f; approved %.3f..%.3f.' % (
        clip_lo, clip_hi, rules['approved_servo_min'], rules['approved_servo_max']), '',
        '```', json.dumps(fit, indent=2), '```']
    if relaunch:
        lines += ['', 'Paused for a wider clip: %.3f..%.3f' % relaunch]
    (out / 'README.md').write_text('\n'.join(lines) + '\n')
    print(json.dumps(fit, indent=2))
    if relaunch:
        print('\nTo approve the next point: stop the launch, then relaunch with\n'
              '  ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py '
              'servo_min:=%.3f servo_max:=%.3f\nand continue with\n'
              '  python3 %s --resume %s' % (relaunch[0], relaunch[1], os.path.realpath(__file__), out))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--resume', help='results folder of an earlier sweep')
    ap.add_argument('--fit', help='fit an existing results folder (offline)')
    ap.add_argument('--extend-right', metavar='DIR',
                    help='right-side limit extension, appended to an earlier sweep folder')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if sum(bool(x) for x in (args.resume, args.fit, args.extend_right)) > 1:
        raise SystemExit('Use only one of --resume, --fit, --extend-right.')
    if args.self_test:
        self_test()
        return
    if args.fit:
        folder = Path(args.fit).expanduser()
        with open(folder / 'points.csv') as fp:
            rows = list(csv.DictReader(fp))
        fit = fit_rows(rows, float(vp.get(vp.load(), 'steering.servo_offset')))
        (folder / 'fit.json').write_text(json.dumps(fit, indent=2))
        print(json.dumps(fit, indent=2))
        return
    if not sys.stdin.isatty():
        raise SystemExit('T2 needs an interactive terminal.')
    try:
        run(args)
    except Abort as exc:
        print('Stopped: %s' % exc)


if __name__ == '__main__':
    main()
