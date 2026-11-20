#!/usr/bin/env python3
"""Offline analysis of one real-car test folder (or a bare ros2 bag). Read-only.

  analyze.py ~/f1tenth_hw_evidence/real_car_tests/<stamp>_<test>_.../
  analyze.py --self-test

Writes into the folder: summary.json, core.csv, motor_cmd.csv, mux_transitions.csv,
stops.png / intervals.png, and an "## Analysis" section in README.md.
Checks: topic rates, scan lengths (F-09), runner publish intervals and the intervals
the bag received on /manual/drive and /joy (mux TTLs 0.22 / 0.20 s, F-13),
mux mode transitions with last_stop_reason, disarmed output = zero, and per stop:
ERPM and current after zero, time to < 100 ERPM, rebound, tachometer counts.
Command side (the bag never records /drive or /commands/*): vesc_speed_audit.log copied
by the runner, else the mux output in /control/status.
"""
import argparse
import csv
import json
import math
import os
import statistics
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402
from run_test import (REBOUND_FAIL_ERPM, UNDER_ERPM, ReboundTracker,  # noqa: E402
                      front_range, interval_stats, parse_audit_line, window_ranges)

STOP_WINDOW_S = 3.0
STEADY_FRACTION = 0.8       # a stop counts for the step gate only from >= 80 % of the step ERPM
REASON_MATCH_S = 0.5        # runner ZERO_COMMAND event <-> measured zero time
JOY_TTL_S, COMMAND_TTL_S = 0.20, 0.22   # physical_drive_mux.py
EXPECTED_HZ = {'/scan': 29.6, '/odom_rf2o': 20.0, '/imu/data_raw': 100.0,
               '/control/status': 20.0, '/manual/drive': 50.0, '/joy': 50.0}

# Reference palette (dataviz skill): series 1 blue, series 2 orange; ink and grid neutrals.
BLUE, ORANGE, INK, MUTED, GRID = '#2a78d6', '#eb6834', '#0b0b0b', '#52514e', '#e4e3df'
REPEAT_COLORS = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']


# ---------------------------------------------------------------- pure logic

def stop_metrics(t, erpm, current, tach, t0, t_end):
    """Metrics for one stop. t0 = time the zero command was sent. Series sorted by t."""
    idx = [i for i, x in enumerate(t) if t0 <= x <= t_end]
    if not idx:
        return {'samples': 0}
    a = [abs(erpm[i]) for i in idx]
    under = next((t[i] - t0 for k, i in enumerate(idx) if a[k] < UNDER_ERPM), None)
    before = [i for i, x in enumerate(t) if x < t0]
    erpm_at_zero = abs(erpm[before[-1]]) if before else a[0]
    tr = ReboundTracker(erpm_at_zero)   # same definition as the runner's live line
    for k, i in enumerate(idx):
        tr.add(t[i] - t0, a[k])
    rise, rise_at, rise_from, peak_after_min = tr.rise, tr.rise_t, tr.from_t, tr.peak
    out = dict(samples=len(idx), erpm_at_zero=round(erpm_at_zero, 1),
               time_to_under_100_erpm_s=None if under is None else round(under, 3),
               rebound_rise_erpm=round(rise, 1),
               rebound_peak_erpm=round(peak_after_min, 1),
               rebound_at_s=None if rise_at is None else round(rise_at, 3),
               rebound_from_s=None if rise_from is None else round(rise_from, 3),  # the dip it rises from
               max_abs_current_after_zero_a=round(max(abs(current[i]) for i in idx), 2),
               stopped=under is not None,
               # A stop that never got under 100 ERPM in the window fails as well.
               pass_no_rebound_over_300=under is not None and rise <= REBOUND_FAIL_ERPM)
    if tach:
        out['tach_counts_after_zero'] = tach[idx[-1]] - (tach[before[-1]] if before else tach[idx[0]])
    return out


def zero_transitions(t, cmd, eps=1.0):
    """Times where the commanded ERPM goes from > eps to <= eps."""
    return [t[i] for i in range(1, len(t)) if abs(cmd[i - 1]) > eps and abs(cmd[i]) <= eps]


def command_series(audit_lines, statuses, legacy, gain):
    """ERPM command actually sent, best source first. The bag never records /drive or
    /commands/* (they are mux-protected), so: vesc_driver audit log (sent_erpm), else
    mux output speed_mps x gain from /control/status, else /commands/motor/speed in old bags."""
    audit = [a for a in (parse_audit_line(x) for x in audit_lines) if a]
    if audit:
        return 'vesc_speed_audit.log (sent_erpm)', [a[0] for a in audit], [a[2] for a in audit]
    if statuses:
        return ('/control/status speed_mps x %.0f' % gain, [t for t, _ in statuses],
                [float(s.get('speed_mps') or 0.0) * gain for _, s in statuses])
    if legacy:
        return '/commands/motor/speed (legacy bag)', [t for t, _ in legacy], [m.data for _, m in legacy]
    return None, [], []


def phase_exits(t1b_statuses):
    """T1b converter: times it left DRIVE (its stop action starts). The audit log cannot show
    these for brake/current0, because no speed command follows the stop."""
    out, prev = [], None
    for t, st in t1b_statuses:
        phase = st.get('phase')
        if prev == 'DRIVE' and phase != 'DRIVE':
            out.append(t)
        prev = phase
    return out


def phase_entries(t1b_statuses):
    """T1b converter: times it entered DRIVE (first nonzero speed command sent to vesc_driver)."""
    out, prev = [], None
    for t, st in t1b_statuses:
        phase = st.get('phase')
        if phase == 'DRIVE' and prev != 'DRIVE':
            out.append(t)
        prev = phase
    return out


# ---------------------------------------------------------------- T4 (ground, straight)

T4_TEST = 't4_straight'
MOTION_ERPM = 150.0        # first motion: |ERPM| >= this on 2 consecutive samples (~0.018 m/s)
MOTION_RANGE_M = 0.02      # ... or the range 2 cm below its pre-start median on 2 consecutive scans
PRE_S = 1.0                # pre-start window before the command: car still, hands off
STILL_STD_M = 0.01         # range spread allowed in the pre-start and standstill windows
SIDE_MAX_M = 4.0           # side walls further than this are not used
WALL_MIN_POINTS, WALL_MAX_RMS = 12, 0.02
WALL_SHARE = 0.9           # wall fit on >= this share of a repeat's scans -> use it, else front median
CURVED_DEG = 5.0           # heading change above this: LiDAR distance corrected by cos(heading)


def wall_fit(scan, half_width_deg):
    """Line x = c + b*y through the front returns (laser frame, +/- half_width): perpendicular
    distance to the wall d = c / sqrt(1 + b^2) and heading to the wall normal psi = atan(b)
    (+ = car turned left). None when too few points or not a plane (box edge, clutter)."""
    pts = window_ranges(scan, half_width_deg)
    if len(pts) < WALL_MIN_POINTS:
        return None
    med = statistics.median(r for _, r in pts)
    xy = [(r * math.cos(a), r * math.sin(a)) for a, r in pts if abs(r - med) < max(0.25, 0.15 * med)]
    rms = float('inf')
    for _ in range(3):
        n = len(xy)
        if n < WALL_MIN_POINTS:
            return None
        mx, my = sum(p[0] for p in xy) / n, sum(p[1] for p in xy) / n
        syy = sum((p[1] - my) ** 2 for p in xy)
        if syy < 1e-9:
            return None
        b = sum((p[1] - my) * (p[0] - mx) for p in xy) / syy
        c = mx - b * my
        res = [p[0] - c - b * p[1] for p in xy]
        rms = math.sqrt(sum(r * r for r in res) / n)
        keep = [p for p, r in zip(xy, res) if abs(r) <= max(3 * rms, 0.01)]
        if len(keep) == n:
            break
        xy = keep
    if rms > WALL_MAX_RMS:
        return None
    return dict(d=c / math.sqrt(1 + b * b), psi=math.atan(b), n=len(xy), rms=rms)


def scan_features(scan, front_deg, wall_deg):
    return dict(front=front_range(scan, front_deg), wall=wall_fit(scan, wall_deg),
                left=front_range(scan, front_deg, 90.0), right=front_range(scan, front_deg, -90.0))


def interp(ts, vs, t):
    """Linear interpolation of a sorted series; None outside it."""
    if not ts or t < ts[0] or t > ts[-1]:
        return None
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        lo, hi = (mid, hi) if ts[mid] <= t else (lo, mid)
    if ts[hi] == ts[lo]:
        return vs[lo]
    return vs[lo] + (vs[hi] - vs[lo]) * (t - ts[lo]) / (ts[hi] - ts[lo])


def in_window(ts, vs, a, b):
    return [v for t, v in zip(ts, vs) if a <= t <= b and v is not None and math.isfinite(v)]


def integrate(ts, vs, a, b):
    """Trapezoid integral of a sorted series over [a, b] (interpolated ends); None if no data."""
    if not ts or b <= a:
        return None
    a, b = max(a, ts[0]), min(b, ts[-1])
    if b <= a:
        return None
    pts = [(a, interp(ts, vs, a))] + [(t, v) for t, v in zip(ts, vs) if a < t < b] + [(b, interp(ts, vs, b))]
    return sum((t2 - t1) * (v1 + v2) / 2 for (t1, v1), (t2, v2) in zip(pts, pts[1:]))


def first_sustained(ts, vs, t_from, pred):
    """First t >= t_from where pred holds on this and the next sample."""
    for i in range(len(ts) - 1):
        if ts[i] >= t_from and pred(vs[i]) and pred(vs[i + 1]):
            return ts[i]
    return None


def slope(ts, vs):
    n = len(ts)
    if n < 3:
        return None
    mt, mv = sum(ts) / n, sum(vs) / n
    stt = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / stt if stt > 0 else None


def med(vals):
    return statistics.median(vals) if vals else None


def unwrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def t4_repeat(t0, t_cmd, t_runner_zero, scan_t, feats, ct, erpm, ot, vx, rt, rpose, gain, hold_s, ramp_accel):
    """T4 metrics of one repeat. t0 = brake command (converter left DRIVE), t_cmd = first nonzero
    speed command (converter entered DRIVE), t_runner_zero = runner's ZERO_COMMAND (or None).
    feats: scan_features per scan time; rpose: RF2O (x, y, yaw). All times: bag receive (wall)."""
    out = {}
    win_feats = [f for t, f in zip(scan_t, feats) if t_cmd - PRE_S <= t <= t0 + hold_s]
    walls = sum(1 for f in win_feats if f['wall'])
    use_wall = bool(win_feats) and walls >= WALL_SHARE * len(win_feats)
    out['range_source'] = 'wall fit (perpendicular)' if use_wall else 'front median +/-3 deg'
    rng = [(f['wall']['d'] if f['wall'] else None) if use_wall else f['front'] for f in feats]
    psi = [f['wall']['psi'] if f['wall'] else None for f in feats]
    rts = [t for t, r in zip(scan_t, rng) if r is not None and math.isfinite(r)]
    rvs = [r for r in rng if r is not None and math.isfinite(r)]
    aerpm = [abs(e) for e in erpm]

    # Pre-start: still, hands off.
    pre = in_window(rts, rvs, t_cmd - PRE_S, t_cmd - 0.02)
    r_pre = med(pre)
    out['pre_range_m'] = r_pre
    pre_erpm = in_window(ct, aerpm, t_cmd - PRE_S, t_cmd)
    out['pre_still'] = (len(pre) >= 3 and statistics.pstdev(pre) <= STILL_STD_M
                        and (not pre_erpm or max(pre_erpm) < UNDER_ERPM))

    # Start delay: command -> first motion.
    t_mv = first_sustained(ct, aerpm, t_cmd, lambda e: e >= MOTION_ERPM)
    t_ml = (first_sustained(rts, rvs, t_cmd, lambda r: r <= r_pre - MOTION_RANGE_M)
            if r_pre is not None else None)
    out['start_delay_erpm_s'] = None if t_mv is None or t_mv > t0 else t_mv - t_cmd
    out['start_delay_lidar_s'] = None if t_ml is None or t_ml > t0 else t_ml - t_cmd
    out['cmd_speed_at_motion_mps'] = (None if out['start_delay_erpm_s'] is None
                                      else ramp_accel * out['start_delay_erpm_s'])
    if out['start_delay_erpm_s'] is None:
        out['moved'] = False
        return out
    out['moved'] = True

    # Brake: range and speed at the brake command.
    r_brake = interp(rts, rvs, t0)
    sl = slope([t for t in rts if t0 - 0.4 <= t <= t0], in_window(rts, rvs, t0 - 0.4, t0))
    v_lidar = None if sl is None else -sl
    vv = in_window(ot, vx, t0 - 0.1, t0)
    v_vesc = sum(vv) / len(vv) if vv else interp(ot, vx, t0)
    out.update(range_at_brake_m=r_brake, v_brake_lidar_mps=v_lidar, v_brake_vesc_mps=v_vesc)

    # Standstill: ERPM < 100, then the range while the brake still holds.
    t_under = next((t for t, e in zip(ct, aerpm) if t >= t0 and e < UNDER_ERPM), None)
    out['stop_time_s'] = None if t_under is None else t_under - t0
    if t_under is None:
        out['stopped'] = False
        return out
    out['stopped'] = True
    a, b = max(t_under + 0.2, t0 + 0.5), t0 + hold_s - 0.1
    still = in_window(rts, rvs, a, b)
    if len(still) < 3:
        b = a + 1.0
        still = in_window(rts, rvs, a, b)
    r_still = med(still)
    out['range_still_m'] = r_still
    out['still_ok'] = len(still) >= 3 and statistics.pstdev(still) <= STILL_STD_M

    def dist(x, y):
        return None if x is None or y is None else x - y
    out['stop_lidar_m'] = dist(r_brake, r_still)
    out['stop_vesc_m'] = integrate(ot, vx, t0, a)
    if t_runner_zero is not None:
        out['stop_from_runner_lidar_m'] = dist(interp(rts, rvs, t_runner_zero), r_still)
        out['runner_to_brake_s'] = t0 - t_runner_zero

    def decel(v, d):
        return v * v / (2 * d) if v is not None and d is not None and d > 0.005 else None
    out['decel_lidar_mps2'] = decel(v_lidar, out['stop_lidar_m'])
    out['decel_vesc_mps2'] = decel(v_vesc, out['stop_vesc_m'])

    # Heading to the wall (wall fit) and RF2O yaw: curvature = steering offset (indicative).
    pts = [(t, p) for t, p in zip(scan_t, psi) if p is not None]
    psi_pre = med([p for t, p in pts if t_cmd - PRE_S <= t <= t_cmd])
    psi_still = med([p for t, p in pts if a <= t <= b])
    psi_drive = med([p for t, p in pts if t_cmd <= t <= t0])
    out['heading_change_wall_deg'] = (None if psi_pre is None or psi_still is None
                                      else math.degrees(psi_still - psi_pre))
    r0 = [interp(rt, [p[k] for p in rpose], t_cmd) for k in range(3)]
    r1 = [interp(rt, [p[k] for p in rpose], t0) for k in range(3)]
    r2 = [interp(rt, [p[k] for p in rpose], a) for k in range(3)]
    have_rf = None not in r0 + r1 + r2
    out['heading_change_rf2o_deg'] = math.degrees(unwrap(r2[2] - r0[2])) if have_rf else None

    # Distance while driving (gain) and in total.
    cos_fix = math.cos(psi_drive) if (use_wall and psi_drive is not None) else 1.0
    out['drive_lidar_m'] = None if dist(r_pre, r_brake) is None else dist(r_pre, r_brake) / cos_fix
    out['drive_vesc_m'] = integrate(ot, vx, t_cmd - 0.2, t0)
    out['drive_rf2o_m'] = math.hypot(r1[0] - r0[0], r1[1] - r0[1]) if have_rf else None
    out['total_lidar_m'] = dist(r_pre, r_still)
    out['total_vesc_m'] = integrate(ot, vx, t_cmd - 0.2, a)
    out['total_rf2o_m'] = math.hypot(r2[0] - r0[0], r2[1] - r0[1]) if have_rf else None
    dl, dv, dr = out['drive_lidar_m'], out['drive_vesc_m'], out['drive_rf2o_m']
    # vesc distance = true distance x gain_true / gain_used  ->  gain_true = gain x d_vesc / d_true
    out['gain_suggested_lidar'] = gain * dv / dl if dl and dv and dl > 0.2 else None
    out['gain_suggested_rf2o'] = gain * dv / dr if dr and dv and dr > 0.2 else None
    curved = [abs(x) for x in (out['heading_change_wall_deg'], out['heading_change_rf2o_deg']) if x is not None]
    out['curved'] = bool(curved) and max(curved) > CURVED_DEG

    # Side drift (+ = left). Wall: sideways along the wall from d and heading; side walls; RF2O
    # relative to the start heading.
    lat = 0.0
    seq = [(t, f['wall']) for t, f in zip(scan_t, feats) if t_cmd <= t <= a and f['wall']]
    for (_, w1), (_, w2) in zip(seq, seq[1:]):
        lat += (w1['d'] - w2['d']) * math.tan((w1['psi'] + w2['psi']) / 2)
    out['drift_wall_m'] = lat if use_wall and len(seq) > 2 else None
    side = []
    for key, sign in (('left', -1.0), ('right', 1.0)):
        s0 = med(in_window(scan_t, [f[key] for f in feats], t_cmd - PRE_S, t_cmd))
        s1 = med(in_window(scan_t, [f[key] for f in feats], a, b))
        out['%s_wall_change_m' % key] = dist(s1, s0)
        if s0 is not None and s1 is not None and s0 <= SIDE_MAX_M and s1 <= SIDE_MAX_M:
            side.append(sign * (s1 - s0))
    out['drift_side_m'] = sum(side) / len(side) if side else None
    if have_rf:
        dx, dy = r2[0] - r0[0], r2[1] - r0[1]
        out['drift_rf2o_m'] = -math.sin(r0[2]) * dx + math.cos(r0[2]) * dy
    else:
        out['drift_rf2o_m'] = None
    return out


T4_KEYS = ['stop_lidar_m', 'stop_vesc_m', 'stop_from_runner_lidar_m', 'decel_lidar_mps2', 'decel_vesc_mps2',
           'stop_time_s', 'gain_suggested_lidar', 'gain_suggested_rf2o', 'start_delay_erpm_s',
           'start_delay_lidar_s', 'drift_wall_m', 'drift_side_m', 'drift_rf2o_m',
           'heading_change_wall_deg', 'heading_change_rf2o_deg', 'rebound_rise_erpm']


def t4_summary(items):
    """Mean / sd / min / max over the counted repeats (stopped at the brake point from steady speed)."""
    use = [x for x in items if x.get('counted') and x.get('stopped')]
    out = dict(counted=len(use), repeats=len(items))
    for k in T4_KEYS:
        vals = [x[k] for x in use if x.get(k) is not None]
        if vals:
            out[k] = dict(n=len(vals), mean=sum(vals) / len(vals), min=min(vals), max=max(vals),
                          sd=statistics.stdev(vals) if len(vals) > 1 else 0.0)
    return out


def fmt(x, nd=2):
    return '-' if x is None else ('%.*f' % (nd, x))


def t4_report(name, t4, gain, table=True):
    lines = ['### T4 %s' % name, '']
    if table:
        lines += ['Range: %s. Times: bag receive. + drift = left. L = LiDAR, V = VESC odom.' % (
            ', '.join(sorted(set(x.get('range_source', '?') for x in t4['items'] if 'range_source' in x))) or '-'), '',
                  '| # | Reason | Counted | v brake m/s L / V | Stop m L / V (from runner) | Decel m/s^2 L / V '
                  '| Stop time s | Drive m L / V / RF2O | Gain sugg. L / RF2O | Start delay s ERPM / LiDAR '
                  '| Drift m wall / side / RF2O | Heading deg wall / RF2O | Rebound ERPM | Flags |',
                  '|---|---|---|---|---|---|---|---|---|---|---|---|---|---|']
    for x in (t4['items'] if table else []):
        flags = [f for f, bad in (('pre-start not still', x.get('pre_still') is False),
                                  ('standstill noisy', x.get('still_ok') is False),
                                  ('curved > %g deg' % CURVED_DEG, x.get('curved')),
                                  ('no motion', x.get('moved') is False),
                                  ('not stopped', x.get('moved') and x.get('stopped') is False)) if bad]
        lines.append('| %d | %s | %s | %s / %s | %s / %s (%s) | %s / %s | %s | %s / %s / %s | %s / %s | %s / %s '
                     '| %s / %s / %s | %s / %s | %s | %s |' % (
                         x['index'] + 1, (x.get('runner_reason') or '-').split('(')[0],
                         'yes' if x.get('counted') else 'no',
                         fmt(x.get('v_brake_lidar_mps')), fmt(x.get('v_brake_vesc_mps')),
                         fmt(x.get('stop_lidar_m'), 3), fmt(x.get('stop_vesc_m'), 3),
                         fmt(x.get('stop_from_runner_lidar_m'), 3),
                         fmt(x.get('decel_lidar_mps2')), fmt(x.get('decel_vesc_mps2')), fmt(x.get('stop_time_s'), 3),
                         fmt(x.get('drive_lidar_m'), 3), fmt(x.get('drive_vesc_m'), 3),
                         fmt(x.get('drive_rf2o_m'), 3),
                         fmt(x.get('gain_suggested_lidar'), 0), fmt(x.get('gain_suggested_rf2o'), 0),
                         fmt(x.get('start_delay_erpm_s'), 3), fmt(x.get('start_delay_lidar_s'), 3),
                         fmt(x.get('drift_wall_m'), 3), fmt(x.get('drift_side_m'), 3), fmt(x.get('drift_rf2o_m'), 3),
                         fmt(x.get('heading_change_wall_deg'), 1), fmt(x.get('heading_change_rf2o_deg'), 1),
                         fmt(x.get('rebound_rise_erpm'), 0), ', '.join(flags) or '-'))
    s = t4['summary']
    lines += ['', 'Summary over %d counted of %d repeats (mean +/- sd, min..max):' % (s['counted'], s['repeats'])]

    def row(label, key, nd=3, unit=''):
        v = s.get(key)
        if v:
            lines.append('- %s: %s +/- %s%s (%s..%s, n=%d)' % (label, fmt(v['mean'], nd), fmt(v['sd'], nd), unit,
                                                             fmt(v['min'], nd), fmt(v['max'], nd), v['n']))
    row('Stopping distance, LiDAR, from the brake command', 'stop_lidar_m', 3, ' m')
    row('Stopping distance, VESC odom', 'stop_vesc_m', 3, ' m')
    row('Stopping distance, LiDAR, from the runner zero (incl. mux pipeline)', 'stop_from_runner_lidar_m', 3, ' m')
    row('Deceleration, LiDAR v^2/2d', 'decel_lidar_mps2', 2, ' m/s^2')
    row('Deceleration, VESC v^2/2d', 'decel_vesc_mps2', 2, ' m/s^2')
    row('Time to < 100 ERPM', 'stop_time_s', 3, ' s')
    row('speed_to_erpm_gain suggested from LiDAR (now %.0f)' % gain, 'gain_suggested_lidar', 0)
    row('speed_to_erpm_gain suggested from RF2O', 'gain_suggested_rf2o', 0)
    row('Start delay, command -> ERPM >= %.0f' % MOTION_ERPM, 'start_delay_erpm_s', 3, ' s')
    row('Start delay, command -> LiDAR -%.0f cm' % (MOTION_RANGE_M * 100), 'start_delay_lidar_s', 3, ' s')
    row('Side drift, wall fit (indicative: steering centre uncertain)', 'drift_wall_m', 3, ' m')
    row('Side drift, side walls', 'drift_side_m', 3, ' m')
    row('Side drift, RF2O vs start heading', 'drift_rf2o_m', 3, ' m')
    row('Heading change, wall fit', 'heading_change_wall_deg', 1, ' deg')
    row('Heading change, RF2O', 'heading_change_rf2o_deg', 1, ' deg')
    row('F-01 ground: rebound rise after the brake', 'rebound_rise_erpm', 0, ' ERPM')
    d = s.get('decel_lidar_mps2')
    if d:
        lines.append('- For the speed-fit checks (conservative): decel = min LiDAR value rounded down = %.1f m/s^2 '
                     '(vehicle_params dynamics.decel_mps2 changes only through a results row).' % (
                         math.floor(d['min'] * 10) / 10))
    return '\n'.join(lines)


def mode_transitions(statuses):
    """statuses: [(t, dict)] -> list of mode changes with the mux's reasons."""
    out, prev = [], None
    for t, st in statuses:
        mode = st.get('mode')
        if mode != prev:
            out.append(dict(t=t, from_mode=prev, to_mode=mode, reason=st.get('reason'),
                            last_stop_reason=st.get('last_stop_reason'),
                            mode_request_result=st.get('mode_request_result'),
                            front_valid_fraction=st.get('front_valid_fraction'),
                            nearest_front_m=st.get('nearest_front_m')))
            prev = mode
    return out


def self_test():
    t = [i * 0.02 for i in range(200)]
    # 12 Nov shape: decay to ~200 ERPM at 0.2 s, rise to 3911 at 0.4 s, then decay
    erpm = []
    for x in t:
        if x < 1.0:
            erpm.append(4200.0)
        elif x < 1.2:
            erpm.append(4200 - (x - 1.0) / 0.2 * 4000)
        elif x < 1.4:
            erpm.append(200 + (x - 1.2) / 0.2 * 3711)
        else:
            erpm.append(max(0.0, 3911 - (x - 1.4) * 4000))
    cur = [18.0 if 1.2 <= x < 1.4 else 1.0 for x in t]
    tach = [int(x * 1000) if x < 1.6 else 1600 for x in t]
    m = stop_metrics(t, erpm, cur, tach, 1.0, 4.0)
    assert not m['pass_no_rebound_over_300'] and 3600 < m['rebound_rise_erpm'] < 3800, m
    assert m['rebound_peak_erpm'] > 3800 and m['max_abs_current_after_zero_a'] == 18.0
    assert m['tach_counts_after_zero'] > 0 and m['time_to_under_100_erpm_s'] > 0.4
    assert abs(m['rebound_from_s'] - 0.2) < 0.03 and abs(m['rebound_at_s'] - 0.4) < 0.03, m
    # T1b brake 5 A stop 5: +198 'rebound' at speed (3930 -> 4128) before the brake acted
    tw = [1.0, 1.02, 1.04, 1.06, 1.08, 1.10, 1.30]
    ew = [3930.0, 4128.0, 3000.0, 1500.0, 150.0, 83.0, 0.0]
    m = stop_metrics(tw, ew, [3.0] * 7, None, 1.01, 3.0)
    assert m['rebound_rise_erpm'] < 100 and m['rebound_peak_erpm'] < 200 and m['pass_no_rebound_over_300'], m
    m = stop_metrics(t, [4200.0] * len(t), cur, None, 1.0, 4.0)        # never slows down
    assert not m['pass_no_rebound_over_300'] and not m['stopped'], m
    clean = [4200.0 if x < 1.0 else max(0.0, 4200 - (x - 1.0) * 8000) for x in t]
    m = stop_metrics(t, clean, cur, None, 1.0, 4.0)
    assert m['pass_no_rebound_over_300'] and abs(m['time_to_under_100_erpm_s'] - 0.52) < 0.03, m
    assert zero_transitions([0, 1, 2, 3, 4], [0, 4207, 4207, 0, 0]) == [3]
    tr = mode_transitions([(0, {'mode': 'STOP'}), (1, {'mode': 'MANUAL_READY'}),
                           (2, {'mode': 'MANUAL_READY'}),
                           (3, {'mode': 'STOP', 'last_stop_reason': 'JOYSTICK_DEADMAN_RELEASED_OR_STALE'})])
    assert [x['to_mode'] for x in tr] == ['STOP', 'MANUAL_READY', 'STOP']
    assert tr[-1]['last_stop_reason'].startswith('JOYSTICK')
    line = '[INFO] [%.3f] [vesc_driver]: VESC_SPEED_AUDIT received_erpm=%.3f sent_erpm=%.3f'
    audit = [line % (10.0, 4207, 3500), line % (10.05, 0, 0), '[INFO] [10.1] [vesc_driver]: other']
    st = [(9.0, {'speed_mps': 0.3}), (9.05, {'speed_mps': 0.0})]
    src, t, e = command_series(audit, st, [], 8414.0)
    assert src.startswith('vesc_speed_audit') and t == [10.0, 10.05] and e == [3500.0, 0.0]
    src, t, e = command_series([], st, [], 8414.0)
    assert src.startswith('/control/status') and abs(e[0] - 2524.2) < 1e-6 and e[1] == 0.0
    assert zero_transitions(t, e) == [9.05]
    assert command_series([], [], [], 8414.0) == (None, [], [])
    ph = [(0, {'phase': 'RELEASED'}), (1, {'phase': 'DRIVE'}), (2, {'phase': 'DRIVE'}),
          (3, {'phase': 'STOP_BRAKE'}), (5, {'phase': 'RELEASED'}), (6, {'phase': 'DRIVE'}),
          (7, {'phase': 'STOP_SPEED0'})]
    assert phase_exits(ph) == [3, 7]
    assert phase_entries(ph) == [1, 6]
    print('PASS: stop metrics (12 Nov rebound shape fails, clean decay passes), zero detection,')
    print('      mux transitions, command source (audit > mux status > legacy),')
    print('      T1b stop times from the converter phase. No bag read.')
    t4_self_test()


def t4_self_test():
    class Scan:
        angle_min, angle_max = -math.pi, math.pi
        angle_increment = 2 * math.pi / 1440
        range_min, range_max = 0.05, 30.0

    def wall_scan(d, psi, noise=0.0):
        """Wall at perpendicular distance d, car turned left by psi; returns only within +/-60 deg."""
        s = Scan()
        s.ranges = []
        for i in range(1440):
            a = s.angle_min + i * s.angle_increment
            c = math.cos(a + psi)        # wall normal at -psi in the car frame
            r = d / c if abs(a) < math.radians(60) and c > 0.1 else float('inf')
            s.ranges.append(r + noise * (((i * 7919) % 13) / 6.0 - 1.0))
        return s
    for d, psi in ((1.5, 0.0), (0.9, math.radians(5)), (2.6, math.radians(-8))):
        w = wall_fit(wall_scan(d, psi, noise=0.005), 10.0)
        assert w and abs(w['d'] - d) < 0.005 and abs(w['psi'] - psi) < math.radians(0.7), (d, psi, w)
    s = wall_scan(1.5, 0.0)
    s.ranges[720 + 20] = 0.3                                                  # one stray return: rejected
    w = wall_fit(s, 10.0)
    assert w and abs(w['d'] - 1.5) < 0.002, w
    s = Scan()
    s.ranges = [float('inf')] * 1440
    s.ranges[715:725] = [1.0] * 10                                            # too few points
    assert wall_fit(s, 10.0) is None
    assert abs(integrate([0, 1, 2], [0, 1, 1], 0.5, 2) - 1.375) < 1e-9 and integrate([0, 1], [1, 1], 2, 3) is None
    assert interp([0, 1], [0, 10], 0.25) == 2.5 and interp([0, 1], [0, 10], 2) is None

    # Simulated T4 repeat: 0.15 s start delay, 0.5 m/s^2 to 0.5 m/s, brake at 0.90 m, 2.0 m/s^2,
    # VESC gain 5 % higher than the converter's (true 8834.7 vs 8414), heading 3 deg left of the
    # wall normal, side walls 1.0 m left / 1.2 m right, RF2O at yaw 0.3 rad.
    gain, true_gain, psi, decel = 8414.0, 8414.0 * 1.05, math.radians(3.0), 2.0
    t_cmd, delay, d0, dt = 10.0, 0.15, 2.8, 0.001
    t, s_, v, t0, hist = 8.0, 0.0, 0.0, None, []
    while t < 18.5:                                                           # brake at ~14.46 s, hold 2 s
        if t0 is None and t >= t_cmd + delay:
            v = min(0.5, v + 0.5 * dt)
        if t0 is None and d0 - s_ * math.cos(psi) <= 0.9:
            t0 = t                                                            # runner and converter brake
        if t0 is not None:
            v = max(0.0, v - decel * dt)
        s_ += v * dt
        hist.append((t, s_, v))
        t += dt

    def at(period):
        return [h for k, h in enumerate(hist) if k % int(round(period / dt)) == 0]
    scan_t, feats = [], []
    for t, s_, v in at(1 / 30.0):
        y = s_ * math.sin(psi)
        d = d0 - s_ * math.cos(psi)
        scan_t.append(t)
        feats.append(dict(front=d / math.cos(psi), wall=dict(d=d, psi=psi), left=(1.0 - y) / math.cos(psi),
                          right=(1.2 + y) / math.cos(psi)))
    core = at(0.02)
    ct, erpm = [h[0] for h in core], [h[2] * true_gain for h in core]
    ot, vx = ct, [e / gain for e in erpm]
    rf = at(0.05)
    rt = [h[0] for h in rf]
    rpose = [(h[1] * math.cos(0.3), h[1] * math.sin(0.3), 0.3) for h in rf]
    x = t4_repeat(t0, t_cmd, t0 - 0.06, scan_t, feats, ct, erpm, ot, vx, rt, rpose, gain, 2.0, 0.5)
    stop_true = 0.25 / (2 * decel) * math.cos(psi)                           # along the wall normal
    assert x['moved'] and x['stopped'] and x['pre_still'] and x['still_ok'], x
    assert x['range_source'].startswith('wall'), x
    assert abs(x['start_delay_erpm_s'] - (delay + MOTION_ERPM / true_gain / 0.5)) < 0.03, x['start_delay_erpm_s']
    assert x['start_delay_lidar_s'] > x['start_delay_erpm_s']
    assert abs(x['stop_lidar_m'] - stop_true) < 0.01, (x['stop_lidar_m'], stop_true)
    assert abs(x['stop_vesc_m'] - 1.05 * 0.0625) < 0.005, x['stop_vesc_m']
    assert abs(x['decel_lidar_mps2'] - decel) < 0.25, x['decel_lidar_mps2']
    assert abs(x['gain_suggested_lidar'] - true_gain) < 0.01 * true_gain, x['gain_suggested_lidar']
    assert abs(x['gain_suggested_rf2o'] - true_gain) < 0.01 * true_gain, x['gain_suggested_rf2o']
    s_end = hist[-1][1]
    assert abs(x['drift_wall_m'] - s_end * math.sin(psi)) < 0.01, (x['drift_wall_m'], s_end * math.sin(psi))
    assert abs(x['drift_side_m'] - s_end * math.sin(psi)) < 0.01, x['drift_side_m']
    assert abs(x['drift_rf2o_m']) < 1e-6 and abs(x['heading_change_rf2o_deg']) < 1e-6
    assert abs(x['heading_change_wall_deg']) < 1e-6 and not x['curved']
    assert x['stop_from_runner_lidar_m'] > x['stop_lidar_m'] - 1e-9
    # Disarmed / no motion: start delay None, not counted
    y = t4_repeat(t0, t_cmd, None, scan_t, [dict(f, wall=None) for f in feats[:1]] * len(feats),
                  ct, [0.0] * len(ct), ot, [0.0] * len(ot), rt, rpose, gain, 2.0, 0.5)
    assert y['moved'] is False and y['range_source'].startswith('front'), y
    items = [dict(x, index=0, counted=True, runner_reason='BRAKE_POINT(ahead 0.90 m <= 0.90 m)'),
             dict(x, index=1, counted=True, gain_suggested_lidar=x['gain_suggested_lidar'] + 100),
             dict(y, index=2, counted=False, runner_reason='HARD_TIME_LIMIT')]
    sm = t4_summary(items)
    assert sm['counted'] == 2 and sm['repeats'] == 3 and abs(sm['gain_suggested_lidar']['sd'] - 70.71) < 0.01, sm
    text = t4_report('selftest', dict(items=items, summary=sm), gain)
    assert 'no motion' in text and 'For the speed-fit checks' in text and text.count('\n| ') == 4, text
    print('PASS: T4 wall fit (distance, heading, stray return, too few), integrate/interp, simulated ground')
    print('      repeat: start delay, stop distance L/V, decel, gain from LiDAR and RF2O (+5 %), drift')
    print('      wall/side/RF2O, heading; no-motion repeat; summary and report.')


# ---------------------------------------------------------------- bag

def read_bag(bag_dir):
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    meta = yaml.safe_load((Path(bag_dir) / 'metadata.yaml').read_text())
    storage = meta['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()}
    data = {name: [] for name in types}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        data[topic].append((t_ns * 1e-9, deserialize_message(raw, types[topic])))
    return data


def read_csv(path):
    if not path.exists():
        return []
    with open(path) as fp:
        return list(csv.DictReader(fp))


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def analyze_t4(info, data, stops, t1b, events, ct, erpm, gain):
    """T4 metrics per stop: the command start is the converter entering DRIVE before the stop
    (else the runner's DRIVE_START), the brake command the stop's t0."""
    p = vp.load()
    front_deg = float(vp.get(p, 'ground_check.front_window_deg'))
    wall_deg = float(vp.get(p, 't4.wall_fit_deg'))
    scans = data.get('/scan', [])
    scan_t = [t for t, _ in scans]
    feats = [scan_features(m, front_deg, wall_deg) for _, m in scans]
    vo = data.get('/odom_vesc_observe', [])
    ot, vx = [t for t, _ in vo], [m.twist.twist.linear.x for _, m in vo]
    rf = data.get('/odom_rf2o', [])
    rt = [t for t, _ in rf]
    rpose = [(m.pose.pose.position.x, m.pose.pose.position.y, quat_yaw(m.pose.pose.orientation)) for _, m in rf]
    entries = phase_entries(t1b)
    starts = [float(e['wall']) for e in events if e['event'] == 'DRIVE_START']
    runner_zeros = [float(e['wall']) for e in events if e['event'] == 'ZERO_COMMAND']
    hold = float((info.get('converter_params') or {}).get('brake_hold_s') or 2.0)
    ramp = float(info.get('ramp_accel_mps2') or vp.get(p, 'runner.ramp_accel_mps2'))
    items, prev_t0 = [], float('-inf')
    for st in stops:
        t0 = st['t0']
        conv = [t for t in entries if prev_t0 < t <= t0]
        run = [t for t in starts if prev_t0 < t <= t0]
        prev_t0 = t0
        reason = st.get('runner_reason') or ''
        base = dict(index=st['index'], t0=t0, runner_reason=reason,
                    rebound_rise_erpm=st.get('rebound_rise_erpm'),
                    rebound_pass=st.get('pass_no_rebound_over_300'),
                    reached_steady=st.get('reached_steady'))
        if not conv and not run:
            items.append(dict(base, moved=False, counted=False, note='no drive start before this stop'))
            continue
        t_cmd = conv[-1] if conv else run[-1]
        rz = [t for t in runner_zeros if t0 - REASON_MATCH_S <= t <= t0 + 1e-3]
        x = t4_repeat(t0, t_cmd, rz[-1] if rz else None, scan_t, feats, ct, erpm, ot, vx, rt, rpose,
                      gain, hold, ramp)
        x.update(base, t_cmd=t_cmd, cmd_source='converter DRIVE entry' if conv else 'runner DRIVE_START',
                 runner_to_cmd_s=(t_cmd - run[-1]) if (conv and run and run[-1] <= t_cmd) else None)
        x['counted'] = bool(st.get('reached_steady')) and reason.startswith('BRAKE_POINT') and x.get('moved', False)
        items.append(x)
    return dict(items=items, summary=t4_summary(items), gain=gain)


def analyze(folder, t4=False):
    folder = Path(folder).expanduser()
    bag_dir = folder / 'bag' if (folder / 'bag' / 'metadata.yaml').exists() else folder
    data = read_bag(bag_dir)
    summary = {'folder': str(folder)}
    run_info = folder / 'run_info.json'
    info = {}
    if run_info.exists():
        info = json.loads(run_info.read_text())
        summary['run'] = {k: info.get(k) for k in (
            'test', 'live', 'started', 'git_commit', 'mux_max_speed_mps', 'mux_max_steer_rad',
            'vesc_speed_max_erpm', 'publish_hz', 'ramp_accel_mps2')}

    # Topic rates and receive intervals (bag receive time).
    rates = {}
    for topic, msgs in sorted(data.items()):
        ts = [m[0] for m in msgs]
        hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] else None
        rates[topic] = dict(count=len(ts), hz=None if hz is None else round(hz, 2),
                            intervals=interval_stats([b - a for a, b in zip(ts, ts[1:])]))
        if topic in EXPECTED_HZ and hz is not None:
            rates[topic]['ok_vs_expected'] = hz >= 0.8 * EXPECTED_HZ[topic]
    missing = [t for t in EXPECTED_HZ if not data.get(t)]
    summary['topics'] = rates
    summary['expected_topics_missing'] = missing

    # F-09: scan lengths.
    summary['scan_lengths'] = dict(Counter(len(m.ranges) for _, m in data.get('/scan', [])))

    # F-13: the inputs as the bag received them, against the mux TTLs.
    for topic, ttl in (('/joy', JOY_TTL_S), ('/manual/drive', COMMAND_TTL_S)):
        iv = rates.get(topic, {}).get('intervals', {})
        if iv.get('count'):
            summary.setdefault('mux_input_freshness', {})[topic] = dict(
                ttl_s=ttl, max_gap_s=iv['max_s'], gaps_over_ttl=sum(
                    1 for a, b in zip(data[topic], data[topic][1:]) if b[0] - a[0] > ttl))
    pub = read_csv(folder / 'publish_intervals.csv')
    if pub:
        summary['runner_publish_intervals'] = interval_stats([float(r['dt_s']) for r in pub[1:]])

    # Mux transitions.
    statuses = []
    for t, m in data.get('/control/status', []):
        try:
            statuses.append((t, json.loads(m.data)))
        except ValueError:
            pass
    trans = mode_transitions(statuses)
    summary['mux_transitions'] = len(trans)
    summary['mux_stop_reasons'] = dict(Counter(x['last_stop_reason'] for x in trans if x['to_mode'] == 'STOP'))
    with open(folder / 'mux_transitions.csv', 'w', newline='') as fp:
        wr = csv.DictWriter(fp, ['t', 'from_mode', 'to_mode', 'reason', 'last_stop_reason',
                                 'mode_request_result', 'front_valid_fraction', 'nearest_front_m'])
        wr.writeheader()
        wr.writerows(trans)
    mux_out = [float(s.get('speed_mps') or 0.0) for _, s in statuses]
    summary['mux_out_max_speed_mps'] = max(mux_out) if mux_out else None

    # VESC telemetry and commands.
    core = data.get('/sensors/core', [])
    ct = [t for t, _ in core]
    erpm = [m.state.speed for _, m in core]
    cur = [m.state.current_motor for _, m in core]
    tach = [m.state.displacement for _, m in core]
    with open(folder / 'core.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['t', 'erpm', 'current_motor_a', 'current_input_a', 'duty', 'voltage_v',
                     'tach_displacement', 'fault'])
        for t, m in core:
            s = m.state
            wr.writerow([t, s.speed, s.current_motor, s.current_input, s.duty_cycle,
                         s.voltage_input, s.displacement, s.fault_code])
    audit_path = folder / 'vesc_speed_audit.log'
    audit_lines = audit_path.read_text().splitlines() if audit_path.exists() else []
    gain = float(vp.get(vp.load(), 'drivetrain.speed_to_erpm_gain'))
    cmd_source, mt, mv = command_series(audit_lines, statuses, data.get('/commands/motor/speed', []), gain)
    summary['command_source'] = cmd_source
    with open(folder / 'motor_cmd.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['t', 'erpm_cmd', 'source'])
        wr.writerows((t, v, cmd_source) for t, v in zip(mt, mv))
    if core:
        summary['vesc'] = dict(max_abs_erpm=round(max(abs(x) for x in erpm), 1),
                               max_abs_current_a=round(max(abs(x) for x in cur), 2),
                               faults=sorted(set(m.state.fault_code for _, m in core)),
                               voltage_min_v=round(min(m.state.voltage_input for _, m in core), 2))
    if mv:
        summary['max_erpm_command'] = max(mv)
    if statuses and not any(s.get('real_actuation_enabled') for _, s in statuses):
        # Disarmed proof: the mux reported zero output AND (if logged) vesc_driver sent zero.
        # None = no data, which is not a pass.
        summary['disarmed_mux_output_all_zero'] = max(mux_out) == 0.0
        audit_sent = [a[2] for a in (parse_audit_line(x) for x in audit_lines) if a]
        summary['disarmed_vesc_sent_all_zero'] = (
            max(abs(x) for x in audit_sent) == 0.0) if audit_sent else None

    # Stop times, best source first: the T1b converter's own switch to its stop method; the
    # sent speed command (audit) falling to zero; the mux output; the runner's events.
    events = read_csv(folder / 'runner_events.csv')
    t1b = []
    for t, m in data.get('/t1b_stop_converter/status', []):
        try:
            t1b.append((t, json.loads(m.data)))
        except ValueError:
            pass
    if t1b:
        summary['t1b_methods'] = sorted(set(s.get('method') for _, s in t1b))
    candidates = [
        ('/t1b_stop_converter/status (left DRIVE)', phase_exits(t1b)),
        (cmd_source, zero_transitions(mt, mv)),
        ('/control/status speed_mps', zero_transitions([t for t, _ in statuses], [x * gain for x in mux_out])),
        ('runner_events.csv (command never nonzero)',
         [float(e['wall']) for e in events if e['event'] == 'ZERO_COMMAND']),
    ]
    source, zeros = next(((s, z) for s, z in candidates if z), (None, []))
    # Reasons by time, not by order: a repeat latched before it moved has no zero transition.
    zero_events = [(float(e['wall']), e['detail']) for e in events if e['event'] == 'ZERO_COMMAND']
    speed = (info.get('profile') or {}).get('speed')
    target_erpm = float(speed) * gain if speed else None
    stops = []
    for k, t0 in enumerate(zeros):
        m = stop_metrics(ct, erpm, cur, tach, t0, t0 + STOP_WINDOW_S) if core else {'samples': 0}
        m.update(index=k, t0=t0, stop_method=info.get('stop_method'))
        near = sorted((abs(t - t0), d) for t, d in zero_events if abs(t - t0) <= REASON_MATCH_S)
        if near:
            m['runner_reason'] = near[0][1]
        after = [s for t, s in t1b if t >= t0]
        if after:  # what the T1b converter actually sent at this stop
            m['t1b_action'] = '%s %s (%s)' % (after[0].get('out'), after[0].get('value'), after[0].get('phase'))
        if target_erpm and m.get('samples'):
            # Only a stop from (near) the step speed counts for the step gate.
            m['target_erpm'] = round(target_erpm)
            m['reached_steady'] = m['erpm_at_zero'] >= STEADY_FRACTION * target_erpm
        stops.append(m)
    summary['stops'] = dict(source=source, count=len(stops), items=stops,
                            steady_stops=sum(1 for s in stops if s.get('reached_steady')))
    if stops and any(s.get('samples') for s in stops):
        summary['stops']['all_pass_no_rebound_over_300'] = all(
            s.get('pass_no_rebound_over_300', True) for s in stops)

    if t4 or info.get('test') == T4_TEST:
        summary['t4'] = analyze_t4(info, data, stops, t1b, events, ct, erpm, gain)

    plots = make_plots(folder, ct, erpm, cur, mt, mv, zeros, pub, data)
    summary['plots'] = plots
    (folder / 'summary.json').write_text(json.dumps(summary, indent=2, default=str))
    write_readme(folder, summary)
    return summary


# ---------------------------------------------------------------- output

def style(ax, title, ylabel):
    ax.set_title(title, loc='left', fontsize=10, color=INK)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
    ax.grid(True, color=GRID, linewidth=0.6)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)


def make_plots(folder, ct, erpm, cur, mt, mv, zeros, pub, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    made = []
    if ct and zeros:
        # One axis per measure (never dual-axis): ERPM on top, motor current below.
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for k, t0 in enumerate(zeros[:len(REPEAT_COLORS)]):
            sel = [i for i, x in enumerate(ct) if t0 - 1.0 <= x <= t0 + STOP_WINDOW_S]
            c = REPEAT_COLORS[k]
            a1.plot([ct[i] - t0 for i in sel], [erpm[i] for i in sel], color=c, lw=2, label='stop %d' % (k + 1))
            a2.plot([ct[i] - t0 for i in sel], [cur[i] for i in sel], color=c, lw=2, label='stop %d' % (k + 1))
        for a in (a1, a2):
            a.axvline(0, color=MUTED, lw=1, ls='--')
        a1.axhline(REBOUND_FAIL_ERPM, color=MUTED, lw=1, ls=':')
        a1.text(STOP_WINDOW_S, REBOUND_FAIL_ERPM, ' 300 ERPM', color=MUTED, fontsize=8, va='bottom', ha='right')
        style(a1, 'Measured ERPM around each zero command (t = 0)', 'ERPM')
        style(a2, 'Motor current', 'A')
        a2.set_xlabel('time from zero command [s]', color=MUTED, fontsize=9)
        a1.legend(frameon=False, fontsize=8, ncol=4)
        fig.tight_layout()
        fig.savefig(folder / 'stops.png', dpi=120)
        plt.close(fig)
        made.append('stops.png')
    joy = [t for t, _ in data.get('/joy', [])]
    if pub or len(joy) > 1:
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(9, 3.2))
        sent = [float(r['dt_s']) * 1000 for r in pub[1:]]
        got = [(b - a) * 1000 for a, b in zip(joy, joy[1:])]
        # Same 0..TTL scale and 2 ms bins on both panels: what matters is the distance to the mux TTL.
        right = max([JOY_TTL_S * 1000 * 1.1] + sent + got)
        bins = [2.0 * k for k in range(int(right / 2) + 2)]
        if sent:
            a1.hist(sent, bins=bins, color=BLUE)
        style(a1, 'Runner publish interval (sent)', 'count')
        a1.set_xlabel('ms', color=MUTED, fontsize=9)
        if got:
            a2.hist(got, bins=bins, color=BLUE)
        for a in (a1, a2):
            a.set_xlim(0, right)
            a.axvline(JOY_TTL_S * 1000, color=MUTED, lw=1, ls='--')
            a.text(JOY_TTL_S * 1000, a.get_ylim()[1] * 0.9, ' JOY_TTL', color=MUTED, fontsize=8)
        style(a2, '/joy interval (bag received)', 'count')
        a2.set_xlabel('ms', color=MUTED, fontsize=9)
        fig.tight_layout()
        fig.savefig(folder / 'intervals.png', dpi=120)
        plt.close(fig)
        made.append('intervals.png')
    return made


def plot_stop_detail(folder, n, before_s=0.5, after_s=1.5):
    """stop<n>_detail.png from core.csv, motor_cmd.csv, summary.json (no bag read):
    measured and sent ERPM, motor current, duty around stop n (1-based)."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    folder = Path(folder).expanduser()
    s = json.loads((folder / 'summary.json').read_text())
    item = s['stops']['items'][n - 1]
    t0 = float(item['t0'])
    core = [r for r in read_csv(folder / 'core.csv') if -before_s <= float(r['t']) - t0 <= after_s]
    cmd = [r for r in read_csv(folder / 'motor_cmd.csv') if -before_s - 0.2 <= float(r['t']) - t0 <= after_s]
    tc = [float(r['t']) - t0 for r in core]
    fig, axes = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
    a1, a2, a3 = axes
    a1.plot(tc, [float(r['erpm']) for r in core], color=BLUE, lw=2, label='measured ERPM (/sensors/core)')
    if cmd:
        a1.step([float(r['t']) - t0 for r in cmd], [float(r['erpm_cmd']) for r in cmd], where='post',
                color=ORANGE, lw=2, label='sent ERPM (%s)' % cmd[0]['source'].split(' ')[0])
    if item.get('rebound_at_s') is not None:
        a1.annotate('peak %.0f ERPM at +%.3f s' % (item['rebound_peak_erpm'], item['rebound_at_s']),
                    xy=(item['rebound_at_s'], item['rebound_peak_erpm']),
                    xytext=(item['rebound_at_s'] + 0.15, item['rebound_peak_erpm']),
                    color=INK, fontsize=8, arrowprops=dict(arrowstyle='-', color=MUTED, lw=1))
    a2.plot(tc, [float(r['current_motor_a']) for r in core], color=BLUE, lw=2)
    a3.plot(tc, [float(r['duty']) for r in core], color=BLUE, lw=2)
    for a in axes:
        a.axvline(0, color=MUTED, lw=1, ls='--')
    a1.text(0, a1.get_ylim()[1], ' zero sent', color=MUTED, fontsize=8, va='top')
    style(a1, 'Stop %d (%s): ERPM, measured and sent' % (n, item.get('stop_method')), 'ERPM')
    style(a2, 'Motor current', 'A')
    style(a3, 'Duty cycle', 'duty')
    a3.set_xlabel('time from zero command [s]', color=MUTED, fontsize=9)
    a1.legend(frameon=False, fontsize=8, loc='upper right')
    fig.tight_layout()
    out = folder / ('stop%d_detail.png' % n)
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out, item


def runs_table(root, repeats=5):
    """Markdown comparison of every analysed T1 / T1b run under root (one row per run)."""
    head = ('| Run | Method | Speed m/s | Steady / measured stops | Stops failing | Max rebound rise (ERPM) '
            '| Max rebound peak (ERPM) | Worst settle < 100 ERPM (s) | Max current after zero (A) '
            '| Max tach after zero | Verdict |\n|---|---|---|---|---|---|---|---|---|---|---|')
    rows = []
    for path in sorted(Path(root).expanduser().glob('*_t1*_stop_v*/summary.json')):
        folder = path.parent
        s = json.loads(path.read_text())
        info = json.loads((folder / 'run_info.json').read_text())
        items = [x for x in s['stops']['items'] if x.get('samples')]
        if not items or not info.get('live'):
            continue
        steady = [x for x in items if x.get('reached_steady')]
        fails = [x for x in items if not x.get('pass_no_rebound_over_300')]
        settle = [x.get('time_to_under_100_erpm_s') for x in items]
        if fails:
            verdict = 'FAIL'
        elif len(steady) >= repeats and s.get('vesc', {}).get('faults') == [0]:
            verdict = 'PASS'
        else:
            verdict = 'INCOMPLETE (%d steady)' % len(steady)
        rows.append('| `%s` | %s | %.2f | %d / %d | %d | %.0f | %.0f | %s | %.1f | %s | %s |' % (
            folder.name[:15], info.get('stop_method'), float(info['profile']['speed']), len(steady),
            len(items), len(fails), max(x['rebound_rise_erpm'] for x in items),
            max(x['rebound_peak_erpm'] for x in items),
            'not stopped' if None in settle else '%.3f' % max(settle),
            max(x['max_abs_current_after_zero_a'] for x in items),
            max((x.get('tach_counts_after_zero') or 0) for x in items), verdict))
    return head + '\n' + '\n'.join(rows)


def write_readme(folder, s):
    lines = ['## Analysis (analyze.py)', '']
    rp = s.get('runner_publish_intervals')
    if rp:
        lines.append('- Runner publish: p50 %.1f ms, p99 %.1f ms, max %.1f ms, %d gaps > 0.20 s' % (
            rp['p50_s'] * 1e3, rp['p99_s'] * 1e3, rp['max_s'] * 1e3, rp['over_0.20_s']))
    for topic, f in s.get('mux_input_freshness', {}).items():
        lines.append('- %s received: max gap %.3f s, %d gaps over TTL %.2f s' % (
            topic, f['max_gap_s'], f['gaps_over_ttl'], f['ttl_s']))
    lines.append('- Scan lengths: %s' % s['scan_lengths'])
    low = [t for t, r in s['topics'].items() if r.get('ok_vs_expected') is False]
    lines.append('- Topic rates below 80 %% of expected: %s; missing: %s' % (low or 'none',
                                                                             s['expected_topics_missing'] or 'none'))
    lines.append('- Mux STOP reasons: %s' % s['mux_stop_reasons'])
    lines.append('- Command source: %s' % s.get('command_source'))
    if 'disarmed_mux_output_all_zero' in s:
        lines.append('- Disarmed: mux output always zero = %s; vesc_driver sent ERPM always zero = %s' % (
            s['disarmed_mux_output_all_zero'], s['disarmed_vesc_sent_all_zero']))
    st = s['stops']
    lines.append('- Stops: %d (from %s), %d from >= 80 %% of the step ERPM (count for the step gate)' % (
        st['count'], st['source'], st.get('steady_stops', 0)))
    for x in st['items']:
        if x.get('samples'):
            lines.append('  - stop %d [%s%s]: at zero %.0f ERPM%s, < 100 ERPM after %s s, rebound +%.0f ERPM '
                         '(peak %.0f, dip +%s s, peak +%s s), max current %.1f A, tach %s, pass=%s, reason %s' % (
                             x['index'] + 1, x.get('stop_method'),
                             ', ' + x['t1b_action'] if x.get('t1b_action') else '',
                             x['erpm_at_zero'], '' if x.get('reached_steady', True) else ' (NOT steady)',
                             x['time_to_under_100_erpm_s'], x['rebound_rise_erpm'], x['rebound_peak_erpm'],
                             x.get('rebound_from_s'), x.get('rebound_at_s'),
                             x['max_abs_current_after_zero_a'], x.get('tach_counts_after_zero'),
                             x['pass_no_rebound_over_300'], x.get('runner_reason')))
    if 't4' in s:
        lines += ['', t4_report(folder.name, s['t4'], s['t4']['gain'])]
    readme = folder / 'README.md'
    text = readme.read_text() if readme.exists() else ''
    text = text.split('## Analysis (analyze.py)')[0].rstrip() + '\n\n' + '\n'.join(lines) + '\n'
    readme.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='*')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--table', nargs='?', const=str(vp.evidence_root()), metavar='ROOT',
                    help='markdown comparison of all analysed T1/T1b runs (default evidence root)')
    ap.add_argument('--stop-plot', type=int, metavar='N',
                    help='after analysis: stopN_detail.png, -0.5..+1.5 s around stop N (1-based)')
    ap.add_argument('--t4', action='store_true',
                    help='T4 metrics per repeat + summary (default: latest t4_straight folder; '
                         'several folders: pooled summary as well)')
    args = ap.parse_args()
    if args.self_test:
        self_test()
    elif args.table:
        print(runs_table(args.table))
    elif args.t4:
        folders = args.folder or [str(p) for p in sorted(vp.evidence_root().glob('*_%s_v*' % T4_TEST))[-1:]]
        if not folders:
            ap.error('no %s folder under %s' % (T4_TEST, vp.evidence_root()))
        pooled, gain = [], None
        for f in folders:
            s = analyze(f, t4=True)
            gain = s['t4']['gain']
            print(t4_report(Path(f).expanduser().name, s['t4'], gain) + '\n')
            pooled += s['t4']['items']
        if len(folders) > 1:
            print(t4_report('pooled (%d folders)' % len(folders),
                            dict(items=pooled, summary=t4_summary(pooled)), gain, table=False))
    elif len(args.folder) > 1:
        ap.error('one folder (several only with --t4)')
    elif args.folder and args.stop_plot and (Path(args.folder[0]).expanduser() / 'summary.json').exists():
        out, item = plot_stop_detail(args.folder[0], args.stop_plot)
        print('%s\n%s' % (out, json.dumps(item, indent=1)))
    elif args.folder:
        s = analyze(args.folder[0])
        print((Path(args.folder[0]).expanduser() / 'README.md').read_text().split('## Analysis (analyze.py)')[-1])
        print('summary.json, CSVs and %s written.' % ', '.join(s['plots'] or ['no plots']))
    else:
        ap.error('folder or --self-test')


if __name__ == '__main__':
    main()
