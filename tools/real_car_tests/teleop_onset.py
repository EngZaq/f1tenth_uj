#!/usr/bin/env python3
"""PS4 teleop stand / floor checks from a teleop_ps4 run folder (28 Nov). Offline, reads the bag + runner_events.csv.

For every stop event of the runner (DEADMAN_RELEASED, BRAKE_BUTTON "L1 pressed", STICK_STOP, KILL, CONTROLLER_LOST):
  reference = the event time, except CONTROLLER_LOST: the last input before the loss (PS4_LINK_LOST: the link
              heartbeat's last report; otherwise the last /ps4/joy message), from the event detail or the bag.
  onset     = first /t1b_stop_converter/status with phase STOP_BRAKE after the reference, minus the reference.
              "not moving" when the converter was not in DRIVE at the reference (nothing to brake).
Controller switched off on the stand: PASS onset < 0.3 s; 0.3..0.5 s MARGINAL; >= 0.5 s -> use the USB cable for the
floor runs (SESSION_HANDOFF). For BRAKE_BUTTON on the floor: speed at the press (/sensors/core ERPM / gain), VESC
distance from the press to |ERPM| < 100, the LiDAR front median (+/-3 deg) change over the same time, and the
estimate v * onset + v^2 / (2 * floor_decel_mps2) (no deadman latency: the operator presses L1).

  python3 teleop_onset.py <run folder>
  python3 teleop_onset.py --self-test
"""
import argparse
import csv
import json
import math
import os
import re
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

STOP_EVENTS = ('DEADMAN_RELEASED', 'BRAKE_BUTTON', 'STICK_STOP', 'KILL', 'CONTROLLER_LOST')
PASS_S, USB_S = 0.3, 0.5
STOPPED_ERPM = 100.0


def wall_from(detail, key):
    m = re.search(r'%s=([0-9.]+)' % key, detail or '')
    return float(m.group(1)) if m else None


def reference(ev, joy_times):
    """Reference wall time of a stop event (see module doc)."""
    t = float(ev['wall'])
    if ev['event'] != 'CONTROLLER_LOST':
        return t, 'event'
    if ev['detail'].startswith('PS4_LINK_LOST'):
        link = wall_from(ev['detail'], 'last_link_wall')
        if link is not None:
            return link, 'last link report'
    joy = wall_from(ev['detail'], 'last_joy_wall')
    before = [x for x in joy_times if x <= t]
    if before:
        return before[-1], 'last /ps4/joy in the bag'
    if joy is not None:
        return joy, 'last /ps4/joy (runner)'
    return t, 'event (no earlier input known)'


def phase_at(conv, t):
    """conv = [(t, phase)] sorted; phase of the last status at or before t (None before the first)."""
    ph = None
    for tc, p in conv:
        if tc > t:
            break
        ph = p
    return ph


def brake_onset(conv, t_ref):
    if phase_at(conv, t_ref) != 'DRIVE':
        return None
    for tc, p in conv:
        if tc > t_ref and p == 'STOP_BRAKE':
            return tc - t_ref
    return float('inf')


def stop_distance(erpm, gain, t0):
    """erpm = [(t, erpm)] sorted -> (speed m/s at t0, VESC distance m until |ERPM| < STOPPED_ERPM, time of the stop)."""
    seg = [(t, e) for t, e in erpm if t >= t0]
    if len(seg) < 2:
        return None, None, None
    v0, dist = abs(seg[0][1]) / gain, 0.0
    for (ta, ea), (tb, eb) in zip(seg, seg[1:]):
        if abs(ea) < STOPPED_ERPM:
            return v0, dist, ta
        dist += 0.5 * (abs(ea) + abs(eb)) / gain * (tb - ta)
    return v0, dist, None


def nearest_scan(scans, t, before=False):
    """scans = [(t, front median m)] -> the first scan at or after t, or (before=True) the last one at or before t."""
    if before:
        prior = [r for ts, r in scans if ts <= t]
        return prior[-1] if prior else None
    for ts, r in scans:
        if ts >= t:
            return r
    return None


def verdict(onset):
    if onset is None:
        return 'not moving'
    if not math.isfinite(onset):
        return 'NO BRAKE SEEN'
    if onset < PASS_S:
        return 'PASS (< %.1f s)' % PASS_S
    if onset < USB_S:
        return 'MARGINAL (%.1f..%.1f s)' % (PASS_S, USB_S)
    return 'FAIL (>= %.1f s): use the USB cable for floor runs' % USB_S


def analyse(events, joy_times, conv, erpm, scans, gain, decel):
    rows = []
    for ev in events:
        if ev['event'] not in STOP_EVENTS:
            continue
        if ev['event'] == 'BRAKE_BUTTON' and not ev['detail'].startswith('L1 pressed'):
            continue
        t_ref, src = reference(ev, joy_times)
        onset = brake_onset(conv, t_ref)
        row = dict(event=ev['event'], wall=float(ev['wall']), detail=ev['detail'], reference=src,
                   reference_wall=t_ref, onset_s=onset, verdict=verdict(onset))
        if ev['event'] == 'BRAKE_BUTTON' and onset is not None:
            v0, dist, t_stop = stop_distance(erpm, gain, t_ref)
            r0, r1 = nearest_scan(scans, t_ref, before=True), (nearest_scan(scans, t_stop) if t_stop else None)
            row.update(speed_mps=v0, vesc_distance_m=dist,
                       lidar_distance_m=(r0 - r1) if (r0 is not None and r1 is not None) else None,
                       estimate_m=(v0 * (onset if math.isfinite(onset) else 0.0) + v0 * v0 / (2 * decel))
                       if v0 is not None else None)
        rows.append(row)
    return rows


def fmt(x, nd=3):
    return '-' if x is None else ('inf' if not math.isfinite(x) else '%.*f' % (nd, x))


def report(rows):
    out = ['| event | reference | brake onset s | verdict | speed m/s | VESC stop m | LiDAR stop m | estimate m |',
           '|---|---|---|---|---|---|---|---|']
    for r in rows:
        out.append('| %s | %s | %s | %s | %s | %s | %s | %s |' % (
            r['event'], r['reference'], fmt(r['onset_s']), r['verdict'], fmt(r.get('speed_mps'), 2),
            fmt(r.get('vesc_distance_m')), fmt(r.get('lidar_distance_m')), fmt(r.get('estimate_m'))))
    return '\n'.join(out)


def load_run(folder):
    import analyze
    from run_test import front_range
    folder = Path(folder)
    with open(folder / 'runner_events.csv') as fp:
        events = list(csv.DictReader(fp))
    data = analyze.read_bag(folder / 'bag')
    joy = [t for t, _ in data.get('/ps4/joy', [])]
    conv = []
    for t, m in data.get('/t1b_stop_converter/status', []):
        try:
            conv.append((t, json.loads(m.data).get('phase')))
        except ValueError:
            continue
    erpm = [(t, float(m.state.speed)) for t, m in data.get('/sensors/core', [])]
    scans = [(t, front_range(m, 3.0)) for t, m in data.get('/scan', [])]
    scans = [(t, r) for t, r in scans if math.isfinite(r)]
    return events, joy, conv, erpm, scans


def self_test():
    gain, decel = 8414.0, 0.9
    conv = ([(10.0 + 0.02 * k, 'DRIVE') for k in range(50)]                          # 10.00 .. 10.98 driving
            + [(11.0 + 0.02 * k, 'STOP_BRAKE') for k in range(100)]                  # brake from 11.00
            + [(13.0 + 0.02 * k, 'RELEASED') for k in range(50)])
    erpm = [(10.0 + 0.01 * k, 2524.0) for k in range(100)] + [
        (11.0 + 0.01 * k, max(0.0, 2524.0 - 2524.0 / 0.33 * 0.01 * k)) for k in range(100)]
    scans = [(10.9, 2.00), (11.5, 1.85)]
    joy = [10.0 + 0.02 * k for k in range(40)]                                       # last /ps4/joy 10.78
    ev = [dict(wall='10.5', event='ARMED', detail=''),
          dict(wall='11.03', event='CONTROLLER_LOST', detail='PS4_STALE(0.21 s) last_joy_wall=10.78 '
                                                             'last_link_wall=None'),
          dict(wall='10.99', event='BRAKE_BUTTON', detail='L1 pressed: brake 5 A'),
          dict(wall='11.20', event='BRAKE_BUTTON', detail='L1 released: centre'),
          dict(wall='14.0', event='KILL', detail='key PS4_KILL')]
    rows = analyse(ev, joy, conv, erpm, scans, gain, decel)
    assert [r['event'] for r in rows] == ['CONTROLLER_LOST', 'BRAKE_BUTTON', 'KILL'], rows
    lost, l1, kill = rows
    assert lost['reference'] == 'last /ps4/joy in the bag' and abs(lost['onset_s'] - 0.22) < 1e-6, lost
    assert lost['verdict'].startswith('PASS')
    assert abs(l1['onset_s'] - 0.01) < 1e-6 and abs(l1['speed_mps'] - 0.3) < 1e-3, l1
    assert 0.03 < l1['vesc_distance_m'] < 0.06 and abs(l1['lidar_distance_m'] - 0.15) < 1e-9, l1
    assert kill['onset_s'] is None and kill['verdict'] == 'not moving'
    link_ev = dict(wall='11.03', event='CONTROLLER_LOST', detail='PS4_LINK_LOST(no report for 0.20 s) '
                                                                 'last_joy_wall=11.02 last_link_wall=10.50')
    assert reference(link_ev, joy) == (10.50, 'last link report')
    assert verdict(0.29).startswith('PASS') and verdict(0.35).startswith('MARGINAL')
    assert 'USB' in verdict(0.5) and verdict(float('inf')) == 'NO BRAKE SEEN'
    assert brake_onset([(1.0, 'DRIVE'), (2.0, 'DRIVE')], 1.5) == float('inf')
    assert '| CONTROLLER_LOST |' in report(rows)
    print('PASS: teleop_onset: reference per event (last /ps4/joy, last link report, event time), brake onset from')
    print('      the converter STOP_BRAKE, verdict < 0.3 / 0.3-0.5 / >= 0.5 s (USB), L1 stop distance (VESC, LiDAR,')
    print('      estimate), not-moving events, released-L1 rows skipped. No ROS, no bag.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.folder:
        ap.error('run folder or --self-test')
    p, ps4 = vp.load(), vp.load_ps4()
    rows = analyse(*load_run(args.folder), float(vp.get(p, 'drivetrain.speed_to_erpm_gain')),
                   float(ps4['floor_decel_mps2']))
    text = report(rows)
    print(text)
    (Path(args.folder) / 'teleop_onset.md').write_text(text + '\n')
    (Path(args.folder) / 'teleop_onset.json').write_text(json.dumps(rows, indent=2, default=str))
    print('written: %s' % (Path(args.folder) / 'teleop_onset.md'))


if __name__ == '__main__':
    main()
