#!/usr/bin/env python3
"""VESC firmware timeout check on the stand (21 Nov). Reads the bag of a run_test.py vesc_timeout_check run.

Method: wheels spin at 0.5 m/s (speed command, converter t1b in DRIVE); the operator kills the converter with
SIGKILL (no goodbye message) while vesc_driver keeps reporting /sensors/core at 50 Hz. From then on the VESC
gets no command at all, so only its own timeout (VESC Tool App Settings > General: timeout_msec,
timeout_brake_current) can act. vesc_driver sends no keep-alive (vesc_driver.cpp timer: requestState +
requestImuData only), so the polling does not reset the timeout.
  t_kill  = last /t1b_stop_converter/status (50 Hz, published with every command)
  onset   = first /sensors/core sample after t_kill whose motor current fell below 25 % of the pre-kill
            drive current, or ERPM below 90 % of the pre-kill ERPM
  braking = minimum current_motor in [onset - 0.02, onset + 0.20] s; stand T1b 14 Nov: brake 5 A gives
            -2.3..-3.7 A in the first 40 ms, release (current 0) gives exactly 0.00 A
PASS: onset - t_kill <= vesc_timeout_check.onset_max_s AND braking <= -brake_current_min_a.
INVALID: converter not in DRIVE at its last message (SPACE released first), or it came back within
converter_gap_min_s. Writes timeout_check.json (run_test.py t3_circle reads it: a PASS lifts the T3 fallback).
Run it once BEFORE changing the VESC settings (expected FAIL: ~1 s, coasting) and once after.
  vesc_timeout_check.py <vesc_timeout_check run folder>
  vesc_timeout_check.py --self-test
"""
import argparse
import json
import os
import statistics
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402


def timeout_metrics(status, core, cfg):
    """status = [(t, phase)], core = [(t, erpm, current_motor)] (bag receive time, s). -> result dict."""
    if not status or not core:
        return dict(pass_=False, verdict='INVALID: no converter status or no /sensors/core')
    status, core = sorted(status), sorted(core)
    # the kill = the first gap >= converter_gap_min_s after DRIVE (the converter may have run before)
    gap_min = float(cfg['converter_gap_min_s'])
    t_kill, phase = None, None
    for (t, ph), (t2, _) in zip(status, status[1:] + [(float('inf'), None)]):
        if t2 - t >= gap_min and core[-1][0] - t >= gap_min:
            t_kill, phase = t, ph
            break
    if t_kill is None:
        return dict(pass_=False, verdict='INVALID: the converter never stopped for >= %.1f s' % gap_min)
    if phase != 'DRIVE':
        return dict(pass_=False, t_kill=t_kill, last_phase=phase,
                    verdict='INVALID: converter was in %s at its last message (kill it while SPACE is held)' % phase)
    pre = [(e, c) for t, e, c in core if t_kill - 1.0 <= t <= t_kill]
    if len(pre) < 10:
        return dict(pass_=False, verdict='INVALID: < 10 telemetry samples in the 1 s before the kill')
    e0, c0 = statistics.mean(e for e, _ in pre), statistics.mean(c for _, c in pre)
    if abs(e0) < 1000 or c0 <= 0.3 or min(abs(e) for e, _ in pre) < 0.8 * abs(e0):
        return dict(pass_=False, verdict='INVALID: wheels not driven steadily in the 1 s before the kill '
                    '(mean %.0f ERPM, %.2f A)' % (e0, c0))
    after = [(t, e, c) for t, e, c in core if t > t_kill]
    onset = next(((t, e, c) for t, e, c in after if c < 0.25 * c0 or abs(e) < 0.9 * abs(e0)), None)
    if onset is None:
        return dict(pass_=False, t_kill=t_kill, erpm_pre=e0, current_pre=c0,
                    verdict='FAIL: the VESC kept driving after the converter died (no timeout?)')
    win = [c for t, e, c in core if onset[0] - 0.02 <= t <= onset[0] + 0.20]
    i_min = min(win)
    stop = next((t for t, e, c in after if t >= onset[0] and abs(e) < 300), None)
    delay = onset[0] - t_kill
    braked = i_min <= -float(cfg['brake_current_min_a'])
    ok = delay <= float(cfg['onset_max_s']) and braked
    verdict = '%s: timeout onset %.2f s after the last command (limit %.2f), %s (min current %+.2f A)' % (
        'PASS' if ok else 'FAIL', delay, float(cfg['onset_max_s']),
        'BRAKED' if braked else 'released / coasting', i_min)
    return dict(pass_=ok, verdict=verdict, t_kill=t_kill, onset_delay_s=delay, erpm_pre=e0, current_pre=c0,
                current_min_after_onset_a=i_min, stop_below_300erpm_s=(stop - t_kill) if stop else None)


def self_test():
    cfg = vp.get(vp.load(), 'vesc_timeout_check')

    def synth(timeout_s, brake, kill_phase='DRIVE', comeback=False):
        status = [(k * 0.02, 'DRIVE') for k in range(250)]            # 5 s at 50 Hz, killed at 4.98 s
        status[-1] = (status[-1][0], kill_phase)
        if comeback:                                                  # 0.5 s hiccup, then runs to the end
            status += [(5.5 + k * 0.02, 'DRIVE') for k in range(200)]
        core, t = [], 0.0
        while t < 9.0:
            if t <= 4.98 + timeout_s:
                e, c = 4200.0, 1.8
            else:
                dt = t - 4.98 - timeout_s
                e = max(0.0, 4200.0 * (1 - dt / (0.06 if brake else 0.12)))
                c = (-3.2 if dt < 0.04 else -0.1) if brake else 0.0
                c = c if e > 0 else 0.0
            core.append((t + 0.001, e, c))
            t += 0.02
        return status, core
    r = timeout_metrics(*synth(0.25, True), cfg)
    assert r['pass_'] and 0.24 <= r['onset_delay_s'] <= 0.30 and r['verdict'].startswith('PASS'), r
    r = timeout_metrics(*synth(1.0, False), cfg)                       # 24 Oct behaviour: ~1 s, coasting
    assert not r['pass_'] and 0.99 <= r['onset_delay_s'] <= 1.05 and 'coasting' in r['verdict'], r
    r = timeout_metrics(*synth(0.25, False), cfg)                      # fast timeout but brake current 0
    assert not r['pass_'] and 'coasting' in r['verdict'], r
    r = timeout_metrics(*synth(1.0, True), cfg)                        # brakes, but too late
    assert not r['pass_'] and 'BRAKED' in r['verdict'], r
    r = timeout_metrics(*synth(0.25, True, kill_phase='STOP_BRAKE'), cfg)
    assert r['verdict'].startswith('INVALID: converter was in STOP_BRAKE'), r
    r = timeout_metrics(*synth(0.25, True, comeback=True), cfg)       # gap 0.5 s < 1 s: not a kill
    assert r['verdict'].startswith('INVALID: the converter never stopped'), r
    st_, co_ = synth(0.25, True)
    r = timeout_metrics(st_ + [(5.6, 'DRIVE')], co_, cfg)               # killed later, wheels already stopped
    assert r['verdict'].startswith('INVALID: wheels not driven steadily'), r
    print('PASS: VESC timeout check: 0.25 s + brake -> PASS; 1 s coasting (24 Oct), 0.25 s coasting, 1 s braked')
    print('      -> FAIL; killed outside DRIVE / converter back within 1 s / wheels not steady -> INVALID. No ROS.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.folder:
        ap.error('folder or --self-test')
    import analyze as A
    folder = Path(args.folder).expanduser()
    info = json.loads((folder / 'run_info.json').read_text())
    if info.get('test') != 'vesc_timeout_check' or not info.get('live'):
        raise SystemExit('%s is not an armed vesc_timeout_check run' % folder)
    data = A.read_bag(folder / 'bag')
    status = [(t, json.loads(m.data).get('phase')) for t, m in data.get('/t1b_stop_converter/status', [])]
    core = [(t, m.state.speed, m.state.current_motor) for t, m in data.get('/sensors/core', [])]
    res = timeout_metrics(status, core, vp.get(vp.load(), 'vesc_timeout_check'))
    res['pass'] = res.pop('pass_')
    res['folder'] = str(folder)
    (folder / 'timeout_check.json').write_text(json.dumps(res, indent=1))
    print(res['verdict'])
    for k in ('erpm_pre', 'current_pre', 'onset_delay_s', 'current_min_after_onset_a', 'stop_below_300erpm_s'):
        if res.get(k) is not None:
            print('  %-26s %.3f' % (k, res[k]))
    print('Wrote %s' % (folder / 'timeout_check.json'))
    return 0 if res['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
