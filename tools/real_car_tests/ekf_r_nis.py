#!/usr/bin/env python3
"""Shadow EKF noise check (23 Nov eve): measurement R from steady cruise, and NIS of the C1 replay. Reads bags only; the
NIS replay publishes only in its own isolated ROS domain (default 88; refuses while a car stack runs).

--r <run folders>   Steady cruise windows (runner events STEADY_REACHED + PAD_S .. ZERO_COMMAND - 0.1 s, >= MIN_S) of
                    T4 / T3 runs. Per sensor (VESC vx = ERPM / gain at 50 Hz, RF2O vx at 20 Hz): white noise
                    sigma_hf = std(first differences) / sqrt(2), robust spread about the window's linear trend
                    (1.4826 MAD), and VESC - RF2O (VESC interpolated to RF2O times): mean (bias) and robust std.
                    Proposed R = (2 x robust spread)^2 (the 21 Nov k = 2 rule); the true speed is not constant in a
                    window (PID ripple), so the spread is an upper bound of the sensor noise.
--nis <run folders> C1 (ekf_real_shadow_c1) replayed per repeat with the current ekf_shadow_inputs (YAW_MODE, R),
                    recording the filtered twist AND its covariance. For each fused measurement z with variance R:
                    NIS = (z - x)^2 / (P + Q dt + R), x / P = the filter output just before the measurement stamp
                    (prior proxy; robot_localization publishes no innovations). Consistent: mean NIS ~ 1 (1 dof),
                    95 % of samples < 3.84. Channels: VESC vx (de-weighted samples separately), vy = 0, IMU yaw rate,
                    RF2O vx.
  ekf_r_nis.py --r <folders> | --nis <folders> [--domain 88] [--work DIR] | --self-test
"""
import argparse
import bisect
import csv
import json
import math
import os
import statistics
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import ekf_shadow_inputs as inp  # noqa: E402

PAD_S = 0.5            # cruise window starts this long after STEADY_REACHED (speed PID settle)
MIN_S = 2.0            # shortest cruise window used
Q_VX, Q_VY, Q_VYAW = 0.05, 0.025, 0.02     # ekf_real_shadow.yaml process noise (per second) of the 3 twist states
DT_FILTER = 1.0 / 30.0                     # frequency 30 Hz


# ---------------------------------------------------------------- pure helpers (self-tested)

def robust_std(vals):
    if len(vals) < 3:
        return float('nan')
    med = statistics.median(vals)
    return 1.4826 * statistics.median(abs(v - med) for v in vals)


def sigma_hf(vals):
    d = [b - a for a, b in zip(vals, vals[1:])]
    return statistics.pstdev(d) / math.sqrt(2.0) if len(d) > 2 else float('nan')


def detrend(ts, vals):
    n = len(ts)
    mt, mv = sum(ts) / n, sum(vals) / n
    sxx = sum((t - mt) ** 2 for t in ts)
    b = sum((t - mt) * (v - mv) for t, v in zip(ts, vals)) / sxx if sxx else 0.0
    return [v - (mv + b * (t - mt)) for t, v in zip(ts, vals)]


def interp(ts, vs, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0 or i >= len(ts):
        return None
    t0, t1 = ts[i - 1], ts[i]
    return vs[i - 1] + (vs[i] - vs[i - 1]) * (t - t0) / (t1 - t0) if t1 > t0 else vs[i]


def nis_series(meas, out_t, out_x, out_p, q, dt=DT_FILTER):
    """meas: [(t, z, R)]; filter outputs (sorted times, state, variance). -> [NIS] using the last output before t."""
    res = []
    for t, z, r in meas:
        i = bisect.bisect_left(out_t, t) - 1
        if i < 0 or t - out_t[i] > 0.2:
            continue
        s = out_p[i] + q * dt + r
        res.append((z - out_x[i]) ** 2 / s)
    return res


def nis_summary(vals):
    if not vals:
        return None
    srt = sorted(vals)
    return dict(n=len(vals), mean=statistics.mean(vals), median=statistics.median(vals),
                p95=srt[int(0.95 * (len(srt) - 1))], share_over_3_84=sum(v > 3.84 for v in vals) / len(vals))


def cruise_windows(events):
    """[(t0, t1)] wall-time cruise windows from runner_events rows."""
    out, t_st = [], None
    for e in events:
        if e['event'] == 'STEADY_REACHED':
            t_st = float(e['wall'])
        elif e['event'] == 'ZERO_COMMAND':
            if t_st is not None:
                a, b = t_st + PAD_S, float(e['wall']) - 0.1
                if b - a >= MIN_S:
                    out.append((a, b))
            t_st = None
    return out


def self_test():
    import random
    rnd = random.Random(1)
    v = [rnd.gauss(0, 0.02) for _ in range(5000)]
    assert abs(robust_std(v) - 0.02) < 0.002 and abs(sigma_hf(v) - 0.02) < 0.002
    ts = [0.02 * i for i in range(500)]
    ramp = [0.3 + 0.1 * t + x for t, x in zip(ts, v)]
    assert abs(statistics.pstdev(detrend(ts, ramp)) - 0.02) < 0.002
    assert interp([0.0, 1.0], [0.0, 2.0], 0.25) == 0.5 and interp([0.0, 1.0], [0.0, 2.0], 1.5) is None
    # consistent filter: x ~ truth with variance P, z = truth + noise R -> NIS mean ~ 1
    out_t = [0.01 * i for i in range(20000)]
    P, R = 1e-3, 4e-3
    truth = 0.3
    out_x = [truth + rnd.gauss(0, math.sqrt(P)) for _ in out_t]
    meas = [(t + 0.005, truth + rnd.gauss(0, math.sqrt(R)), R) for t in out_t[::2]]
    s = nis_summary(nis_series(meas, out_t, out_x, [P] * len(out_t), 0.0))
    assert 0.93 < s['mean'] < 1.07 and 0.03 < s['share_over_3_84'] < 0.07, s
    s2 = nis_summary(nis_series([(t, z, R / 10) for t, z, R in meas], out_t, out_x, [P] * len(out_t), 0.0))
    assert s2['mean'] > 2.5                            # R too small -> NIS >> 1
    ev = [dict(event='STEADY_REACHED', wall='10.0'), dict(event='ZERO_COMMAND', wall='15.0'),
          dict(event='STEADY_REACHED', wall='20.0'), dict(event='ZERO_COMMAND', wall='21.0')]
    assert cruise_windows(ev) == [(10.5, 14.9)]
    print('PASS: robust std / white noise / detrend, interpolation, NIS (consistent ~1, R too small >> 1),')
    print('      cruise windows.')


# ---------------------------------------------------------------- --r

def read_events(folder):
    with open(folder / 'runner_events.csv') as fp:
        return list(csv.DictReader(fp))


def measure_r(folders, gain):
    import analyze as A
    rows = []
    for folder in folders:
        wins = cruise_windows(read_events(folder))
        if not wins:
            continue
        data = A.read_bag(folder / 'bag')
        core = [(t, m.state.speed / gain) for t, m in data.get('/sensors/core', [])]
        rf = [(t, m.twist.twist.linear.x) for t, m in data.get('/odom_rf2o', [])]
        ct, cv = [c[0] for c in core], [c[1] for c in core]
        for a, b in wins:
            ve = [(t, x) for t, x in core if a <= t <= b]
            re_ = [(t, x) for t, x in rf if a <= t <= b]
            if len(ve) < 50 or len(re_) < 20:
                continue
            vt, vv = zip(*ve)
            rt, rv = zip(*re_)
            diff = [x - interp(ct, cv, t) for t, x in re_ if interp(ct, cv, t) is not None]
            rows.append(dict(run=folder.name[:15], test=folder.name.split('_')[2], t0=a, dur_s=b - a,
                             vesc_mean=statistics.mean(vv), rf2o_mean=statistics.mean(rv),
                             vesc_hf=sigma_hf(vv), vesc_spread=robust_std(detrend(vt, vv)),
                             rf2o_hf=sigma_hf(rv), rf2o_spread=robust_std(detrend(rt, rv)),
                             rf2o_minus_vesc_mean=statistics.mean(diff), rf2o_minus_vesc_rstd=robust_std(diff)))
    return rows


def print_r(rows):
    print('| run | test | window s | VESC vx mean | RF2O vx mean | VESC sigma_hf | VESC spread | RF2O sigma_hf | '
          'RF2O spread | RF2O - VESC mean | RF2O - VESC robust std |')
    print('|---|---|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        print('| %s | %s | %.1f | %.3f | %.3f | %.4f | %.4f | %.4f | %.4f | %+.3f | %.4f |' % (
            r['run'], r['test'], r['dur_s'], r['vesc_mean'], r['rf2o_mean'], r['vesc_hf'], r['vesc_spread'],
            r['rf2o_hf'], r['rf2o_spread'], r['rf2o_minus_vesc_mean'], r['rf2o_minus_vesc_rstd']))
    out = {}
    for test in sorted({r['test'] for r in rows}):
        g = [r for r in rows if r['test'] == test]
        med = {k: statistics.median(r[k] for r in g) for k in ('vesc_hf', 'vesc_spread', 'rf2o_hf', 'rf2o_spread',
                                                               'rf2o_minus_vesc_mean', 'rf2o_minus_vesc_rstd')}
        med.update(windows=len(g), R_vesc_vx=(2 * med['vesc_spread']) ** 2, R_rf2o_vx=(2 * med['rf2o_spread']) ** 2)
        out[test] = med
        print('%s (%d windows, medians): VESC sigma_hf %.4f spread %.4f -> R (2 x spread)^2 = %.1e (now %.1e); '
              'RF2O sigma_hf %.4f spread %.4f -> R %.1e (now %.1e); RF2O - VESC %+.3f m/s (robust std %.3f)' % (
                  test, len(g), med['vesc_hf'], med['vesc_spread'], med['R_vesc_vx'], inp.R_VESC_VX,
                  med['rf2o_hf'], med['rf2o_spread'], med['R_rf2o_vx'], inp.R_RF_VX, med['rf2o_minus_vesc_mean'],
                  med['rf2o_minus_vesc_rstd']))
    return out


# ---------------------------------------------------------------- --nis

def nis_replay(folders, work, env, gain):
    import analyze as A
    import ekf_shadow_replay as rp
    from nav_msgs.msg import Odometry  # noqa: F401  (Recorder subscription type)

    class CovRecorder(rp.Recorder):
        def on_odom(self, m):
            c = m.twist.covariance
            with self.lock:
                self.samples.append((rp.stamp(m), m.twist.twist.linear.x, m.twist.twist.linear.y,
                                     m.twist.twist.angular.z, c[0], c[7], c[35]))
    rec, allv = CovRecorder(), {}
    per = []
    try:
        for folder in folders:
            info = json.loads((folder / 'run_info.json').read_text())
            hold = float((info.get('converter_params') or {}).get('brake_hold_s') or 2.0)
            ev = read_events(folder)
            starts = [float(e['wall']) for e in ev if e['event'] == 'DRIVE_START']
            zeros = [float(e['wall']) for e in ev if e['event'] == 'ZERO_COMMAND']
            data = A.read_bag(folder / 'bag')
            imu = data['/imu/data_raw']
            st_t, st_b = [], []
            for t, m in data.get('/t1b_stop_converter/status', []):
                st_t.append(t)
                st_b.append(inp.brake_active(m.data))
            for i, tc in enumerate(starts):
                tz = next((t for t in zeros if t > tc), None)
                if tz is None or tz - tc < 3.0:
                    continue
                cal = rp.calib_window(imu, tc - rp.BIAS_S, tc - 0.02)
                if not cal or cal['moving']:
                    continue
                w0, w1 = tc - rp.BIAS_S - 0.2, tz + hold + 0.3
                name = '%s_r%d_nis_%s_r%.1e' % (folder.name[:15], i, inp.YAW_MODE, inp.R_IMU_WZ)
                in_bag = work / (name + '_in')
                if not in_bag.exists():
                    rp.build_inputs(data, w0, w1, cal, gain, in_bag)
                samples, _, _ = rp.run_node(rp.ekf_cmd(rp.VARIANTS['C1'][0]), in_bag, env, work / (name + '.log'), rec)
                if not samples:
                    continue
                ot = [s[0] for s in samples]
                meas = dict(vesc_vx=[], vesc_vx_dw=[], vy0=[], imu_wz=[], rf2o_vx=[])
                for t, m in data['/sensors/core']:
                    if w0 <= t <= w1:
                        br = rp.state_at(st_t, st_b, t)
                        o = inp.vesc_msg(m.header.stamp, m.state.speed, br, gain, True)
                        key = 'vesc_vx_dw' if inp.vesc_deweighted(m.state.speed, br) else 'vesc_vx'
                        meas[key].append((rp.stamp(m), o.twist.twist.linear.x, o.twist.covariance[0]))
                        meas['vy0'].append((rp.stamp(m), 0.0, inp.R_VY))
                for t, m in imu:
                    if w0 <= t <= w1:
                        a = m.linear_acceleration
                        o = inp.imu_msg(m.header.stamp, rp.gyro_of(m), (a.x, a.y, a.z), cal)
                        meas['imu_wz'].append((rp.stamp(m), o.angular_velocity.z, inp.R_IMU_WZ))
                for t, m in data['/odom_rf2o']:
                    if w0 <= t <= w1:
                        meas['rf2o_vx'].append((rp.stamp(m), m.twist.twist.linear.x, inp.R_RF_VX))
                cols = dict(vesc_vx=(1, 4, Q_VX), vesc_vx_dw=(1, 4, Q_VX), vy0=(2, 5, Q_VY), imu_wz=(3, 6, Q_VYAW),
                            rf2o_vx=(1, 4, Q_VX))
                res = dict(run=folder.name[:15], repeat=i)
                for k, (xi, pi, q) in cols.items():
                    v = nis_series(sorted(meas[k]), ot, [s[xi] for s in samples], [s[pi] for s in samples], q)
                    allv.setdefault(k, []).extend(v)
                    res[k] = nis_summary(v)
                per.append(res)
                print('  %s r%d: %s' % (res['run'], i, ' | '.join('%s %.2f' % (k, res[k]['mean']) for k in cols
                                                                  if res.get(k))))
    finally:
        rec.close()
    total = {k: nis_summary(v) for k, v in allv.items()}
    print('\nC1 NIS (yaw %s, R_IMU_WZ %.1e) over %d repeats: mean / median / p95 / share > 3.84 (consistent: ~1 / '
          '0.45 / 3.84 / 5 %%)' % (inp.YAW_MODE, inp.R_IMU_WZ, len(per)))
    for k, s in total.items():
        if s:
            print('  %-11s n %6d  mean %7.3f  median %6.3f  p95 %7.3f  >3.84 %5.1f %%' % (
                k, s['n'], s['mean'], s['median'], s['p95'], 100 * s['share_over_3_84']))
    return dict(yaw_mode=inp.YAW_MODE, r=dict(vesc_vx=inp.R_VESC_VX, vy=inp.R_VY, imu_wz=inp.R_IMU_WZ,
                                              rf2o_vx=inp.R_RF_VX), q=dict(vx=Q_VX, vy=Q_VY, vyaw=Q_VYAW),
                total=total, repeats=per)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--r', nargs='+', metavar='FOLDER')
    ap.add_argument('--nis', nargs='+', metavar='FOLDER')
    ap.add_argument('--domain', type=int, default=88)
    ap.add_argument('--work', default=None)
    ap.add_argument('--out', default=None, help='JSON result file')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    gain = inp.gain_from_params()
    result = {}
    if args.r:
        result['r'] = print_r(measure_r([Path(f).expanduser() for f in args.r], gain))
    if args.nis:
        import ekf_shadow_replay as rp
        busy = rp.stack_running()
        if busy:
            raise SystemExit('REFUSED: a car stack is running: ' + '; '.join(busy))
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        work = Path(args.work or tempfile.mkdtemp(prefix='ekf_nis_'))
        work.mkdir(parents=True, exist_ok=True)
        result['nis'] = nis_replay([Path(f).expanduser() for f in args.nis], work, dict(os.environ), gain)
    if args.out and result:
        Path(args.out).write_text(json.dumps(result, indent=1, default=str))
    if not result:
        ap.error('--r, --nis or --self-test')
    return 0


if __name__ == '__main__':
    sys.exit(main())
