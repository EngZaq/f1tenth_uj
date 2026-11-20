#!/usr/bin/env python3
"""T4 yaw-rate profile (20 Nov: constant steering angle from the start, or ~0 then a step?).

For every counted repeat of the given T4 run folders (analyze.py --t4 must have run):
- BMI160 yaw rate from /imu/data_raw: the IMU is mounted upside down (base_link -> imu_link roll pi),
  so base_link yaw rate = -gz; bias = mean over the 1 s before the command. The sign is checked
  against RF2O (integrated headings) and reported.
- RF2O yaw rate: derivative of the /odom_rf2o yaw.
- Distance: LiDAR range progress toward the wall (same range source and pre-start range as
  analyze.py --t4).
- Curvature kappa = IMU yaw rate / VESC vx (v >= MIN_V) against distance. A constant steering
  angle gives a constant kappa from the first metre on (the yaw rate itself grows with speed in
  the ramp); play that moves under load gives kappa ~0 first, then a step. Step test: one level vs
  two levels (best split) vs a straight-line trend; STEP only if two levels beat both.
- Frozen IMU (20 Nov session 1: the BMI160 stream repeated one value for 129.6 s, I2C Errno 121
  kind): if > FROZEN_SHARE of consecutive 3-axis samples in a repeat are identical, that repeat uses
  the RF2O yaw rate (smoothed RF_SMOOTH_S) for the curvature and the step test, and is flagged.
Marks: motion start (ERPM >= 150, analyze start delay), command ramp end (v / ramp_accel), brake.
Writes t4_yaw.png and t4_yaw.json into each run folder. Reads the bag only.

  t4_yaw_plot.py <folder> [<folder> ...]
  t4_yaw_plot.py --self-test
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

BIAS_S = 1.0          # IMU bias window before the command
SMOOTH_S = 0.10       # moving average for the IMU yaw rate
MIN_V = 0.20          # m/s: kappa only where the VESC speed is at least this (sensorless below)
MIN_SIDE = 0.15       # step test: each level keeps at least this share of the samples
STEP_SSE_GAIN = 0.5   # step: two levels cut the SSE by >= this ...
STEP_RATIO = 0.35     # ... and |kappa before| <= this x |kappa after|
CONST_SSE_GAIN = 0.3  # constant: two levels cut the SSE by < this
STEP_VS_LINEAR = 0.5  # step: two levels also leave <= this share of a straight-line trend's SSE
FROZEN_SHARE = 0.5    # IMU frozen in a repeat: identical consecutive samples above this share
RF_SMOOTH_S = 0.20    # moving average for the RF2O yaw rate (20 Hz pose derivative)
CONST_SPREAD = 0.5    # constant: sd of kappa <= this x |mean kappa| (and <= CONST_SD_ABS)
CONST_SD_ABS = 0.05   # 1/m
OSC_EXPLAINED = 0.5   # oscillating: neither two levels nor a line explain this share of the variance
TURN_MIN = 0.05       # rad/s: turning points (extremes between sign changes) at least this large


# ---------------------------------------------------------------- pure logic

def moving_average(ts, vs, half_s):
    out, j0, j1, acc = [], 0, 0, 0.0
    for t in ts:
        while j1 < len(ts) and ts[j1] <= t + half_s:
            acc += vs[j1]
            j1 += 1
        while ts[j0] < t - half_s:
            acc -= vs[j0]
            j0 += 1
        out.append(acc / (j1 - j0))
    return out


def derivative(ts, vs):
    """Central difference (unwrapped angles) at the inner samples."""
    out_t, out_v = [], []
    for i in range(1, len(ts) - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt > 0:
            d = math.atan2(math.sin(vs[i + 1] - vs[i - 1]), math.cos(vs[i + 1] - vs[i - 1]))
            out_t.append(ts[i])
            out_v.append(d / dt)
    return out_t, out_v


def step_fit(xs, ys):
    """One level vs the best two-level split vs a straight line. Returns dict(level, sse1, split_x,
    before, after, sse2, gain, sse_lin, slope) or None if too few samples."""
    n = len(ys)
    if n < 10:
        return None
    mean = sum(ys) / n
    sse1 = sum((y - mean) ** 2 for y in ys)
    k_min = max(2, int(MIN_SIDE * n))
    pre_s, pre_q = [0.0], [0.0]
    for y in ys:
        pre_s.append(pre_s[-1] + y)
        pre_q.append(pre_q[-1] + y * y)
    best = None
    for k in range(k_min, n - k_min + 1):
        a_s, a_q, b_s, b_q = pre_s[k], pre_q[k], pre_s[n] - pre_s[k], pre_q[n] - pre_q[k]
        sse = (a_q - a_s * a_s / k) + (b_q - b_s * b_s / (n - k))
        if best is None or sse < best[0]:
            best = (sse, k, a_s / k, b_s / (n - k))
    sse2, k, before, after = best
    mx = sum(xs) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    slope = sum((x - mx) * (y - mean) for x, y in zip(xs, ys)) / sxx if sxx > 0 else 0.0
    sse_lin = sum((y - mean - slope * (x - mx)) ** 2 for x, y in zip(xs, ys))
    return dict(level=mean, sse1=sse1, split_x=xs[k], before=before, after=after, sse2=sse2,
                gain=0.0 if sse1 <= 0 else 1.0 - sse2 / sse1, sse_lin=sse_lin, slope=slope)


def frozen_share(samples):
    """Share of consecutive identical samples (tuples)."""
    if len(samples) < 2:
        return 1.0
    return sum(1 for a, b in zip(samples, samples[1:]) if a == b) / (len(samples) - 1)


def classify(fit, n=None):
    if fit is None:
        return 'too few samples'
    small_first = abs(fit['before']) <= STEP_RATIO * abs(fit['after'])
    if fit['gain'] >= STEP_SSE_GAIN and small_first and fit['sse2'] <= STEP_VS_LINEAR * fit['sse_lin']:
        return 'STEP (~0, then turning)'
    sd = math.sqrt(fit['sse1'] / n) if n else 0.0
    if fit['gain'] < CONST_SSE_GAIN and sd <= max(CONST_SD_ABS, CONST_SPREAD * abs(fit['level'])):
        return 'CONSTANT from the start'
    lin_gain = 0.0 if fit['sse1'] <= 0 else 1.0 - fit['sse_lin'] / fit['sse1']
    if max(fit['gain'], lin_gain) < OSC_EXPLAINED:
        return 'OSCILLATING (neither constant nor a step)'
    return 'GRADUAL / mixed'


def turning_points(xs, ys, min_abs=TURN_MIN):
    """Largest |y| between sign changes, kept if >= min_abs: [(index, y)]."""
    out, seg = [], []
    for i, y in enumerate(ys):
        if seg and (y > 0) != (ys[seg[0]] > 0):
            k = max(seg, key=lambda j: abs(ys[j]))
            if abs(ys[k]) >= min_abs:
                out.append((k, ys[k]))
            seg = []
        seg.append(i)
    if seg:
        k = max(seg, key=lambda j: abs(ys[j]))
        if abs(ys[k]) >= min_abs:
            out.append((k, ys[k]))
    return out


def self_test():
    xs = [i * 0.01 for i in range(150)]
    const = [0.15 + 0.01 * math.sin(40 * x) for x in xs]
    assert classify(step_fit(xs, const), len(xs)) == 'CONSTANT from the start', step_fit(xs, const)
    osc = [0.1 + 0.3 * math.sin(2 * math.pi * x / 0.55) for x in xs]
    assert classify(step_fit(xs, osc), len(xs)) == 'OSCILLATING (neither constant nor a step)'
    tp = turning_points(xs, [0.3 * math.sin(2 * math.pi * x / 0.55) for x in xs])
    assert [round(xs[i], 2) for i, _ in tp][:2] == [0.14, 0.41], tp
    step = [(0.005 if x < 0.6 else 0.16) + 0.01 * math.sin(40 * x) for x in xs]
    f = step_fit(xs, step)
    assert classify(f, len(xs)) == 'STEP (~0, then turning)' and abs(f['split_x'] - 0.6) <= 0.03, f
    ramp = [0.16 * x / 1.5 for x in xs]
    assert classify(step_fit(xs, ramp), len(xs)) == 'GRADUAL / mixed', step_fit(xs, ramp)
    assert step_fit(xs[:5], const[:5]) is None
    assert frozen_share([(1, 2, 3)] * 50) == 1.0 and frozen_share([(i, 0, 0) for i in range(50)]) == 0.0
    ts = [i * 0.01 for i in range(11)]
    assert abs(moving_average(ts, [float(i) for i in range(11)], 0.025)[5] - 5.0) < 1e-9
    dt, dv = derivative([0.0, 0.1, 0.2], [3.1, -3.1, -2.9])   # unwrap across +/-pi
    assert abs(dv[0] - (2 * math.pi - 6.0) / 0.2) < 1e-9, dv
    noisy_step = [(0.005 if x < 0.6 else 0.16) + 0.03 * math.sin(97 * x) for x in xs]
    assert classify(step_fit(xs, noisy_step), len(xs)) == 'STEP (~0, then turning)', step_fit(xs, noisy_step)
    print('PASS: step test (constant / step at 0.6 m, clean and noisy / gradual ramp != step /')
    print('      oscillation), turning points,')
    print('      moving average, unwrapped')
    print('      yaw derivative. No bag read.')


# ---------------------------------------------------------------- per run

def interp(ts, vs, t):
    import bisect
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    a, b = ts[i - 1], ts[i]
    return vs[i - 1] + (vs[i] - vs[i - 1]) * (t - a) / (b - a) if b > a else vs[i]


def run_folder(folder):
    import analyze as an
    import params as vp
    folder = Path(folder).expanduser()
    summary = json.loads((folder / 'summary.json').read_text())
    info = json.loads((folder / 'run_info.json').read_text())
    t4 = summary.get('t4')
    if not t4:
        raise SystemExit('%s: no T4 analysis (run analyze.py --t4 first)' % folder)
    items = [x for x in t4['items'] if x.get('counted')]
    p = vp.load()
    data = an.read_bag(folder / 'bag')
    front_deg = float(vp.get(p, 'ground_check.front_window_deg'))
    wall_deg = float(vp.get(p, 't4.wall_fit_deg'))
    scans = data.get('/scan', [])
    scan_t = [t for t, _ in scans]
    feats = [an.scan_features(m, front_deg, wall_deg) for _, m in scans]
    imu = data.get('/imu/data_raw', [])
    it_, gz = [t for t, _ in imu], [m.angular_velocity.z for _, m in imu]
    i3 = [(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z) for _, m in imu]
    rf = data.get('/odom_rf2o', [])
    rt, ryaw = [t for t, _ in rf], [an.quat_yaw(m.pose.pose.orientation) for _, m in rf]
    rdt, rdv = derivative(rt, ryaw)
    rdv_s = moving_average(rdt, rdv, RF_SMOOTH_S / 2)
    vo = data.get('/odom_vesc_observe', [])
    ot, vx = [t for t, _ in vo], [m.twist.twist.linear.x for _, m in vo]
    speed = float(info.get('speed') or (info.get('profile') or {}).get('speed') or 0.5)
    ramp_accel = float(info.get('ramp_accel_mps2') or vp.get(p, 'runner.ramp_accel_mps2'))
    off = (info.get('converter_params') or {}).get('servo_offset')
    off = float(off) if off is not None else float(vp.get(p, 'steering.servo_offset'))

    reps = []
    for x in items:
        tc, t0 = x['t_cmd'], x['t0']
        wall = x.get('range_source', '').startswith('wall')
        rng_t, rng_v = [], []
        for t, f in zip(scan_t, feats):
            if tc - 1.5 <= t <= t0 + 1.0:
                r = (f['wall']['d'] if f['wall'] else None) if wall else f['front']
                if r is not None and math.isfinite(r):
                    rng_t.append(t)
                    rng_v.append(r)
        r_pre = x['pre_range_m']
        bias_v = [g for t, g in zip(it_, gz) if tc - BIAS_S <= t < tc - 0.02]
        bias = sum(bias_v) / len(bias_v)
        win = [(t, -(g - bias)) for t, g in zip(it_, gz) if tc - 0.5 <= t <= t0 + 1.0]
        wt = [t for t, _ in win]
        wr = moving_average(wt, [r for _, r in win], SMOOTH_S / 2)
        dist = [r_pre - interp(rng_t, rng_v, t) for t in wt]
        rf_win = [(t, v) for t, v in zip(rdt, rdv_s) if tc - 0.5 <= t <= t0 + 1.0]
        fz = frozen_share([v for t, v in zip(it_, i3) if tc - BIAS_S <= t <= t0])
        imu_frozen = fz > FROZEN_SHARE
        # Sign check: IMU heading change vs RF2O over command -> brake + 1 s.
        h_imu = sum(r * (b - a) for (a, r), (b, _) in zip(win, win[1:]) if a >= tc)
        h_rf = math.atan2(math.sin(interp(rt, ryaw, t0 + 1.0) - interp(rt, ryaw, tc)),
                          math.cos(interp(rt, ryaw, t0 + 1.0) - interp(rt, ryaw, tc)))
        t_move = tc + (x.get('start_delay_erpm_s') or 0.0)
        t_ramp = tc + speed / ramp_accel
        ks, kk, kt = [], [], []
        if imu_frozen:   # curvature from RF2O instead
            src = [(t, v, r_pre - interp(rng_t, rng_v, t)) for t, v in rf_win]
        else:
            src = list(zip(wt, wr, dist))
        # Turning points: yaw rate (same source as kappa, smoothed 0.3 s) from motion to brake.
        mv = [(t, r, d) for t, r, d in src if t_move <= t <= t0]
        src_t = [t for t, _, _ in mv]
        src_d = [d for _, _, d in mv]
        src_r = moving_average(src_t, [r for _, r, _ in mv], 0.15) if mv else []
        for t, r, d in src:
            v = interp(ot, vx, t)
            if t_move <= t <= t0 and v >= MIN_V:
                ks.append(d)
                kk.append(r / v)
                kt.append(t)
        fit = step_fit(ks, kk)
        split_t = None
        if fit:
            split_t = kt[ks.index(fit['split_x'])] - tc
        reps.append(dict(
            index=x['index'], bias_rad_s=bias, heading_imu_deg=math.degrees(h_imu),
            heading_rf2o_deg=math.degrees(h_rf),
            sign_check_ok=None if imu_frozen else (abs(h_imu) > math.radians(1.0) and h_imu * h_rf > 0),
            imu_frozen=imu_frozen, imu_frozen_share=fz, kappa_source='RF2O' if imu_frozen else 'IMU',
            t_move_s=t_move - tc, t_ramp_end_s=t_ramp - tc, t_brake_s=t0 - tc,
            d_move_m=r_pre - interp(rng_t, rng_v, t_move), d_ramp_end_m=r_pre - interp(rng_t, rng_v, t_ramp),
            d_brake_m=r_pre - interp(rng_t, rng_v, t0),
            yaw_rate_steady_mean=(lambda v: sum(v) / len(v) if v else None)(
                [r for t, r, _ in src if t_ramp <= t <= t0]),
            kappa_fit=fit, split_t_s=split_t, verdict=classify(fit, len(kk)),
            kappa_mean=(sum(kk) / len(kk)) if kk else None,
            kappa_sd=(math.sqrt(sum((k - sum(kk) / len(kk)) ** 2 for k in kk) / len(kk))) if kk else None,
            turning_points=[dict(t_s=src_t[i] - tc, d_m=src_d[i], range_m=r_pre - src_d[i], yaw_rate=y)
                            for i, y in turning_points(src_d, src_r)],
            pre_range_m=r_pre,
            _plot=dict(t=[t - tc for t in wt], r=wr, d=dist, rf_t=[t - tc for t, _ in rf_win],
                       rf_r=[v for _, v in rf_win],
                       rf_d=[r_pre - interp(rng_t, rng_v, t) for t, _ in rf_win], ks=ks, kk=kk)))
    return dict(folder=str(folder), servo_offset=off, speed=speed, repeats=reps)


def plot(res, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import analyze as an
    fig, axes = plt.subplots(3, 1, figsize=(10, 11))
    for i, r in enumerate(res['repeats']):
        c = an.REPEAT_COLORS[i % len(an.REPEAT_COLORS)]
        q = r['_plot']
        lab = 'repeat %d' % (r['index'] + 1)
        fr = r['imu_frozen']
        if not fr:
            axes[0].plot(q['t'], q['r'], color=c, lw=1.4, label=lab + ' IMU')
            axes[1].plot(q['d'], q['r'], color=c, lw=1.4, label=lab + ' IMU')
        axes[0].plot(q['rf_t'], q['rf_r'], color=c, lw=1.4 if fr else 1.0, ls='-' if fr else '--',
                     alpha=1.0 if fr else 0.7, label=lab + (' RF2O (IMU frozen)' if fr else ' RF2O'))
        axes[1].plot(q['rf_d'], q['rf_r'], color=c, lw=1.4 if fr else 1.0, ls='-' if fr else '--',
                     alpha=1.0 if fr else 0.7)
        axes[2].plot(q['ks'], q['kk'], color=c, lw=1.2, label='%s (%s)' % (lab, r['kappa_source']))
        f = r['kappa_fit']
        if f:
            x0, x1 = q['ks'][0], q['ks'][-1]
            axes[2].plot([x0, f['split_x'], f['split_x'], x1], [f['before'], f['before'], f['after'], f['after']],
                         color=c, lw=2.2, alpha=0.45)
        mt, mr = (q['rf_t'], q['rf_r']) if fr else (q['t'], q['r'])
        for key_t, key_d, mk in (('t_move_s', 'd_move_m', 'o'), ('t_ramp_end_s', 'd_ramp_end_m', 's'),
                                 ('t_brake_s', 'd_brake_m', 'v')):
            axes[0].plot([r[key_t]], [interp(mt, mr, r[key_t])], mk, color=c, ms=7, mec='white')
            axes[1].plot([r[key_d]], [interp(mt, mr, r[key_t])], mk, color=c, ms=7, mec='white')
    r0 = res['repeats'][0] if res['repeats'] else None
    if r0:
        axes[0].axvspan(0, r0['t_ramp_end_s'], color=an.GRID, alpha=0.35, lw=0)
        axes[0].axvspan(r0['t_ramp_end_s'], r0['t_brake_s'], color=an.GRID, alpha=0.12, lw=0)
    title = 'T4 %s, servo_offset %.3f' % (Path(res['folder']).name[:15], res['servo_offset'])
    an.style(axes[0], title + ': yaw rate vs time from the command (o motion, s ramp end, v brake; '
             'dark band = command ramp)', 'yaw rate rad/s (+ left)')
    axes[0].set_xlabel('s from the speed command', color=an.MUTED, fontsize=9)
    an.style(axes[1], 'yaw rate vs distance (LiDAR progress toward the wall)', 'yaw rate rad/s (+ left)')
    axes[1].set_xlabel('m', color=an.MUTED, fontsize=9)
    an.style(axes[2], 'curvature = yaw rate / VESC vx (v >= %.2f m/s; RF2O where the IMU froze), '
             'best two-level fit' % MIN_V, 'curvature 1/m (+ left)')
    axes[2].set_xlabel('m', color=an.MUTED, fontsize=9)
    for ax in axes:
        ax.axhline(0, color=an.MUTED, lw=0.6)
    axes[0].legend(fontsize=7, ncol=3, frameon=False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return
    for folder in args.folders:
        res = run_folder(folder)
        plot(res, Path(res['folder']) / 't4_yaw.png')
        clean = dict(res, repeats=[{k: v for k, v in r.items() if k != '_plot'} for r in res['repeats']])
        (Path(res['folder']) / 't4_yaw.json').write_text(json.dumps(clean, indent=2))
        print('%s (servo_offset %.3f): t4_yaw.png, t4_yaw.json' % (Path(res['folder']).name, res['servo_offset']))
        for r in res['repeats']:
            f = r['kappa_fit'] or {}
            print('  repeat %d turning points (yaw rate rad/s @ distance m / time s / range to wall m): %s' % (
                r['index'] + 1, ', '.join('%+.2f @ %.2f / %.2f / %.2f' % (
                    tp['yaw_rate'], tp['d_m'], tp['t_s'], tp['range_m']) for tp in r['turning_points'])))
            print('     kappa mean %.3f sd %.3f 1/m, start range %.3f m' % (
                r['kappa_mean'] or float('nan'), r['kappa_sd'] or float('nan'), r['pre_range_m']))
            print('  repeat %d: %s | kappa before %.3f / after %.3f 1/m, split %.2f m at %.2f s, SSE -%.0f %% | '
                  'steady yaw rate %.3f rad/s (%s) | heading IMU %.1f vs RF2O %.1f deg (sign %s) | '
                  'motion %.2f s / %.2f m, ramp end %.2f s / %.2f m, brake %.2f s / %.2f m' % (
                      r['index'] + 1, r['verdict'], f.get('before', float('nan')), f.get('after', float('nan')),
                      f.get('split_x', float('nan')), r['split_t_s'] if r['split_t_s'] is not None else float('nan'),
                      100 * f.get('gain', float('nan')), r['yaw_rate_steady_mean'] or float('nan'),
                      r['kappa_source'], r['heading_imu_deg'], r['heading_rf2o_deg'],
                      {None: 'n/a, IMU FROZEN %.0f %%' % (100 * r['imu_frozen_share']), True: 'ok',
                       False: 'MISMATCH'}[r['sign_check_ok']],
                      r['t_move_s'], r['d_move_m'], r['t_ramp_end_s'], r['d_ramp_end_m'],
                      r['t_brake_s'], r['d_brake_m']))


if __name__ == '__main__':
    main()
