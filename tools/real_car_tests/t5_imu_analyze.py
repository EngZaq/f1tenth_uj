#!/usr/bin/env python3
"""T5 IMU (BMI160 /imu/data_raw) noise on the car (21 Nov). Reads bags only.

The bmi160 node subtracts its own start-up gyro bias (500 samples, car still) before publishing, so every
"bias" here is the RESIDUAL after that calibration. Base yaw rate = -gz (IMU z down; vehicle_params imu).
Exact duplicate samples (all six values equal to the previous one: the 100 Hz timer re-reading the register,
0.3-1.3 % in healthy T4 bags) are dropped before noise estimates; runs of >= FROZEN_RUN of them are freezes.
White-noise level per sample: sigma_hf^2 = var(second difference) / 6 (removes a slowly varying true rate,
so it also works while driving).

  driving <T4 folders>  (T5 a) steady-cruise windows of t4_straight runs vs the 1 s still windows before each
                        start: gx / gy / gz std and sigma_hf, gz vs RF2O yaw rate (pose differences, header
                        stamps, best lag), compared with the 11 Nov bench R_gz 5.4e-7. Recommends a driving R_gz.
  stand <T5b folder>    (T5 b) run_test.py t5b_stand_imu: still / 90 s steady (motor on, car not moving) /
                        after the brake: per axis mean, std, sigma_hf; gz bias shift with the motor running,
                        drift in the steady window, vibration spectrum peaks vs wheel frequency.
  static <T5a folder>   (T5 c) 15 min static recording: residual bias, R (variance), Allan deviation (angle random
                        walk, bias instability), 60 s block drift, integrated heading drift, frozen-sample check.
Writes t5_<mode>.json into the (first) folder.
  t5_imu_analyze.py --self-test
"""
import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

AXES = ('gx', 'gy', 'gz', 'ax', 'ay', 'az')
FROZEN_RUN = 10          # >= this many identical consecutive samples (0.1 s) = frozen stream
GAP_S = 0.03             # header-stamp gap counted as a dropout (100 Hz expected)
C1_R_GZ = 8.0e-3         # C1 shadow EKF R for gyro vyaw (ekf_shadow_inputs.R_IMU_WZ; gyro - reference, T4 cruise)
START_DELAY_S = 0.3      # command -> motion (T4)
SETTLE_S = 1.0           # after the ramp (as t3_analyze)


# ---------------------------------------------------------------- pure logic (numpy)

def dedup(rows):
    """rows = [(t, gx, gy, gz, ax, ay, az)] sorted by t -> (rows without exact repeats, frozen report)."""
    out, runs, run = [], [], 0
    for i, r in enumerate(rows):
        if i and r[1:] == rows[i - 1][1:]:
            run += 1
            continue
        if run:
            runs.append(run)
        run = 0
        out.append(r)
    if run:
        runs.append(run)
    n = max(1, len(rows) - 1)
    return out, dict(samples=len(rows), duplicates=sum(runs), duplicate_share=sum(runs) / n,
                     longest_run=max(runs) if runs else 0, frozen_runs=sum(1 for x in runs if x >= FROZEN_RUN - 1),
                     frozen=any(x >= FROZEN_RUN - 1 for x in runs))


def timing(ts):
    import numpy as np
    t = np.asarray(ts, dtype=float)
    if len(t) < 3:
        return dict(rate_hz=None)
    d = np.diff(t)
    return dict(rate_hz=float((len(t) - 1) / (t[-1] - t[0])), max_gap_s=float(d.max()),
                gaps_over_30ms=int((d > GAP_S).sum()), duration_s=float(t[-1] - t[0]))


def hf_var(x):
    """White-noise variance per sample from the second difference (a ramp cancels): var(d2) / 6."""
    import numpy as np
    x = np.asarray(x, dtype=float)
    if len(x) < 10:
        return None
    return float(np.var(np.diff(x, 2)) / 6.0)


def allan(x, dt, taus):
    """Overlapping Allan deviation of a rate signal x (uniform dt) at the given taus -> [(tau, adev)]."""
    import numpy as np
    x = np.asarray(x, dtype=float)
    theta = np.concatenate([[0.0], np.cumsum(x) * dt])
    n = len(x)
    out = []
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= n:
            continue
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        out.append((m * dt, float(math.sqrt(np.mean(d * d) / (2 * (m * dt) ** 2)))))
    return out


def allan_summary(ad):
    """ARW = adev at tau 1 s (rate x sqrt(s)), bias instability = min adev / 0.664 (and its tau)."""
    if not ad:
        return {}
    arw = min(ad, key=lambda p: abs(math.log(p[0])))[1]
    tmin, amin = min(ad, key=lambda p: p[1])
    return dict(arw_at_1s=arw, bias_instability=amin / 0.664, tau_min_s=tmin)


def block_drift(ts, x, block_s):
    """Mean per block of block_s -> (block means, max - min, linear slope per minute)."""
    import numpy as np
    t, x = np.asarray(ts, dtype=float), np.asarray(x, dtype=float)
    k = ((t - t[0]) // block_s).astype(int)
    means = [float(x[k == i].mean()) for i in range(k.max() + 1) if (k == i).sum() > 0.5 * block_s * 50]
    sl = float(np.polyfit(t - t[0], x, 1)[0] * 60.0) if len(t) > 10 else None
    return means, (max(means) - min(means)) if means else None, sl


def rate_from_poses(poses):
    """poses = [(t_header, yaw)] -> [(t_prev, t, mean yaw rate over the interval)]."""
    out = []
    for (t0, y0), (t1, y1) in zip(poses, poses[1:]):
        if t1 - t0 > 1e-3:
            out.append((t0, t1, ((y1 - y0 + math.pi) % (2 * math.pi) - math.pi) / (t1 - t0)))
    return out


def gyro_vs_intervals(ts, wz, intervals, lag):
    """Mean of wz over each interval shifted by -lag (RF2O stamps late by lag) -> [(ref, gyro)]."""
    import numpy as np
    t = np.asarray(ts, dtype=float)
    w = np.asarray(wz, dtype=float)
    G = np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * np.diff(t))])
    out = []
    for a, b, ref in intervals:
        a2, b2 = a - lag, b - lag
        if a2 < t[0] or b2 > t[-1]:
            continue
        out.append((ref, float((np.interp(b2, t, G) - np.interp(a2, t, G)) / (b2 - a2))))
    return out


def best_lag(ts, wz, intervals, lags):
    """Lag minimising the residual variance -> (lag, residual std, n, residual mean)."""
    import numpy as np
    best = None
    for lag in lags:
        pairs = gyro_vs_intervals(ts, wz, intervals, lag)
        if len(pairs) < 10:
            continue
        r = np.array([g - ref for ref, g in pairs])
        if best is None or r.var() < best[1] ** 2:
            best = (lag, float(r.std()), len(pairs), float(r.mean()))
    return best


def spectrum_peaks(x, dt, seg_s=4.0, n_peaks=3, fmin=0.5):
    """Welch-style averaged amplitude spectrum -> [(Hz, rms in that bin)] of the largest peaks above fmin."""
    import numpy as np
    x = np.asarray(x, dtype=float) - np.mean(x)
    n = int(seg_s / dt)
    if len(x) < n:
        return []
    segs = [x[i:i + n] * np.hanning(n) for i in range(0, len(x) - n + 1, n // 2)]
    p = np.mean([np.abs(np.fft.rfft(s)) ** 2 for s in segs], axis=0)
    f = np.fft.rfftfreq(n, dt)
    idx = [i for i in range(1, len(p) - 1) if f[i] >= fmin and p[i] >= p[i - 1] and p[i] >= p[i + 1]]
    idx.sort(key=lambda i: -p[i])
    scale = math.sqrt(8.0 / 3.0) * math.sqrt(2.0) / n      # hanning power correction, one-sided rms
    return [(float(f[i]), float(math.sqrt(p[i]) * scale)) for i in idx[:n_peaks]]


def axis_stats(rows, k):
    """mean, std, sigma_hf of axis k (1..6) over rows."""
    import numpy as np
    x = np.array([r[k] for r in rows], dtype=float)
    if len(x) < 10:
        return None
    hv = hf_var(x)
    return dict(n=len(x), mean=float(x.mean()), std=float(x.std()), hf_std=math.sqrt(hv) if hv else None)


def pooled(windows, k):
    """Pool several windows (each mean removed): std and sigma_hf of axis k."""
    import numpy as np
    res, hfs, n = [], [], 0
    for rows in windows:
        x = np.array([r[k] for r in rows], dtype=float)
        if len(x) < 10:
            continue
        res.append(x - x.mean())
        hfs.append((hf_var(x), len(x)))
        n += len(x)
    if not res:
        return None
    allr = np.concatenate(res)
    hv = sum(v * m for v, m in hfs) / sum(m for _, m in hfs)
    return dict(n=n, std=float(allr.std()), hf_std=math.sqrt(hv))


def recommend_r_gz(hf_drive, bias_unc=None):
    """Driving R for the gyro yaw rate: (2 sigma_hf,drive)^2 (k = 2 rule of the other R values), plus the
    residual bias uncertainty^2 when known (no bias state in the EKF)."""
    r = (2.0 * hf_drive) ** 2
    if bias_unc:
        r += bias_unc ** 2
    return r


def self_test():
    import numpy as np
    rng = np.random.default_rng(7)
    dt, n = 0.01, 60000                              # 10 min at 100 Hz
    s_w, s_rw = 0.002, 2e-5                          # white rad/s per sample, random walk per step
    x = s_w * rng.standard_normal(n) + np.cumsum(s_rw * rng.standard_normal(n))
    ad = dict(allan(x, dt, [0.01, 0.1, 1.0, 10.0]))
    # white part: adev(tau) = s_w * sqrt(dt / tau)
    assert abs(ad[0.01] / s_w - 1) < 0.05 and abs(ad[0.1] / (s_w * math.sqrt(0.1)) - 1) < 0.1, ad
    assert abs(hf_var(x) / s_w ** 2 - 1) < 0.05, hf_var(x)
    # hf_var ignores a large smooth true rate (T4 weave 0.3 rad/s over 0.4 s)
    t = np.arange(400) * dt
    smooth = 0.3 * np.sin(2 * math.pi * 1.2 * t)
    y = smooth + 0.01 * rng.standard_normal(len(t))
    assert abs(math.sqrt(hf_var(y)) / 0.01 - 1) < 0.15, math.sqrt(hf_var(y))
    assert np.std(y) > 0.15                                     # a plain std would be dominated by the motion
    # frozen / duplicate detection
    rows = [(k * dt, float(k), 0.0, 0.0, 0.0, 0.0, 9.8) for k in range(100)]
    rows[50:65] = [(r[0],) + rows[49][1:] for r in rows[50:65]]  # 15 identical = frozen
    rows[80] = (rows[80][0],) + rows[79][1:]                     # one duplicate = not frozen
    d, rep = dedup(rows)
    assert rep['duplicates'] == 16 and rep['longest_run'] == 15 and rep['frozen'] and rep['frozen_runs'] == 1, rep
    assert len(d) == 84
    _, rep2 = dedup([rows[k] for k in range(100) if not 50 <= k < 65])
    assert not rep2['frozen'] and rep2['duplicates'] == 1
    # gyro vs RF2O: RF2O at 20 Hz, stamps 50 ms late -> best lag 0.05, residual ~ RF2O noise
    tg = np.arange(0, 20, dt)
    wz = 0.4 * np.sin(2 * math.pi * 0.5 * tg)
    yaw = np.cumsum(wz) * dt
    tr = np.arange(0.2, 19.8, 0.05)
    poses = [(tt + 0.05, float(np.interp(tt, tg, yaw)) + 0.0005 * rng.standard_normal()) for tt in tr]
    lag, rstd, nn, rmean = best_lag(tg, wz, rate_from_poses(poses), [k * 0.01 for k in range(-5, 16)])
    assert abs(lag - 0.05) < 0.011 and rstd < 0.03 and nn > 300, (lag, rstd)
    # block drift: slope 0.01 per minute recovered
    tb = np.arange(0, 900, 0.02)
    xb = 0.01 * tb / 60 + 0.001 * rng.standard_normal(len(tb))
    means, span, sl = block_drift(tb, xb, 60.0)
    assert abs(sl - 0.01) < 1e-4 and len(means) == 15 and abs(span - 0.14) < 0.01, (sl, span)
    # spectrum: 10 Hz vibration of rms 0.05 found
    tv = np.arange(0, 90, dt)
    v = 0.05 * math.sqrt(2) * np.sin(2 * math.pi * 10.0 * tv) + 0.01 * rng.standard_normal(len(tv))
    pk = spectrum_peaks(v, dt)
    assert abs(pk[0][0] - 10.0) < 0.3 and abs(pk[0][1] / 0.05 - 1) < 0.25, pk
    assert abs(recommend_r_gz(0.01) - 4e-4) < 1e-12 and abs(recommend_r_gz(0.01, 0.01) - 5e-4) < 1e-12
    print('PASS: Allan deviation (white slope), second-difference noise (ignores a 0.3 rad/s weave),')
    print('      duplicate vs frozen runs, gyro vs RF2O best lag 50 ms, 60 s block drift slope, vibration')
    print('      spectrum peak (frequency + rms), R recommendation. No ROS.')


# ---------------------------------------------------------------- bags

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def imu_rows(data):
    rows = sorted((stamp(m), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                   m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
                  for _, m in data.get('/imu/data_raw', []))
    return rows


def events(folder):
    with open(folder / 'runner_events.csv') as fp:
        return list(csv.DictReader(fp))


def drive_windows(folder, p):
    """[(t_drive, steady_a, steady_b)] in wall time (= header time on the Jetson) for each repeat."""
    info = json.loads((folder / 'run_info.json').read_text())
    v = float(info['profile']['speed'])
    ramp = float(info.get('ramp_accel_mps2') or vp.get(p, 'runner.ramp_accel_mps2'))
    ev = events(folder)
    starts = [float(e['wall']) for e in ev if e['event'] == 'DRIVE_START']
    zeros = [float(e['wall']) for e in ev if e['event'] == 'ZERO_COMMAND']
    out = []
    for t0 in starts:
        tz = next((t for t in zeros if t > t0), None)
        if tz is not None:
            out.append((t0, t0 + START_DELAY_S + v / ramp + SETTLE_S, tz - 0.1))
    return out, info


def sel(rows, a, b):
    return [r for r in rows if a <= r[0] <= b]


def fmt(v, f='%.4f'):
    return '-' if v is None else f % v


def mode_driving(folders, p):
    import numpy as np
    import analyze as A
    bench = float(vp.get(p, 't5.bench_gz_var_rad2_s2'))
    rows_out, all_still, all_drive, resid = [], {k: [] for k in (1, 2, 3)}, {k: [] for k in (1, 2, 3, 4)}, []
    for folder in folders:
        data = A.read_bag(folder / 'bag')
        raw = imu_rows(data)
        if not raw:
            print('%s: no IMU data, skipped' % folder.name)
            continue
        imu, rep = dedup(raw)
        if rep['frozen']:
            print('%s: IMU FROZEN (longest run %d samples), skipped' % (folder.name, rep['longest_run']))
            continue
        wins, info = drive_windows(folder, p)
        still = [sel(imu, t0 - 1.0, t0 - 0.02) for t0, _, _ in wins]
        drive = [sel(imu, a, b) for _, a, b in wins if b - a >= 0.5]
        res = dict(folder=folder.name, repeats=len(wins), steady_s=sum((b - a) for _, a, b in wins if b - a >= 0.5),
                   duplicates=rep['duplicate_share'])
        for k, name in ((1, 'gx'), (2, 'gy'), (3, 'gz')):
            res['still_' + name] = pooled(still, k)
            res['drive_' + name] = pooled(drive, k)
            all_still[k] += still
            all_drive[k] += drive
        res['drive_ax'] = pooled(drive, 4)
        all_drive[4] += drive
        # gz vs RF2O over the steady windows (base yaw rate = -(gz - bias), bias = still window mean)
        rf = sorted((stamp(m), A.quat_yaw(m.pose.pose.orientation)) for _, m in data.get('/odom_rf2o', []))
        ints, ts, wz = [], [], []
        for (t0, a, b), st in zip(wins, still):
            if b - a < 0.5 or not st:
                continue
            bias = float(np.mean([r[3] for r in st]))
            seg = sel(imu, a - 0.3, b + 0.3)
            ts += [r[0] for r in seg]
            wz += [-(r[3] - bias) for r in seg]
            ints += [iv for iv in rate_from_poses([q for q in rf if a <= q[0] <= b])]
        if ints and len(ts) > 50:
            order = np.argsort(ts)
            tsa, wza = np.asarray(ts)[order], np.asarray(wz)[order]
            bl = best_lag(tsa, wza, ints, [k * 0.01 for k in range(-5, 16)])
            if bl:
                res['rf2o'] = dict(lag_s=bl[0], residual_std=bl[1], n=bl[2], residual_mean=bl[3],
                                   ref_std=float(np.std([iv[2] for iv in ints])))
                resid.append((bl[1], bl[2]))
        rows_out.append(res)
        (folder / 't5_driving.json').write_text(json.dumps(res, indent=1))
    print('| T4 run | repeats | steady s | dup % | still gz std / hf | drive gz hf | drive gx / gy std | '
          'drive gx / gy hf | drive ax std m/s^2 | gz - RF2O std (lag ms) | RF2O ref std |')
    print('|' + '---|' * 11)
    for r in rows_out:
        def g(key, f='std'):
            return r[key][f] if r.get(key) else None
        rr = r.get('rf2o') or {}
        print('| %s | %d | %.1f | %.1f | %s / %s | %s | %s / %s | %s / %s | %s | %s (%s) | %s |' % (
            r['folder'][:15], r['repeats'], r['steady_s'], 100 * r['duplicates'],
            fmt(g('still_gz')), fmt(g('still_gz', 'hf_std')), fmt(g('drive_gz', 'hf_std')),
            fmt(g('drive_gx')), fmt(g('drive_gy')), fmt(g('drive_gx', 'hf_std')), fmt(g('drive_gy', 'hf_std')),
            fmt(g('drive_ax'), '%.3f'), fmt(rr.get('residual_std')),
            fmt(rr.get('lag_s') * 1000 if rr else None, '%.0f'), fmt(rr.get('ref_std'), '%.3f')))
    if not rows_out:
        return None
    tot = dict(still_gz=pooled(all_still[3], 3), drive_gz=pooled(all_drive[3], 3), drive_gx=pooled(all_drive[1], 1),
               drive_gy=pooled(all_drive[2], 2), still_gx=pooled(all_still[1], 1), still_gy=pooled(all_still[2], 2))
    rstd = math.sqrt(sum(s * s * n for s, n in resid) / sum(n for _, n in resid)) if resid else None
    hf_d, hf_s = tot['drive_gz']['hf_std'], tot['still_gz']['hf_std']
    per_run = sorted(r['drive_gz']['hf_std'] for r in rows_out if r.get('drive_gz'))
    rec = recommend_r_gz(per_run[-1])                       # worst run (floor): conservative, covers every floor seen
    rec_typ = recommend_r_gz(per_run[len(per_run) // 2])     # median run
    print('\nPooled (all healthy runs), rad/s:')
    print('  gz sigma_hf: still %.5f (var %.2e) | driving %.5f (var %.2e) | bench 11 Nov var %.1e (std %.5f)'
          % (hf_s, hf_s ** 2, hf_d, hf_d ** 2, bench, math.sqrt(bench)))
    print('  gx / gy std driving %.4f / %.4f (still %.4f / %.4f): roll / pitch vibration, true rates ~0 on a flat floor'
          % (tot['drive_gx']['std'], tot['drive_gy']['std'], tot['still_gx']['std'], tot['still_gy']['std']))
    if rstd:
        print('  gz - RF2O residual std %.4f (var %.2e): gyro (averaged over the RF2O interval) + RF2O noise + lag'
              % (rstd, rstd ** 2))
    print('  C1 shadow EKF uses R_gz %.1e (= (2 x robust std of gyro - reference)^2, includes the reference noise)'
          % C1_R_GZ)
    print('  RECOMMENDED driving R_gz = (2 x sigma_hf of the worst run %.4f)^2 = %.1e rad^2/s^2 (median run: %.1e).'
          % (per_run[-1], rec, rec_typ))
    print('  = gyro white noise incl. motor / floor vibration; bias and scale error are not in it (T5a / T3).')
    out = dict(runs=rows_out, pooled=tot, rf2o_residual_std=rstd, recommended_r_gz=rec, median_run_r_gz=rec_typ,
               bench_r_gz=bench, c1_r_gz=C1_R_GZ)
    (folders[0] / 't5_driving_pooled.json').write_text(json.dumps(out, indent=1))
    return out


def mode_stand(folder, p):
    import numpy as np
    import analyze as A
    t5 = vp.get(p, 't5')
    gain = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    dwheel = float(vp.get(p, 'vehicle.wheel_diameter_m'))
    data = A.read_bag(folder / 'bag')
    raw = imu_rows(data)
    imu, rep = dedup(raw)
    wins, info = drive_windows(folder, p)
    if info.get('test') != 't5b_stand_imu':
        print('WARNING: %s is a %s run, not t5b_stand_imu' % (folder.name, info.get('test')))
    if not wins:
        raise SystemExit('no DRIVE_START / ZERO_COMMAND pair in runner_events.csv')
    t0, a, b = max(wins, key=lambda w: w[2] - w[1])           # the longest repeat; b = zero command - 0.1
    tz = b + 0.1
    a = a - SETTLE_S + float(t5['settle_s'])
    phases = dict(still=sel(imu, t0 - float(t5['still_s']), t0 - 0.02), steady=sel(imu, a, b),
                  after=sel(imu, tz + 2.5, tz + 2.5 + float(t5['still_s'])))
    core = [(t, m.state.speed) for t, m in data.get('/sensors/core', []) if a <= t <= b]
    erpm = float(np.mean([e for _, e in core])) if core else None
    wheel_hz = abs(erpm) / gain / (math.pi * dwheel) if erpm else None
    res = dict(folder=folder.name, steady_s=b - a, imu=rep, timing=timing([r[0] for r in raw]),
               erpm_mean=erpm, erpm_std=float(np.std([e for _, e in core])) if core else None, wheel_hz=wheel_hz,
               phases={ph: {ax: axis_stats(rows, k + 1) for k, ax in enumerate(AXES)} for ph, rows in phases.items()})
    st, sd = res['phases']['still'], res['phases']['steady']
    if st['gz'] and sd['gz']:
        shift = sd['gz']['mean'] - st['gz']['mean']
        res['gz_bias_shift_motor_on'] = shift
        res['heading_drift_deg_per_min_from_shift'] = math.degrees(shift) * 60.0
    ts = [r[0] for r in phases['steady']]
    if len(ts) > 100:
        _, span, sl = block_drift(ts, [r[3] for r in phases['steady']], 10.0)
        res['steady_gz_slope_per_min'], res['steady_gz_10s_block_span'] = sl, span
        res['spectrum'] = {ax: spectrum_peaks([r[k + 1] for r in phases['steady']], 0.01) for k, ax in enumerate(AXES)}
    (folder / 't5_stand.json').write_text(json.dumps(res, indent=1))
    print('T5b %s: steady %.1f s at %s ERPM (sd %s), wheel %s Hz | IMU %s Hz, dup %.1f %%, longest run %d%s'
          % (folder.name, b - a, fmt(erpm, '%.0f'), fmt(res['erpm_std'], '%.0f'), fmt(wheel_hz, '%.2f'),
             fmt(res['timing'].get('rate_hz'), '%.1f'), 100 * rep['duplicate_share'], rep['longest_run'],
             ' FROZEN' if rep['frozen'] else ''))
    print('| axis | still mean / std / hf | steady mean / std / hf | after mean / std / hf | steady peaks Hz (rms) |')
    print('|---|---|---|---|---|')
    for ax in AXES:
        cells = []
        for ph in ('still', 'steady', 'after'):
            s = res['phases'][ph][ax]
            cells.append('-' if not s else '%+.4f / %.4f / %s' % (s['mean'], s['std'], fmt(s['hf_std'])))
        pk = ', '.join('%.1f (%.3f)' % x for x in res.get('spectrum', {}).get(ax, []))
        print('| %s | %s | %s |' % (ax, ' | '.join(cells), pk or '-'))
    if 'gz_bias_shift_motor_on' in res:
        print('gz bias shift with the motor on: %+.5f rad/s (%+.3f deg/s) -> %+.1f deg heading per minute if '
              'uncorrected; steady gz slope %s rad/s per min, 10 s block span %s'
              % (res['gz_bias_shift_motor_on'], math.degrees(res['gz_bias_shift_motor_on']),
                 res['heading_drift_deg_per_min_from_shift'], fmt(res.get('steady_gz_slope_per_min'), '%+.5f'),
                 fmt(res.get('steady_gz_10s_block_span'), '%.5f')))
    if sd['gz'] and sd['gz']['hf_std']:
        print('R_gz from the stand (2 x sigma_hf,steady)^2 = %.1e (still: %.1e); the car does not move, so this is '
              'pure sensor + vibration noise' % (recommend_r_gz(sd['gz']['hf_std']),
                                                 recommend_r_gz(st['gz']['hf_std']) if st['gz'] else float('nan')))
    return res


def mode_static(folder, p):
    import numpy as np
    import analyze as A
    t5 = vp.get(p, 't5')
    bench = float(vp.get(p, 't5.bench_gz_var_rad2_s2'))
    bag = folder / 'bag' if (folder / 'bag').exists() else folder
    data = A.read_bag(bag)
    raw = imu_rows(data)
    if not raw:
        raise SystemExit('no /imu/data_raw in %s' % bag)
    tim = timing([r[0] for r in raw])
    imu, rep = dedup(raw)
    imu = [r for r in imu if r[0] >= imu[0][0] + 5.0]          # skip the first 5 s
    ts = np.array([r[0] for r in imu])
    dt = float(np.median(np.diff(ts)))
    res = dict(folder=str(folder), timing=tim, imu=rep, dt_s=dt, axes={})
    if tim['duration_s'] < float(t5['static_min_s']):
        print('WARNING: only %.0f s (T5a wants >= %.0f s)' % (tim['duration_s'], float(t5['static_min_s'])))
    taus = [0.01, 0.03, 0.1, 0.3, 1, 3, 10, 30, 100, 300]
    for k, ax in enumerate(AXES):
        x = np.array([r[k + 1] for r in imu])
        ad = allan(x, dt, taus)
        means, span, sl = block_drift(ts, x, float(t5['block_s']))
        res['axes'][ax] = dict(mean=float(x.mean()), std=float(x.std()), var=float(x.var()), hf_std=math.sqrt(hf_var(x)),
                               allan=ad, **allan_summary(ad), block_means=means, block_span=span, slope_per_min=sl)
    gz = np.array([r[3] for r in imu])
    heading = -np.cumsum(gz) * dt                                # base yaw from the node-calibrated gz
    first = gz[ts <= ts[0] + 10.0].mean()
    heading_rm = -np.cumsum(gz - first) * dt                     # as the runner: bias = first 10 s
    res['heading_drift_deg'] = dict(node_calibrated=math.degrees(float(heading[-1])),
                                    first_10s_bias_removed=math.degrees(float(heading_rm[-1])),
                                    over_s=float(ts[-1] - ts[0]))
    log = folder / 'imu_launch.log'
    if log.exists():
        res['node_log_bias_lines'] = [ln.strip() for ln in log.read_text(errors='replace').splitlines()
                                      if 'bias' in ln.lower()][:4]
    (folder / 't5_static.json').write_text(json.dumps(res, indent=1))
    print('T5a %s: %.0f s, %.1f Hz, max gap %.3f s, gaps > 30 ms %d | duplicates %.2f %%, longest run %d -> %s'
          % (folder.name, tim['duration_s'], tim['rate_hz'], tim['max_gap_s'], tim['gaps_over_30ms'],
             100 * rep['duplicate_share'], rep['longest_run'], 'FROZEN' if rep['frozen'] else 'no freeze'))
    print('| axis | mean (residual bias) | std | var = R | sigma_hf | ARW adev(1 s) | bias instab. (tau) | '
          '60 s block span | slope / min |')
    print('|' + '---|' * 9)
    for ax in AXES:
        a = res['axes'][ax]
        print('| %s | %+.5f | %.5f | %.2e | %.5f | %s | %s (%s s) | %s | %s |' % (
            ax, a['mean'], a['std'], a['var'], a['hf_std'], fmt(a.get('arw_at_1s'), '%.2e'),
            fmt(a.get('bias_instability'), '%.2e'), fmt(a.get('tau_min_s'), '%.0f'), fmt(a['block_span'], '%.5f'),
            fmt(a['slope_per_min'], '%+.6f')))
    g = res['axes']['gz']
    print('gz (rad/s): residual bias %+.5f (%+.4f deg/s); R %.2e vs bench %.1e; ARW %.3f deg/sqrt(h); bias '
          'instability %.4f deg/s (%.1f deg/h)' % (
              g['mean'], math.degrees(g['mean']), g['var'], bench, math.degrees(g['arw_at_1s']) * 60.0,
              math.degrees(g['bias_instability']), math.degrees(g['bias_instability']) * 3600.0))
    h = res['heading_drift_deg']
    print('Heading drift over %.0f s: %+.1f deg with the node calibration only, %+.1f deg after removing the '
          'first-10 s mean (what a start-up bias estimate leaves)' % (h['over_s'], h['node_calibrated'],
                                                                      h['first_10s_bias_removed']))
    for ln in res.get('node_log_bias_lines', []):
        print('  node log: %s' % ln)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('mode', nargs='?', choices=['driving', 'stand', 'static'])
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.mode:
        ap.error('mode or --self-test')
    p = vp.load()
    folders = [Path(f).expanduser() for f in args.folders]
    if args.mode == 'driving':
        if not folders:
            folders = sorted(vp.evidence_root().glob('*_t4_straight_*_armed'))
        mode_driving(folders, p)
    elif args.mode == 'stand':
        folders = folders or sorted(vp.evidence_root().glob('*_t5b_stand_imu_*_armed'))[-1:]
        mode_stand(folders[0], p)
    else:
        folders = folders or sorted(vp.evidence_root().glob('*_t5a_static'))[-1:]
        if not folders:
            raise SystemExit('no *_t5a_static folder')
        mode_static(folders[0], p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
