#!/usr/bin/env python3
"""T3 analysis (21 Nov): constant-steering circles (run_test.py t3_circle). Reads bags only.

Per run (each runner folder = one --steer; every repeat that reached steady speed is analysed, the one
stopped by TURNS_DONE is "counted"):
- Steady window: drive start + start delay (0.3 s) + v / ramp accel + SETTLE_S .. zero command - 0.1 s.
- Yaw rate: BMI160 base yaw rate -(gz - bias), bias = mean gz over the 1 s before the drive start; and the
  RF2O yaw slope (scale check gyro / RF2O).
- Speed: C1 shadow EKF vx (live /ekf_shadow/odom, if a *_ekf_shadow_live recording covers the run), RF2O
  twist vx and VESC ERPM / speed_to_erpm_gain.
- Radius at the rear axle (base_link): R = v / yaw rate (per speed source) and an algebraic (Kasa) circle fit
  of the RF2O path and of the EKF path. Real bicycle angle delta = atan(wheelbase / R) (from the RF2O fit,
  and from v_EKF / gyro) vs the command and vs T2 (the T2 19 Nov per-side fit at the servo actually sent,
  /sensors/servo_position_command = after the VESC clip).
- Lateral slip at the rear axle: beta = direction of motion (RF2O path) - RF2O yaw; vy = v sin(beta). Small
  beta supports the EKF vy = 0 constraint (the EKF itself cannot show it: its vy is constrained).
- IMU y-axis sign: specific force along IMU +y made perpendicular to the still gravity, vs the expected
  centripetal v x yaw rate (+ toward the circle centre = base +y in a left turn): ratio ~ +1 -> IMU y = base y,
  ~ -1 -> IMU y = -base y. Body roll in the turn adds to the magnitude with the same sign.
Across runs: ground per-side gain g = (servo - centre) / delta_real (centre = vehicle_params servo_offset).
FULL LOCK runs (run_test.py --full-lock, servo exactly at the clip 0.10 / 0.68) are reported in their own table:
real minimum turning radius at the rear axle (circle fit and v / yaw rate), real wheel angle vs the T2 stand angle
at that clip (steering.max_left_rad / max_right_rad), and the radius swept by the outer front body corner. They are
not used for the gain (the command is a clip, not an angle).
Writes t3.json into each run folder.
  t3_analyze.py <t3 folder> [...] [--shadow <ekf_shadow_live folder>] [--t2 <T2 folder>]
  t3_analyze.py --self-test
"""
import argparse
import csv
import json
import math
import os
import statistics
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

START_DELAY_S = 0.3     # command -> motion (T4: 0.26-0.30 s)
SETTLE_S = 1.0          # after the ramp: steering and speed settled
SLIP_DT_S = 0.2         # path direction from positions this far apart
T2_DEFAULT = '20261119_134151_t2_servo_sweep'


# ---------------------------------------------------------------- pure logic

def circle_fit(xy):
    """Algebraic (Kasa) circle fit: x^2 + y^2 + D x + E y + F = 0 -> (cx, cy, R, rms radial residual)."""
    import numpy as np
    a = np.array(xy, dtype=float)
    if len(a) < 5:
        return None
    A = np.column_stack([a[:, 0], a[:, 1], np.ones(len(a))])
    b = -(a[:, 0] ** 2 + a[:, 1] ** 2)
    (D, E, F), *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = -D / 2, -E / 2
    r2 = cx * cx + cy * cy - F
    if r2 <= 0:
        return None
    R = math.sqrt(r2)
    res = np.hypot(a[:, 0] - cx, a[:, 1] - cy) - R
    return float(cx), float(cy), float(R), float(math.sqrt(float(np.mean(res ** 2))))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def unwrap(vals):
    out = []
    for v in vals:
        out.append(v if not out else out[-1] + wrap(v - out[-1]))
    return out


def slope(ts, vs):
    n = len(ts)
    if n < 3:
        return None
    mt, mv = sum(ts) / n, sum(vs) / n
    stt = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / stt if stt > 0 else None


def slip_angles(poses, dt=SLIP_DT_S):
    """poses [(t, x, y, yaw)] -> [beta] = chord direction (positions >= dt apart) - yaw interpolated at the
    chord's mid-time (on a circle the chord is parallel to the tangent at its mid-time)."""
    ts = [q[0] for q in poses]
    yu = unwrap([q[3] for q in poses])
    out, j = [], 0
    for i, (t, x, y, _) in enumerate(poses):
        while j < len(poses) and poses[j][0] < t + dt:
            j += 1
        if j >= len(poses):
            break
        t2, x2, y2, _ = poses[j]
        if math.hypot(x2 - x, y2 - y) < 0.02:
            continue
        tm = (t + t2) / 2
        k = max(i, min(j - 1, next(n for n in range(i, j + 1) if ts[n + 1] >= tm)))
        f = (tm - ts[k]) / (ts[k + 1] - ts[k]) if ts[k + 1] > ts[k] else 0.0
        mid = yu[k] + f * (yu[k + 1] - yu[k])
        out.append(wrap(math.atan2(y2 - y, x2 - x) - mid))
    return out


def bicycle_angle(wheelbase, R, side):
    return side * math.atan(wheelbase / R)


def t2_angle(fit, servo):
    """T2 19 Nov per-side fit: bicycle angle (rad, + = left) at a servo value."""
    side = 'left' if servo < fit['offset'] else 'right'
    return (servo - fit['%s_offset' % side]) / fit['%s_gain' % side]


def run_tables(info):
    """The run's converter steering tables [(delta, servo)] (left, right) from run_info converter_params, or None
    (gain mode / runs before 22 Nov)."""
    cp = info.get('converter_params') or {}
    tl, tr = cp.get('steer_table_left') or [], cp.get('steer_table_right') or []
    if not (tl and tr):
        return None
    return tuple(list(zip([float(v) for v in t[0::2]], [float(v) for v in t[1::2]])) for t in (tl, tr))


def table_angle(table, servo):
    """Inverse of one side's table: the angle the table assigns to this servo value (None outside the table)."""
    for (d0, s0), (d1, s1) in zip(table, table[1:]):
        if min(s0, s1) - 1e-9 <= servo <= max(s0, s1) + 1e-9 and s1 != s0:
            return d0 + (d1 - d0) * (servo - s0) / (s1 - s0)
    return None


def lateral_axis(gravity):
    """Unit IMU +y made perpendicular to the still gravity vector (IMU frame)."""
    n = math.sqrt(sum(g * g for g in gravity))
    gh = [g / n for g in gravity]
    l = [(1.0 if i == 1 else 0.0) - gh[1] * gh[i] for i in range(3)]
    ln = math.sqrt(sum(x * x for x in l))
    return [x / ln for x in l], n


def self_test():
    import random
    rnd = random.Random(5)
    # Circle fit on a noisy 1.5-turn arc, R 0.98.
    pts = [(2 + 0.98 * math.cos(a) + rnd.gauss(0, 0.005), -1 + 0.98 * math.sin(a) + rnd.gauss(0, 0.005))
           for a in [k * 0.02 for k in range(470)]]
    cx, cy, R, rms = circle_fit(pts)
    assert abs(R - 0.98) < 0.005 and abs(cx - 2) < 0.01 and abs(cy + 1) < 0.01 and rms < 0.01, (cx, cy, R, rms)
    # Slip: a car on a left circle whose heading points 3 deg inward of its path -> beta = -3 deg.
    poses, yaw_off = [], math.radians(-3.0)
    for k in range(600):
        t = k * 0.02
        a = 0.5 / 0.98 * t
        x, y = 0.98 * math.sin(a), 0.98 * (1 - math.cos(a))
        poses.append((t, x, y, a - yaw_off))
    b = slip_angles(poses)
    assert abs(math.degrees(statistics.mean(b)) - (-3.0)) < 0.2, statistics.mean(b)
    b20 = slip_angles(poses[::5])                                  # 20 Hz, as RF2O
    assert abs(math.degrees(statistics.mean(b20)) - (-3.0)) < 0.2, math.degrees(statistics.mean(b20))
    assert abs(math.degrees(bicycle_angle(0.26, 0.26 / math.tan(0.26), 1.0)) - math.degrees(0.26)) < 1e-9
    fit = dict(offset=0.3636, left_offset=0.3563, left_gain=-0.8743, right_offset=0.3442, right_gain=-1.0624)
    assert abs(t2_angle(fit, 0.10) - (0.10 - 0.3563) / -0.8743) < 1e-12 and t2_angle(fit, 0.10) > 0
    assert t2_angle(fit, 0.60) < 0
    # IMU y: tilted, upside-down mount (still 1.8 / -0.55 / -10.02); +y_imu = base +y here.
    l, gn = lateral_axis((1.8, -0.55, -10.02))
    assert abs(sum(a * b for a, b in zip(l, (1.8, -0.55, -10.02)))) < 1e-9 and l[1] > 0.99
    u = unwrap([3.1, -3.1, -2.9])
    assert u[1] > 3.1 and abs(slope([0, 1, 2], [0, 2, 4]) - 2) < 1e-12
    # steering table (22 Nov): inverse, run detection, column label
    info_t = dict(converter_params=dict(steer_table_left=[0.0, 0.335, 0.2042, 0.178, 0.28, 0.11],
                                        steer_table_right=[0.0, 0.335, -0.1414, 0.526, -0.28, 0.660]))
    tl, tr = run_tables(info_t)
    assert abs(table_angle(tl, 0.178) - 0.2042) < 1e-12 and abs(table_angle(tl, 0.11) - 0.28) < 1e-12
    assert abs(table_angle(tr, 0.593) - (-0.1414 + (-0.1386) * (0.593 - 0.526) / 0.134)) < 1e-9
    assert table_angle(tl, 0.335) == 0.0 and table_angle(tl, 0.10) is None
    assert run_tables(dict(converter_params=dict(servo_offset=0.335))) is None and run_tables({}) is None
    assert ref_label([('a', dict(yaw_rate_gyro=0, delta_ref='table'))]) == ('delta table', False)
    assert ref_label([('a', dict(yaw_rate_gyro=0, delta_ref='T2'))]) == ('delta T2', False)
    assert ref_label([('a', dict(yaw_rate_gyro=0))]) == ('delta T2', False)                     # pre-22 Nov runs
    lab, mx = ref_label([('a', dict(yaw_rate_gyro=0, delta_ref='T2')), ('b', dict(yaw_rate_gyro=0, delta_ref='table'))])
    assert mx and ref_cell(dict(delta_ref='table', delta_table_deg=11.7), mx) == '+11.7 (table)'
    assert ref_cell(dict(delta_ref='table', delta_table_lock_deg=16.04), False, lock=True) == '+16.0'
    print('PASS: Kasa circle fit (noisy 1.5 turns), slip angle sign/size, bicycle angle, T2 per-side angle,')
    print('      gravity-perpendicular IMU y axis, unwrap, slope; steering-table inverse, "delta table" column. No ROS.')


# ---------------------------------------------------------------- bags

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def shadow_covering(t0, t1):
    """The *_ekf_shadow_live recording whose bag spans [t0, t1], or None."""
    import yaml
    for folder in sorted(vp.evidence_root().glob('*_ekf_shadow_live'), reverse=True):
        meta = folder / 'bag' / 'metadata.yaml'
        if not meta.exists():
            continue
        info = yaml.safe_load(meta.read_text())['rosbag2_bagfile_information']
        s = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
        e = s + info['duration']['nanoseconds'] * 1e-9
        if s <= t0 and t1 <= e:
            return folder
    return None


def events(folder):
    with open(folder / 'runner_events.csv') as fp:
        return list(csv.DictReader(fp))


def window(series, a, b):
    return [(t, v) for t, v in series if a <= t <= b]


def analyze_run(folder, data, ekf, p, fit_t2):
    info = json.loads((folder / 'run_info.json').read_text())
    circ = info.get('circle') or {}
    steer = float(circ.get('steer_rad', info['profile'].get('steer')))
    lock = circ.get('full_lock') or info['profile'].get('full_lock')
    tables = run_tables(info)
    side = 1.0 if steer > 0 else -1.0
    veh = vp.get(p, 'vehicle')
    hw, front_x = float(veh['body_width_m']) / 2.0, float(veh['front_bumper_x_m'])
    L = float(vp.get(p, 'vehicle.wheelbase_m'))
    gain = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    centre = float(circ.get('servo_offset', vp.get(p, 'steering.servo_offset')))
    v_cmd = float(info['profile']['speed'])
    ramp = float(info.get('ramp_accel_mps2') or vp.get(p, 'runner.ramp_accel_mps2'))
    ev = events(folder)
    starts = [(float(e['wall']), e) for e in ev if e['event'] == 'DRIVE_START']
    zeros = [(float(e['wall']), e['detail']) for e in ev if e['event'] == 'ZERO_COMMAND']
    imu = [(t, m) for t, m in data['/imu/data_raw']]
    gz = [(t, m.angular_velocity.z) for t, m in imu]
    rf = sorted(((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y,
                  math.atan2(2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z),
                             1 - 2 * m.pose.pose.orientation.z ** 2), m.twist.twist.linear.x)
                 for _, m in data['/odom_rf2o']), key=lambda r: r[0])
    core = [(t, m.state.speed / gain) for t, m in data['/sensors/core']]
    servo = [(t, m.data) for t, m in data.get('/sensors/servo_position_command', [])]
    out = []
    for t_drive, _ in starts:
        tz = next((t for t, _ in zeros if t > t_drive), None)
        reason = next((d for t, d in zeros if t > t_drive), '')
        if tz is None:
            continue
        a = t_drive + START_DELAY_S + v_cmd / ramp + SETTLE_S
        b = tz - 0.1
        res = dict(drive_start=t_drive, zero=tz, reason=reason, counted=reason.startswith('TURNS_DONE'),
                   steer_cmd_rad=steer, full_lock=lock, window_s=b - a)
        if b - a < 3.0:
            res['note'] = 'steady window < 3 s: not analysed'
            out.append(res)
            continue
        pre = [v for t, v in gz if t_drive - 1.0 <= t < t_drive - 0.02]
        bias = statistics.mean(pre)
        wz = [-(v - bias) for _, v in window(gz, a, b)]
        w_gyro = statistics.mean(wz)
        rfw = [r for r in rf if a <= r[0] <= b]
        yaw_u = unwrap([r[3] for r in rfw])
        w_rf = slope([r[0] for r in rfw], yaw_u)
        v_rf = statistics.mean(r[4] for r in rfw)
        v_vesc = statistics.mean(v for _, v in window(core, a, b))
        ekw = [e for e in ekf if a <= e[0] <= b] if ekf else []
        v_ekf = statistics.mean(e[4] for e in ekw) if ekw else None
        fit_rf = circle_fit([(r[1], r[2]) for r in rfw])
        fit_ekf = circle_fit([(e[1], e[2]) for e in ekw]) if len(ekw) > 10 else None
        sv = [v for _, v in window(servo, a, b)]
        servo_sent = statistics.mean(sv) if sv else None
        beta = slip_angles([(r[0], r[1], r[2], r[3]) for r in rfw])
        # IMU y sign
        still = [m.linear_acceleration for t, m in imu if t_drive - 1.0 <= t < t_drive - 0.02]
        grav = [statistics.mean(getattr(x, k) for x in still) for k in 'xyz']
        lat, g_norm = lateral_axis(grav)
        ay = 9.80665 / g_norm * statistics.mean(sum(c * getattr(m.linear_acceleration, k) for c, k in zip(lat, 'xyz'))
                             for t, m in imu if a <= t <= b)
        v_ref = v_ekf if v_ekf is not None else v_rf
        ay_expected = v_ref * w_gyro
        R_v = {k: (v / abs(w_gyro)) for k, v in (('ekf', v_ekf), ('rf2o', v_rf), ('vesc', v_vesc)) if v is not None}
        R_fit = fit_rf[2] if fit_rf else None
        d_fit = bicycle_angle(L, R_fit, side) if R_fit else None
        d_v = bicycle_angle(L, R_v.get('ekf', R_v['rf2o']), side)
        turned = sum(-(v - bias) * (t2 - t1) for (t1, v), (t2, _) in zip(window(gz, t_drive, tz), window(gz, t_drive, tz)[1:]))
        res.update(
            gyro_bias=bias, yaw_rate_gyro=w_gyro, yaw_rate_rf2o=w_rf,
            gyro_over_rf2o=(w_gyro / w_rf) if w_rf else None, turned_deg=math.degrees(turned),
            v_ekf=v_ekf, v_rf2o=v_rf, v_vesc=v_vesc, R_v=R_v,
            R_fit_rf2o=R_fit, fit_rf2o_rms=fit_rf[3] if fit_rf else None,
            R_fit_ekf=fit_ekf[2] if fit_ekf else None, fit_ekf_rms=fit_ekf[3] if fit_ekf else None,
            delta_real_fit_deg=math.degrees(d_fit) if d_fit is not None else None,
            delta_real_v_deg=math.degrees(d_v), delta_cmd_deg=math.degrees(steer),
            servo_sent=servo_sent, servo_planned=circ.get('servo_planned'),
            delta_t2_deg=math.degrees(t2_angle(fit_t2, servo_sent)) if (fit_t2 and servo_sent) else None,
            delta_ref='table' if tables else 'T2',
            delta_table_deg=(math.degrees(table_angle(tables[0] if side > 0 else tables[1], servo_sent))
                             if (tables and servo_sent and table_angle(tables[0] if side > 0 else tables[1],
                                                                       servo_sent) is not None) else None),
            ground_gain=(((servo_sent - centre) / (d_fit if d_fit is not None else d_v))
                         if (servo_sent and not lock) else None),
            delta_t2_clip_deg=(math.degrees(float(vp.get(p, 'steering.max_left_rad' if side > 0 else
                                                         'steering.max_right_rad'))) if lock else None),
            delta_table_lock_deg=(math.degrees((tables[0] if side > 0 else tables[1])[-1][0])
                                  if (lock and tables) else None),
            r_outer_corner=math.hypot((R_fit if R_fit else R_v.get('ekf', R_v['rf2o'])) + hw, front_x),
            slip_beta_deg=math.degrees(statistics.mean(beta)) if beta else None,
            slip_beta_sd_deg=math.degrees(statistics.pstdev(beta)) if len(beta) > 1 else None,
            vy_mps=v_ref * math.sin(statistics.mean(beta)) if beta else None,
            imu_ay_lateral=ay, ay_expected=ay_expected, imu_y_ratio=ay / ay_expected if ay_expected else None,
            imu_y_sign=('+' if ay * ay_expected > 0 else '-') if ay_expected else None, ekf_source=bool(ekw))
        out.append(res)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--shadow', help='ekf_shadow_live folder (default: the one whose bag covers each run)')
    ap.add_argument('--t2', default=None, help='T2 folder with fit.json (default %s)' % T2_DEFAULT)
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    import analyze as A
    p = vp.load()
    t2 = Path(args.t2).expanduser() if args.t2 else vp.evidence_root() / T2_DEFAULT
    fit_t2 = json.loads((t2 / 'fit.json').read_text()) if (t2 / 'fit.json').exists() else None
    folders = [Path(f).expanduser() for f in args.folders] or \
        sorted(vp.evidence_root().glob('*_t3_circle_*_armed'))[-4:]
    rows = []
    for folder in folders:
        data = A.read_bag(folder / 'bag')
        t_first = min(t for t, _ in data['/sensors/core'])
        t_last = max(t for t, _ in data['/sensors/core'])
        sh = Path(args.shadow).expanduser() if args.shadow else shadow_covering(t_first, t_last)
        ekf = None
        if sh:
            sb = A.read_bag(sh / 'bag' if (sh / 'bag').exists() else sh)
            ekf = sorted(((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, 0.0, m.twist.twist.linear.x)
                          for _, m in sb.get('/ekf_shadow/odom', [])), key=lambda e: e[0])
        res = analyze_run(folder, data, ekf, p, fit_t2)
        (folder / 't3.json').write_text(json.dumps(dict(folder=str(folder), shadow=str(sh) if sh else None,
                                                        t2_fit=str(t2), runs=res), indent=1))
        rows += [(folder.name[:15], r) for r in res]
    print_table(rows, p)
    return 0


def ref_label(rows):
    """Column name for the reference angle: 'delta table' / 'delta T2', or both (then each cell is marked)."""
    refs = {r.get('delta_ref', 'T2') for _, r in rows if 'yaw_rate_gyro' in r}
    return ('delta table' if refs == {'table'} else 'delta T2' if refs <= {'T2'} else 'delta T2 or table'), len(refs) > 1


def ref_cell(r, mixed, fmt='%+.1f', lock=False):
    tab = r.get('delta_ref') == 'table'
    v = (r.get('delta_table_lock_deg') if tab else r.get('delta_t2_clip_deg')) if lock else \
        (r.get('delta_table_deg') if tab else r.get('delta_t2_deg'))
    return ('-' if v is None else fmt % v) + ((' (table)' if tab else ' (T2)') if mixed else '')


def print_table(rows, p):
    def f(v, fmt):
        return '-' if v is None else fmt % v
    locks = [(run, r) for run, r in rows if r.get('full_lock')]
    rows = [(run, r) for run, r in rows if not r.get('full_lock')]
    ref, mixed = ref_label(rows)
    print('| run | cnt | steer cmd deg | servo | v EKF / RF2O / VESC m/s | yaw rate gyro rad/s (gyro/RF2O) | '
          'R v/w EKF m | R fit RF2O m (rms) | R fit EKF m | delta real fit / v deg | %s deg | ground gain | '
          'slip beta deg (vy m/s) | IMU y ratio |' % ref)
    print('|' + '---|' * 14)
    for run, r in rows:
        if 'yaw_rate_gyro' not in r:
            print('| %s | %s | %+.1f | - | %s |' % (run, 'y' if r['counted'] else 'n', math.degrees(r['steer_cmd_rad']),
                                                  r.get('note', '-')))
            continue
        print('| %s | %s | %+.1f | %s | %s / %.3f / %.3f | %+.3f (%s) | %s | %s (%s) | %s | %s / %+.1f | %s | %s | '
              '%s (%s) | %s |' % (
                  run, 'y' if r['counted'] else 'n', r['delta_cmd_deg'], f(r['servo_sent'], '%.3f'),
                  f(r['v_ekf'], '%.3f'), r['v_rf2o'], r['v_vesc'], r['yaw_rate_gyro'], f(r['gyro_over_rf2o'], '%.3f'),
                  f(r['R_v'].get('ekf'), '%.3f'), f(r['R_fit_rf2o'], '%.3f'), f(r['fit_rf2o_rms'], '%.3f'),
                  f(r['R_fit_ekf'], '%.3f'), f(r['delta_real_fit_deg'], '%+.1f'), r['delta_real_v_deg'],
                  ref_cell(r, mixed), f(r['ground_gain'], '%.3f'), f(r['slip_beta_deg'], '%+.1f'),
                  f(r['vy_mps'], '%+.3f'), f(r['imu_y_ratio'], '%+.2f')))
    for side, name in ((1, 'left'), (-1, 'right')):
        g = [r['ground_gain'] for _, r in rows if r.get('ground_gain') and r['steer_cmd_rad'] * side > 0]
        if g:
            print('Ground %s gain (servo - centre) / delta_real: %s -> mean %.3f (T2 stand fit %s, converter T3 %.3f)'
                  % (name, ', '.join('%.3f' % x for x in g), statistics.mean(g),
                     {'left': -0.874, 'right': -1.062}[name], float(vp.get(p, 't3.servo_gain_%s' % name))))
    if locks:
        lref, lmixed = ref_label(locks)
        print('\nFULL LOCK (servo at the driving clip or the table end; minimum turning radius, not a gain):')
        print('| run | cnt | side | servo | v EKF / RF2O m/s | yaw rate gyro rad/s | R min fit RF2O m (rms) | R v/w EKF m | '
              'delta real fit / v deg | %s at the lock deg | outer front corner radius m | slip deg | IMU y |' % lref)
        print('|' + '---|' * 13)
        for run, r in locks:
            if 'yaw_rate_gyro' not in r:
                print('| %s | %s | %s | - | %s |' % (run, 'y' if r['counted'] else 'n', r['full_lock'], r.get('note', '-')))
                continue
            print('| %s | %s | %s | %s | %s / %.3f | %+.3f | %s (%s) | %s | %s / %+.1f | %s | %.3f | %s | %s |' % (
                run, 'y' if r['counted'] else 'n', r['full_lock'], f(r['servo_sent'], '%.3f'), f(r['v_ekf'], '%.3f'),
                r['v_rf2o'], r['yaw_rate_gyro'], f(r['R_fit_rf2o'], '%.3f'), f(r['fit_rf2o_rms'], '%.3f'),
                f(r['R_v'].get('ekf'), '%.3f'), f(r['delta_real_fit_deg'], '%+.1f'), r['delta_real_v_deg'],
                ref_cell(r, lmixed, lock=True), r['r_outer_corner'], f(r['slip_beta_deg'], '%+.1f'),
                f(r['imu_y_ratio'], '%+.2f')))
        rows = rows + locks                                       # IMU y sign from every run
    signs = [r['imu_y_sign'] for _, r in rows if r.get('imu_y_sign')]
    if signs:
        print('IMU y axis: %s -> %s' % (' '.join(signs), 'IMU +y = base +y' if set(signs) == {'+'} else
                                        'IMU +y = base -y' if set(signs) == {'-'} else 'INCONSISTENT'))


if __name__ == '__main__':
    sys.exit(main())
