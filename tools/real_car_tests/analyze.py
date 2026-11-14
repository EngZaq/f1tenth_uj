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
"""
import argparse
import csv
import json
import os
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from run_test import interval_stats  # noqa: E402

UNDER_ERPM = 100.0
REBOUND_FAIL_ERPM = 300.0   # T1 pass: no rebound > 300 ERPM
STOP_WINDOW_S = 3.0
JOY_TTL_S, COMMAND_TTL_S = 0.20, 0.22   # physical_drive_mux.py
EXPECTED_HZ = {'/scan': 29.6, '/odom_rf2o': 20.0, '/imu/data_raw': 100.0,
               '/control/status': 20.0, '/manual/drive': 50.0, '/joy': 50.0, '/drive': 20.0}

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
    running_min, rise, rise_at, peak_after_min = a[0], 0.0, None, 0.0
    for k, i in enumerate(idx):
        running_min = min(running_min, a[k])
        if a[k] - running_min > rise:
            rise, rise_at = a[k] - running_min, t[i] - t0
            peak_after_min = a[k]
    under = next((t[i] - t0 for k, i in enumerate(idx) if a[k] < UNDER_ERPM), None)
    before = [i for i, x in enumerate(t) if x < t0]
    erpm_at_zero = abs(erpm[before[-1]]) if before else a[0]
    out = dict(samples=len(idx), erpm_at_zero=round(erpm_at_zero, 1),
               time_to_under_100_erpm_s=None if under is None else round(under, 3),
               rebound_rise_erpm=round(rise, 1),
               rebound_peak_erpm=round(peak_after_min, 1),
               rebound_at_s=None if rise_at is None else round(rise_at, 3),
               max_abs_current_after_zero_a=round(max(abs(current[i]) for i in idx), 2),
               pass_no_rebound_over_300=rise <= REBOUND_FAIL_ERPM)
    if tach:
        out['tach_counts_after_zero'] = tach[idx[-1]] - (tach[before[-1]] if before else tach[idx[0]])
    return out


def zero_transitions(t, cmd, eps=1.0):
    """Times where the commanded ERPM goes from > eps to <= eps."""
    return [t[i] for i in range(1, len(t)) if abs(cmd[i - 1]) > eps and abs(cmd[i]) <= eps]


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
    clean = [4200.0 if x < 1.0 else max(0.0, 4200 - (x - 1.0) * 8000) for x in t]
    m = stop_metrics(t, clean, cur, None, 1.0, 4.0)
    assert m['pass_no_rebound_over_300'] and abs(m['time_to_under_100_erpm_s'] - 0.52) < 0.03, m
    assert zero_transitions([0, 1, 2, 3, 4], [0, 4207, 4207, 0, 0]) == [3]
    tr = mode_transitions([(0, {'mode': 'STOP'}), (1, {'mode': 'MANUAL_READY'}),
                           (2, {'mode': 'MANUAL_READY'}),
                           (3, {'mode': 'STOP', 'last_stop_reason': 'JOYSTICK_DEADMAN_RELEASED_OR_STALE'})])
    assert [x['to_mode'] for x in tr] == ['STOP', 'MANUAL_READY', 'STOP']
    assert tr[-1]['last_stop_reason'].startswith('JOYSTICK')
    print('PASS: stop metrics (12 Nov rebound shape fails, clean decay passes), zero detection,')
    print('      mux transitions. No bag read; nothing published.')


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


def analyze(folder):
    folder = Path(folder).expanduser()
    bag_dir = folder / 'bag' if (folder / 'bag' / 'metadata.yaml').exists() else folder
    data = read_bag(bag_dir)
    summary = {'folder': str(folder)}
    run_info = folder / 'run_info.json'
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
    drive = [m.drive.speed for _, m in data.get('/drive', [])]
    summary['drive_max_speed_mps'] = max(drive) if drive else None
    if statuses and not any(s.get('real_actuation_enabled') for _, s in statuses):
        # None = no /drive in the bag: nothing proven, not a pass.
        summary['disarmed_output_all_zero'] = (max(drive) == 0.0) if drive else None

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
    cmd = data.get('/commands/motor/speed', [])
    mt = [t for t, _ in cmd]
    mv = [m.data for _, m in cmd]
    with open(folder / 'motor_cmd.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['t', 'erpm_cmd'])
        wr.writerows(zip(mt, mv))
    if core:
        summary['vesc'] = dict(max_abs_erpm=round(max(abs(x) for x in erpm), 1),
                               max_abs_current_a=round(max(abs(x) for x in cur), 2),
                               faults=sorted(set(m.state.fault_code for _, m in core)),
                               voltage_min_v=round(min(m.state.voltage_input for _, m in core), 2))
    if mv:
        summary['max_erpm_command'] = max(mv)

    # Stops: zero transitions on the real motor command; else the runner's ZERO_COMMAND events.
    events = read_csv(folder / 'runner_events.csv')
    zeros = zero_transitions(mt, mv)
    source = '/commands/motor/speed'
    if not zeros:
        zeros = [float(e['wall']) for e in events if e['event'] == 'ZERO_COMMAND']
        source = 'runner_events.csv (motor command never nonzero)'
    reasons = [e['detail'] for e in events if e['event'] == 'ZERO_COMMAND']
    stops = []
    for k, t0 in enumerate(zeros):
        m = stop_metrics(ct, erpm, cur, tach, t0, t0 + STOP_WINDOW_S) if core else {'samples': 0}
        m.update(index=k, t0=t0)
        if k < len(reasons) and source.startswith('/commands'):
            m['runner_reason'] = reasons[k]
        stops.append(m)
    summary['stops'] = dict(source=source, count=len(stops), items=stops)
    if stops and any(s.get('samples') for s in stops):
        summary['stops']['all_pass_no_rebound_over_300'] = all(
            s.get('pass_no_rebound_over_300', True) for s in stops)

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
    if 'disarmed_output_all_zero' in s:
        lines.append('- Disarmed: /drive always zero = %s' % s['disarmed_output_all_zero'])
    st = s['stops']
    lines.append('- Stops: %d (from %s)' % (st['count'], st['source']))
    for x in st['items']:
        if x.get('samples'):
            lines.append('  - stop %d: at zero %.0f ERPM, < 100 ERPM after %s s, rebound +%.0f ERPM '
                         '(peak %.0f), max current %.1f A, tach %s, pass=%s' % (
                             x['index'] + 1, x['erpm_at_zero'], x['time_to_under_100_erpm_s'],
                             x['rebound_rise_erpm'], x['rebound_peak_erpm'],
                             x['max_abs_current_after_zero_a'], x.get('tach_counts_after_zero'),
                             x['pass_no_rebound_over_300']))
    readme = folder / 'README.md'
    text = readme.read_text() if readme.exists() else ''
    text = text.split('## Analysis (analyze.py)')[0].rstrip() + '\n\n' + '\n'.join(lines) + '\n'
    readme.write_text(text)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
    elif args.folder:
        s = analyze(args.folder)
        print((Path(args.folder).expanduser() / 'README.md').read_text().split('## Analysis (analyze.py)')[-1])
        print('summary.json, CSVs and %s written.' % ', '.join(s['plots'] or ['no plots']))
    else:
        ap.error('folder or --self-test')


if __name__ == '__main__':
    main()
