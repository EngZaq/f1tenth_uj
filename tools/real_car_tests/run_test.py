#!/usr/bin/env python3
"""F1TENTH UJ real-car test runner. Drives ONLY through physical_drive_mux.

Publishes /manual/drive, /joy (button 5 = deadman) and /control/request_mode at
50 Hz: the same interfaces as ~/continuous_keyboard_mux.py. Never /drive, never
/commands/*. Records its own ros2 bag. Not an emergency stop: the battery
disconnect / kill switch stays in the operator's hand.

Keys over SSH (no Enter): n = arm, hold SPACE = drive, s = skip, q = quit.
SPACE is a deadman from terminal auto-repeat: 0.7 s after the first press,
then 0.25 s between repeats. No fresh SPACE -> zero, and zero is kept >= 1 s.

  run_test.py --self-test          offline, no ROS
  run_test.py --list
  run_test.py --test dry_run       mux must report REAL_ACTUATION disabled
"""
import argparse
import csv
import json
import math
import os
import re
import select
import signal
import statistics
import subprocess
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

# place: 'stand' (wheels off the ground) or 'ground' (needs --t1-passed-confirmed).
# Every profile ends by itself after ramp + steady; SPACE release ends it earlier.
PROFILES = {
    'dry_run': dict(
        place='stand', speed=0.30, steer=0.0, steady_s=2.0, repeats=2, post_zero_s=2.0,
        live_allowed=False,
        note='Disarmed end-to-end check: keyboard deadman, mux handshake, bag, analysis.'),
}

# The bag is a subscriber: it must never touch the actuator chain, or the mux counts
# 2 subscribers and latches ACTUATOR_GRAPH_OWNERSHIP_FAULT (dry run 14 Nov). Command side
# comes from /control/status (mux output speed_mps / steer_rad), /manual/drive,
# /sensors/servo_position_command and the vesc_driver VESC_SPEED_AUDIT log (sent ERPM).
BAG_TOPICS = [
    '/scan', '/odom_rf2o', '/odom_vesc_observe', '/imu/data_raw', '/sensors/core',
    '/sensors/servo_position_command', '/control/status',
    '/manual/drive', '/joy', '/control/request_mode', '/tf', '/tf_static',
]
PROTECTED_TOPICS = ('/drive', '/drive_calibrated')
PROTECTED_PREFIXES = ('/commands/',)


def is_protected(topic):
    """Actuator-chain topics nothing in these tools may publish to or subscribe to."""
    return topic in PROTECTED_TOPICS or topic.startswith(PROTECTED_PREFIXES)

# Topics this runner must own alone, and actuator topics nobody may publish in this stack.
OWN_TOPICS = ['/manual/drive', '/joy', '/control/request_mode']
FORBIDDEN_PUBLISHED = ['/commands/motor/brake', '/commands/motor/current',
                       '/commands/motor/duty_cycle', '/commands/motor/position']


# ---------------------------------------------------------------- pure logic

class Deadman:
    """SPACE auto-repeat deadman. Time is injected so it is testable."""

    def __init__(self, first_s, repeat_s):
        self.first_s, self.repeat_s = first_s, repeat_s
        self.reset()

    def reset(self):
        self.last = None
        self.count = 0

    def press(self, now):
        if self.last is None or now - self.last > self.first_s:
            self.count = 0  # a new press, not an auto-repeat
        self.last = now
        self.count += 1

    def held(self, now):
        if self.last is None:
            return False
        limit = self.repeat_s if self.count >= 2 else self.first_s
        return now - self.last < limit


def ramp_speed(target, elapsed, accel):
    return min(target, max(0.0, accel * elapsed))


def required_length(v, ramp_accel, decel, steady_m, margin_m, bumper_m):
    """LiDAR range needed ahead for a straight run at v (baseline speed-fit rule)."""
    return v * v / (2 * ramp_accel) + steady_m + v * v / (2 * decel) + margin_m + bumper_m


def front_range(scan, half_width_deg):
    """Median finite range around 0 rad (car front; LiDAR yaw is 0). From reference script."""
    if scan is None or not scan.angle_increment:
        return float('nan')
    n = len(scan.ranges)
    full = (scan.angle_max - scan.angle_min) > 6.2
    hw = math.radians(half_width_deg)
    lo = int(math.floor((-hw - scan.angle_min) / scan.angle_increment))
    hi = int(math.ceil((hw - scan.angle_min) / scan.angle_increment))
    vals = []
    for i in range(lo, hi + 1):
        if full:
            i %= n
        elif not 0 <= i < n:
            continue
        r = scan.ranges[i]
        if math.isfinite(r) and scan.range_min < r < scan.range_max:
            vals.append(r)
    return statistics.median(vals) if vals else float('nan')


def interval_stats(dts):
    if not dts:
        return {'count': 0}
    s = sorted(dts)

    def q(f):
        return s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))]
    return {'count': len(s), 'mean_s': round(sum(s) / len(s), 5), 'p50_s': round(q(.5), 5),
            'p95_s': round(q(.95), 5), 'p99_s': round(q(.99), 5), 'max_s': round(s[-1], 5),
            'over_0.10_s': sum(d > .10 for d in s), 'over_0.20_s': sum(d > .20 for d in s)}


class Machine:
    """Runner state machine. step() gets time, one key, deadman state and a world
    snapshot; returns (speed, steer, deadman_button, mode_request or None)."""

    def __init__(self, prof, p, live, ground_ok=False):
        self.prof, self.live = prof, live
        r = vp.get(p, 'runner')
        self.ramp = float(r['ramp_accel_mps2'])
        self.status_ttl = float(r['status_ttl_s'])
        self.arm_timeout = float(r['arm_timeout_s'])
        self.post_zero = max(float(r['min_post_stop_zero_s']), float(prof['post_zero_s']))
        self.decel = vp.decel_for_checks(p)
        self.bumper = vp.bumper_ahead_of_lidar(p)
        g = vp.get(p, 'ground_check')
        self.steady_m, self.margin = float(g['steady_length_m']), float(g['margin_m'])
        self.ground = prof['place'] == 'ground'
        self.ground_ok = ground_ok
        v = float(prof['speed'])
        self.ramp_s = v / self.ramp
        self.steady_s = self.steady_m / v if self.ground else float(prof['steady_s'])
        self.hard_s = self.ramp_s + self.steady_s + 0.5
        self.state = 'IDLE'
        self.repeat = 0
        self.quit = False
        self.events = []
        self.repeats = []
        self.reset_deadman = False
        self.message = 'Press n to arm (SPACE released).'

    def log(self, now, event, detail=''):
        self.events.append(dict(wall=time.time(), mono=now, repeat=self.repeat,
                                state=self.state, event=event, detail=detail))
        self.message = '%s %s' % (event, detail)

    def fault(self, w):
        st = w.get('status')
        if st is None or w.get('status_age', 1e9) > self.status_ttl:
            return 'MUX_STATUS_STALE'
        if bool(st.get('real_actuation_enabled')) != self.live:
            return 'REAL_FLAG_MISMATCH(real=%s,live=%s)' % (st.get('real_actuation_enabled'), self.live)
        for k in ('drive_publisher_count', 'drive_subscriber_count',
                  'motor_publisher_count', 'motor_subscriber_count'):
            if st.get(k) != 1:
                return 'ACTUATOR_GRAPH_CHANGED(%s=%s)' % (k, st.get(k))
        if w.get('graph_fault'):
            return w['graph_fault']
        if w.get('keyboard_lost'):
            return 'KEYBOARD_LOST'
        if self.live and w.get('telemetry_age', 1e9) > 0.5:
            return 'VESC_TELEMETRY_STALE'
        if w.get('vesc_fault'):
            return 'VESC_FAULT_%s' % w['vesc_fault']
        if self.ground and w.get('scan_age', 1e9) > 0.3:
            return 'SCAN_STALE'
        return None

    def ground_fit(self, w):
        v = float(self.prof['speed'])
        need = required_length(v, self.ramp, self.decel, self.steady_m, self.margin, self.bumper)
        have = w.get('front_m', float('nan'))
        return (math.isfinite(have) and have >= need), need, have

    def stop(self, now, reason):
        self.state = 'STOPPING'
        self.t_stop = now
        self.log(now, 'ZERO_COMMAND', reason)
        if self.repeats and self.repeats[-1]['repeat'] == self.repeat and 'zero_wall' not in self.repeats[-1]:
            self.repeats[-1].update(zero_wall=time.time(), zero_reason=reason)
        return 0.0, 0.0, 0, 'STOP'

    def step(self, now, key, held, w):
        st = w.get('status') or {}
        mode = st.get('mode')
        if key == 'q' or w.get('keyboard_lost'):
            self.quit = True  # lost SSH terminal: finish with zero, never wait in IDLE
        if self.state == 'IDLE':
            if self.quit:
                self.state = 'DONE'
                return 0.0, 0.0, 0, 'STOP'
            if key == 's':
                self.log(now, 'SKIP')
                self.repeat += 1
                if self.repeat >= self.prof['repeats']:
                    self.state = 'DONE'
            elif key == 'n':
                reason = self.fault(w)
                if reason is None and held:
                    reason = 'RELEASE_SPACE_FIRST'
                if reason is None and self.ground:
                    if not self.ground_ok:
                        reason = 'GROUND_NEEDS_T1_PASS_CONFIRMED'
                    else:
                        ok, need, have = self.ground_fit(w)
                        self.log(now, 'SPEED_FIT', 'need=%.2f m have=%.2f m' % (need, have))
                        if not ok:
                            reason = 'SPEED_DOES_NOT_FIT(need %.2f m, have %.2f m)' % (need, have)
                if reason:
                    self.log(now, 'ARM_REFUSED', reason)
                else:
                    self.state = 'ARMING'
                    self.t_arm = self.t_req = now
                    self.log(now, 'ARM_REQUEST')
                    return 0.0, 0.0, 0, 'MANUAL'
            return 0.0, 0.0, 0, None
        if self.state == 'ARMING':
            reason = self.fault(w)
            if self.quit or reason:
                self.state = 'IDLE'
                self.log(now, 'ARM_ABORTED', reason or 'QUIT')
                return 0.0, 0.0, 0, 'STOP'
            if mode == 'MANUAL_READY' and st.get('last_mode_request') == 'MANUAL':
                self.state = 'READY'
                self.log(now, 'MANUAL_READY', 'hold SPACE to drive')
            elif now - self.t_arm > self.arm_timeout:
                self.state = 'IDLE'
                self.log(now, 'ARM_REFUSED', 'MANUAL_READY_TIMEOUT(mode=%s,result=%s)' % (
                    mode, st.get('mode_request_result')))
                return 0.0, 0.0, 0, 'STOP'
            elif now - self.t_req > 0.25 and mode == 'STOP':
                self.t_req = now
                return 0.0, 0.0, 0, 'MANUAL'
            return 0.0, 0.0, 0, None
        if self.state == 'READY':
            reason = self.fault(w)
            if reason or self.quit or key == 's' or mode != 'MANUAL_READY':
                if key == 's':
                    self.repeats.append(dict(repeat=self.repeat, skipped=True))
                return self.stop(now, reason or ('QUIT' if self.quit else 'SKIP' if key == 's'
                                                 else 'MUX_LEFT_MANUAL_MODE(%s)' % mode))
            if held:
                self.state = 'DRIVING'
                self.t_drive = now
                self.steady_logged = False
                self.repeats.append(dict(repeat=self.repeat, target_mps=self.prof['speed'],
                                         steer_rad=self.prof['steer'], start_wall=time.time()))
                self.log(now, 'DRIVE_START', 'target=%.2f m/s' % self.prof['speed'])
            else:
                return 0.0, 0.0, 0, None
        if self.state == 'DRIVING':
            elapsed = now - self.t_drive
            v = ramp_speed(float(self.prof['speed']), elapsed, self.ramp)
            reason = self.fault(w)
            if reason is None and not held:
                reason = 'DEADMAN_RELEASED'
            if reason is None and mode not in ('MANUAL_READY', 'MANUAL_ACTIVE'):
                reason = 'MUX_LEFT_MANUAL_MODE(%s,%s)' % (mode, st.get('last_stop_reason'))
            if reason is None and self.quit:
                reason = 'QUIT'
            if reason is None and key == 's':
                reason = 'SKIP'
            if reason is None and elapsed >= self.ramp_s + self.steady_s:
                reason = 'PROFILE_DONE'
            if reason is None and elapsed >= self.hard_s:
                reason = 'HARD_TIME_LIMIT'
            if reason is None and self.ground:
                have = w.get('front_m', float('nan')) - self.bumper
                if not math.isfinite(have) or have <= v * v / (2 * self.decel) + self.margin:
                    reason = 'WALL_STOP_DISTANCE(%.2f m)' % have
            if reason:
                return self.stop(now, reason)
            if not self.steady_logged and v >= float(self.prof['speed']):
                self.steady_logged = True
                self.log(now, 'STEADY_REACHED')
            return v, float(self.prof['steer']), 1, None
        if self.state == 'STOPPING':
            if now - self.t_stop < self.post_zero:
                return 0.0, 0.0, 0, None
            self.log(now, 'REPEAT_END')
            self.repeat += 1
            self.reset_deadman = True
            self.state = 'DONE' if (self.quit or self.repeat >= self.prof['repeats']) else 'IDLE'
            if self.state == 'IDLE':
                self.message = 'Repeat %d/%d: release SPACE, press n to arm.' % (
                    self.repeat + 1, self.prof['repeats'])
            return 0.0, 0.0, 0, 'STOP'
        return 0.0, 0.0, 0, None


def self_test():
    p = vp.load()
    assert vp.config_mismatches(p) == [], vp.config_mismatches(p)
    d = Deadman(0.7, 0.25)
    d.press(0.0)
    assert d.held(0.69) and not d.held(0.71)
    d.reset()
    d.press(0.0)
    d.press(0.6)  # first auto-repeat
    assert d.held(0.84) and not d.held(0.86)
    d.press(1.0)  # gap 0.4 s < 0.7 s: still the same hold
    assert d.count == 3 and not d.held(1.26)
    assert abs(required_length(1.0, 0.5, 1.0, 1.0, 0.5, 0.09) - 3.09) < 1e-9
    assert ramp_speed(1.0, 0.5, 0.5) == 0.25 and ramp_speed(1.0, 9, 0.5) == 1.0

    class Scan:
        angle_min, angle_max = -math.pi, math.pi
        angle_increment = 2 * math.pi / 1440
        range_min, range_max = 0.05, 30.0
        ranges = [5.0] * 1440
    s = Scan()
    s.ranges[720] = float('inf')
    assert front_range(s, 3.0) == 5.0
    s.ranges = [float('inf')] * 1440
    assert math.isnan(front_range(s, 3.0))

    ok_status = dict(mode='STOP', real_actuation_enabled=False, drive_publisher_count=1,
                     drive_subscriber_count=1, motor_publisher_count=1, motor_subscriber_count=1)

    def world(**kw):
        st = dict(ok_status, **kw.pop('st', {}))
        return dict(dict(status=st, status_age=0.05), **kw)
    prof = PROFILES['dry_run']
    m = Machine(prof, p, live=False)
    assert m.step(0.0, 'n', False, world())[3] == 'MANUAL' and m.state == 'ARMING'
    m.step(0.1, None, False, world(st=dict(mode='MANUAL_READY', last_mode_request='MANUAL')))
    assert m.state == 'READY'
    ready = world(st=dict(mode='MANUAL_READY', last_mode_request='MANUAL'))
    active = world(st=dict(mode='MANUAL_ACTIVE', last_mode_request='MANUAL'))
    assert m.step(0.2, None, True, ready)[:3] == (0.0, 0.0, 1) and m.state == 'DRIVING'
    out = m.step(0.4, None, True, active)
    assert m.state == 'DRIVING' and abs(out[0] - 0.1) < 1e-9 and out[2] == 1
    # stale status -> zero on the same tick
    out = m.step(0.5, None, True, world(status_age=0.3, st=dict(mode='MANUAL_ACTIVE')))
    assert out[:3] == (0.0, 0.0, 0) and m.state == 'STOPPING'
    assert m.step(1.0, None, False, world())[0] == 0.0 and m.state == 'STOPPING'
    m.step(2.6, None, False, world())
    assert m.state == 'IDLE' and m.repeat == 1 and m.reset_deadman
    # second repeat runs to PROFILE_DONE; deadman release also stops at once
    m.step(3.0, 'n', False, world())
    m.step(3.1, None, False, ready)
    m.step(3.2, None, True, ready)
    assert m.step(3.2 + m.ramp_s + m.steady_s - 0.01, None, True, active)[0] == prof['speed']
    assert m.step(3.2 + m.ramp_s + m.steady_s, None, True, active)[0] == 0.0
    assert m.events[-1]['detail'] == 'PROFILE_DONE'
    m.step(9.0, None, False, world())
    assert m.state == 'DONE'
    # refusals: armed mux for a disarmed profile, extra command source, SPACE held at arm
    m = Machine(prof, p, live=False)
    m.step(0.0, 'n', False, world(st=dict(real_actuation_enabled=True)))
    assert m.state == 'IDLE' and 'REAL_FLAG_MISMATCH' in m.events[-1]['detail']
    m.step(0.1, 'n', False, world(graph_fault='EXTRA_PUBLISHER(/commands/motor/brake=1)'))
    assert m.state == 'IDLE'
    m.step(0.2, 'n', True, world())
    assert m.state == 'IDLE' and m.events[-1]['detail'] == 'RELEASE_SPACE_FIRST'
    # mux leaving MANUAL while driving (F-13 path) -> zero
    m.step(0.3, 'n', False, world())
    m.step(0.4, None, False, ready)
    m.step(0.5, None, True, ready)
    assert m.step(0.6, None, True, world(st=dict(mode='STOP')))[0] == 0.0
    # ground: refuse without T1 confirmation, refuse when too short, wall stop while driving
    gprof = dict(prof, place='ground', speed=1.0)
    g = Machine(gprof, p, live=False)
    g.step(0.0, 'n', False, world(front_m=10.0, scan_age=0.05))
    assert g.state == 'IDLE' and g.events[-1]['detail'] == 'GROUND_NEEDS_T1_PASS_CONFIRMED'
    g = Machine(gprof, p, live=False, ground_ok=True)
    g.step(0.0, 'n', False, world(front_m=3.0, scan_age=0.05))
    assert g.state == 'IDLE' and 'SPEED_DOES_NOT_FIT' in g.events[-1]['detail']
    g.step(0.1, 'n', False, world(front_m=6.0, scan_age=0.05))
    assert g.state == 'ARMING'
    g.step(0.2, None, False, world(front_m=6.0, scan_age=0.05, st=dict(mode='MANUAL_READY', last_mode_request='MANUAL')))
    g.step(0.3, None, True, world(front_m=6.0, scan_age=0.05, st=dict(mode='MANUAL_READY')))
    assert g.step(1.3, None, True, world(front_m=1.5, scan_age=0.05, st=dict(mode='MANUAL_ACTIVE')))[0] > 0
    # 0.55 m/s: stop zone 0.55^2/2 + 0.5 = 0.65 m from the bumper; 0.6 m LiDAR = 0.51 m bumper
    assert g.step(1.4, None, True, world(front_m=0.6, scan_age=0.05, st=dict(mode='MANUAL_ACTIVE')))[0] == 0.0
    assert g.events[-1]['detail'].startswith('WALL_STOP_DISTANCE')
    # lost terminal while idle -> DONE (no endless idle loop)
    m = Machine(prof, p, live=False)
    m.step(0.0, None, False, world(keyboard_lost=True))
    assert m.state == 'DONE'
    st = interval_stats([0.02] * 99 + [0.25])
    assert st['over_0.20_s'] == 1 and st['p50_s'] == 0.02
    # The bag must never subscribe to the actuator chain (dry run 14 Nov: drive_subscriber_count=2).
    bad = [t for t in BAG_TOPICS if is_protected(t)]
    assert not bad, 'protected topics in BAG_TOPICS: %s' % bad
    assert all(is_protected(t) for t in ('/drive', '/drive_calibrated', '/commands/motor/speed',
                                         '/commands/motor/brake', '/commands/servo/position'))
    assert not any(is_protected(t) for t in ('/control/status', '/manual/drive', '/sensors/core'))
    a = parse_audit_line('[INFO] [1794653015.653271431] [vesc_driver]: '
                         'VESC_SPEED_AUDIT received_erpm=4207.000 sent_erpm=3500.000')
    assert a == (1794653015.653271431, 4207.0, 3500.0), a
    assert parse_audit_line('[INFO] [1.0] [vesc_driver]: Connected to VESC') is None
    print('PASS: deadman timing, ramp, speed-fit length, front range, config = vehicle_params,')
    print('      arm handshake, zero on stale status / deadman release / mux leaving MANUAL,')
    print('      refusals (real flag, extra publisher, SPACE held, ground gates), wall stop,')
    print('      no protected topic in the bag list, audit-log parsing.')
    # Clean shutdown on every exit path, in child processes on an isolated domain
    # (ROS_DOMAIN_ID 77, localhost only). The probe node has no publishers.
    env = dict(os.environ, ROS_DOMAIN_ID='77', ROS_LOCALHOST_ONLY='1')
    for mode, want in (('normal', 0), ('systemexit', 3), ('keyboardinterrupt', None)) * 2:
        res = subprocess.run([sys.executable, os.path.realpath(__file__), '--shutdown-probe', mode],
                             env=env, capture_output=True, text=True, timeout=60)
        aborted = res.returncode in (-signal.SIGABRT, 134) or 'terminate called' in res.stderr
        assert not aborted, 'abort on %s exit:\n%s' % (mode, res.stderr[-800:])
        assert want is None or res.returncode == want, (mode, res.returncode, res.stderr[-800:])
        assert 'CHILD STILL RUNNING' not in res.stderr, mode
    print('PASS: shutdown probe x2 (normal, SystemExit, KeyboardInterrupt): no abort, spin thread')
    print('      joined, child process group stopped. Isolated domain 77; nothing published.')


# ---------------------------------------------------------------- processes and logs

AUDIT_RE = re.compile(r'\[(\d+\.\d+)\] \[vesc_driver\]: VESC_SPEED_AUDIT '
                      r'received_erpm=(-?[\d.]+) sent_erpm=(-?[\d.]+)')


def parse_audit_line(line):
    """vesc_driver audit line (teammates' uncommitted build) -> (wall_s, received, sent) or None."""
    m = AUDIT_RE.search(line)
    return (float(m.group(1)), float(m.group(2)), float(m.group(3))) if m else None


def ros_log_dir():
    if os.environ.get('ROS_LOG_DIR'):
        return Path(os.environ['ROS_LOG_DIR'])
    return Path(os.environ.get('ROS_HOME') or os.path.expanduser('~/.ros')) / 'log'


def vesc_driver_pid():
    """The single running vesc_driver_node, else None (none or several)."""
    out = subprocess.run(['pgrep', '-f', 'vesc_driver/vesc_driver_node'],
                         capture_output=True, text=True).stdout.split()
    return int(out[0]) if len(out) == 1 else None


def copy_vesc_audit(out, pid, t0, t1):
    """Copy VESC_SPEED_AUDIT lines in [t0-2, t1+2] into the results folder: the record of
    the ERPM vesc_driver actually sent. Returns a note for runner_summary.json."""
    if pid is None:
        return 'no /vesc_driver in this ROS graph, or not exactly one vesc_driver process'
    files = sorted(ros_log_dir().glob('vesc_driver_node_%d_*.log' % pid))
    if not files:
        return 'no log file for vesc_driver pid %d in %s' % (pid, ros_log_dir())
    n = 0
    with open(files[-1]) as src, open(out / 'vesc_speed_audit.log', 'w') as dst:
        for line in src:
            a = parse_audit_line(line)
            if a and t0 - 2.0 <= a[0] <= t1 + 2.0:
                dst.write(line)
                n += 1
    if not n:
        return 'no VESC_SPEED_AUDIT lines in the run window (audit build not active?)'
    return 'copied %d VESC_SPEED_AUDIT lines from %s' % (n, files[-1])


def stop_process_group(proc, timeout=15.0):
    """SIGINT the whole group (ros2 bag finalises its file), then SIGTERM, then SIGKILL."""
    if proc is None or proc.poll() is not None:
        return None if proc is None else proc.returncode
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            return proc.wait(timeout=wait)
        except subprocess.TimeoutExpired:
            continue
    return proc.poll()


class RosSession:
    """rclpy context + one spin thread, torn down in order on every exit path:
    executor shutdown, join the spin thread, destroy the node, rclpy shutdown.
    Shutting rclpy down under a running spin thread aborts the process
    ("terminate called without an active exception", dry run 14 Nov)."""

    def __init__(self, node_factory):
        self.node_factory = node_factory

    def __enter__(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        # No rclpy SIGINT handler: it would shut the context down before the zero commands
        # in `finally` are published. Ctrl+C -> KeyboardInterrupt -> finally -> zero.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        try:
            self.node = self.node_factory()
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
        except BaseException:
            rclpy.try_shutdown()
            raise
        self.thread = threading.Thread(target=self.executor.spin, name='ros_spin', daemon=True)
        self.thread.start()
        return self.node

    def __exit__(self, *exc):
        import rclpy
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            say('WARNING: ROS spin thread did not stop')
        self.node.destroy_node()
        rclpy.try_shutdown()
        return False


def shutdown_probe(mode):
    """Self-test helper, run in a child process on an isolated domain. No publishers.
    Starts a session and a process group like the bag, then leaves through `mode`."""
    from rclpy.node import Node
    child = subprocess.Popen(['sleep', '30'], start_new_session=True)
    try:
        with RosSession(lambda: Node('shutdown_probe')):
            time.sleep(0.3)
            if mode == 'systemexit':
                raise SystemExit(3)
            if mode == 'keyboardinterrupt':
                raise KeyboardInterrupt
    finally:
        stop_process_group(child)
        if child.poll() is None:
            print('CHILD STILL RUNNING', file=sys.stderr)


# ---------------------------------------------------------------- terminal

class Keyboard:
    """Single keys from the SSH terminal (no Enter) in a background thread."""

    def __init__(self, deadman):
        if not sys.stdin.isatty():
            raise SystemExit('stdin is not a terminal: the SPACE deadman needs a real TTY')
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.deadman = deadman
        self.lock = threading.Lock()
        self.pending = []
        self.log = []  # (wall, mono, key)
        self.eof = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.eof:
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if not ready:
                continue
            try:
                ch = os.read(self.fd, 1)
            except (BlockingIOError, InterruptedError):
                continue  # another reader took the byte; not a lost terminal
            except OSError:
                ch = b''
            if not ch:
                self.eof = True
                break
            now = time.monotonic()
            key = 'space' if ch == b' ' else ch.decode(errors='ignore').lower()
            with self.lock:
                self.log.append((time.time(), now, key))
                if key == 'space':
                    self.deadman.press(now)
                else:
                    self.pending.append(key)

    def held(self, now):
        with self.lock:
            return not self.eof and self.deadman.held(now)

    def get_key(self):
        with self.lock:
            return self.pending.pop(0) if self.pending else None

    def reset_deadman(self):
        with self.lock:
            self.deadman.reset()

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


def say(text):
    try:
        print(text, flush=True)
    except OSError:  # SSH terminal gone
        pass


# ---------------------------------------------------------------- ROS

def run(args):
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ackermann_msgs.msg import AckermannDriveStamped
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import Joy, LaserScan
    from std_msgs.msg import String
    try:
        from vesc_msgs.msg import VescStateStamped
    except ImportError:
        VescStateStamped = None

    p = vp.load(args.params)
    prof = PROFILES[args.test]
    if args.live and not prof['live_allowed']:
        raise SystemExit('profile %s never runs with real actuation' % args.test)
    if prof['place'] == 'ground' and not args.t1_passed_confirmed:
        raise SystemExit('ground profile: needs --t1-passed-confirmed (Moatasem written T1 pass)')
    mism = vp.config_mismatches(p)
    if mism:
        raise SystemExit('config/real_car disagrees with vehicle_params.yaml:\n  ' + '\n  '.join(mism))
    r = vp.get(p, 'runner')

    class RunnerNode(Node):
        def __init__(self):
            super().__init__('real_car_test_runner')
            self.lock = threading.Lock()
            self.manual = self.create_publisher(AckermannDriveStamped, '/manual/drive', 10)
            self.joy = self.create_publisher(Joy, '/joy', 10)
            self.mode = self.create_publisher(String, '/control/request_mode', 10)
            self.status = self.status_at = None
            self.status_log = []
            self.tele = self.tele_at = None
            self.scan = self.scan_at = None
            self.create_subscription(String, '/control/status', self.on_status, 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
            if VescStateStamped is not None:
                self.create_subscription(VescStateStamped, '/sensors/core', self.on_core, 10)

        def on_status(self, msg):
            try:
                st = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            with self.lock:
                self.status, self.status_at = st, time.monotonic()
                self.status_log.append((time.time(), msg.data))

        def on_scan(self, msg):
            with self.lock:
                self.scan, self.scan_at = msg, time.monotonic()

        def on_core(self, msg):
            with self.lock:
                self.tele = dict(erpm=msg.state.speed, current=msg.state.current_motor,
                                 fault=msg.state.fault_code)
                self.tele_at = time.monotonic()

        def emit(self, speed, steer, deadman):
            # Command before deadman, as in continuous_keyboard_mux.py.
            cmd = AckermannDriveStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.drive.speed = float(speed)
            cmd.drive.steering_angle = float(steer)
            self.manual.publish(cmd)
            joy = Joy()
            joy.header.stamp = cmd.header.stamp
            joy.buttons = [0, 0, 0, 0, 0, int(deadman)]
            self.joy.publish(joy)

        def request(self, mode):
            self.mode.publish(String(data=mode))

        def graph_fault(self):
            for t in OWN_TOPICS:
                n = len(self.get_publishers_info_by_topic(t))
                if n != 1:
                    return 'EXTRA_PUBLISHER(%s=%d)' % (t, n)
            for t in FORBIDDEN_PUBLISHED:
                n = len(self.get_publishers_info_by_topic(t))
                if n:
                    return 'EXTRA_PUBLISHER(%s=%d)' % (t, n)
            return None

        def world(self, now, half_width):
            with self.lock:
                w = dict(status=self.status,
                         status_age=(now - self.status_at) if self.status_at else 1e9,
                         telemetry_age=(now - self.tele_at) if self.tele_at else 1e9,
                         vesc_fault=(self.tele or {}).get('fault'),
                         scan_age=(now - self.scan_at) if self.scan_at else 1e9,
                         front_m=front_range(self.scan, half_width))
                w['erpm'] = (self.tele or {}).get('erpm')
            return w

        def vesc_speed_max(self):
            cli = self.create_client(GetParameters, '/vesc_driver/get_parameters')
            if not cli.wait_for_service(timeout_sec=2.0):
                return None
            fut = cli.call_async(GetParameters.Request(names=['speed_max']))
            t_end = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < t_end:
                time.sleep(0.02)
            if not fut.done() or not fut.result().values:
                return None
            v = fut.result().values[0]
            return v.double_value if v.type == 3 else None

    if any(is_protected(t) for t in BAG_TOPICS):
        raise SystemExit('BAG_TOPICS contains an actuator-chain topic: refusing.')
    # Every exit below (return, refusal, SystemExit, Ctrl+C) leaves through RosSession,
    # which stops the spin thread before rclpy shuts down.
    with RosSession(RunnerNode) as node:
        drive_session(args, p, prof, r, node)


def drive_session(args, p, prof, r, node):
    period = 1.0 / float(r['publish_hz'])
    t_end = time.monotonic() + 3.0
    while node.status is None and time.monotonic() < t_end:
        time.sleep(0.05)
    if node.status is None:
        raise SystemExit('No /control/status in 3 s: start the test stack first.')
    st = dict(node.status)
    real = bool(st.get('real_actuation_enabled'))
    if real != args.live:
        raise SystemExit('Mux real_actuation_enabled=%s but --live=%s: refusing.' % (real, args.live))
    cap = float(st.get('max_speed_mps', vp.get(p, 'caps.mux_default_max_speed_mps')))
    steer_cap = float(st.get('max_steer_rad', vp.get(p, 'caps.mux_default_max_steer_rad')))
    if prof['speed'] > cap + 1e-9 or abs(prof['steer']) > steer_cap + 1e-9:
        raise SystemExit('Profile %.2f m/s / %.2f rad exceeds mux cap %.2f m/s / %.2f rad.' % (
            prof['speed'], prof['steer'], cap, steer_cap))
    speed_max = node.vesc_speed_max()
    need_erpm = prof['speed'] * float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    if speed_max is not None and need_erpm > speed_max:
        raise SystemExit('Profile needs %.0f ERPM but vesc_driver speed_max is %.0f.' % (need_erpm, speed_max))

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = '%s_%s_v%.2f_erpmmax%s_%s' % (stamp, args.test, prof['speed'],
                                         int(speed_max) if speed_max is not None else 'NA',
                                         'armed' if real else 'disarmed')
    out = (Path(args.out_root).expanduser() if args.out_root else vp.evidence_root()) / name
    out.mkdir(parents=True, exist_ok=False)
    commit = subprocess.run(['git', '-C', str(vp.REPO), 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    info = dict(test=args.test, profile=prof, live=args.live, started=stamp, git_commit=commit,
                mux_status_at_start=st, mux_max_speed_mps=cap, mux_max_steer_rad=steer_cap,
                vesc_speed_max_erpm=speed_max, publish_hz=r['publish_hz'],
                ramp_accel_mps2=r['ramp_accel_mps2'], vehicle_params=p, bag_topics=BAG_TOPICS)
    (out / 'run_info.json').write_text(json.dumps(info, indent=2, default=str))
    (out / 'README.md').write_text(
        '# %s (%s)\n\n%s\n\n- Place: %s; target %.2f m/s, steer %.2f rad, steady %.1f s, %d repeats\n'
        '- Mux real actuation: %s; mux caps %.2f m/s / %.2f rad; vesc_driver speed_max: %s ERPM\n'
        '- Runner: %.0f Hz, ramp %.2f m/s^2; git %s\n\n## Operator notes\n\n(fill in)\n' % (
            args.test, stamp, prof['note'], prof['place'], prof['speed'], prof['steer'],
            prof['steady_s'], prof['repeats'], real, cap, steer_cap, speed_max,
            float(r['publish_hz']), float(r['ramp_accel_mps2']), commit))

    machine = Machine(prof, p, live=args.live, ground_ok=args.t1_passed_confirmed)
    kb = Keyboard(Deadman(float(r['deadman_first_press_s']), float(r['deadman_repeat_s'])))
    # Audit log only from the vesc_driver of THIS ROS graph (a process on another domain is not ours).
    in_graph = 'vesc_driver' in node.get_node_names()
    audit_pid, t_run0 = (vesc_driver_pid() if in_graph else None), time.time()
    bag_log = open(out / 'bag.log', 'w')
    # stdin DEVNULL: the bag must never share the terminal that carries the SPACE deadman.
    bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out / 'bag')] + BAG_TOPICS,
                           stdin=subprocess.DEVNULL, stdout=bag_log, stderr=subprocess.STDOUT,
                           start_new_session=True)

    def on_signal(signum, _frame):
        raise SystemExit('signal %d' % signum)
    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGHUP, on_signal)

    intervals = []
    half_width = float(vp.get(p, 'ground_check.front_window_deg'))
    try:
        # Zero commands while the bag starts, so the mux sees fresh inputs from the start.
        t_bag = time.monotonic() + 2.0
        while time.monotonic() < t_bag:
            node.emit(0.0, 0.0, 0)
            time.sleep(period)
        if bag.poll() is not None:
            raise SystemExit('ros2 bag record exited; see %s' % (out / 'bag.log'))
        say('\n%s -> %s' % (args.test, out))
        say('Keys: n = arm, hold SPACE = drive, s = skip, q = quit. ' + machine.message)
        last_msg = None
        graph = None
        tick = 0
        next_t = last_pub = time.monotonic()
        while machine.state != 'DONE':
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()  # overrun: resync; shows up in the interval log
            now = time.monotonic()
            if tick % 5 == 0:
                graph = node.graph_fault()
            tick += 1
            w = node.world(now, half_width)
            w['graph_fault'] = graph
            w['keyboard_lost'] = kb.eof
            speed, steer, dead, req = machine.step(now, kb.get_key(), kb.held(now), w)
            node.emit(speed, steer, dead)
            intervals.append((time.time(), now - last_pub, speed, dead))
            last_pub = now
            if req:
                node.request(req)
            if machine.reset_deadman:
                kb.reset_deadman()
                machine.reset_deadman = False
            if machine.message != last_msg:
                last_msg = machine.message
                say('[%s] %s | mux %s %s | ERPM %s' % (
                    machine.state, machine.message, (w['status'] or {}).get('mode'),
                    (w['status'] or {}).get('reason'), w.get('erpm')))
    finally:
        # Each cleanup step is guarded: a failure in one must not skip the zero or the bag stop.
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # a second Ctrl+C must not cut the zero short
        try:
            # Zero for at least min_post_stop_zero_s whatever happened above.
            t_zero = time.monotonic() + float(r['min_post_stop_zero_s'])
            while time.monotonic() < t_zero:
                node.emit(0.0, 0.0, 0)
                node.request('STOP')
                time.sleep(period)
        except Exception as exc:  # noqa: BLE001
            say('ZERO AFTER STOP FAILED: %s: use the physical cutoff' % exc)
        try:
            kb.restore()
        except termios.error:
            pass
        time.sleep(1.0)  # bag keeps the tail after the last zero
        rc = stop_process_group(bag)
        if bag.poll() is None:
            say('WARNING: ros2 bag record still running (pid %d)' % bag.pid)
        bag_log.close()
        try:
            audit = copy_vesc_audit(out, audit_pid, t_run0, time.time())
        except OSError as exc:
            audit = 'audit copy failed: %s' % exc
        try:
            write_outputs(out, machine, intervals, kb.log, node.status_log,
                          dict(vesc_speed_audit=audit, bag_returncode=rc))
        except Exception as exc:  # noqa: BLE001
            say('Writing runner outputs failed: %s' % exc)
        say('VESC audit: %s' % audit)
        say('Results: %s\nNext: python3 %s %s' % (
            out, Path(__file__).with_name('analyze.py'), out))


def write_outputs(out, machine, intervals, keys, status_log, extra):
    with open(out / 'runner_events.csv', 'w', newline='') as fp:
        wr = csv.DictWriter(fp, ['wall', 'mono', 'repeat', 'state', 'event', 'detail'])
        wr.writeheader()
        wr.writerows(machine.events)
    with open(out / 'publish_intervals.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['wall', 'dt_s', 'speed_cmd_mps', 'deadman'])
        wr.writerows(intervals)
    with open(out / 'keys.csv', 'w', newline='') as fp:
        wr = csv.writer(fp)
        wr.writerow(['wall', 'mono', 'key'])
        wr.writerows(keys)
    with open(out / 'status.jsonl', 'w') as fp:
        for wall, raw in status_log:
            fp.write(json.dumps({'wall': wall, 'status': json.loads(raw)}) + '\n')
    space = [k[1] for k in keys if k[2] == 'space']
    gaps = [b - a for a, b in zip(space, space[1:])]
    summary = dict(repeats=machine.repeats,
                   publish_intervals=interval_stats([i[1] for i in intervals[1:]]),
                   space_key_gaps=interval_stats(gaps), **extra)
    (out / 'runner_summary.json').write_text(json.dumps(summary, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--test', choices=sorted(PROFILES))
    ap.add_argument('--live', action='store_true',
                    help='Mux real actuation is enabled for an approved step. Never for dry_run.')
    ap.add_argument('--t1-passed-confirmed', action='store_true',
                    help='Ground profiles only, after Moatasem confirmed T1 in writing.')
    ap.add_argument('--params', help='vehicle_params.yaml (default config/real_car/)')
    ap.add_argument('--out-root', help='results root (default ~/f1tenth_hw_evidence/real_car_tests)')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--shutdown-probe', choices=['normal', 'systemexit', 'keyboardinterrupt'],
                    help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.shutdown_probe:
        shutdown_probe(args.shutdown_probe)
    elif args.self_test:
        self_test()
    elif args.list:
        for k, v in PROFILES.items():
            print('%-10s %s' % (k, v['note']))
    elif args.test:
        run(args)
    else:
        ap.error('--test, --list or --self-test')


if __name__ == '__main__':
    main()
