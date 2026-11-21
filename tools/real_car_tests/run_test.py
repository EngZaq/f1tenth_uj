#!/usr/bin/env python3
"""F1TENTH UJ real-car test runner. Drives ONLY through physical_drive_mux.

Publishes /manual/drive, /joy (button 5 = deadman) and /control/request_mode at
50 Hz: the same interfaces as ~/continuous_keyboard_mux.py. Never /drive, never
/commands/*. Records its own ros2 bag. Not an emergency stop: the battery
disconnect / kill switch stays in the operator's hand.

Keys over SSH (no Enter): n = arm, hold SPACE = drive, s = skip (idle only), q = quit,
m = placement readout (ground profiles: range ahead, arm window, brake point, side walls;
t3_circle: clear disk, clearance, 360 deg mux visibility, RF2O).
KILL: while arming, armed, driving or stopping, ANY key but SPACE = zero (converter brake), latched
until n. q there kills too, then the runner ends.
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
# steps: one speed per run (--speed); step N+1 only after step N passed (step_gate).
# converter: the node that must own /drive -> /commands/* for this profile.
PROFILES = {
    'dry_run': dict(
        place='stand', speed=0.30, steer=0.0, steady_s=2.0, repeats=2, post_zero_s=2.0,
        live_allowed=False, converter='ackermann_to_vesc_node',
        note='Disarmed end-to-end check: keyboard deadman, mux handshake, bag, analysis.'),
    't1_stop': dict(
        place='stand', steps=[0.5, 1.0, 1.5], steer=0.0, steady_s=3.0, repeats=5, extra_repeats=3,
        post_zero_s=4.0,
        live_allowed=True, converter='ackermann_to_vesc_node',
        note='T1a (F-01), wheels off the ground: ramp, 3 s steady, standard stop (speed 0). 5 repeats per step.'),
    't1b_stop': dict(
        place='stand', steps=[0.5, 1.0, 1.5], steer=0.0, steady_s=3.0, repeats=5, extra_repeats=3,
        post_zero_s=4.0,
        live_allowed=True, converter='t1b_stop_converter',
        note='T1b (F-01), wheels off the ground: as T1a, stop method from t1b_stop_converter.'),
    't2_servo_sweep': dict(
        kind='servo', live_allowed=True,
        note='T2 servo sweep, servo-only launch, motor phases disconnected (t2_servo_sweep.py).'),
    # T4 drives until the brake point (vehicle_params t4), not for a fixed time; steady_s is the
    # minimum (ground_check steady 1.0 m at 0.5 m/s). Only stops AT the brake point count.
    # Steps 1.0 / 1.5 are added only after 0.5 passed, with the measured decel (Moatasem).
    't4_straight': dict(
        place='ground', steps=[0.5], steer=0.0, steady_s=2.0, repeats=3, extra_repeats=2,
        post_zero_s=4.0, live_allowed=True, converter='t1b_stop_converter',
        brake_at_range=True, stop_method='brake5A', default_steer_cap=True,
        note='T4 ground, straight (steer 0) toward a wall, automatic brake 5 A at the brake point. 3 repeats.'),
    # T3 (21 Nov): constant steering (--steer, + = left) at 0.5 (or 0.3) m/s until the gyro heading change
    # reaches t3.target_turns, then brake 5 A. Needs the T3 converter gains, a VESC clip containing the
    # planned servo and a mux steer cap |steer|..0.30. No step gate (<= the T4-passed 0.5 m/s).
    't3_circle': dict(
        place='ground', steps=[0.3, 0.5], no_step_gate=True, steer=None, steady_s=2.0, repeats=1,
        extra_repeats=2, post_zero_s=4.0, live_allowed=True, converter='t1b_stop_converter',
        stop_method='brake5A', circle=True, count_reason='TURNS_DONE',
        note='T3 ground circles: constant steering, brake 5 A after target_turns of gyro heading. 1 counted run.'),
}
STEP_TOL = 1e-6
SPEED_MAX_ROUNDING = 5.0  # ERPM: yaml values are rounded (4630 for 4627.7)

# The bag is a subscriber: it must never touch the actuator chain, or the mux counts
# 2 subscribers and latches ACTUATOR_GRAPH_OWNERSHIP_FAULT (dry run 14 Nov). Command side
# comes from /control/status (mux output speed_mps / steer_rad), /manual/drive,
# /sensors/servo_position_command and the vesc_driver VESC_SPEED_AUDIT log (sent ERPM).
BAG_TOPICS = [
    '/scan', '/odom_rf2o', '/odom_vesc_observe', '/imu/data_raw', '/sensors/core',
    '/sensors/servo_position_command', '/control/status',
    '/manual/drive', '/joy', '/control/request_mode', '/tf', '/tf_static',
    '/t1b_stop_converter/status',
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
T1B_ONLY_TOPICS = ('/commands/motor/brake', '/commands/motor/current')  # t1b_stop_converter only


# ---------------------------------------------------------------- pure logic

# BMI160 (/imu/data_raw) watch, 20 Nov: after one I2C timeout ([Errno 110]) the stream kept its
# 100 Hz rate but repeated one sample until the node restarted (two sessions). Rate checks miss it.
IMU_WINDOW_S = 1.0        # look at the last second of samples
IMU_FROZEN_SHARE = 0.5    # frozen: more than this share of consecutive samples identical
IMU_STALE_S = 0.3         # stale: no sample for this long
IMU_MIN_SAMPLES = 50      # fewer in the window (100 Hz expected) -> IMU_STALE
# 20 Nov 21:35: the bmi160 node EXITS (exit code 0, launch says "finished cleanly") when its start-up
# gyro calibration sees motion (std > gyro_motion_std_limit_dps 0.5). 0 publishers = node gone.
# 10 s = 2 s warm-up + 5 s calibration (500 samples at 100 Hz) + margin.
IMU_NODE_DOWN = ('IMU_STALE(IMU node not running: restart the stack and keep the car still for 10 s)')


class ImuWatch:
    """Frozen / stale detection on (gyro xyz, accel xyz) tuples. status(now) -> None or reason."""

    def __init__(self):
        self.samples = []   # (t, tuple)

    def add(self, t, sample):
        self.samples.append((t, tuple(sample)))
        cut = t - IMU_WINDOW_S
        while self.samples and self.samples[0][0] < cut:
            self.samples.pop(0)

    def status(self, now, publishers=None):
        """publishers: count on /imu/data_raw (None = unknown). 0 -> the node is not running."""
        if publishers == 0:
            return IMU_NODE_DOWN
        if not self.samples or now - self.samples[-1][0] > IMU_STALE_S:
            return 'IMU_STALE(no /imu/data_raw for > %.1f s)' % IMU_STALE_S
        win = [v for t, v in self.samples if t >= now - IMU_WINDOW_S]
        if len(win) < IMU_MIN_SAMPLES:
            return 'IMU_STALE(%d samples in %.1f s)' % (len(win), IMU_WINDOW_S)
        same = sum(1 for a, b in zip(win, win[1:]) if a == b) / (len(win) - 1)
        if same > IMU_FROZEN_SHARE:
            return 'IMU_FROZEN(%.0f %% identical samples in %.1f s; restart the stack)' % (100 * same, IMU_WINDOW_S)
        return None


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


def window_ranges(scan, half_width_deg, center_deg=0.0):
    """[(angle_rad, range)] of finite returns within +/- half_width around center
    (REP-103: 0 = car front, +90 deg = left; LiDAR yaw is 0)."""
    if scan is None or not scan.angle_increment:
        return []
    n = len(scan.ranges)
    full = (scan.angle_max - scan.angle_min) > 6.2
    c, hw = math.radians(center_deg), math.radians(half_width_deg)
    lo = int(math.floor((c - hw - scan.angle_min) / scan.angle_increment))
    hi = int(math.ceil((c + hw - scan.angle_min) / scan.angle_increment))
    out = []
    for i in range(lo, hi + 1):
        if full:
            i %= n
        elif not 0 <= i < n:
            continue
        r = scan.ranges[i]
        if math.isfinite(r) and scan.range_min < r < scan.range_max:
            out.append((scan.angle_min + i * scan.angle_increment, r))
    return out


def front_range(scan, half_width_deg, center_deg=0.0):
    """Median finite range around center_deg (0 = car front). From reference script."""
    vals = [r for _, r in window_ranges(scan, half_width_deg, center_deg)]
    return statistics.median(vals) if vals else float('nan')


def sector_min(scan, half_width_deg):
    """Nearest finite return in +/- half_width around the front: the value the mux gate uses
    (physical_drive_mux.py front_evidence, MANUAL stop below 0.80 m)."""
    vals = [r for _, r in window_ranges(scan, half_width_deg)]
    return min(vals) if vals else float('nan')


# ---------------------------------------------------------------- T3 circle geometry (21 Nov)
# Circles that start at the same point and heading and turn to the same side are NESTED: a circle of
# radius r <= R_max (centre (0, side r) in the start base_link frame) lies inside the disk of radius R_max
# around (0, side R_max). So one clear disk around the largest plausible circle covers every real radius
# in [r_factor_min, r_factor_max] x L / tan(steer), and the same disk is the drive geofence.

def t3_geometry(steer, t3, wheelbase, body_length, body_width, front_x):
    """Clear-disk geometry in the start base_link frame (x forward, y left; + steer = left)."""
    side = 1.0 if steer > 0 else -1.0
    r_nom = wheelbase / math.tan(abs(steer))
    r_min, r_max = r_nom * float(t3['r_factor_min']), r_nom * float(t3['r_factor_max'])
    hw, rear_x = body_width / 2.0, front_x - body_length
    reach = max(math.hypot(r_max + hw, front_x), math.hypot(r_max + hw, rear_x))   # outer body corner
    clear_r = reach + float(t3['clear_margin_m'])
    return dict(side=side, r_nom=r_nom, r_min=r_min, r_max=r_max, center=(0.0, side * r_max), reach=reach,
                clear_r=clear_r, diameter_m=2.0 * clear_r, inner_free_r=max(0.0, r_min - hw))


def scan_points_base(scan, laser_x):
    """[(x, y, bearing_deg)] of finite returns in base_link (laser at (laser_x, 0), yaw 0)."""
    if scan is None or not scan.angle_increment:
        return []
    out = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min < r <= scan.range_max:
            a = scan.angle_min + i * scan.angle_increment
            out.append((laser_x + r * math.cos(a), r * math.sin(a), math.degrees(a)))
    return out


def own_body(x, y, body_length, body_width, front_x, pad=0.05):
    return front_x - body_length - pad <= x <= front_x + pad and abs(y) <= body_width / 2.0 + pad


def t3_clearance(scan, geom, laser_x, body_length, body_width, front_x):
    """Smallest (distance to the clear-disk centre - clear radius) over all returns (car body excluded):
    < 0 = something inside the space the car may sweep. -> dict(min_m, bearing_deg, n) or None (no scan)."""
    pts = scan_points_base(scan, laser_x)
    if not pts:
        return None
    cx, cy = geom['center']
    best = None
    for x, y, b in pts:
        if own_body(x, y, body_length, body_width, front_x):
            continue
        d = math.hypot(x - cx, y - cy) - geom['clear_r']
        if best is None or d < best[0]:
            best = (d, b)
    if best is None:
        return dict(min_m=float('inf'), bearing_deg=None, n=0)
    return dict(min_m=best[0], bearing_deg=best[1], n=len(pts))


def t3_visibility(scan, half_width_deg=20.0, step_deg=5.0):
    """Mux front-visibility prediction for every heading the car takes on a circle: the share of finite
    returns in +/- half_width around each heading (laser frame, as physical_drive_mux front_evidence).
    -> dict(min_fraction, heading_deg) or None."""
    if scan is None or not scan.angle_increment or len(scan.ranges) < 100:
        return None
    n = len(scan.ranges)
    ok = [1 if (math.isfinite(r) and scan.range_min < r <= scan.range_max) else 0 for r in scan.ranges]
    half = int(round(math.radians(half_width_deg) / scan.angle_increment))
    best = None
    h = -180.0
    while h < 180.0:
        c = int(round((math.radians(h) - scan.angle_min) / scan.angle_increment))
        idx = [(c + k) % n for k in range(-half, half + 1)]
        f = sum(ok[i] for i in idx) / len(idx)
        if best is None or f < best[0]:
            best = (f, h)
        h += step_deg
    return dict(min_fraction=best[0], heading_deg=best[1])


def t3_fence(start, geom, pose):
    """Distance (m) of the base_link pose beyond the largest circle (> 0 = outside), all in the RF2O odom
    frame; start = (x, y, yaw) at the drive start."""
    x0, y0, h0 = start
    cx = x0 - geom['center'][1] * math.sin(h0)
    cy = y0 + geom['center'][1] * math.cos(h0)
    return math.hypot(pose[0] - cx, pose[1] - cy) - geom['r_max']


def interval_stats(dts):
    if not dts:
        return {'count': 0}
    s = sorted(dts)

    def q(f):
        return s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))]
    return {'count': len(s), 'mean_s': round(sum(s) / len(s), 5), 'p50_s': round(q(.5), 5),
            'p95_s': round(q(.95), 5), 'p99_s': round(q(.99), 5), 'max_s': round(s[-1], 5),
            'over_0.10_s': sum(d > .10 for d in s), 'over_0.20_s': sum(d > .20 for d in s)}


KILL_BANNER = ('*** KILL *** zero sent (converter brakes) - LATCHED: no driving until you release SPACE '
               'and press n')

REBOUND_FAIL_ERPM = 300.0   # T1 pass: no rebound > 300 ERPM
UNDER_ERPM = 100.0          # "stopped"
DECEL_FRACTION = 0.5        # a rebound counts only after |ERPM| fell to this share of its value at the zero


class ReboundTracker:
    """F-01 rebound after a zero command, the same definition live (runner) and offline (analyze):
    the largest rise of |ERPM| above its running minimum, counted only once |ERPM| has fallen to
    DECEL_FRACTION of its value at the zero (or below UNDER_ERPM). A wobble at speed before the
    stop acts is not a rebound (T1b brake 5 A stop 5: live said peak 4128, the ERPM at the zero)."""

    def __init__(self, erpm_at_zero=None):
        self.start = None if erpm_at_zero is None else abs(erpm_at_zero)
        self.decelerating = False
        self.min = self.min_t = self.rise_t = self.from_t = None
        self.rise = self.peak = 0.0

    def add(self, t, erpm):
        a = abs(erpm)
        if self.start is None:
            self.start = a
        if not self.decelerating:
            if a > max(DECEL_FRACTION * self.start, UNDER_ERPM):
                return
            self.decelerating = True
        if self.min is None or a < self.min:
            self.min, self.min_t = a, t
        if a - self.min > self.rise:
            self.rise, self.peak, self.rise_t, self.from_t = a - self.min, a, t, self.min_t


def step_mismatch(speed, mux_cap, speed_max, gain, margin):
    """A stepped run must use exactly this step's caps: mux max_speed_mps == step speed and
    vesc_driver speed_max within [step ERPM, step ERPM x margin]. Returns a reason or None."""
    if mux_cap is None or abs(mux_cap - speed) > STEP_TOL:
        return 'mux max_speed_mps=%s, step needs %.2f (launch max_speed_mps:=%.2f)' % (mux_cap, speed, speed)
    need = speed * gain
    if speed_max is None:
        return 'vesc_driver speed_max unreadable'
    if not need <= speed_max <= need * margin + SPEED_MAX_ROUNDING:
        return 'vesc_driver speed_max=%.0f, step %.2f m/s needs %.0f..%.0f (wrong vesc_yaml)' % (
            speed_max, speed, need, need * margin + SPEED_MAX_ROUNDING)
    return None


def step_gate(root, test, method, speed, steps, repeats):
    """Step N+1 only after step N passed: an analysed, armed run of the same test and stop
    method at the previous speed with >= `repeats` measured stops, none rebounding
    > 300 ERPM, VESC fault 0. Returns (ok, reason)."""
    idx = [i for i, s in enumerate(steps) if abs(s - speed) <= STEP_TOL]
    if not idx:
        return False, 'speed %.2f is not a step of %s %s' % (speed, test, steps)
    if idx[0] == 0:
        return True, 'first step'
    prev = steps[idx[0] - 1]
    seen = []
    for info_path in sorted(Path(root).expanduser().glob('*_%s_v*/run_info.json' % test)):
        folder = info_path.parent
        try:
            info = json.loads(info_path.read_text())
        except ValueError:
            continue
        if (not info.get('live') or info.get('stop_method') != method
                or abs(float(info['profile'].get('speed', -1)) - prev) > STEP_TOL):
            continue
        summary = folder / 'summary.json'
        if not summary.exists():
            seen.append('%s not analysed (run analyze.py)' % folder.name)
            continue
        s = json.loads(summary.read_text())
        stops = s.get('stops', {})
        items = [x for x in stops.get('items', []) if x.get('samples')]
        # Only stops from >= 80 % of the step ERPM count (analyze.py reached_steady); a rebound
        # on ANY stop fails the run.
        steady = [x for x in items if x.get('reached_steady')]
        rebounds = [x for x in items if not x.get('pass_no_rebound_over_300')]
        ok = (len(steady) >= repeats and not rebounds
              and s.get('vesc', {}).get('faults') == [0]
              and not str(stops.get('source', '')).startswith('runner_events'))
        if ok:
            return True, 'step %.2f m/s passed in %s' % (prev, folder.name)
        seen.append('%s: %d steady stops (need %d), %d rebound > 300, faults %s' % (
            folder.name, len(steady), repeats, len(rebounds), s.get('vesc', {}).get('faults')))
    return False, 'step %.2f m/s (%s) has no passed run: %s' % (prev, method, '; '.join(seen) or 'none found')


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
        t4 = vp.get(p, 't4')
        self.gate, self.guard = float(t4['mux_gate_m']), float(t4['gate_guard_m'])
        self.reaction, self.slack = float(t4['reaction_s']), float(t4['time_slack_s'])
        self.max_approach = float(t4['max_approach_m'])
        self.ground = prof['place'] == 'ground'
        # circle (T3): constant steering, stop after target_turns of gyro heading; clear disk + geofence.
        self.circle = bool(prof.get('circle'))
        if self.circle:
            self.t3 = vp.get(p, 't3')
            veh = vp.get(p, 'vehicle')
            self.wheelbase = float(veh['wheelbase_m'])
            self.geom = t3_geometry(float(prof['steer']), self.t3, self.wheelbase, float(veh['body_length_m']),
                                    float(veh['body_width_m']), float(veh['front_bumper_x_m']))
            self.target_rad = 2 * math.pi * float(self.t3['target_turns'])
            self.min_visible = 0.75 + float(self.t3['visibility_margin'])   # updated from the mux status
        self.ground_ok = ground_ok
        # brake_at_range (T4): drive until the brake point; the time limit is set per repeat.
        self.brake_at_range = bool(prof.get('brake_at_range'))
        # Only stops with this reason count (None = any stop from steady speed).
        self.count_prefix = 'BRAKE_POINT' if self.brake_at_range else prof.get('count_reason')
        v = float(prof['speed'])
        self.ramp_s = v / self.ramp
        self.steady_s = self.steady_m / v if self.ground else float(prof['steady_s'])
        self.hard_s = self.ramp_s + self.steady_s + 0.5
        self.state = 'IDLE'
        self.repeat = 0
        # Stepped profiles: a repeat that ended before steady speed does not count; up to
        # extra_repeats more are offered (the step gate itself still needs 5 steady stops).
        self.steady_count = 0
        self.max_repeats = prof['repeats'] + prof.get('extra_repeats', 0)
        self.stop_steady = False
        self.quit = False
        self.killed = False      # KILL latch: cleared only by a successful n (arm request)
        self.events = []
        self.repeats = []
        self.reset_deadman = False
        self.last_erpm = None
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

    def brake_range(self, v):
        """LiDAR range (from the laser) at which the runner sends zero at speed v: never later
        than the stop rule (bumper + reaction travel + v^2/2a + margin) and always before the
        mux gate, so the mux 0.80 m gate stays the backstop, not the stop."""
        rule = self.bumper + v * self.reaction + v * v / (2 * self.decel) + self.margin
        return max(self.gate + self.guard, rule)

    @staticmethod
    def ahead(w):
        """Range the arm window and the brake trigger use: the front median (+/-3 deg) only, so a
        side object never brakes early (20 Nov, narrow floor). A side object closer than 0.80 m in
        the mux +/-20 deg sector makes the MUX latch STOP -> zero -> converter brake 5 A (backstop).
        NaN if the front window has no return."""
        return w.get('front_m', float('nan'))

    def ground_window(self):
        """(need, max) LiDAR range ahead for arming. need = ramp + steady + brake point, and
        never less than the baseline rule; max only for brake_at_range profiles."""
        v = float(self.prof['speed'])
        need = max(required_length(v, self.ramp, self.decel, self.steady_m, self.margin, self.bumper),
                   v * v / (2 * self.ramp) + self.steady_m + self.brake_range(v))
        top = self.brake_range(v) + self.max_approach if self.brake_at_range else float('inf')
        return need, top

    def ground_fit(self, w):
        need, top = self.ground_window()
        have = self.ahead(w)
        return (math.isfinite(have) and need <= have <= top), need, have

    def circle_text(self, w):
        g, c, v = self.geom, w.get('t3_clear'), w.get('t3_vis')
        return ('circle %s steer %+.2f rad: R nominal %.2f m (assumed %.2f..%.2f), clear disk %.2f m diameter, centre '
                '%.2f m to the %s | clearance %s | mux visibility min %s (needs >= %.2f) | RF2O %s' % (
                    'LEFT' if g['side'] > 0 else 'RIGHT', self.prof['steer'], g['r_nom'], g['r_min'], g['r_max'],
                    g['diameter_m'], g['r_max'], 'left' if g['side'] > 0 else 'right',
                    '-' if c is None else '%+.2f m (worst at %s deg)' % (c['min_m'], '-' if c['bearing_deg'] is None
                                                                       else '%.0f' % c['bearing_deg']),
                    '-' if v is None else '%.2f at heading %.0f deg' % (v['min_fraction'], v['heading_deg']),
                    self.min_visible, 'OK' if w.get('rf2o_age', 1e9) <= float(self.t3['rf2o_stale_s']) else 'STALE'))

    def circle_refusal(self, w):
        mux_min = (w.get('status') or {}).get('manual_min_front_valid_fraction')
        if mux_min is not None:
            self.min_visible = float(mux_min) + float(self.t3['visibility_margin'])
        c, v = w.get('t3_clear'), w.get('t3_vis')
        if c is None or v is None:
            return 'CIRCLE_NO_SCAN'
        if c['min_m'] < 0:
            return 'CIRCLE_NOT_CLEAR(return %.2f m inside the %.2f m clear disk at %.0f deg)' % (
                -c['min_m'], self.geom['diameter_m'], c['bearing_deg'])
        if v['min_fraction'] < self.min_visible:
            return 'CIRCLE_LOW_VISIBILITY(%.2f < %.2f at heading %.0f deg: the mux would stop there)' % (
                v['min_fraction'], self.min_visible, v['heading_deg'])
        if w.get('rf2o_age', 1e9) > float(self.t3['rf2o_stale_s']) or w.get('rf2o_pose') is None:
            return 'CIRCLE_RF2O_STALE(geofence source)'
        if w.get('gz') is None:
            return 'CIRCLE_NO_GYRO'
        return None

    def placement(self, w):
        if self.circle:
            return self.circle_text(w)
        need, top = self.ground_window()
        return ('front %.2f m | arm window %.2f..%.2f m | brake point %.2f m | mux nearest +/-20deg %.2f m '
                '(arm needs >= %.2f, mux brakes < %.2f) | side L %.2f R %.2f m' % (
                    self.ahead(w), need, top, self.brake_range(float(self.prof['speed'])),
                    w.get('sector_min_m', float('nan')), self.gate + self.guard, self.gate,
                    w.get('left_m', float('nan')), w.get('right_m', float('nan'))))

    def circle_step(self, now, elapsed, v, w):
        """T3 while driving: integrate the gyro heading (base yaw rate = -(gz - bias)) and the RF2O yaw;
        stop reason or None. The mux front gate stays the obstacle backstop."""
        t3 = self.t3
        dt, self.last_t = now - self.last_t, now
        if w.get('imu_problem') or w.get('gz') is None:
            return 'CIRCLE_IMU_PROBLEM(%s)' % (w.get('imu_problem') or 'no gyro')
        wz = -(w['gz'] - self.bias)
        self.turned += wz * dt
        self.wz_hist = [(t, x) for t, x in self.wz_hist if now - t <= 0.5] + [(now, wz)]
        pose = w.get('rf2o_pose')
        if pose is None or self.start_pose is None or w.get('rf2o_age', 1e9) > float(t3['rf2o_stale_s']):
            return 'CIRCLE_RF2O_STALE(%.2f s)' % w.get('rf2o_age', float('inf'))
        d = pose[2] - self.rf_last_yaw
        self.turned_rf += (d + math.pi) % (2 * math.pi) - math.pi
        self.rf_last_yaw = pose[2]
        out = t3_fence(self.start_pose, self.geom, pose)
        if out > float(t3['geofence_margin_m']):
            return 'GEOFENCE(%.2f m outside the largest circle)' % out
        side = self.geom['side']
        if elapsed >= float(t3['turn_check_s']) and v > 0.1:
            expect = v * math.tan(abs(float(self.prof['steer']))) / self.wheelbase
            mean = sum(x for _, x in self.wz_hist) / len(self.wz_hist)
            if side * mean < float(t3['turn_check_fraction']) * expect:
                return 'TURN_CHECK_FAILED(yaw rate %+.3f rad/s, expected %+.3f)' % (mean, side * expect)
        if side * self.turned >= self.target_rad:
            return 'TURNS_DONE(gyro %.0f deg, RF2O %.0f deg)' % (math.degrees(self.turned), math.degrees(self.turned_rf))
        if side * self.turned_rf >= self.target_rad + 0.5:
            return 'TURNS_RF2O_BACKUP(gyro %.0f deg, RF2O %.0f deg)' % (math.degrees(self.turned),
                                                                       math.degrees(self.turned_rf))
        return None

    def finished(self):
        if 'extra_repeats' not in self.prof:
            return self.quit or self.repeat >= self.prof['repeats']
        return (self.quit or self.steady_count >= self.prof['repeats']
                or self.repeat >= self.max_repeats)

    def stop(self, now, reason):
        self.stop_steady = (self.state == 'DRIVING' and getattr(self, 'steady_logged', False)
                            and not reason.startswith('KILL')
                            and (self.count_prefix is None or reason.startswith(self.count_prefix)))
        self.state = 'STOPPING'
        self.t_stop = now
        self.tracker = ReboundTracker(self.last_erpm)  # ERPM at the zero = the latest telemetry
        self.log(now, 'ZERO_COMMAND', reason)
        if self.repeats and self.repeats[-1]['repeat'] == self.repeat and 'zero_wall' not in self.repeats[-1]:
            self.repeats[-1].update(zero_wall=time.time(), zero_reason=reason)
        return 0.0, 0.0, 0, 'STOP'

    def kill(self, now, key):
        """Software kill: any key but SPACE while arming, armed, driving or stopping. Same zero
        path as a SPACE release (runner zero -> mux STOP -> converter brake), latched until n.
        Not faster than SPACE release and not an emergency stop."""
        self.killed = True
        out, was = None, self.state
        if self.state in ('READY', 'DRIVING'):
            out = self.stop(now, 'KILL(key %r)' % key)
        elif self.state == 'ARMING':
            self.state = 'IDLE'
            out = (0.0, 0.0, 0, 'STOP')
        self.log(now, 'KILL', 'key %r in %s' % (key, was))
        self.message = KILL_BANNER
        return out or (0.0, 0.0, 0, None)

    def step(self, now, key, held, w):
        st = w.get('status') or {}
        mode = st.get('mode')
        if w.get('erpm') is not None and self.state != 'STOPPING':
            self.last_erpm = w['erpm']
        if key == 'q' or w.get('keyboard_lost'):
            self.quit = True  # lost SSH terminal: finish with zero, never wait in IDLE
        if key is not None and self.state in ('ARMING', 'READY', 'DRIVING'):
            return self.kill(now, key)
        if key is not None and self.state == 'STOPPING':
            self.kill(now, key)  # already sending zero; latch and show it, keep the stop running
            key = None
        if self.state == 'IDLE':
            if self.quit:
                self.state = 'DONE'
                return 0.0, 0.0, 0, 'STOP'
            if key == 's':
                self.log(now, 'SKIP')
                self.repeat += 1
                if self.finished():
                    self.state = 'DONE'
            elif key == 'm' and self.ground:
                self.log(now, 'PLACEMENT', self.placement(w))
            elif key == 'n':
                reason = self.fault(w)
                if reason is None and held:
                    reason = 'RELEASE_SPACE_FIRST'
                if reason is None and w.get('imu_problem'):
                    reason = w['imu_problem']   # arm only; a freeze while driving is logged, not a stop
                if reason is None and self.ground:
                    if not self.ground_ok:
                        reason = 'GROUND_NEEDS_T1_PASS_CONFIRMED'
                    elif w.get('erpm') is not None and abs(w['erpm']) >= UNDER_ERPM:
                        reason = 'CAR_MOVING(%.0f ERPM)' % w['erpm']
                    elif w.get('sector_min_m', float('inf')) < self.gate + self.guard:
                        # The mux would latch STOP on the first tick: refuse with the reason instead.
                        reason = 'MUX_SECTOR_OBJECT(nearest +/-20deg %.2f m < %.2f m)' % (
                            w['sector_min_m'], self.gate + self.guard)
                    elif self.circle:
                        reason = self.circle_refusal(w)
                        self.log(now, 'CIRCLE_FIT', self.circle_text(w))
                    else:
                        ok, need, have = self.ground_fit(w)
                        top = self.ground_window()[1]
                        self.log(now, 'SPEED_FIT', 'need=%.2f m have=%.2f m max=%.2f m' % (need, have, top))
                        if not ok:
                            reason = 'SPEED_DOES_NOT_FIT(need %.2f..%.2f m, have %.2f m)' % (need, top, have)
                if reason:
                    self.log(now, 'ARM_REFUSED', reason)
                else:
                    self.state = 'ARMING'
                    self.t_arm = self.t_req = now
                    self.killed = False  # n is the only way out of a KILL latch
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
            if reason or self.quit or mode != 'MANUAL_READY':
                return self.stop(now, reason or ('QUIT' if self.quit else 'MUX_LEFT_MANUAL_MODE(%s)' % mode))
            if held:
                self.state = 'DRIVING'
                self.t_drive = now
                self.steady_logged = False
                self.repeats.append(dict(repeat=self.repeat, target_mps=self.prof['speed'],
                                         steer_rad=self.prof['steer'], start_wall=time.time()))
                detail = 'target=%.2f m/s' % self.prof['speed']
                if self.brake_at_range:
                    # Time limit = expected time to the brake point + slack. A slower car (gain
                    # off, wheel slip) or a lost front return stops early, never late.
                    v = float(self.prof['speed'])
                    ramp_m = v * v / (2 * self.ramp)
                    d0 = self.ahead(w)
                    approach = max(0.0, (d0 if math.isfinite(d0) else 0.0) - self.brake_range(v) - ramp_m)
                    self.hard_s = self.ramp_s + approach / v + self.slack
                    self.repeats[-1].update(ahead_at_start_m=d0, brake_point_m=self.brake_range(v))
                    detail += ' ahead=%.2f m brake_point=%.2f m limit=%.1f s' % (
                        d0, self.brake_range(v), self.hard_s)
                if self.circle:
                    v = float(self.prof['speed'])
                    self.bias = w.get('gz_mean_1s') or 0.0
                    self.turned = self.turned_rf = 0.0
                    pose = w.get('rf2o_pose')
                    self.last_t = now
                    self.rf_last_yaw = pose[2] if pose else None
                    self.start_pose = tuple(pose) if pose else None
                    self.wz_hist = []
                    self.hard_s = self.ramp_s + self.target_rad * self.geom['r_max'] / v + float(self.t3['time_slack_s'])
                    self.repeats[-1].update(gyro_bias=self.bias, gyro_std_1s=w.get('gz_std_1s'),
                                            start_pose=self.start_pose, geometry=self.geom)
                    detail += ' steer=%+.2f rad R=%.2f m target %.2f turns limit=%.1f s gyro bias %.5f (std %s)' % (
                        self.prof['steer'], self.geom['r_nom'], float(self.t3['target_turns']), self.hard_s, self.bias,
                        '-' if w.get('gz_std_1s') is None else '%.4f' % w['gz_std_1s'])
                self.log(now, 'DRIVE_START', detail)
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
                reason = 'QUIT'  # lost terminal (keys are KILL, handled above)
            if reason is None and not self.brake_at_range and not self.circle and elapsed >= self.ramp_s + self.steady_s:
                reason = 'PROFILE_DONE'
            if reason is None and elapsed >= self.hard_s:
                reason = 'HARD_TIME_LIMIT'
            if reason is None and self.circle:
                reason = self.circle_step(now, elapsed, v, w)
            if reason is None and self.ground and not self.circle:
                have = self.ahead(w)
                if not math.isfinite(have):
                    reason = 'FRONT_RANGE_LOST'
                elif have <= self.brake_range(v):
                    reason = 'BRAKE_POINT(ahead %.2f m <= %.2f m)' % (have, self.brake_range(v))
            if reason:
                return self.stop(now, reason)
            if not self.steady_logged and v >= float(self.prof['speed']):
                self.steady_logged = True
                self.log(now, 'STEADY_REACHED')
            return v, float(self.prof['steer']), 1, None
        if self.state == 'STOPPING':
            # Live F-01 readout from /sensors/core (same definition as analyze.py, which uses the
            # converter's own stop time and gives the recorded numbers).
            tr = self.tracker
            if w.get('erpm') is not None:
                tr.add(now - self.t_stop, w['erpm'])
            if now - self.t_stop < self.post_zero:
                return 0.0, 0.0, 0, None
            if tr.start is None:
                detail = 'no ERPM telemetry'
            elif not tr.decelerating:
                detail = 'ERPM never fell below half of %.0f: NOT STOPPED, STEP FAILS' % tr.start
            else:
                detail = 'live rebound +%.0f ERPM (peak %.0f at +%.2f s)%s' % (
                    tr.rise, tr.peak, tr.rise_t or 0.0,
                    ' > 300: STEP FAILS' if tr.rise > REBOUND_FAIL_ERPM else '')
            if self.repeats and self.repeats[-1]['repeat'] == self.repeat:
                self.repeats[-1]['live_rebound_rise_erpm'] = round(tr.rise, 1)
            if self.stop_steady:
                self.steady_count += 1
            elif 'extra_repeats' in self.prof:
                detail += (' | not stopped at the brake point: NOT counted' if self.brake_at_range
                           else ' | turns not completed: NOT counted' if self.circle
                           else ' | ended before steady speed: NOT counted')
            self.log(now, 'REPEAT_END', detail)
            self.repeat += 1
            self.reset_deadman = True
            self.state = 'DONE' if self.finished() else 'IDLE'
            if self.state == 'IDLE' and 'extra_repeats' in self.prof:
                self.message = '%s | %s stops %d/%d, repeat %d of max %d: release SPACE, press n.' % (
                    detail, 'brake-point' if self.brake_at_range else 'full-turn' if self.circle else 'steady',
                    self.steady_count,
                    self.prof['repeats'], self.repeat + 1, self.max_repeats)
                if self.brake_at_range or self.circle:
                    self.message += ' Carry the car back to the start marks, hands off ~2 s, m to check.'
            elif self.state == 'IDLE':
                self.message = '%s | Repeat %d/%d: release SPACE, press n to arm.' % (
                    detail, self.repeat + 1, self.prof['repeats'])
            if self.killed:
                self.message = '%s | %s' % (KILL_BANNER, self.message)
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
    # 0.55 m/s: brake point max(mux 0.80 + 0.10, 0.09 + 0.11 + 0.15 + 0.5) = 0.90 m LiDAR
    assert g.step(1.4, None, True, world(front_m=0.6, scan_age=0.05, st=dict(mode='MANUAL_ACTIVE')))[0] == 0.0
    assert g.events[-1]['detail'].startswith('BRAKE_POINT')
    # ---- T4 (t4_straight): brake point, arm window, placement, counting, time limit
    tp = resolve_profile('t4_straight', 0.5)
    for bad in (None, 1.0):
        try:
            resolve_profile('t4_straight', bad)
        except SystemExit:
            continue
        raise AssertionError('t4 resolved %s' % bad)
    t4 = Machine(tp, p, live=False, ground_ok=True)
    assert abs(t4.brake_range(0.5) - 0.90) < 1e-9, t4.brake_range(0.5)       # mux gate + guard rules
    assert t4.brake_range(0.5) > t4.gate                                   # runner always before the mux
    rule_only = t4.bumper + 1.5 * t4.reaction + 1.5 ** 2 / (2 * t4.decel) + t4.margin
    assert abs(t4.brake_range(1.5) - rule_only) < 1e-9 and rule_only > 0.9  # faster: stop rule rules
    need, top = t4.ground_window()
    assert abs(need - 2.15) < 1e-9 and abs(top - 3.90) < 1e-9, (need, top)
    assert need >= required_length(0.5, t4.ramp, t4.decel, t4.steady_m, t4.margin, t4.bumper)

    def gw(front, sector=None, mode='STOP', **kw):
        st = dict(mode=mode, last_mode_request='MANUAL')
        return world(front_m=front, sector_min_m=front if sector is None else sector, scan_age=0.05,
                     left_m=1.0, right_m=1.2, st=st, **kw)
    t4.step(0.0, 'm', False, gw(2.6))
    assert t4.state == 'IDLE' and t4.events[-1]['event'] == 'PLACEMENT' and 'arm window 2.15..3.90' in \
        t4.events[-1]['detail'], t4.events[-1]
    for front, sector, why in ((2.0, None, 'SPEED_DOES_NOT_FIT'), (4.2, None, 'SPEED_DOES_NOT_FIT'),
                               (2.8, 0.85, 'MUX_SECTOR_OBJECT'),    # mux would latch on the first tick
                               (float('nan'), None, 'SPEED_DOES_NOT_FIT')):
        t4.step(0.0, 'n', False, gw(front, sector))
        assert t4.state == 'IDLE' and why in t4.events[-1]['detail'], t4.events[-1]
    t4.step(0.0, 'n', False, gw(2.8, erpm=400.0))
    assert t4.state == 'IDLE' and t4.events[-1]['detail'].startswith('CAR_MOVING')

    def t4_repeat(t, d0, release_at=None, front_at=None):
        """One repeat from d0 at the commanded speed; front_at(elapsed) overrides the range."""
        t4.step(t, 'n', False, gw(d0))
        assert t4.state == 'ARMING', t4.events[-1]
        t4.step(t + 0.05, None, False, gw(d0, mode='MANUAL_READY'))
        t4.step(t + 0.1, None, True, gw(d0, mode='MANUAL_READY'))
        assert t4.state == 'DRIVING'
        k, d = 0.0, d0
        while t4.state == 'DRIVING' and k < 30.0:
            k += 0.02
            d -= ramp_speed(0.5, k, t4.ramp) * 0.02
            held = release_at is None or k < release_at
            r = front_at(k) if front_at else d
            t4.step(t + 0.1 + k, None, held, gw(r, mode='MANUAL_ACTIVE'))
        reason = [e for e in t4.events if e['event'] == 'ZERO_COMMAND'][-1]['detail']
        t4.step(t + k + 0.5, None, False, gw(d))
        t4.step(t + k + 10.0, None, False, gw(d))                            # post-zero over
        return t + k + 11.0, k, reason
    t, k, why = t4_repeat(0.0, 2.8)
    assert why.startswith('BRAKE_POINT') and t4.steady_count == 1, why
    # expected 1.0 s ramp (0.25 m) + (2.8 - 0.9 - 0.25) / 0.5 = 4.3 s; no PROFILE_DONE at ramp + steady (3.0 s)
    assert 4.2 < k < 4.4 and abs(t4.hard_s - 5.3) < 1e-9, (k, t4.hard_s)
    assert t4.repeats[-1]['brake_point_m'] == 0.9 and t4.state == 'IDLE'
    t, k, why = t4_repeat(t, 2.8, release_at=2.0)                           # operator released SPACE
    assert why == 'DEADMAN_RELEASED' and t4.steady_count == 1 and 'brake point: NOT counted' in t4.message
    t, k, why = t4_repeat(t, 2.8, front_at=lambda e: 2.8)                    # never closer (no motion)
    assert why == 'HARD_TIME_LIMIT' and abs(k - (1.0 + 1.65 / 0.5 + 1.0)) < 0.05 and t4.steady_count == 1, k
    t, k, why = t4_repeat(t, 2.8, front_at=lambda e: 2.0 if e < 1.0 else float('nan'))
    assert why == 'FRONT_RANGE_LOST' and t4.steady_count == 1
    assert t4.state == 'IDLE' and t4.repeat == 4                            # 3 + 2 extra: one attempt left
    t, k, why = t4_repeat(t, 3.0)
    assert t4.state == 'DONE' and t4.steady_count == 2                      # cap reached at 5 attempts
    # Narrow floor: a side object in the mux sector (1.2 m, later 0.5 m) never brakes the runner;
    # only the front median does. At < 0.80 m the MUX latches STOP and the runner follows it.
    sd = Machine(tp, p, live=False, ground_ok=True)
    sd.step(0.0, 'n', False, gw(2.8, 1.2))
    assert sd.state == 'ARMING', sd.events[-1]
    sd.step(0.05, None, False, gw(2.8, 1.2, mode='MANUAL_READY'))
    sd.step(0.1, None, True, gw(2.8, 1.2, mode='MANUAL_READY'))
    assert sd.step(1.5, None, True, gw(2.3, 0.5, mode='MANUAL_ACTIVE'))[0] > 0 and sd.state == 'DRIVING'
    assert sd.step(1.55, None, True, gw(2.3, 0.5, mode='STOP'))[0] == 0.0
    assert sd.events[-1]['detail'].startswith('MUX_LEFT_MANUAL_MODE') and sd.steady_count == 0

    # ---- T3 (t3_circle): steer validation, geometry, clearance, visibility, geofence, drive stops
    for bad_steer in (None, 0.04, 0.31, float('nan')):
        try:
            resolve_profile('t3_circle', 0.5, bad_steer, p)
        except SystemExit:
            continue
        raise AssertionError('t3 resolved steer %s' % bad_steer)
    try:
        resolve_profile('t4_straight', 0.5, 0.1, p)
        raise AssertionError('t4 accepted --steer')
    except SystemExit:
        pass
    t3p = vp.get(p, 't3')
    veh = vp.get(p, 'vehicle')
    L = float(veh['wheelbase_m'])
    body = (float(veh['body_length_m']), float(veh['body_width_m']), float(veh['front_bumper_x_m']))
    import random as _rnd
    rnd = _rnd.Random(3)
    for st_ in (0.26, -0.26, 0.15):
        g3 = t3_geometry(st_, t3p, L, *body)
        assert abs(g3['r_nom'] - L / math.tan(abs(st_))) < 1e-12 and g3['center'][1] * st_ > 0
        for _ in range(200):           # nested: every tangent circle with r <= r_max stays in the disk
            r_ = rnd.uniform(g3['r_min'], g3['r_max'])
            a_ = rnd.uniform(0, 2 * math.pi)
            px, py = r_ * math.sin(a_), g3['side'] * r_ * (1 - math.cos(a_))
            assert math.hypot(px - g3['center'][0], py - g3['center'][1]) <= g3['r_max'] + 1e-9
    g26 = t3_geometry(0.26, t3p, L, *body)
    assert 3.2 < g26['diameter_m'] < 3.4, g26                       # hall: ~3.3 m for +/-0.26 rad

    class Scan3:
        angle_min, angle_max = -math.pi, math.pi
        angle_increment = 2 * math.pi / 1440
        range_min, range_max = 0.0, 100.0

        def __init__(self, r=6.0):
            self.ranges = [r] * 1440

        def put(self, bearing_deg, r, width_deg=2.0):
            for i in range(1440):
                a = math.degrees(self.angle_min + i * self.angle_increment)
                if abs((a - bearing_deg + 180) % 360 - 180) <= width_deg / 2:
                    self.ranges[i] = r
            return self
    lx = float(vp.get(p, 'lidar.x_m'))
    assert t3_clearance(Scan3(), g26, lx, *body)['min_m'] > 1.0           # 6 m hall walls: clear
    box_left = Scan3().put(90.0, 1.0)
    assert t3_clearance(box_left, g26, lx, *body)['min_m'] < 0             # inside the LEFT circle's disk
    assert t3_clearance(box_left, t3_geometry(-0.26, t3p, L, *body), lx, *body)['min_m'] > 0   # not the right one
    assert t3_clearance(Scan3().put(180.0, 0.05), g26, lx, *body)['min_m'] > 1.0   # own body (behind laser) ignored
    assert t3_visibility(Scan3())['min_fraction'] == 1.0
    gap = Scan3().put(90.0, float('inf'), 60.0)
    v3 = t3_visibility(gap)
    assert v3['min_fraction'] < 0.05 and abs(v3['heading_deg'] - 90) <= 10, v3
    st0 = (1.0, 2.0, math.pi / 2)                      # facing +y; left circle centre at (1 - r_max, 2)
    assert abs(t3_fence(st0, g26, (1.0 - 2 * g26['r_max'], 2.0, 0)) - 0.0) < 1e-9
    assert t3_fence(st0, g26, (1.0, 2.0, 0)) == 0.0 - 0.0 and t3_fence(st0, g26, (1.5, 2.0, 0)) > 0.4

    def c3world(pose, gz, mode='STOP', clear=2.0, vis=1.0, rf_age=0.05, sector=3.0, **kw):
        st = dict(mode=mode, last_mode_request='MANUAL', manual_min_front_valid_fraction=0.75)
        return world(front_m=float('nan'), sector_min_m=sector, scan_age=0.05, left_m=2.0, right_m=2.0, st=st,
                     gz=gz, gz_mean_1s=0.003, gz_std_1s=0.0005, rf2o_pose=pose, rf2o_age=rf_age,
                     t3_clear=dict(min_m=clear, bearing_deg=45.0, n=1440),
                     t3_vis=dict(min_fraction=vis, heading_deg=10.0), **kw)

    def circle_run(steer, r_true_factor=1.0, turn_sign=1.0, fail_at=None, fail_kw=None, limit=80.0):
        """Kinematic car on a circle (r_true = factor x nominal, turning turn_sign x commanded side);
        raw gz = -(yaw rate) + bias (IMU z down). Returns (machine, zero reason, elapsed)."""
        prof3 = resolve_profile('t3_circle', 0.5, steer, p)
        m3 = Machine(prof3, p, live=False, ground_ok=True)
        x, y, yaw, bias = 0.0, 0.0, 0.0, 0.003
        m3.step(0.0, 'n', False, c3world((x, y, yaw), bias))
        assert m3.state == 'ARMING', m3.events[-1]
        m3.step(0.05, None, False, c3world((x, y, yaw), bias, mode='MANUAL_READY'))
        m3.step(0.1, None, True, c3world((x, y, yaw), bias, mode='MANUAL_READY'))
        assert m3.state == 'DRIVING', m3.events[-1]
        k = 0.0
        r_true = r_true_factor * m3.geom['r_nom']
        side = (1.0 if steer > 0 else -1.0) * turn_sign
        while m3.state == 'DRIVING' and k < limit:
            k += 0.02
            v = ramp_speed(0.5, k, m3.ramp)
            wz = 0.0 if r_true_factor == 0 else side * v / r_true
            yaw += wz * 0.02
            x += v * math.cos(yaw) * 0.02
            y += v * math.sin(yaw) * 0.02
            kw = dict(mode='MANUAL_ACTIVE')
            if fail_at is not None and k >= fail_at:
                kw.update(fail_kw or {})
            out3 = m3.step(0.1 + k, None, True, c3world((x, y, yaw), -wz + bias, **kw))
            if m3.state == 'DRIVING':
                assert out3[1] == steer and out3[2] == 1
        zero = [e for e in m3.events if e['event'] == 'ZERO_COMMAND'][-1]['detail']
        m3.step(0.1 + k + 0.5, None, False, c3world((x, y, yaw), bias))
        m3.step(0.1 + k + 10.0, None, False, c3world((x, y, yaw), bias))
        return m3, zero, k
    m3, why, k = circle_run(0.26)
    assert why.startswith('TURNS_DONE') and m3.steady_count == 1 and m3.state == 'DONE', (why, m3.state)
    expect_k = 1.0 + (2 * 2 * math.pi * m3.geom['r_nom'] - 0.25) / 0.5   # ramp 1 s (0.25 m) + 2 turns at 0.5
    assert abs(k - expect_k) < 0.1, (k, expect_k)
    assert m3.hard_s > k and abs(m3.repeats[-1]['gyro_bias'] - 0.003) < 1e-12
    m3, why, k = circle_run(-0.26)
    assert why.startswith('TURNS_DONE') and m3.steady_count == 1                       # right circle
    m3, why, k = circle_run(0.26, turn_sign=-1.0)                                      # turns the wrong way
    # wrong way: the circle on the other side leaves the disk first (GEOFENCE ~1.5 s), the turn check by 2 s
    assert why.startswith(('GEOFENCE', 'TURN_CHECK_FAILED')) and k < 2.1 and m3.steady_count == 0, (why, k)
    m3, why, k = circle_run(0.26, r_true_factor=0.0)                                   # drives straight
    assert why.startswith(('GEOFENCE', 'TURN_CHECK_FAILED')) and k < 2.1 and m3.steady_count == 0, (why, k)
    m3, why, k = circle_run(0.26, r_true_factor=1.8)                                   # far larger than R_max
    assert why.startswith('GEOFENCE') and m3.steady_count == 0, why
    m3, why, k = circle_run(0.26, fail_at=5.0, fail_kw=dict(rf_age=0.5))
    assert why.startswith('CIRCLE_RF2O_STALE') and 5.0 <= k < 5.05, (why, k)
    m3, why, k = circle_run(0.26, fail_at=5.0, fail_kw=dict(imu_problem='IMU_FROZEN(test)'))
    assert why.startswith('CIRCLE_IMU_PROBLEM') and m3.steady_count == 0, why
    m3, why, k = circle_run(0.26, fail_at=3.0, fail_kw=dict(mode='STOP'))
    assert why.startswith('MUX_LEFT_MANUAL_MODE') and m3.steady_count == 0, why       # mux gate = backstop
    pr3 = resolve_profile('t3_circle', 0.5, 0.26, p)
    for kw, why_ in ((dict(clear=-0.2), 'CIRCLE_NOT_CLEAR'), (dict(vis=0.78), 'CIRCLE_LOW_VISIBILITY'),
                     (dict(rf_age=1.0), 'CIRCLE_RF2O_STALE'), (dict(gz=None), 'CIRCLE_NO_GYRO')):
        a3 = Machine(pr3, p, live=False, ground_ok=True)
        gz_ = kw.pop('gz', 0.003)
        a3.step(0.0, 'n', False, c3world((0, 0, 0), gz_, **kw))
        assert a3.state == 'IDLE' and a3.events[-1]['detail'].startswith(why_), (why_, a3.events[-1])
    a3 = Machine(pr3, p, live=False, ground_ok=True)
    a3.step(0.0, 'm', False, c3world((0, 0, 0), 0.003))
    assert 'clear disk 3.31 m diameter' in a3.events[-1]['detail'], a3.events[-1]['detail']
    a3.step(0.0, 'n', False, c3world((0, 0, 0), 0.003, sector=0.85))   # the T4 mux-sector rule still applies
    assert a3.events[-1]['detail'].startswith('MUX_SECTOR_OBJECT'), a3.events[-1]
    t3c = dict(servo_gain_left=-0.874, servo_gain_right=-1.062, servo_offset=0.335)
    wide = dict(servo_min=0.10, servo_max=0.68)
    ci = t3_precheck(pr3, p, 0.30, t3c, 0.335, wide)
    assert abs(ci['servo_planned'] - 0.10776) < 1e-9 and ci['geometry']['side'] == 1.0
    assert abs(t3_precheck(resolve_profile('t3_circle', 0.5, -0.26, p), p, 0.30, t3c, 0.335, wide)['servo_planned']
               - 0.61112) < 1e-9
    for args_, why_ in (((0.40, t3c, wide), 'max_steer_rad'),                                  # cap too high
                        ((0.30, dict(t3c, servo_gain_left=-0.5), wide), 'T3 converter gains'),  # baseline gains
                        ((0.30, dict(servo_offset=0.335), wide), 'T3 converter gains'),         # old converter
                        ((0.30, t3c, dict(servo_min=0.27, servo_max=0.37)), 'vesc_driver clip'),  # T4 yaml
                        ((0.30, t3c, {}), 'vesc_driver clip')):                                 # unreachable
        try:
            t3_precheck(pr3, p, args_[0], args_[1], 0.335, args_[2])
            raise AssertionError('t3_precheck accepted %s' % why_)
        except SystemExit as e:
            assert why_ in str(e), (why_, str(e))
    try:
        t3_precheck(resolve_profile('t3_circle', 0.5, 0.30, p), p, 0.30, t3c, 0.335, wide)  # needs 0.073 < 0.10
        raise AssertionError('servo below the converter clip accepted')
    except SystemExit as e:
        assert 'converter clip' in str(e)
    print('PASS: T3 circle: --steer limits, nested clear disk, clearance (box, own body), visibility gap,')
    print('      geofence; drives: 2 gyro turns counted (L/R, bias removed), wrong direction / straight ->')
    print('      GEOFENCE or TURN_CHECK_FAILED by 2.1 s, too large -> GEOFENCE, RF2O stale, IMU fault, mux STOP')
    print('      backstop; arm refusals (not clear, low visibility, RF2O stale, no gyro, mux sector object);')
    print('      pre-run refusals (steer cap, converter gains, converter clip, VESC clip / T4 yaml).')

    # ---- KILL: any key but SPACE while arming / armed / driving / stopping -> zero, latched until n
    def armed(m, front=2.8):
        m.step(0.0, 'n', False, gw(front))
        m.step(0.05, None, False, gw(front, mode='MANUAL_READY'))
        assert m.state == 'READY', m.events[-1]
    for key in ('x', 's', 'm', 'q', '\r'):
        kd = Machine(tp, p, live=False, ground_ok=True)
        armed(kd)
        kd.step(0.1, None, True, gw(2.8, mode='MANUAL_READY'))
        assert kd.step(2.0, None, True, gw(2.2, mode='MANUAL_ACTIVE'))[0] > 0  # steady speed
        out = kd.step(2.02, key, True, gw(2.2, mode='MANUAL_ACTIVE'))           # SPACE still held
        assert out == (0.0, 0.0, 0, 'STOP') and kd.state == 'STOPPING' and kd.killed, (key, out)
        assert kd.message.startswith(KILL_BANNER) and kd.events[-1]['event'] == 'KILL'
        zero = [e for e in kd.events if e['event'] == 'ZERO_COMMAND'][-1]['detail']
        assert zero.startswith('KILL') and not kd.stop_steady, zero
        assert kd.step(2.5, None, True, gw(2.2))[:3] == (0.0, 0.0, 0)          # held SPACE: still zero
        kd.step(9.0, None, True, gw(2.2))
        assert kd.steady_count == 0 and kd.message.startswith(KILL_BANNER)
        if key == 'q':
            assert kd.state == 'DONE'                                           # q: kill, then end
            continue
        assert kd.state == 'IDLE' and kd.killed
        assert kd.step(9.1, None, True, gw(2.8))[:3] == (0.0, 0.0, 0) and kd.state == 'IDLE'  # SPACE alone: nothing
        kd.step(9.2, 'n', False, gw(2.8))
        assert kd.state == 'ARMING' and not kd.killed                           # only n clears the latch
    kd = Machine(tp, p, live=False, ground_ok=True)
    armed(kd)
    assert kd.step(0.1, 'k', False, gw(2.8, mode='MANUAL_READY')) == (0.0, 0.0, 0, 'STOP')   # READY
    assert kd.state == 'STOPPING' and kd.killed
    kd = Machine(tp, p, live=False, ground_ok=True)
    kd.step(0.0, 'n', False, gw(2.8))
    assert kd.step(0.02, 'k', False, gw(2.8)) == (0.0, 0.0, 0, 'STOP') and kd.state == 'IDLE'  # ARMING
    assert kd.killed and kd.events[-1]['detail'] == "key 'k' in ARMING"
    kd = Machine(tp, p, live=False, ground_ok=True)
    armed(kd)
    kd.step(0.1, None, True, gw(2.8, mode='MANUAL_READY'))
    kd.step(0.5, None, False, gw(2.7, mode='MANUAL_ACTIVE'))                    # SPACE release: brake
    assert kd.state == 'STOPPING' and not kd.killed
    assert kd.step(0.6, 'z', False, gw(2.7))[:3] == (0.0, 0.0, 0) and kd.state == 'STOPPING'   # STOPPING
    assert kd.killed and kd.message.startswith(KILL_BANNER)
    # T1 profile: a KILL from steady speed is not a counted stop
    k1 = Machine(dict(PROFILES['t1b_stop'], speed=0.5), p, live=False)
    k1.step(0.0, 'n', False, world())
    k1.step(0.05, None, False, world(st=dict(mode='MANUAL_READY', last_mode_request='MANUAL')))
    k1.step(0.1, None, True, world(st=dict(mode='MANUAL_READY', last_mode_request='MANUAL')))
    k1.step(2.0, None, True, world(st=dict(mode='MANUAL_ACTIVE')))
    k1.step(2.02, 'x', True, world(st=dict(mode='MANUAL_ACTIVE')))
    k1.step(9.0, None, False, world())
    assert k1.steady_count == 0 and k1.state == 'IDLE' and k1.killed
    # sector_min = the mux's nearest in +/-20 deg; side ranges at +/-90 deg (REP-103: +90 = left)
    s = Scan()
    s.ranges = [5.0] * 1440
    s.ranges[720 + 60] = 1.0                                                 # +15 deg
    s.ranges[720 + 100] = 0.5                                                # +25 deg: outside the sector
    s.ranges[720 + 360] = 0.7                                                # +90 deg (left)
    assert front_range(s, 3.0) == 5.0 and sector_min(s, 20.0) == 1.0
    assert front_range(s, 3.0, 90.0) == 5.0 and min(r for _, r in window_ranges(s, 3.0, 90.0)) == 0.7
    s.ranges[720 - 360 - 2: 720 - 360 + 3] = [0.8] * 5                      # -90 deg (right)
    assert front_range(s, 0.6, -90.0) == 0.8
    assert math.isnan(sector_min(None, 20.0))
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
    # Stepped profiles: --speed must be a step; caps and vesc yaml must match the step exactly.
    assert resolve_profile('t1_stop', 0.5)['speed'] == 0.5
    for test, speed in (('t1_stop', None), ('t1_stop', 0.7), ('dry_run', 0.5)):
        try:
            resolve_profile(test, speed)
        except SystemExit:
            continue
        raise AssertionError('resolved %s %s' % (test, speed))
    assert step_mismatch(0.5, 0.5, 4630.0, 8414.0, 1.10) is None
    assert step_mismatch(0.5, 1.5, 4630.0, 8414.0, 1.10)            # mux cap above the step
    assert step_mismatch(0.5, 0.5, 13883.0, 8414.0, 1.10)           # 1.5 m/s yaml at step 0.5
    assert step_mismatch(0.5, 0.5, 3500.0, 8414.0, 1.10)            # yaml below the step
    assert step_mismatch(1.0, 1.0, 9255.0, 8414.0, 1.10) is None
    assert step_mismatch(1.5, 1.5, 13883.0, 8414.0, 1.10) is None
    import tempfile
    with tempfile.TemporaryDirectory() as root:
        steps = [0.5, 1.0, 1.5]
        assert step_gate(root, 't1_stop', 'speed0', 0.5, steps, 5)[0]
        assert not step_gate(root, 't1_stop', 'speed0', 1.0, steps, 5)[0]

        def fake(name, speed, live, method, n_pass, n_items, faults=(0,), analysed=True, n_steady=None):
            d = Path(root) / name
            d.mkdir()
            (d / 'run_info.json').write_text(json.dumps(dict(
                live=live, stop_method=method, profile=dict(speed=speed))))
            if analysed:
                steady = n_items if n_steady is None else n_steady
                items = [dict(samples=50, pass_no_rebound_over_300=k < n_pass, reached_steady=k < steady)
                         for k in range(n_items)]
                (d / 'summary.json').write_text(json.dumps(dict(
                    stops=dict(source='vesc_speed_audit.log (sent_erpm)', items=items),
                    vesc=dict(faults=list(faults)))))
        fake('1_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 4, 5)       # 1 rebound
        fake('2_t1_stop_v0.50_speed0_erpmmax4630_disarmed', 0.5, False, 'speed0', 5, 5)   # not live
        fake('3_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 5, 5, analysed=False)
        fake('4_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 3, 3)       # too few
        fake('5_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 5, 5, faults=(0, 4))
        ok, why = step_gate(root, 't1_stop', 'speed0', 1.0, steps, 5)
        assert not ok and 'not analysed' in why and '1 rebound' in why, why
        fake('4b_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 5, 5, n_steady=3)  # short
        ok, why = step_gate(root, 't1_stop', 'speed0', 1.0, steps, 5)
        assert not ok and '3 steady stops' in why, why
        fake('6_t1b_stop_v0.50_brake5A_erpmmax4630_armed', 0.5, True, 'brake5A', 5, 5)    # other test
        assert not step_gate(root, 't1_stop', 'speed0', 1.0, steps, 5)[0]
        fake('7_t1_stop_v0.50_speed0_erpmmax4630_armed', 0.5, True, 'speed0', 5, 5)
        assert step_gate(root, 't1_stop', 'speed0', 1.0, steps, 5)[0]
        assert not step_gate(root, 't1_stop', 'speed0', 1.5, steps, 5)[0]                 # 1.0 not passed
        assert not step_gate(root, 't1_stop', 'brake5A', 1.0, steps, 5)[0]                # per method
    # Live rebound readout in STOPPING (12 Nov shape: dip then rise)
    m = Machine(dict(PROFILES['t1_stop'], speed=0.5), p, live=True)
    live = dict(st=dict(real_actuation_enabled=True))
    m.step(0.0, 'n', False, world(telemetry_age=0.02, **live))
    m.step(0.1, None, False, world(telemetry_age=0.02, st=dict(real_actuation_enabled=True, mode='MANUAL_READY',
                                                                last_mode_request='MANUAL')))
    m.step(0.2, None, True, world(telemetry_age=0.02, st=dict(real_actuation_enabled=True, mode='MANUAL_READY')))
    m.step(0.3, None, False, world(telemetry_age=0.02, erpm=4200.0, st=dict(real_actuation_enabled=True,
                                                                          mode='MANUAL_ACTIVE')))
    assert m.state == 'STOPPING'
    for t, e in ((0.4, 2000.0), (0.5, 150.0), (0.7, 3911.0), (1.0, 0.0)):
        m.step(t, None, False, world(telemetry_age=0.02, erpm=e, **live))
    m.step(5.0, None, False, world(telemetry_age=0.02, erpm=0.0, **live))
    assert m.repeats[-1]['live_rebound_rise_erpm'] == 3761.0 and 'STEP FAILS' in m.events[-1]['detail']
    # Stepped profile: a repeat released during the ramp is not counted; DONE after 5 steady stops
    m = Machine(dict(PROFILES['t1b_stop'], speed=1.0), p, live=True)
    t = 0.0

    def one_repeat(t, hold_s):
        rdy = dict(real_actuation_enabled=True, mode='MANUAL_READY', last_mode_request='MANUAL')
        m.step(t, 'n', False, world(telemetry_age=0.02, st=dict(real_actuation_enabled=True)))
        m.step(t + 0.05, None, False, world(telemetry_age=0.02, st=rdy))
        m.step(t + 0.1, None, True, world(telemetry_age=0.02, st=rdy))     # mux sees the deadman next tick
        k = 0.15
        while k < hold_s:
            m.step(t + k, None, True, world(telemetry_age=0.02, erpm=8000.0, st=dict(rdy, mode='MANUAL_ACTIVE')))
            k += 0.05
            if m.state != 'DRIVING':
                break
        m.step(t + k, None, False, world(telemetry_age=0.02, erpm=8000.0, st=dict(rdy, mode='MANUAL_ACTIVE')))
        m.step(t + k + 0.1, None, False, world(telemetry_age=0.02, erpm=0.0, st=dict(rdy, mode='STOP')))
        m.step(t + k + 10.0, None, False, world(telemetry_age=0.02, erpm=0.0, st=dict(rdy, mode='STOP')))
        return t + k + 11.0
    t = one_repeat(t, 1.0)                                                # released in the ramp
    assert m.steady_count == 0 and m.state == 'IDLE' and 'NOT counted' in m.events[-1]['detail']
    for _ in range(4):
        t = one_repeat(t, 9.0)                                            # PROFILE_DONE each
    assert m.steady_count == 4 and m.state == 'IDLE'
    t = one_repeat(t, 9.0)
    assert m.steady_count == 5 and m.state == 'DONE' and m.repeat == 6, (m.steady_count, m.state, m.repeat)
    m = Machine(dict(PROFILES['t1b_stop'], speed=1.0), p, live=True)
    t = 0.0
    for _ in range(8):
        t = one_repeat(t, 1.0)
    assert m.state == 'DONE' and m.steady_count == 0 and m.repeat == 8     # cap: 5 + 3 extra
    # T1b brake 5 A stop 5: a wobble at speed before the brake acts (3930 -> 4128) is not a rebound
    tr = ReboundTracker(3930.0)
    for t, e in ((0.02, 4128.0), (0.04, 3000.0), (0.06, 1500.0), (0.08, 150.0), (0.10, 83.0), (0.3, 0.0)):
        tr.add(t, e)
    assert tr.decelerating and tr.rise == 0.0 and tr.peak == 0.0, vars(tr)
    tr = ReboundTracker(4200.0)
    for t in range(10):
        tr.add(t * 0.1, 4150.0 + 30 * (t % 2))                       # never slows down
    assert not tr.decelerating
    # IMU watch (20 Nov freeze) and arm refusal
    iw = ImuWatch()
    assert iw.status(0.0).startswith('IMU_STALE')
    for i in range(100):
        iw.add(i * 0.01, (0.001 * (i % 7), 0.0, 0.002 * (i % 5), 0.1, 0.2, 9.8 + 0.01 * (i % 3)))
    assert iw.status(0.995) is None, iw.status(0.995)
    for i in range(100, 200):
        iw.add(i * 0.01, (0.003, -0.001, 0.004, 0.12, 0.21, 9.79))
    assert iw.status(1.995).startswith('IMU_FROZEN'), iw.status(1.995)
    assert iw.status(2.5).startswith('IMU_STALE')                          # no sample for 0.5 s
    sparse = ImuWatch()
    for i in range(20):
        sparse.add(i * 0.05, (float(i), 0, 0, 0, 0, 0))
    assert sparse.status(0.96).startswith('IMU_STALE(20 samples')
    assert iw.status(1.995, publishers=0) == IMU_NODE_DOWN                  # node exited (calibration)
    assert iw.status(1.995, publishers=1).startswith('IMU_FROZEN')         # node up: data rule as before
    assert ImuWatch().status(0.0, publishers=1).startswith('IMU_STALE(no /imu')
    mi = Machine(PROFILES['dry_run'], p, live=False)
    mi.step(0.0, 'n', False, world(imu_problem='IMU_FROZEN(100 % identical samples in 1.0 s; restart the stack)'))
    assert mi.state == 'IDLE' and mi.events[-1]['event'] == 'ARM_REFUSED' \
        and mi.events[-1]['detail'].startswith('IMU_FROZEN'), mi.events[-1]
    assert mi.step(0.1, 'n', False, world())[3] == 'MANUAL' and mi.state == 'ARMING'   # healthy -> arms
    print('PASS: IMU watch: healthy / frozen / stale / too few samples / node not running; arm refused on IMU_FROZEN.')
    print('PASS: deadman timing, ramp, speed-fit length, front range, config = vehicle_params,')
    print('      arm handshake, zero on stale status / deadman release / mux leaving MANUAL,')
    print('      refusals (real flag, extra publisher, SPACE held, ground gates), wall stop,')
    print('      no protected topic in the bag list, audit-log parsing, step resolution,')
    print('      step/yaml/cap match, step gate (rebound, unanalysed, too few, not steady, fault, method), live rebound,')
    print('      unsteady repeats not counted (extra repeats, capped).')
    print('PASS: T4 brake point 0.90 m (> mux gate 0.80 m), arm window 2.15..3.90 m (front median), object')
    print('      < 0.90 m in the mux sector / car moving refused, placement readout, drive to the brake point,')
    print('      side object in the sector never brakes the runner (mux does), only brake-point stops count,')
    print('      time limit, front range lost, sector min / side ranges.')
    print('PASS: KILL: any key but SPACE (x s m q Enter) while driving -> zero at once even with SPACE held,')
    print('      banner, latched (SPACE alone does nothing) until n; also in READY, ARMING, STOPPING; q ends;')
    print('      KILL stops never counted (T4 and T1).')
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


def get_node_params(node, node_name, names):
    """{name: value} read from a running node (bool/int/double/string); {} if unreachable.
    Needs `node` spinning in another thread (RosSession)."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/%s/get_parameters' % node_name)
    try:
        if not cli.wait_for_service(timeout_sec=2.0):
            return {}
        fut = cli.call_async(GetParameters.Request(names=list(names)))
        t_end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            return {}
        return {name: {1: v.bool_value, 2: v.integer_value, 3: v.double_value,
                       4: v.string_value}.get(v.type)
                for name, v in zip(names, fut.result().values)}
    finally:
        node.destroy_client(cli)


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

def resolve_profile(test, speed, steer=None, p=None):
    """Profile with its speed fixed: stepped profiles need --speed equal to one step; circle profiles
    need --steer (+ = left) within t3.steer_min_rad..steer_max_rad."""
    prof = dict(PROFILES[test])
    if prof.get('circle'):
        t3 = vp.get(p or vp.load(), 't3')
        lo, hi = float(t3['steer_min_rad']), float(t3['steer_max_rad'])
        if steer is None or not (math.isfinite(steer) and lo - 1e-9 <= abs(steer) <= hi + 1e-9):
            raise SystemExit('%s needs --steer with |steer| in %.2f..%.2f rad (+ = left)' % (test, lo, hi))
        prof['steer'] = float(steer)
    elif steer is not None:
        raise SystemExit('%s has a fixed steering angle; do not pass --steer' % test)
    if 'steps' in prof:
        if speed is None or not any(abs(speed - s) <= STEP_TOL for s in prof['steps']):
            raise SystemExit('%s needs --speed, one of %s' % (test, prof['steps']))
        prof['speed'] = float(speed)
    elif speed is not None:
        raise SystemExit('%s has a fixed speed; do not pass --speed' % test)
    return prof


def run(args):
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import Imu, Joy, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    try:
        from vesc_msgs.msg import VescStateStamped
    except ImportError:
        VescStateStamped = None

    p = vp.load(args.params)
    prof = resolve_profile(args.test, args.speed, args.steer, p)
    if args.live and not prof['live_allowed']:
        raise SystemExit('profile %s never runs with real actuation' % args.test)
    if prof['place'] == 'ground' and not args.t1_passed_confirmed:
        raise SystemExit('ground profile: needs --t1-passed-confirmed (Moatasem written T1 pass)')
    mism = vp.config_mismatches(p)
    if mism:
        raise SystemExit('config/real_car disagrees with vehicle_params.yaml:\n  ' + '\n  '.join(mism))
    r = vp.get(p, 'runner')
    mux_sector = float(vp.get(p, 't4.mux_sector_deg'))
    laser_x = float(vp.get(p, 'lidar.x_m'))
    veh = vp.get(p, 'vehicle')
    body = (float(veh['body_length_m']), float(veh['body_width_m']), float(veh['front_bumper_x_m']))
    t3_geom = (t3_geometry(prof['steer'], vp.get(p, 't3'), float(veh['wheelbase_m']), *body)
               if prof.get('circle') else None)

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
            self.imu_watch = ImuWatch()
            self.gz, self.gz_hist = None, []           # T3: latest raw gz and the last 1 s
            self.rf2o = self.rf2o_at = None
            self.t3_cache, self.t3_cache_at, self.t3_scan = (None, None), 0.0, None
            self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
            if prof.get('circle'):
                self.create_subscription(Odometry, '/odom_rf2o', self.on_rf2o, qos_profile_sensor_data)
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

        def on_imu(self, msg):
            a, g = msg.linear_acceleration, msg.angular_velocity
            now = time.monotonic()
            with self.lock:
                self.imu_watch.add(now, (g.x, g.y, g.z, a.x, a.y, a.z))
                self.gz = g.z
                self.gz_hist.append((now, g.z))
                while self.gz_hist and now - self.gz_hist[0][0] > 1.0:
                    self.gz_hist.pop(0)

        def on_rf2o(self, msg):
            q = msg.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            with self.lock:
                self.rf2o = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
                self.rf2o_at = time.monotonic()

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
                pubs = [i.node_name for i in self.get_publishers_info_by_topic(t)]
                # T1b only: the t1b converter, alone, may publish brake and current.
                allowed = t in T1B_ONLY_TOPICS and prof['converter'] == 't1b_stop_converter'
                if pubs and not (allowed and pubs == ['t1b_stop_converter']):
                    return 'EXTRA_PUBLISHER(%s=%s)' % (t, pubs)
            return None

        def world(self, now, half_width, imu_publishers=None):
            with self.lock:
                w = dict(status=self.status,
                         status_age=(now - self.status_at) if self.status_at else 1e9,
                         telemetry_age=(now - self.tele_at) if self.tele_at else 1e9,
                         vesc_fault=(self.tele or {}).get('fault'),
                         scan_age=(now - self.scan_at) if self.scan_at else 1e9,
                         front_m=front_range(self.scan, half_width))
                if prof['place'] == 'ground':
                    w.update(sector_min_m=sector_min(self.scan, mux_sector),
                             left_m=front_range(self.scan, half_width, 90.0),
                             right_m=front_range(self.scan, half_width, -90.0))
                w['erpm'] = (self.tele or {}).get('erpm')
                w['imu_problem'] = self.imu_watch.status(now, imu_publishers)
                if prof.get('circle'):
                    gz = [x for _, x in self.gz_hist]
                    w.update(gz=self.gz, gz_mean_1s=statistics.mean(gz) if gz else None,
                             gz_std_1s=statistics.pstdev(gz) if len(gz) > 1 else None,
                             rf2o_pose=self.rf2o, rf2o_age=(now - self.rf2o_at) if self.rf2o_at else 1e9)
                    scan = self.scan
            if prof.get('circle'):
                # Clearance + visibility: ~1440 points, at most 5 Hz and only for a new scan (cached).
                if scan is not None and scan is not self.t3_scan and now - self.t3_cache_at >= 0.2:
                    self.t3_cache = (t3_clearance(scan, t3_geom, laser_x, *body), t3_visibility(scan))
                    self.t3_cache_at, self.t3_scan = now, scan
                w['t3_clear'], w['t3_vis'] = self.t3_cache
            return w

        def get_params(self, node_name, names):
            return get_node_params(self, node_name, names)

        def vesc_speed_max(self):
            return self.get_params('vesc_driver', ['speed_max']).get('speed_max')

    if any(is_protected(t) for t in BAG_TOPICS):
        raise SystemExit('BAG_TOPICS contains an actuator-chain topic: refusing.')
    # Every exit below (return, refusal, SystemExit, Ctrl+C) leaves through RosSession,
    # which stops the spin thread before rclpy shuts down.
    with RosSession(RunnerNode) as node:
        drive_session(args, p, prof, r, node)


def t3_precheck(prof, p, steer_cap, converter_params, base_off, clip):
    """T3 refusals before anything is armed (SystemExit with the fix): mux steer cap <= t3.steer_max_rad,
    converter per-side gains = the T3 values, planned servo inside the converter clip (not clamped) and
    inside the vesc_driver servo clip. -> circle info for run_info.json."""
    import t1b_stop_converter as conv
    t3, veh = vp.get(p, 't3'), vp.get(p, 'vehicle')
    steer = float(prof['steer'])
    if steer_cap > float(t3['steer_max_rad']) + 1e-9:
        raise SystemExit('t3_circle: mux max_steer_rad %.2f > t3.steer_max_rad %.2f (launch max_steer_rad:=0.30)'
                         % (steer_cap, float(t3['steer_max_rad'])))
    gl, gr = converter_params.get('servo_gain_left'), converter_params.get('servo_gain_right')
    if (gl is None or gr is None or abs(float(gl) - float(t3['servo_gain_left'])) > 1e-6
            or abs(float(gr) - float(t3['servo_gain_right'])) > 1e-6):
        raise SystemExit('t3_circle needs the T3 converter gains %.3f / %.3f, the converter has %s / %s '
                         '(launch converter_params_file:=/home/ftenth/f1tenth_uj/config/real_car/'
                         't1b_converter_t3_t2gain.yaml)' % (float(t3['servo_gain_left']),
                                                            float(t3['servo_gain_right']), gl, gr))
    off = float(converter_params.get('servo_offset') or base_off)
    gside = float(gl) if steer > 0 else float(gr)
    servo = conv.servo_command(off, float(gl), float(gr), steer)
    if abs(servo - (off + gside * steer)) > 1e-9:
        raise SystemExit('t3_circle: steer %+.2f rad needs servo %.3f, outside the converter clip %s: '
                         'choose a smaller --steer' % (steer, off + gside * steer, conv.SERVO_DRIVE_CLIP))
    if (len(clip) < 2 or None in clip.values()
            or not (float(clip['servo_min']) - 1e-9 <= servo <= float(clip['servo_max']) + 1e-9)):
        raise SystemExit('t3_circle: planned servo %.3f outside the vesc_driver clip %s (launch '
                         'vesc_yaml:=.../vesc_t3_erpm<N>.yaml)' % (servo, clip))
    return dict(steer_rad=steer, servo_planned=servo, servo_offset=off, servo_gain_side=gside,
                vesc_servo_clip=clip, t3=t3,
                geometry=t3_geometry(steer, t3, float(veh['wheelbase_m']), float(veh['body_length_m']),
                                     float(veh['body_width_m']), float(veh['front_bumper_x_m'])))


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
    gain = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    need_erpm = prof['speed'] * gain
    if speed_max is not None and need_erpm > speed_max:
        raise SystemExit('Profile needs %.0f ERPM but vesc_driver speed_max is %.0f.' % (need_erpm, speed_max))

    # Exactly the converter this profile expects owns /drive -> /commands/* (never both).
    names = set(node.get_node_names())
    other = {'ackermann_to_vesc_node', 't1b_stop_converter'} - {prof['converter']}
    if prof['converter'] not in names or names & other:
        raise SystemExit('%s needs converter %s and not %s; graph has %s (launch converter:=...)' % (
            args.test, prof['converter'], sorted(other), sorted(names & (other | {prof['converter']}))))
    converter_params = {}
    base_off = float(vp.get(p, 'steering.servo_offset'))
    method = 'speed0'
    if prof['converter'] == 't1b_stop_converter':
        converter_params = node.get_params('t1b_stop_converter',
                                           ['method', 'brake_current_a', 'brake_hold_s', 'servo_offset',
                                            'servo_gain_left', 'servo_gain_right'])
        base_off = float(vp.get(p, 'steering.servo_offset'))
        if converter_params.get('servo_offset') is not None and \
                abs(float(converter_params['servo_offset']) - base_off) > 1e-9:
            print('TEST servo_offset %.3f in the converter (baseline %.3f): recorded in run_info.json.'
                  % (float(converter_params['servo_offset']), base_off))
        method = converter_params.get('method')
        if method == 'brake':
            method = 'brake%gA' % converter_params.get('brake_current_a')
        if not method:
            raise SystemExit('t1b_stop_converter parameters unreadable: %s' % converter_params)
    if prof.get('stop_method') and method != prof['stop_method']:
        raise SystemExit('%s needs stop method %s, the converter runs %s (launch converter:=t1b '
                         'stop_method:=brake brake_current_a:=5.0)' % (args.test, prof['stop_method'], method))
    default_steer = float(vp.get(p, 'caps.mux_default_max_steer_rad'))
    if prof.get('default_steer_cap') and abs(steer_cap - default_steer) > 1e-9:
        raise SystemExit('%s keeps the mux steer cap at its default %.2f rad (mux has %.2f): '
                         'do not pass max_steer_rad' % (args.test, default_steer, steer_cap))
    circle_info = None
    if prof.get('circle'):
        circle_info = t3_precheck(prof, p, steer_cap, converter_params, base_off,
                                  node.get_params('vesc_driver', ['servo_min', 'servo_max']))
        g = circle_info['geometry']
        say('T3 circle %s %+.2f rad -> servo %.3f; R nominal %.2f m; clear disk %.2f m diameter, centre %.2f m '
            'to the %s of the start' % ('LEFT' if prof['steer'] > 0 else 'RIGHT', prof['steer'],
                                        circle_info['servo_planned'], g['r_nom'], g['diameter_m'], g['r_max'],
                                        'left' if prof['steer'] > 0 else 'right'))
    gate = 'not a stepped profile'
    if 'steps' in prof:
        bad = step_mismatch(prof['speed'], cap, speed_max, gain, float(vp.get(p, 'caps.vesc_speed_max_margin')))
        if bad:
            raise SystemExit('Step check: ' + bad)
        if prof.get('no_step_gate'):
            gate = 'no step gate (%s <= the T4-passed 0.5 m/s on the ground)' % args.test
        elif args.live:
            ok, gate = step_gate(args.out_root or vp.evidence_root(), args.test, method,
                                 prof['speed'], prof['steps'], prof['repeats'])
            if not ok:
                raise SystemExit('Step gate: ' + gate)
        say('Step %.2f m/s, stop method %s: %s' % (prof['speed'], method, gate))

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    name = '%s_%s_v%.2f_%s_erpmmax%s_%s' % (stamp, args.test, prof['speed'], method,
                                            int(speed_max) if speed_max is not None else 'NA',
                                            'armed' if real else 'disarmed')
    out = (Path(args.out_root).expanduser() if args.out_root else vp.evidence_root()) / name
    out.mkdir(parents=True, exist_ok=False)
    commit = subprocess.run(['git', '-C', str(vp.REPO), 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    info = dict(test=args.test, profile=prof, live=args.live, started=stamp, git_commit=commit,
                stop_method=method, converter=prof['converter'], converter_params=converter_params,
                step_gate=gate,
                mux_status_at_start=st, mux_max_speed_mps=cap, mux_max_steer_rad=steer_cap,
                vesc_speed_max_erpm=speed_max, publish_hz=r['publish_hz'], circle=circle_info,
                ramp_accel_mps2=r['ramp_accel_mps2'], vehicle_params=p, bag_topics=BAG_TOPICS)
    (out / 'run_info.json').write_text(json.dumps(info, indent=2, default=str))
    (out / 'README.md').write_text(
        '# %s (%s)\n\n%s\n\n- Place: %s; target %.2f m/s, steer %.2f rad, steady %.1f s, %d repeats\n'
        '- Mux real actuation: %s; mux caps %.2f m/s / %.2f rad; vesc_driver speed_max: %s ERPM\n'
        '- Converter %s, stop method %s; step gate: %s\n'
        '- Runner: %.0f Hz, ramp %.2f m/s^2; git %s\n\n## Operator notes\n\n(fill in)\n' % (
            args.test, stamp, prof['note'], prof['place'], prof['speed'], prof['steer'],
            prof['steady_s'], prof['repeats'], real, cap, steer_cap, speed_max,
            prof['converter'], method, gate,
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
        if machine.ground:
            need, top = machine.ground_window()
            say('Ground: arm only with %.2f..%.2f m free ahead (LiDAR); runner brakes at %.2f m '
                '(mux gate %.2f m = backstop). m = placement readout.' % (
                    need, top, machine.brake_range(prof['speed']), machine.gate))
        say('Keys: n = arm, hold SPACE = drive, s = skip, q = quit (idle). While armed/driving ANY other '
            'key = KILL (brake, latched until n). ' + machine.message)
        last_msg = None
        graph, imu_pubs = None, None
        last_imu = 'unknown'
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
                imu_pubs = node.count_publishers('/imu/data_raw')
            tick += 1
            w = node.world(now, half_width, imu_pubs)
            w['graph_fault'] = graph
            w['keyboard_lost'] = kb.eof
            if w.get('imu_problem') != last_imu:
                last_imu = w.get('imu_problem')
                machine.log(now, 'IMU_STATUS', last_imu or 'OK')
                say('IMU: %s' % (last_imu or 'OK'))
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
                if machine.message.startswith(KILL_BANNER):
                    say('\a\n' + '!' * 72)
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
    ap.add_argument('--steer', type=float, help='t3_circle: constant steering angle, rad (+ = left)')
    ap.add_argument('--speed', type=float, help='stepped profiles (t1_stop, t1b_stop, t4_straight): this step, m/s')
    ap.add_argument('--shutdown-probe', choices=['normal', 'systemexit', 'keyboardinterrupt'],
                    help=argparse.SUPPRESS)
    args, rest = ap.parse_known_args()
    if args.test and PROFILES[args.test].get('kind') == 'servo':
        # T2 is its own tool (servo-only stack, no mux); pass the remaining arguments through.
        tool = os.path.join(os.path.dirname(os.path.realpath(__file__)), 't2_servo_sweep.py')
        os.execv(sys.executable, [sys.executable, tool] + rest)
    if rest:
        ap.error('unrecognized arguments: %s' % ' '.join(rest))
    if args.shutdown_probe:
        shutdown_probe(args.shutdown_probe)
    elif args.self_test:
        self_test()
    elif args.list:
        for k, v in PROFILES.items():
            print('%-15s %s%s' % (k, v['note'], '  steps %s m/s' % v['steps'] if 'steps' in v else ''))
    elif args.test:
        run(args)
    else:
        ap.error('--test, --list or --self-test')


if __name__ == '__main__':
    main()
