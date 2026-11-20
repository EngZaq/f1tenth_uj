#!/usr/bin/env python3
"""F1TENTH UJ real-car test runner. Drives ONLY through physical_drive_mux.

Publishes /manual/drive, /joy (button 5 = deadman) and /control/request_mode at
50 Hz: the same interfaces as ~/continuous_keyboard_mux.py. Never /drive, never
/commands/*. Records its own ros2 bag. Not an emergency stop: the battery
disconnect / kill switch stays in the operator's hand.

Keys over SSH (no Enter): n = arm, hold SPACE = drive, s = skip, q = quit,
m = placement readout (ground profiles: range ahead, arm window, brake point, side walls).
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


def interval_stats(dts):
    if not dts:
        return {'count': 0}
    s = sorted(dts)

    def q(f):
        return s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))]
    return {'count': len(s), 'mean_s': round(sum(s) / len(s), 5), 'p50_s': round(q(.5), 5),
            'p95_s': round(q(.95), 5), 'p99_s': round(q(.99), 5), 'max_s': round(s[-1], 5),
            'over_0.10_s': sum(d > .10 for d in s), 'over_0.20_s': sum(d > .20 for d in s)}


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
        self.ground_ok = ground_ok
        # brake_at_range (T4): drive until the brake point; the time limit is set per repeat.
        self.brake_at_range = bool(prof.get('brake_at_range'))
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
        """Range the ground checks use: the smaller of the front median (+/-3 deg) and the
        mux-style nearest in +/-20 deg. NaN if the front window has no return."""
        f, s = w.get('front_m', float('nan')), w.get('sector_min_m', float('nan'))
        if not math.isfinite(f):
            return float('nan')
        return min(f, s) if math.isfinite(s) else f

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

    def placement(self, w):
        need, top = self.ground_window()
        return ('ahead %.2f m (front %.2f, nearest +/-20deg %.2f) | arm window %.2f..%.2f m | brake point '
                '%.2f m | side L %.2f R %.2f m' % (
                    self.ahead(w), w.get('front_m', float('nan')), w.get('sector_min_m', float('nan')),
                    need, top, self.brake_range(float(self.prof['speed'])),
                    w.get('left_m', float('nan')), w.get('right_m', float('nan'))))

    def finished(self):
        if 'extra_repeats' not in self.prof:
            return self.quit or self.repeat >= self.prof['repeats']
        return (self.quit or self.steady_count >= self.prof['repeats']
                or self.repeat >= self.max_repeats)

    def stop(self, now, reason):
        self.stop_steady = (self.state == 'DRIVING' and getattr(self, 'steady_logged', False)
                            and (not self.brake_at_range or reason.startswith('BRAKE_POINT')))
        self.state = 'STOPPING'
        self.t_stop = now
        self.tracker = ReboundTracker(self.last_erpm)  # ERPM at the zero = the latest telemetry
        self.log(now, 'ZERO_COMMAND', reason)
        if self.repeats and self.repeats[-1]['repeat'] == self.repeat and 'zero_wall' not in self.repeats[-1]:
            self.repeats[-1].update(zero_wall=time.time(), zero_reason=reason)
        return 0.0, 0.0, 0, 'STOP'

    def step(self, now, key, held, w):
        st = w.get('status') or {}
        mode = st.get('mode')
        if w.get('erpm') is not None and self.state != 'STOPPING':
            self.last_erpm = w['erpm']
        if key == 'q' or w.get('keyboard_lost'):
            self.quit = True  # lost SSH terminal: finish with zero, never wait in IDLE
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
                if reason is None and self.ground:
                    if not self.ground_ok:
                        reason = 'GROUND_NEEDS_T1_PASS_CONFIRMED'
                    elif w.get('erpm') is not None and abs(w['erpm']) >= UNDER_ERPM:
                        reason = 'CAR_MOVING(%.0f ERPM)' % w['erpm']
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
                reason = 'QUIT'
            if reason is None and key == 's':
                reason = 'SKIP'
            if reason is None and not self.brake_at_range and elapsed >= self.ramp_s + self.steady_s:
                reason = 'PROFILE_DONE'
            if reason is None and elapsed >= self.hard_s:
                reason = 'HARD_TIME_LIMIT'
            if reason is None and self.ground:
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
                           else ' | ended before steady speed: NOT counted')
            self.log(now, 'REPEAT_END', detail)
            self.repeat += 1
            self.reset_deadman = True
            self.state = 'DONE' if self.finished() else 'IDLE'
            if self.state == 'IDLE' and 'extra_repeats' in self.prof:
                self.message = '%s | %s stops %d/%d, repeat %d of max %d: release SPACE, press n.' % (
                    detail, 'brake-point' if self.brake_at_range else 'steady', self.steady_count,
                    self.prof['repeats'], self.repeat + 1, self.max_repeats)
                if self.brake_at_range:
                    self.message += ' Carry the car back to the start marks, hands off ~2 s, m to check.'
            elif self.state == 'IDLE':
                self.message = '%s | Repeat %d/%d: release SPACE, press n to arm.' % (
                    detail, self.repeat + 1, self.prof['repeats'])
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
                               (2.8, 1.2, 'SPEED_DOES_NOT_FIT'),    # object in the mux +/-20 deg sector
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
    print('PASS: deadman timing, ramp, speed-fit length, front range, config = vehicle_params,')
    print('      arm handshake, zero on stale status / deadman release / mux leaving MANUAL,')
    print('      refusals (real flag, extra publisher, SPACE held, ground gates), wall stop,')
    print('      no protected topic in the bag list, audit-log parsing, step resolution,')
    print('      step/yaml/cap match, step gate (rebound, unanalysed, too few, not steady, fault, method), live rebound,')
    print('      unsteady repeats not counted (extra repeats, capped).')
    print('PASS: T4 brake point 0.90 m (> mux gate 0.80 m), arm window 2.15..3.90 m, object in the mux')
    print('      sector / car moving refused, placement readout, drive to the brake point (no fixed time),')
    print('      only brake-point stops count, time limit, front range lost, sector min / side ranges.')
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

def resolve_profile(test, speed):
    """Profile with its speed fixed: stepped profiles need --speed equal to one step."""
    prof = dict(PROFILES[test])
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
    from sensor_msgs.msg import Joy, LaserScan
    from std_msgs.msg import String
    try:
        from vesc_msgs.msg import VescStateStamped
    except ImportError:
        VescStateStamped = None

    p = vp.load(args.params)
    prof = resolve_profile(args.test, args.speed)
    if args.live and not prof['live_allowed']:
        raise SystemExit('profile %s never runs with real actuation' % args.test)
    if prof['place'] == 'ground' and not args.t1_passed_confirmed:
        raise SystemExit('ground profile: needs --t1-passed-confirmed (Moatasem written T1 pass)')
    mism = vp.config_mismatches(p)
    if mism:
        raise SystemExit('config/real_car disagrees with vehicle_params.yaml:\n  ' + '\n  '.join(mism))
    r = vp.get(p, 'runner')
    mux_sector = float(vp.get(p, 't4.mux_sector_deg'))

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
                pubs = [i.node_name for i in self.get_publishers_info_by_topic(t)]
                # T1b only: the t1b converter, alone, may publish brake and current.
                allowed = t in T1B_ONLY_TOPICS and prof['converter'] == 't1b_stop_converter'
                if pubs and not (allowed and pubs == ['t1b_stop_converter']):
                    return 'EXTRA_PUBLISHER(%s=%s)' % (t, pubs)
            return None

        def world(self, now, half_width):
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
    method = 'speed0'
    if prof['converter'] == 't1b_stop_converter':
        converter_params = node.get_params('t1b_stop_converter', ['method', 'brake_current_a', 'brake_hold_s'])
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
    gate = 'not a stepped profile'
    if 'steps' in prof:
        bad = step_mismatch(prof['speed'], cap, speed_max, gain, float(vp.get(p, 'caps.vesc_speed_max_margin')))
        if bad:
            raise SystemExit('Step check: ' + bad)
        if args.live:
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
                vesc_speed_max_erpm=speed_max, publish_hz=r['publish_hz'],
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
