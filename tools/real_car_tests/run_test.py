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
import struct
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
    # T5b (21 Nov): IMU noise with the motor running and the car NOT moving (stand), steady 0.5 m/s for 90 s,
    # brake 5 A. Hands off the car >= 5 s before n (still reference). Analysis: t5_imu_analyze.py stand.
    't5b_stand_imu': dict(
        place='stand', steps=[0.5], steer=0.0, steady_s=90.0, repeats=1, extra_repeats=1, post_zero_s=4.0,
        live_allowed=True, converter='t1b_stop_converter', stop_method='brake5A',
        note='T5b, wheels off the ground: 0.5 m/s steady for 90 s (IMU vibration noise), brake 5 A. 1 run.'),
    # VESC timeout check (21 Nov): stand, 0.5 m/s steady 20 s; during steady the operator kills the converter
    # (pkill -9, vesc_driver keeps reporting) so the VESC gets no command: its own timeout must brake.
    # Analysis: vesc_timeout_check.py (writes timeout_check.json; a pass unlocks T3 without the fallback rules).
    'vesc_timeout_check': dict(
        place='stand', steps=[0.5], steer=0.0, steady_s=20.0, repeats=1, extra_repeats=1, post_zero_s=4.0,
        live_allowed=True, converter='t1b_stop_converter', stop_method='brake5A',
        note='VESC timeout, wheels off the ground: 0.5 m/s, kill -9 the converter during steady. 1 run.'),
    # Teleop (23 Nov, hall-loop mapping drive): keyboard driving through the same chain (runner -> mux MANUAL ->
    # t1b converter brake 5 A + steering TABLE -> vesc_driver). Every drive key is the deadman: hold w / SPACE =
    # straight, a = left, d = right at --speed; release everything = zero -> mux STOP -> brake 5 A. Steering moves
    # at TELEOP_STEER_RATE to +/-0.28 rad (table limit) and back to the centre when the key changes. No brake
    # point / time profile: the mux 0.80 m front gate is the obstacle backstop. Segments repeat until q.
    'teleop': dict(
        place='ground', steps=[0.3, 0.5], no_step_gate=True, steer=0.28, steady_s=0.0, repeats=100000,
        post_zero_s=1.0, live_allowed=True, converter='t1b_stop_converter', stop_method='brake5A',
        teleop=True, teleop_max_s=300.0,
        note='Teleop (hall-loop mapping): hold w/SPACE straight, a left, d right; release = brake 5 A. Until q.'),
}
# Teleop with a PS4 controller on the Jetson (23 Nov eve, after the day's Wi-Fi SSH key stalls): same chain + limits as
# 'teleop'; the deadman is R1 on /ps4/joy (joy_node on the Jetson, no network in the loop), steering = left stick
# (proportional, same 0.7 rad/s rate, +/-0.28 rad table limit). The keyboard keeps n / m / s / q, and every key
# (SPACE too) is a KILL while armed.
# 28 Nov (Moatasem): stick layout from config/real_car/ps4_teleop.yaml: R1 held = deadman, right stick Y = speed
# (fixed | proportional), left stick X = steer, L1 held = brake (stays armed), circle = KILL, no jump start after any
# stop. --speed = teleop_max_speed_mps (default 0.3, hard ceiling 1.5; no steps). Mux safety_mode teleop allowed.
PROFILES['teleop_ps4'] = dict(
    {k: v for k, v in PROFILES['teleop'].items() if k != 'steps'}, ps4=True,
    note='Teleop with the PS4 controller on the Jetson: options = arm, HOLD R1 = deadman, right stick up = speed, '
         'left stick = steer, L1 = brake (stays armed), circle = KILL; release R1 = brake 5 A. Until q (keyboard).')
TELEOP_KEYS = {'space': 0, 'w': 0, 'a': 1, 'd': -1}   # drive key -> steering direction (+ = left)
TELEOP_LOWVIS_MAX_SPEED = 0.3                          # m/s: teleop with a lowered mux visibility threshold
TELEOP_STEER_RATE = 0.7                                # rad/s: 0 -> 0.28 in 0.4 s, and back on key change
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
    '/t1b_stop_converter/status', '/ps4/joy',
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
HANDS_OFF_S = 1.0         # T3 gyro bias = mean gz over this window before SPACE (--fast: FAST_HANDS_OFF_S)
FAST_HANDS_OFF_S = 0.5
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


# ---------------------------------------------------------------- PS4 teleop (23 Nov eve; layout 28 Nov)

THROTTLE_MODES = ('fixed', 'proportional')
TELEOP_SPEED_CEILING = 1.5                 # m/s, hard: --speed above is refused (= mux MAX_SPEED_CEILING)
INPUT_EVENT = struct.Struct('llHHi')       # struct input_event on 64-bit Linux: timeval (16 B), type, code, value


def ps4_input_devices(text, match, vendor):
    """/proc/bus/input/devices text -> dict(bus, gamepad, motion): bus of the DS4 gamepad ('0005' Bluetooth, '0003'
    USB, None = no controller), its name, and the event device of its motion sensors (name ending with `match`)."""
    out = dict(bus=None, gamepad=None, motion=None)
    for block in text.strip().split('\n\n'):
        bus = ven = event = None
        name = ''
        for line in block.splitlines():
            if line.startswith('I:'):
                f = dict(kv.split('=', 1) for kv in line[2:].split() if '=' in kv)
                bus, ven = f.get('Bus'), (f.get('Vendor') or '').lower()
            elif line.startswith('N: Name='):
                name = line[len('N: Name='):].strip().strip('"')
            elif line.startswith('H: Handlers='):
                ev = [h for h in line[len('H: Handlers='):].split() if h.startswith('event')]
                event = '/dev/input/' + ev[0] if ev else None
        if ven != str(vendor).lower():
            continue
        if name.endswith(match):
            out['motion'] = event
            out['bus'] = out['bus'] or bus
        elif 'Touchpad' not in name and out['gamepad'] is None:
            out['gamepad'], out['bus'] = name, bus
    return out


class LinkWatch:
    """Bluetooth link heartbeat (28 Nov). joy_node publishes on changes plus an autorepeat of the LAST state with a
    fresh header stamp, and the kernel keeps that state when the Bluetooth link drops until its supervision timeout
    (seconds): /ps4/joy then stays 'fresh' with R1 held. The DS4 motion-sensor input device gets an event with every
    input report (MSC_TIMESTAMP + IMU), whatever the hands do, so its silence is the link loss. A reader thread
    counts raw input_event records; read error / end of file = device removed. Time injected for tests."""

    def __init__(self, ttl):
        self.ttl = float(ttl)
        self.at = self.wall = self.path = self.lost = None
        self.count, self.max_gap = 0, 0.0
        self.recent = []
        self.stop = False
        self.lock = threading.Lock()

    def event(self, now, wall=None):
        with self.lock:
            if self.at is not None:
                gap = now - self.at
                self.max_gap = max(self.max_gap, gap)
                self.recent = (self.recent + [gap])[-500:]
            self.at, self.wall = now, wall or time.time()
            self.count += 1

    def set_lost(self, why):
        with self.lock:
            self.lost = why

    def problem(self, now):
        with self.lock:
            if self.lost:
                return 'PS4_LINK_LOST(%s)' % self.lost
            if self.at is None:
                return 'PS4_LINK_LOST(no motion-sensor event yet)'
            if now - self.at >= self.ttl:
                return 'PS4_LINK_LOST(no report for %.2f s)' % (now - self.at)
        return None

    def read_loop(self, fd, clock=time.monotonic):
        while not self.stop:
            try:
                ready, _, _ = select.select([fd], [], [], 0.05)
                if not ready:
                    continue
                data = os.read(fd, INPUT_EVENT.size * 64)
            except OSError as exc:
                self.set_lost('device error: %s' % (exc.strerror or exc))
                return
            if not data:
                self.set_lost('device closed')
                return
            self.event(clock())

    def start(self, path):
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)     # PermissionError -> caller refuses
        self.path = path
        threading.Thread(target=self.read_loop, args=(fd,), daemon=True).start()

    def stats(self):
        with self.lock:
            return dict(path=self.path, events=self.count, max_gap_s=round(self.max_gap, 4), lost=self.lost,
                        recent_max_gap_s=round(max(self.recent), 4) if self.recent else None)


class Ps4Pad:
    """PS4 controller input from /ps4/joy (joy_node on the Jetson), layout from config/real_car/ps4_teleop.yaml.
    Thread-safe; time injected (testable). problem(): no /ps4/joy for joy_ttl_s, header stamp older than
    stamp_max_age_s or missing, too few buttons / axes, or the link heartbeat (LinkWatch) lost. held(): R1 down in a
    fresh message. Keys come from rising edges only (a button already down in the first message never arms or
    kills): options -> 'n', triangle -> 'm', kill buttons -> 'PS4_KILL'. Sticks: sign so that LEFT / UP read +,
    deadzone. centre_at / brake_at = the latest message with the throttle stick in the deadzone / L1 down, so a
    flick through the centre or an L1 tap between two runner ticks is not missed."""

    def __init__(self, cfg, link=None):
        self.dead, self.brake = int(cfg['deadman_button']), int(cfg['brake_button'])
        self.kill = [int(b) for b in cfg['kill_buttons']]
        self.keymap = {int(cfg['arm_button']): 'n', int(cfg['placement_button']): 'm'}
        self.keymap.update({b: 'PS4_KILL' for b in self.kill})
        self.ttl, self.stamp_ttl = float(cfg['joy_ttl_s']), float(cfg['stamp_max_age_s'])
        self.dz = float(cfg['stick_deadzone'])
        self.steer_axis, self.steer_sign = int(cfg['steer_axis']), float(cfg['steer_sign'])
        self.thr_axis, self.thr_sign = int(cfg['throttle_axis']), float(cfg['throttle_sign'])
        self.need = max([self.dead, self.brake] + list(self.keymap)) + 1
        self.need_axes = max(self.steer_axis, self.thr_axis) + 1
        self.link = link
        self.buttons = self.axes = self.at = self.lag = self.wall = None
        self.centre_at = self.brake_at = None
        self.pending, self.log, self.gaps = [], [], []
        self.lock = threading.Lock()

    def update(self, now, buttons, axes, lag=None, wall=None):
        """lag = ROS time at receipt - header.stamp, s (None = no stamp)."""
        b, a = [int(x) for x in buttons], [float(x) for x in axes]
        with self.lock:
            if self.at is not None:
                self.gaps.append(now - self.at)
            prev, self.buttons, self.axes, self.at, self.lag = self.buttons, b, a, now, lag
            self.wall = wall or time.time()
            if self.thr_axis < len(a) and abs(a[self.thr_axis]) < self.dz:
                self.centre_at = now
            if self.brake < len(b) and b[self.brake]:
                self.brake_at = now
            if prev is None:
                return
            for i, (was, isnow) in enumerate(zip(prev, b)):
                if isnow and not was:
                    if i in self.keymap:
                        self.pending.append(self.keymap[i])
                    self.log.append((self.wall, now, 'press %d' % i))
                elif was and not isnow:
                    self.log.append((self.wall, now, 'release %d' % i))

    def problem(self, now):
        with self.lock:
            if self.at is None or now - self.at >= self.ttl:
                return 'PS4_STALE(%s)' % ('no /ps4/joy' if self.at is None else '%.2f s' % (now - self.at))
            if self.lag is None:
                return 'PS4_NO_STAMP(/ps4/joy header.stamp is zero)'
            age = self.lag + (now - self.at)
            if not -self.stamp_ttl < age < self.stamp_ttl:
                return 'PS4_STAMP_OLD(%.2f s)' % age
            if len(self.buttons) < self.need:
                return 'PS4_TOO_FEW_BUTTONS(%d < %d)' % (len(self.buttons), self.need)
            if len(self.axes) < self.need_axes:
                return 'PS4_TOO_FEW_AXES(%d < %d)' % (len(self.axes), self.need_axes)
        return self.link.problem(now) if self.link is not None else None

    def held(self, now):
        if self.problem(now):
            return False
        with self.lock:
            return self.buttons[self.dead] == 1

    def _axis(self, idx, sign):
        if self.axes is None or idx >= len(self.axes) or not math.isfinite(self.axes[idx]):
            return 0.0
        return max(-1.0, min(1.0, sign * self.axes[idx]))

    def steer_frac(self):
        """Left stick: + = left, deadzone, scaled to -1..+1."""
        with self.lock:
            x = self._axis(self.steer_axis, self.steer_sign)
        if abs(x) < self.dz:
            return 0.0
        return math.copysign((abs(x) - self.dz) / (1.0 - self.dz), x)

    def machine_inputs(self):
        """World entries for Machine.ps4_drive (and the CONTROLLER_LOST detail)."""
        with self.lock:
            return dict(ps4_thr=self._axis(self.thr_axis, self.thr_sign),
                        ps4_brake_down=bool(self.buttons and self.brake < len(self.buttons)
                                            and self.buttons[self.brake]),
                        ps4_brake_at=self.brake_at, ps4_centre_at=self.centre_at, ps4_joy_wall=self.wall,
                        ps4_link_wall=self.link.wall if self.link is not None else None)

    def get_key(self):
        with self.lock:
            return self.pending.pop(0) if self.pending else None

    def snapshot(self):
        with self.lock:
            return (None if self.buttons is None else list(self.buttons),
                    None if self.axes is None else list(self.axes), self.at)


def throttle_target(thr, mode, vmax, dz, fixed_threshold, vmin):
    """Right stick (UP = +) -> (speed request m/s, stop reason or None); a stop = 0 m/s = converter brake.
    CENTRE (|thr| < deadzone), DOWN (brake; reverse not implemented), NONFINITE. fixed: above fixed_threshold =
    exactly vmax, else BELOW_FIXED. proportional: deadzone..1 -> 0..vmax, below vmin = BELOW_MIN (no low speed)."""
    if not math.isfinite(thr):
        return 0.0, 'NONFINITE'
    if abs(thr) < dz:
        return 0.0, 'CENTRE'
    if thr < 0:
        return 0.0, 'DOWN'
    if mode == 'fixed':
        return (float(vmax), None) if thr > fixed_threshold else (0.0, 'BELOW_FIXED')
    v = float(vmax) * min(1.0, (thr - dz) / (1.0 - dz))
    return (v, None) if v >= vmin - 1e-9 else (0.0, 'BELOW_MIN')


def stopping_distance(v, latency_s, decel):
    """Estimate: deadman latency travel + braking distance, v * latency + v^2 / (2 decel)."""
    return v * latency_s + v * v / (2.0 * decel)


def teleop_speed(speed, cfg):
    """--speed (= teleop_max_speed_mps; default speed_default_mps) within proportional_min_speed_mps .. the hard
    ceiling TELEOP_SPEED_CEILING (a lower speed_ceiling_mps in the yaml is honoured). SystemExit otherwise."""
    ceil = min(TELEOP_SPEED_CEILING, float(cfg['speed_ceiling_mps']))
    vmin = float(cfg['proportional_min_speed_mps'])
    v = float(cfg['speed_default_mps']) if speed is None else float(speed)
    if not math.isfinite(v) or v > ceil + 1e-9:
        raise SystemExit('teleop_ps4: --speed %s m/s REFUSED: teleop_max_speed_mps has a hard ceiling of %.2f m/s'
                         % (speed, ceil))
    if v < vmin - 1e-9:
        raise SystemExit('teleop_ps4: --speed %.2f m/s REFUSED: below %.2f m/s (proportional_min_speed_mps; the '
                         'sensorless VESC speed is unreliable that slow)' % (v, vmin))
    return v


def teleop_speed_problem(speed, mux_cap, speed_max, gain):
    """teleop_ps4: the running stack must carry teleop_max_speed_mps: mux cap == --speed (launch max_speed_mps), and
    vesc_driver speed_max must reach it (vesc_teleop_erpm12620.yaml for anything above ~0.33 m/s). -> reason/None."""
    if mux_cap is None or abs(mux_cap - speed) > STEP_TOL:
        return ('mux max_speed_mps=%s but teleop_max_speed_mps (--speed) is %.2f: launch max_speed_mps:=%.2f'
                % (mux_cap, speed, speed))
    if speed_max is None:
        return 'vesc_driver speed_max unreadable'
    if speed * gain > speed_max + SPEED_MAX_ROUNDING:
        return ('vesc_driver speed_max=%.0f ERPM < %.0f ERPM for %.2f m/s: launch vesc_yaml:=/home/ftenth/f1tenth_uj/'
                'config/real_car/vesc_teleop_erpm12620.yaml' % (speed_max, speed * gain, speed))
    return None


TELEOP_BRAKE_CHECK_MPS = 1.0       # --speed above this needs an analysed L1 stop from >= 0.9 m/s (teleop_onset.py)
TELEOP_BRAKE_CHECK_MARGIN = 1.25   # pass: VESC stop distance <= margin x v^2 / (2 floor_decel) + 0.05 m


def teleop_brake_gate(root, speed, decel):
    """teleop_ps4 above TELEOP_BRAKE_CHECK_MPS only after the floor brake-distance check (SESSION_HANDOFF): an armed
    teleop_ps4 run analysed by teleop_onset.py with an L1 (BRAKE_BUTTON) stop from >= 0.9 m/s whose VESC stop
    distance is within TELEOP_BRAKE_CHECK_MARGIN of the estimate. -> (ok, reason)."""
    if speed <= TELEOP_BRAKE_CHECK_MPS + 1e-9:
        return True, '%.2f m/s <= %.1f m/s: no brake-distance gate' % (speed, TELEOP_BRAKE_CHECK_MPS)
    seen = []
    for f in sorted(Path(root).expanduser().glob('*_teleop_ps4_v*_armed/teleop_onset.json'), reverse=True):
        try:
            rows = json.loads(f.read_text())
        except ValueError:
            continue
        for r in rows:
            v, d = r.get('speed_mps'), r.get('vesc_distance_m')
            if r.get('event') != 'BRAKE_BUTTON' or v is None or d is None or v < 0.9:
                continue
            lim = TELEOP_BRAKE_CHECK_MARGIN * v * v / (2.0 * decel) + 0.05
            if d <= lim:
                return True, 'L1 stop from %.2f m/s in %.2f m (<= %.2f m) in %s' % (v, d, lim, f.parent.name)
            seen.append('%s: %.2f m from %.2f m/s > %.2f m' % (f.parent.name, d, v, lim))
    return False, ('--speed %.2f > %.1f m/s needs the floor brake-distance check first (SESSION_HANDOFF: 1.0 m/s at a '
                   'wall, L1 at the mark, then teleop_onset.py): %s' % (speed, TELEOP_BRAKE_CHECK_MPS,
                                                                        '; '.join(seen) or 'no analysed L1 stop'))


def safety_mode_gate(prof, status):
    """Mux safety_mode from /control/status (missing = 'autonomy': a mux built before 28 Nov). 'teleop' (perception
    gates relaxed) is for the PS4 runner only; the mux refuses the keyboard runner's plain MANUAL request too.
    -> (mode, refusal or None)."""
    mode = (status or {}).get('safety_mode') or 'autonomy'
    if mode not in ('autonomy', 'teleop'):
        return mode, 'mux safety_mode %r unknown' % mode
    if mode == 'teleop' and not prof.get('ps4'):
        return mode, ('mux safety_mode teleop (perception gates relaxed) is for the PS4 controller only: run '
                      '--test teleop_ps4, or relaunch without safety_mode:=teleop for this profile')
    return mode, None


VESC_MATCH_KEYS = ('speed_min', 'speed_max', 'current_min', 'current_max', 'brake_min', 'brake_max',
                   'servo_min', 'servo_max')


def vesc_yaml_match(vesc_params, config_dir):
    """config/real_car/vesc_*.yaml files whose limits equal the running vesc_driver's parameters (the runner cannot
    read the launch argument itself). -> [file names]."""
    import yaml
    names = []
    for f in sorted(Path(config_dir).glob('vesc_*.yaml')):
        try:
            rp = yaml.safe_load(f.read_text())['vesc_driver']['ros__parameters']
            if all(vesc_params.get(k) is not None and abs(float(rp[k]) - float(vesc_params[k])) < 1e-6
                   for k in VESC_MATCH_KEYS):
                names.append(f.name)
        except (KeyError, TypeError, ValueError, OSError):
            continue
    return names


def ps4_controller_check(pad, cfg, clock, sleep, say):
    """Before anything can arm (28 Nov layout, ps4_teleop.yaml): (1) hands off 1.5 s: /ps4/joy max gap <=
    check_max_gap_s, no button down, controller not lost; (2) LEFT stick fully LEFT: the axis that moves most (>= 0.8;
    triggers resting at +/-1 ignored) must be steer_axis and read + after steer_sign; (3) RIGHT stick fully UP: the
    same for throttle_axis / throttle_sign; (4) press and release R1, then L1, then circle: any other button first
    = refusal. SystemExit naming the index / sign seen, so ps4_teleop.yaml can be fixed. -> dict for run_info.json."""
    timeout, max_gap = float(cfg['check_timeout_s']), float(cfg['check_max_gap_s'])

    def wait(pred, what):
        t_end = clock() + timeout
        while clock() < t_end:
            r = pred()
            if r:
                return r
            sleep(0.02)
        raise SystemExit('PS4 check: %s not seen within %.0f s' % (what, timeout))
    say('PS4 check 1/4: hands off the controller (sticks centred, no button) for 1.5 s ...')
    wait(lambda: pad.snapshot()[2] is not None, '/ps4/joy (start joy_node: see SESSION_HANDOFF)')
    sleep(1.5)
    with pad.lock:
        recent = pad.gaps[-60:]
    gap = max(recent) if recent else float('inf')
    if not recent or gap > max_gap:
        raise SystemExit('PS4 check: /ps4/joy max gap %.3f s > %.2f s: start joy_node with -p autorepeat_rate:=50.0'
                         % (gap, max_gap))
    prob = pad.problem(clock())
    if prob:
        raise SystemExit('PS4 check: %s' % prob)
    buttons, rest, _ = pad.snapshot()
    if any(buttons):
        raise SystemExit('PS4 check: button(s) %s pressed at rest: release everything and start again'
                         % [i for i, x in enumerate(buttons) if x])
    if len(rest) < pad.need_axes or len(buttons) < pad.need:
        raise SystemExit('PS4 check: /ps4/joy has %d buttons / %d axes, ps4_teleop.yaml needs %d / %d'
                         % (len(buttons), len(rest), pad.need, pad.need_axes))
    found = {}

    def moved():
        axes = pad.snapshot()[1]
        cand = [(abs(a - r), i, a - r) for i, (a, r) in enumerate(zip(axes, rest)) if abs(r) < 0.5]
        best = max(cand) if cand else (0.0, None, 0.0)
        return best if best[0] >= 0.8 else None
    for n, what, key, axis, sign in ((2, 'the LEFT stick fully to the LEFT', 'steer', pad.steer_axis, pad.steer_sign),
                                     (3, 'the RIGHT stick fully UP', 'throttle', pad.thr_axis, pad.thr_sign)):
        say('PS4 check %d/4: push %s and hold it ...' % (n, what))
        _, idx, delta = wait(moved, 'a stick axis moving >= 0.8')
        if idx != axis or sign * delta <= 0:
            raise SystemExit('PS4 check: %s moved axis %d by %+.2f, but ps4_teleop.yaml has %s_axis: %d, %s_sign: %+.1f'
                             ' -> set %s_axis: %d and %s_sign: %+.1f there (index and sign as seen now)'
                             % (what, idx, delta, key, axis, key, sign, key, idx, key, 1.0 if delta > 0 else -1.0))
        found[key] = dict(axis=idx, raw_delta=round(delta, 3))
        say('PS4: %s = axis %d, sign %+.0f OK; now let the stick go back to the centre.' % (key, idx, sign))
        wait(lambda: abs(pad.snapshot()[1][idx] - rest[idx]) < 0.2, 'the stick back at the centre')
    for name, key, idx in (('R1 (deadman)', 'deadman_button', pad.dead), ('L1 (brake)', 'brake_button', pad.brake),
                           ('circle (KILL)', 'kill_buttons[0]', pad.kill[0])):
        say('PS4 check 4/4: press and release %s once ...' % name)

        def pressed():
            b = pad.snapshot()[0]
            other = [i for i, x in enumerate(b) if x and i != idx]
            if other:
                raise SystemExit('PS4 check: button %s pressed while waiting for %s (ps4_teleop.yaml %s: %d): if '
                                 'that was %s, set %s: %d' % (other, name, key, idx, name, key, other[0]))
            return b[idx] == 1
        wait(pressed, '%s (button %d)' % (name, idx))
        wait(lambda: pad.snapshot()[0][idx] == 0, '%s released' % name)
    with pad.lock:
        pad.pending.clear()                    # nothing pressed during the check arms or kills later
    say('PS4 check PASSED: R1 %d, L1 %d, circle %d, steer axis %d (%+.0f), throttle axis %d (%+.0f), max gap %.3f s.'
        % (pad.dead, pad.brake, pad.kill[0], pad.steer_axis, pad.steer_sign, pad.thr_axis, pad.thr_sign, gap))
    return dict(deadman_button=pad.dead, brake_button=pad.brake, kill_buttons=pad.kill, steer_axis=pad.steer_axis,
                steer_sign=pad.steer_sign, throttle_axis=pad.thr_axis, throttle_sign=pad.thr_sign, max_gap_s=gap,
                n_buttons=len(buttons), n_axes=len(rest), sticks_seen=found)


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

def t3_geometry(steer, t3, wheelbase, body_length, body_width, front_x, extra_m=0.0):
    """Clear-disk geometry in the start base_link frame (x forward, y left; + steer = left). steer = the
    EXPECTED real angle (the command, or for full lock the T2 angle at the clip). extra_m widens the clear
    disk only (VESC timeout fallback), not the geofence."""
    side = 1.0 if steer > 0 else -1.0
    r_nom = wheelbase / math.tan(abs(steer))
    r_min, r_max = r_nom * float(t3['r_factor_min']), r_nom * float(t3['r_factor_max'])
    hw, rear_x = body_width / 2.0, front_x - body_length
    reach = max(math.hypot(r_max + hw, front_x), math.hypot(r_max + hw, rear_x))   # outer body corner
    clear_r = reach + float(t3['clear_margin_m']) + float(extra_m)
    return dict(side=side, r_nom=r_nom, r_min=r_min, r_max=r_max, center=(0.0, side * r_max), reach=reach,
                clear_r=clear_r, diameter_m=2.0 * clear_r, inner_free_r=max(0.0, r_min - hw), extra_m=extra_m)


def t3_command(t3, steering, steer=None, full_lock=None):
    """T3 command -> (commanded steer, expected real angle). --steer: within the per-side radian limits;
    --full-lock left|right: the per-side limit itself (converter clamp -> servo exactly at the clip), expected
    angle = the T2 angle at that clip (steering.max_left_rad / max_right_rad). SystemExit on a bad command."""
    lo = float(t3['steer_min_rad'])
    lim = {1.0: float(t3['steer_max_left_rad']), -1.0: float(t3['steer_max_right_rad'])}
    if (steer is None) == (full_lock is None):
        raise SystemExit('t3_circle needs exactly one of --steer (+ = left) or --full-lock left|right')
    if full_lock is not None:
        if full_lock not in ('left', 'right'):
            raise SystemExit('--full-lock must be left or right')
        side = 1.0 if full_lock == 'left' else -1.0
        return side * lim[side], float(steering['max_left_rad' if side > 0 else 'max_right_rad'])
    side = 1.0 if steer > 0 else -1.0
    if not (math.isfinite(steer) and lo - 1e-9 <= abs(steer) <= lim[side] + 1e-9):
        raise SystemExit('t3_circle needs --steer in +%.2f..+%.3f (left) or -%.2f..-%.3f (right) rad'
                         % (lo, lim[1.0], lo, lim[-1.0]))
    return float(steer), float(steer)


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


def masked(bearing_deg, r, mask):
    """True if a return (laser-frame bearing, range) is the car itself (lidar.self_mask sectors)."""
    return any(a <= bearing_deg <= b and r < rmax for a, b, rmax in (mask or ()))


def t3_clearance(scan, geom, laser_x, body_length, body_width, front_x, mask=()):
    """Smallest (distance to the clear-disk centre - clear radius) over all returns (car body and the measured
    self_mask excluded): < 0 = something inside the space the car may sweep. -> dict(min_m, bearing_deg, n) or
    None (no scan). The LiDAR blind sector is not seen: the area behind the car is checked by eye (checklist)."""
    pts = scan_points_base(scan, laser_x)
    if not pts:
        return None
    cx, cy = geom['center']
    best = None
    for x, y, b in pts:
        if own_body(x, y, body_length, body_width, front_x) or masked(b, math.hypot(x - laser_x, y), mask):
            continue
        d = math.hypot(x - cx, y - cy) - geom['clear_r']
        if best is None or d < best[0]:
            best = (d, b)
    if best is None:
        return dict(min_m=float('inf'), bearing_deg=None, n=0)
    return dict(min_m=best[0], bearing_deg=best[1], n=len(pts))


def t3_visibility(scan, half_width_deg=20.0, step_deg=5.0, fov=None, mask=()):
    """Mux front-visibility prediction for every heading the car takes on a circle: the share of finite
    returns in +/- half_width around each heading (laser frame, as physical_drive_mux front_evidence).
    Beams outside the LiDAR field of view `fov` = (min_deg, max_deg) and beams with a self_mask return are
    UNKNOWN (what the mux will see there is not in this scan): a heading is predicted only if its whole
    window lies inside the fov, and its share counts known beams only (>= half of the window must be known).
    -> dict(min_fraction, heading_deg, predicted, skipped) or None (no scan / nothing predictable)."""
    if scan is None or not scan.angle_increment or len(scan.ranges) < 100:
        return None
    n = len(scan.ranges)
    lo, hi = fov if fov else (-180.0, 180.0)
    state = []                                       # 1 return, 0 no return, None unknown
    for i, r in enumerate(scan.ranges):
        b = math.degrees(scan.angle_min + i * scan.angle_increment)
        b = (b + 180.0) % 360.0 - 180.0
        ok = math.isfinite(r) and scan.range_min < r <= scan.range_max
        if not lo <= b <= hi or (ok and masked(b, r, mask)):
            state.append(None)
        else:
            state.append(1 if ok else 0)
    half = int(round(math.radians(half_width_deg) / scan.angle_increment))
    best, predicted, skipped = None, 0, 0
    h = -180.0
    while h < 180.0:
        if fov and not (lo <= h - half_width_deg and h + half_width_deg <= hi):
            skipped += 1
            h += step_deg
            continue
        c = int(round((math.radians(h) - scan.angle_min) / scan.angle_increment))
        win = [state[(c + k) % n] for k in range(-half, half + 1)]
        known = [x for x in win if x is not None]
        if len(known) < 0.5 * len(win):
            skipped += 1
        else:
            predicted += 1
            f = sum(known) / len(known)
            if best is None or f < best[0]:
                best = (f, h)
        h += step_deg
    if best is None:
        return None
    return dict(min_fraction=best[0], heading_deg=best[1], predicted=predicted, skipped=skipped)


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


def live_mode(real, live, live_allowed):
    """Mux real actuation vs --live -> (real, live, one-line note or None). 23 Nov: mux real + no --live -> live is
    switched on (one line) instead of refusing; --live with the mux disarmed and real on a never-live profile
    still refuse."""
    if real and not live_allowed:
        raise SystemExit('Mux real_actuation_enabled=True but this profile never runs with real actuation: refusing.')
    if live and not real:
        raise SystemExit('Mux real_actuation_enabled=False but --live given: refusing (launch the armed stack, or '
                         'drop --live for a disarmed run).')
    if real and not live:
        return real, True, 'Mux real actuation is ENABLED: live mode on (--live implied).'
    return real, live, None


def bag_command(out_dir, topics):
    """ros2 bag record, zstd per file (23 Nov; analyze.read_bag reads both kinds)."""
    return (['ros2', 'bag', 'record', '-o', str(out_dir), '--compression-mode', 'file',
             '--compression-format', 'zstd'] + list(topics))


def timeout_verified(root):
    """VESC firmware timeout verified on the stand: an armed vesc_timeout_check run whose timeout_check.json
    (vesc_timeout_check.py) passed. -> (ok, reason)."""
    seen = []
    for f in sorted(Path(root).expanduser().glob('*_vesc_timeout_check_*_armed/timeout_check.json'), reverse=True):
        try:
            res = json.loads(f.read_text())
        except ValueError:
            continue
        if res.get('pass') is True:
            return True, 'VESC timeout verified in %s' % f.parent.name
        seen.append('%s: %s' % (f.parent.name, res.get('verdict', 'not passed')))
    return False, 'VESC timeout NOT verified (%s)' % ('; '.join(seen[:3]) or 'no analysed vesc_timeout_check run')


def lowvis_gate(test, status, timeout_ok, speed=None, t3_min=0.35):
    """Mux low-visibility mode (manual_allow_reduced_visibility), only with the VESC timeout verified. T3 (Moatasem
    22 Nov, launch t3_low_visibility:=true) with a threshold >= t3_min; teleop (23 Nov corridor, launch
    manual_min_front_valid:=<f>) only at --speed <= TELEOP_LOWVIS_MAX_SPEED. Every other profile refuses.
    -> (active, refusal or None)."""
    st = status or {}
    if not st.get('manual_low_visibility_enabled'):
        return False, None
    frac = float(st.get('manual_min_front_valid_fraction') or 0.0)
    if test in ('teleop', 'teleop_ps4'):
        if speed is None or speed > TELEOP_LOWVIS_MAX_SPEED + 1e-9:
            return True, ('mux low-visibility mode (MANUAL front >= %.2f): teleop only at --speed <= %.1f'
                          % (frac, TELEOP_LOWVIS_MAX_SPEED))
    elif test == 't3_circle':
        if frac < t3_min - 1e-9:
            return True, ('mux MANUAL front threshold %.2f < %.2f: T3 keeps its value (relaunch with '
                          't3_low_visibility:=true, without manual_min_front_valid)' % (frac, t3_min))
    else:
        return True, ('mux low-visibility mode is for T3 and teleop <= %.1f m/s ONLY: relaunch without '
                      't3_low_visibility / manual_min_front_valid for %s' % (TELEOP_LOWVIS_MAX_SPEED, test))
    if not timeout_ok:
        return True, 'mux low-visibility mode needs the verified VESC timeout (vesc_timeout_check PASS)'
    return True, None


def t3_slow_gate(root, prof, slow=0.3):
    """VESC-timeout fallback: 0.5 m/s only after a counted (TURNS_DONE) live run of the same command at
    `slow` m/s. -> (ok, reason)."""
    if prof['speed'] <= slow + STEP_TOL:
        return True, '%.2f m/s is the fallback speed' % prof['speed']
    for info_path in sorted(Path(root).expanduser().glob('*_t3_circle_v%.2f_*_armed/run_info.json' % slow)):
        try:
            info = json.loads(info_path.read_text())
        except ValueError:
            continue
        pr = info.get('profile', {})
        if (not info.get('live') or pr.get('full_lock') != prof.get('full_lock')
                or abs(float(pr.get('steer', 99)) - float(prof['steer'])) > 1e-6):
            continue
        ev = info_path.parent / 'runner_events.csv'
        if ev.exists():
            with open(ev) as fp:
                if any(e['event'] == 'ZERO_COMMAND' and e['detail'].startswith('TURNS_DONE')
                       for e in csv.DictReader(fp)):
                    return True, 'counted %.2f m/s run of this command: %s' % (slow, info_path.parent.name)
    return False, ('%.2f m/s needs a counted %.2f m/s run of the same command first (%s)'
                   % (prof['speed'], slow, 'mux low-visibility mode rule' if prof.get('mux_low_visibility')
                      else 'VESC timeout not verified'))


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
            self.expect_steer = float(prof.get('expect_steer', prof['steer']))   # full lock: T2 angle at the clip
            self.geom = t3_geometry(self.expect_steer, self.t3, self.wheelbase, float(veh['body_length_m']),
                                    float(veh['body_width_m']), float(veh['front_bumper_x_m']),
                                    float(prof.get('clear_extra_m', 0.0)))
            self.target_rad = 2 * math.pi * float(self.t3['target_turns'])
            self.min_visible = 0.75 + float(self.t3['visibility_margin'])   # updated from the mux status
        self.ground_ok = ground_ok
        # --fast (23 Nov): no T3 clear-disk / visibility refusal at arm, hands-off 0.5 s (gyro bias window).
        # Kept: SPACE-release brake, converter brake, caps, IMU freeze refusal, VESC status, geofence.
        self.fast = bool(prof.get('fast'))
        self.teleop = bool(prof.get('teleop'))
        self.ps4 = bool(prof.get('ps4'))           # teleop_ps4: the deadman is R1 on the controller, arm = options
        # Mux safety_mode teleop arms only on the PS4 runner's request (physical_drive_mux TELEOP_MANUAL_REQUEST).
        self.manual_request = prof.get('manual_request', 'MANUAL')
        self.safety_mode = prof.get('safety_mode', 'autonomy')
        self.last_warn = None
        self.ps4_lost = False
        if self.ps4:
            pc = prof['ps4_cfg']
            self.thr_mode, self.dz = prof['throttle_mode'], float(pc['stick_deadzone'])
            self.fixed_thr, self.vmin = float(pc['fixed_threshold']), float(pc['proportional_min_speed_mps'])
            self.brake_a = float(pc['brake_current_a'])
            self.v_cmd, self.allow, self.block_t, self.prev_t = 0.0, False, 0.0, 0.0
            self.brake_on = self.block_logged = False
        self.steer_now, self.steer_t = 0.0, None
        self.hands_off_s = float(prof.get('hands_off_s', HANDS_OFF_S))
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
        if w.get('ps4_problem'):
            return w['ps4_problem']        # teleop_ps4: controller stale / disconnected -> zero, arm refused
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

    def sync_mux_visibility(self, w):
        """Runner arm threshold = the mux's MANUAL threshold (from its status) + t3.visibility_margin; returns the
        LOW-VISIBILITY banner ('' in the standard mode)."""
        st = w.get('status') or {}
        if st.get('manual_min_front_valid_fraction') is not None:
            self.min_visible = float(st['manual_min_front_valid_fraction']) + float(self.t3['visibility_margin'])
        if not st.get('manual_low_visibility_enabled'):
            return ''
        return ('LOW-VISIBILITY MODE ACTIVE (T3 only: mux MANUAL front >= %.2f, AUTO and the 0.80 m obstacle gate '
                'unchanged; spotter, look behind the car) | ' % float(st.get('manual_min_front_valid_fraction', 0)))

    def circle_text(self, w):
        lowvis = self.sync_mux_visibility(w)
        g, c, v = self.geom, w.get('t3_clear'), w.get('t3_vis')
        what = ('FULL LOCK (command %+.3f rad, expected %+.3f)' % (self.prof['steer'], self.expect_steer)
                if self.prof.get('full_lock') else 'steer %+.2f rad' % self.prof['steer'])
        fb = (' incl. +%.1f m VESC-timeout FALLBACK' % g['extra_m']) if g.get('extra_m') else ''
        fast = ('FAST: clear disk + visibility NOT checked (look at the disk yourself) | ' if self.fast else '')
        return fast + lowvis + (
            'circle %s %s: R nominal %.2f m (assumed %.2f..%.2f), clear disk %.2f m diameter%s, centre '
                '%.2f m to the %s | clearance %s | mux visibility min %s (needs >= %.2f) | RF2O %s' % (
                    'LEFT' if g['side'] > 0 else 'RIGHT', what, g['r_nom'], g['r_min'], g['r_max'],
                    g['diameter_m'], fb, g['r_max'], 'left' if g['side'] > 0 else 'right',
                    '-' if c is None else '%+.2f m (worst at %s deg)' % (c['min_m'], '-' if c['bearing_deg'] is None
                                                                       else '%.0f' % c['bearing_deg']),
                    '-' if v is None else '%.2f at heading %.0f deg (%d headings predicted, %d blind/masked: mux '
                    'is the backstop there)' % (v['min_fraction'], v['heading_deg'], v.get('predicted', 0),
                                                 v.get('skipped', 0)),
                    self.min_visible, 'OK' if w.get('rf2o_age', 1e9) <= float(self.t3['rf2o_stale_s']) else 'STALE')
                + ' | LiDAR rear blind sector NOT checked: look behind the car (disk empty, nobody inside)')

    def circle_refusal(self, w):
        self.sync_mux_visibility(w)
        c, v = w.get('t3_clear'), w.get('t3_vis')
        if self.fast:
            c = v = None                      # --fast: clear disk + visibility not checked (operator + mux)
        elif c is None or v is None:
            return 'CIRCLE_NO_SCAN'
        if c is not None and c['min_m'] < 0:
            return 'CIRCLE_NOT_CLEAR(return %.2f m inside the %.2f m clear disk at %.0f deg)' % (
                -c['min_m'], self.geom['diameter_m'], c['bearing_deg'])
        if v is not None and v['min_fraction'] < self.min_visible:
            return 'CIRCLE_LOW_VISIBILITY(%.2f < %.2f at heading %.0f deg: the mux would stop there)' % (
                v['min_fraction'], self.min_visible, v['heading_deg'])
        if w.get('rf2o_age', 1e9) > float(self.t3['rf2o_stale_s']) or w.get('rf2o_pose') is None:
            return 'CIRCLE_RF2O_STALE(geofence source)'
        if w.get('gz') is None:
            return 'CIRCLE_NO_GYRO'
        return None

    def placement(self, w):
        if self.circle:
            return self.circle_text(w) + (steer_mode_text(w) if self.prof.get('converter') == 't1b_stop_converter'
                                          else '')
        need, top = self.ground_window()
        return ('front %.2f m | arm window %.2f..%.2f m | brake point %.2f m | mux nearest +/-20deg %.2f m '
                '(arm needs >= %.2f, mux brakes < %.2f) | side L %.2f R %.2f m' % (
                    self.ahead(w), need, top, self.brake_range(float(self.prof['speed'])),
                    w.get('sector_min_m', float('nan')), self.gate + self.guard, self.gate,
                    w.get('left_m', float('nan')), w.get('right_m', float('nan')))
                + (steer_mode_text(w) if self.prof.get('converter') == 't1b_stop_converter' else ''))

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
            expect = v * math.tan(abs(self.expect_steer)) / self.wheelbase
            mean = sum(x for _, x in self.wz_hist) / len(self.wz_hist)
            if side * mean < float(t3['turn_check_fraction']) * expect:
                return 'TURN_CHECK_FAILED(yaw rate %+.3f rad/s, expected %+.3f)' % (mean, side * expect)
        if side * self.turned >= self.target_rad:
            return 'TURNS_DONE(gyro %.0f deg, RF2O %.0f deg)' % (math.degrees(self.turned), math.degrees(self.turned_rf))
        if side * self.turned_rf >= self.target_rad + 0.5:
            return 'TURNS_RF2O_BACKUP(gyro %.0f deg, RF2O %.0f deg)' % (math.degrees(self.turned),
                                                                       math.degrees(self.turned_rf))
        return None

    def teleop_steer(self, now, direction):
        """Teleop steering: move toward direction x steer max (0 = centre) at TELEOP_STEER_RATE."""
        target = float(direction or 0) * abs(float(self.prof['steer']))
        dt = 0.0 if self.steer_t is None else max(0.0, now - self.steer_t)
        self.steer_t = now
        step = TELEOP_STEER_RATE * dt
        self.steer_now += max(-step, min(step, target - self.steer_now))
        return self.steer_now

    STOP_TEXT = dict(CENTRE='right stick centre', DOWN='right stick down (no reverse)', NONFINITE='stick non-finite',
                     BELOW_FIXED='right stick below the fixed threshold', BELOW_MIN='speed below the proportional floor')

    def ps4_drive(self, now, w):
        """teleop_ps4 while DRIVING (R1 held, mux MANUAL). L1 held = brake (speed 0 -> converter brake, stays armed),
        overrides the stick. Right stick -> throttle_target; any stop is speed 0 (converter brake, never a VESC speed
        0). No jump start: after arming, after L1 and after a non-centre stop, motion needs the stick seen in the
        centre first (now, or in any /ps4/joy message since: pad centre_at). Increases ramp at ramp_accel_mps2, a
        stop is immediate. -> (speed, steer)."""
        thr = float(w.get('ps4_thr', 0.0))
        brake_at = w.get('ps4_brake_at')
        brake = bool(w.get('ps4_brake_down')) or (brake_at is not None and brake_at >= self.prev_t)
        if brake:
            if not self.brake_on:
                self.brake_on = True
                self.log(now, 'BRAKE_BUTTON', 'L1 pressed: brake %g A, stays ARMED (command was %.2f m/s)'
                         % (self.brake_a, self.v_cmd))
            self.allow, self.block_t = False, now
        elif self.brake_on:
            self.brake_on = False
            self.log(now, 'BRAKE_BUTTON', 'L1 released: centre the right stick to drive again')
        centre_at = w.get('ps4_centre_at')
        if not brake and not self.allow and (abs(thr) < self.dz or (centre_at is not None
                                                                    and centre_at >= self.block_t)):
            self.allow, self.block_logged = True, False
        target, why = throttle_target(thr, self.thr_mode, float(self.prof['speed']), self.dz, self.fixed_thr,
                                      self.vmin)
        if brake:
            target = 0.0
        elif why is not None:
            if self.v_cmd > 0:
                self.log(now, 'STICK_STOP', '%s -> brake %g A (was %.2f m/s)' % (self.STOP_TEXT[why], self.brake_a,
                                                                               self.v_cmd))
            if why != 'CENTRE':
                self.allow, self.block_t = False, now
        elif not self.allow:
            target = 0.0
            if not self.block_logged:
                self.block_logged = True
                self.log(now, 'NO_JUMP_START', 'right stick up, but not through the centre since the last stop / '
                                               'brake / arm: centre it, then push')
        dt = max(0.0, now - self.prev_t)
        self.prev_t = now
        if target <= 0.0:
            self.v_cmd = 0.0
        else:
            step = self.ramp * dt
            self.v_cmd += max(-step, min(step, target - self.v_cmd))
        return self.v_cmd, self.teleop_steer(now, w.get('drive_dir'))

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
        if self.ps4:
            self.message = KILL_BANNER.replace('release SPACE and press n', 'release R1 and press options (or n)')
        return out or (0.0, 0.0, 0, None)

    def step(self, now, key, held, w):
        st = w.get('status') or {}
        mode = st.get('mode')
        if w.get('erpm') is not None and self.state != 'STOPPING':
            self.last_erpm = w['erpm']
        if self.ps4:
            prob = w.get('ps4_problem')
            if prob and not self.ps4_lost:
                self.ps4_lost = True
                self.log(now, 'CONTROLLER_LOST', '%s last_joy_wall=%s last_link_wall=%s' % (
                    prob, w.get('ps4_joy_wall'), w.get('ps4_link_wall')))
            elif not prob and self.ps4_lost:
                self.ps4_lost = False
                self.log(now, 'CONTROLLER_OK')
        if self.safety_mode == 'teleop' and st.get('teleop_warn') != self.last_warn:
            self.last_warn = st.get('teleop_warn')
            self.log(now, 'WARN' if self.last_warn else 'WARN_CLEAR', '%s (mux safety_mode teleop: no stop) front '
                     'valid %s nearest %s m' % (self.last_warn or 'front gates clear', st.get('front_valid_fraction'),
                                                st.get('nearest_front_m')))
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
                    reason = 'RELEASE_DEADMAN_FIRST(R1)' if self.ps4 else 'RELEASE_SPACE_FIRST'
                if reason is None and w.get('imu_problem'):
                    reason = w['imu_problem']   # arm only; a freeze while driving is logged, not a stop
                if reason is None and self.ground:
                    if not self.ground_ok:
                        reason = 'GROUND_NEEDS_T1_PASS_CONFIRMED'
                    elif w.get('erpm') is not None and abs(w['erpm']) >= UNDER_ERPM:
                        reason = 'CAR_MOVING(%.0f ERPM)' % w['erpm']
                    elif (w.get('sector_min_m', float('inf')) < self.gate + self.guard
                          and self.safety_mode != 'teleop'):
                        # The mux would latch STOP on the first tick: refuse with the reason instead.
                        # (mux safety_mode teleop: the obstacle gate only warns, so this copy of it does too)
                        reason = 'MUX_SECTOR_OBJECT(nearest +/-20deg %.2f m < %.2f m)' % (
                            w['sector_min_m'], self.gate + self.guard)
                    elif self.circle:
                        reason = self.circle_refusal(w)
                        self.log(now, 'CIRCLE_FIT', self.circle_text(w))
                    elif self.teleop:
                        pass                      # no straight-length fit: the operator steers, mux gate = backstop
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
                    self.log(now, 'ARM_REQUEST', self.manual_request)
                    return 0.0, 0.0, 0, self.manual_request
            return 0.0, 0.0, 0, None
        if self.state == 'ARMING':
            reason = self.fault(w)
            if self.quit or reason:
                self.state = 'IDLE'
                self.log(now, 'ARM_ABORTED', reason or 'QUIT')
                return 0.0, 0.0, 0, 'STOP'
            if mode == 'MANUAL_READY' and st.get('last_mode_request') == self.manual_request:
                self.state = 'READY'
                self.log(now, 'MANUAL_READY', 'hold %s to drive' % ('R1' if self.ps4 else 'SPACE'))
                if self.ps4:
                    self.log(now, 'ARMED', 'safety_mode %s: hold R1, centre the right stick, then push it up'
                             % self.safety_mode)
            elif now - self.t_arm > self.arm_timeout:
                self.state = 'IDLE'
                self.log(now, 'ARM_REFUSED', 'MANUAL_READY_TIMEOUT(mode=%s,result=%s)' % (
                    mode, st.get('mode_request_result')))
                return 0.0, 0.0, 0, 'STOP'
            elif now - self.t_req > 0.25 and mode == 'STOP':
                self.t_req = now
                return 0.0, 0.0, 0, self.manual_request
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
                                         steer_rad=self.prof['steer'], full_lock=self.prof.get('full_lock'),
                                         start_wall=time.time()))
                detail = 'target=%.2f m/s' % self.prof['speed']
                if self.ps4:
                    self.v_cmd, self.allow, self.block_t, self.prev_t = 0.0, False, now, now
                    self.brake_on = self.block_logged = False
                if self.teleop:
                    self.steer_now, self.steer_t = 0.0, now
                    self.hard_s = float(self.prof.get('teleop_max_s', 300.0))
                    detail += ' TELEOP steer max %.2f rad, rate %.2f rad/s, segment limit %.0f s' % (
                        abs(float(self.prof['steer'])), TELEOP_STEER_RATE, self.hard_s)
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
                if self.ps4:
                    self.log(now, 'DEADMAN_RELEASED', 'R1 released -> brake %g A; options to arm again' % self.brake_a)
            if reason is None and mode not in ('MANUAL_READY', 'MANUAL_ACTIVE'):
                reason = 'MUX_LEFT_MANUAL_MODE(%s,%s)' % (mode, st.get('last_stop_reason'))
            if reason is None and self.quit:
                reason = 'QUIT'  # lost terminal (keys are KILL, handled above)
            if (reason is None and not self.brake_at_range and not self.circle and not self.teleop
                    and elapsed >= self.ramp_s + self.steady_s):
                reason = 'PROFILE_DONE'
            if reason is None and elapsed >= self.hard_s:
                reason = 'HARD_TIME_LIMIT'
            if reason is None and self.circle:
                reason = self.circle_step(now, elapsed, v, w)
            if reason is None and self.ground and not self.circle and not self.teleop:
                have = self.ahead(w)
                if not math.isfinite(have):
                    reason = 'FRONT_RANGE_LOST'
                elif have <= self.brake_range(v):
                    reason = 'BRAKE_POINT(ahead %.2f m <= %.2f m)' % (have, self.brake_range(v))
            if reason:
                return self.stop(now, reason)
            if self.ps4:
                speed, steer = self.ps4_drive(now, w)
                return speed, steer, 1, None
            if not self.steady_logged and v >= float(self.prof['speed']):
                self.steady_logged = True
                self.log(now, 'STEADY_REACHED')
            if self.teleop:
                return v, self.teleop_steer(now, w.get('drive_dir')), 1, None
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
                    self.message += (' Carry the car back to the start marks, hands off %.1f s, m to check.'
                                     % (self.hands_off_s if self.fast else 2.0))
            elif self.state == 'IDLE' and self.teleop:
                self.message = '%s | teleop segment %d ended: release the keys, n to arm again, q to finish.' % (
                    detail, self.repeat)
            elif self.state == 'IDLE':
                self.message = '%s | Repeat %d/%d: release SPACE, press n to arm.' % (
                    detail, self.repeat + 1, self.prof['repeats'])
            if self.killed:
                self.message = '%s | %s' % (KILL_BANNER.replace('release SPACE and press n', 'release R1 and press '
                                                                'options (or n)') if self.ps4 else KILL_BANNER,
                                            self.message)
            return 0.0, 0.0, 0, 'STOP'
        return 0.0, 0.0, 0, None


def self_test_23nov(p, world):
    """23 Nov: converter parameter read (A), --fast / auto-live / zstd (B), teleop (E)."""
    from rcl_interfaces.msg import ParameterValue
    import t1b_stop_converter as conv

    def pv(t, **kw):
        return ParameterValue(type=t, **kw)
    names = list(T1B_PARAMS_REQUIRED + T1B_PARAMS_OPTIONAL)
    vals = [pv(4, string_value='brake'), pv(3, double_value=5.0), pv(3, double_value=2.0),
            pv(3, double_value=0.335), pv(3, double_value=-0.5), pv(3, double_value=-0.5)]
    table = [pv(8, double_array_value=[0.0, 0.335, 0.2042, 0.178, 0.28, 0.11]),
             pv(8, double_array_value=[0.0, 0.335, -0.1414, 0.526, -0.28, 0.66])]
    # gain mode as rclpy Humble answers it: the batch is EMPTY (unset tables), per name the tables are empty too
    single = dict(zip(names, [[v] for v in vals] + [[], []]))
    gain = params_from_answers(names, [], single.get)
    assert gain['method'] == 'brake' and gain['steer_table_left'] is None and t1b_params_problem(gain) is None, gain
    assert converter_steering(gain, 0.335)['mode'] == 'gain'
    tab = params_from_answers(names, vals + table, None)             # table mode: one full batch answer
    assert t1b_params_problem(tab) is None and converter_steering(tab, 0.335)['limits'] == (-0.28, 0.28)
    assert params_from_answers(names, None, None) == {}               # unreachable
    assert 'unreadable' in t1b_params_problem({})
    assert 'missing: method' in t1b_params_problem(dict(gain, method=None))
    try:
        converter_steering(dict(tab, steer_table_left=[0.0, 0.340, 0.2, 0.178]), 0.335)
        raise AssertionError('bad table accepted')
    except SystemExit as exc:
        assert 'table invalid' in str(exc)
    # auto-live (B3)
    assert live_mode(True, False, True)[1] is True and 'live mode on' in live_mode(True, False, True)[2]
    assert live_mode(True, True, True)[2] is None and live_mode(False, False, True) == (False, False, None)
    for args_ in ((False, True, True), (True, False, False), (True, True, False)):
        try:
            live_mode(*args_)
            raise AssertionError(args_)
        except SystemExit:
            pass
    bc = bag_command('/x/bag', BAG_TOPICS)
    fmt_ = bc[bc.index('--compression-format') + 1]
    assert bc[:3] == ['ros2', 'bag', 'record'] and '--compression-mode' in bc and fmt_ == 'zstd'
    assert not any(is_protected(t) for t in bc[4:] if t.startswith('/'))
    # --fast (B1): a T3 arm that the standard mode refuses (disk not clear, low visibility) arms in fast mode;
    # IMU freeze / VESC telemetry / SPACE still refuse.
    pr = resolve_profile('t3_circle', 0.3, -0.18, p)

    def tw(clear, vis, **kw):
        st = dict(mode='STOP', last_mode_request='MANUAL', manual_min_front_valid_fraction=0.75,
                  real_actuation_enabled=kw.pop('real', False))
        return world(front_m=float('nan'), sector_min_m=3.0, scan_age=0.05, st=st, gz=0.0, gz_mean_1s=0.0,
                     rf2o_pose=(0.0, 0.0, 0.0), rf2o_age=0.05, t3_clear=clear, t3_vis=vis, **kw)
    bad_c, bad_v = dict(min_m=-0.3, bearing_deg=90.0), dict(min_fraction=0.30, heading_deg=0.0)
    std = Machine(pr, p, live=False, ground_ok=True)
    std.step(0.0, 'n', False, tw(bad_c, bad_v))
    assert std.state == 'IDLE' and std.events[-1]['detail'].startswith('CIRCLE_NOT_CLEAR'), std.events[-1]
    fp = dict(pr, fast=True, hands_off_s=FAST_HANDS_OFF_S)
    fm = Machine(fp, p, live=False, ground_ok=True)
    fm.step(0.0, 'm', False, tw(None, None))
    assert fm.events[-1]['detail'].startswith('FAST: clear disk + visibility NOT checked'), fm.events[-1]
    assert fm.step(0.1, 'n', False, tw(None, None))[3] == 'MANUAL' and fm.state == 'ARMING', fm.events[-1]
    for kw, why in ((dict(imu_problem='IMU_FROZEN(x)'), 'IMU_FROZEN'), (dict(vesc_fault=3), 'VESC_FAULT'),
                    (dict(real=True), 'REAL_FLAG_MISMATCH')):
        fm2 = Machine(fp, p, live=False, ground_ok=True)
        fm2.step(0.0, 'n', False, tw(None, None, **kw))
        assert fm2.state == 'IDLE' and fm2.events[-1]['detail'].startswith(why), (why, fm2.events[-1])
    fm2 = Machine(fp, p, live=False, ground_ok=True)
    fm2.step(0.0, 'n', True, tw(None, None))
    assert fm2.events[-1]['detail'] == 'RELEASE_SPACE_FIRST'
    assert fm.hands_off_s == 0.5 and std.hands_off_s == HANDS_OFF_S
    # drive keys (E): a key change restarts the first-press window; SPACE alone behaves as before

    class KS:
        drive_keys, drive_key = TELEOP_KEYS, None
    d, ks = Deadman(0.7, 0.25), KS()
    for t_ in (0.0, 0.5, 0.53, 0.56):
        assert drive_key_event(d, ks, 'w', t_)
    assert d.held(0.80) and not d.held(0.82)                     # auto-repeat: 0.25 s gap
    assert drive_key_event(d, ks, 'a', 0.58) and d.held(1.27) and not d.held(1.29)   # new key: 0.7 s window
    assert not drive_key_event(d, ks, 'x', 0.6) and ks.drive_key == 'a'
    # teleop Machine (E)
    tp = resolve_profile('teleop', 0.5, None, p)
    tm = Machine(tp, p, live=False, ground_ok=True)
    gst = dict(mode='STOP', last_mode_request='MANUAL')

    def tg(front=0.5, mode='STOP', direction=0, **kw):
        return world(front_m=front, sector_min_m=1.2, scan_age=0.05, left_m=1.0, right_m=1.0,
                     st=dict(gst, mode=mode), drive_dir=direction, **kw)
    assert tm.step(0.0, 'n', False, tg())[3] == 'MANUAL', tm.events[-1]     # front 0.5 m: no speed-fit refusal
    tm.step(0.05, None, False, tg(mode='MANUAL_READY'))
    tm.step(0.10, None, True, tg(mode='MANUAL_READY'))
    assert tm.state == 'DRIVING' and 'TELEOP' in tm.events[-1]['detail']
    t_, steer_ = 0.10, []
    for direction, n_ in ((0, 25), (1, 40), (0, 40), (-1, 40)):
        for _ in range(n_):
            t_ += 0.02
            out = tm.step(t_, None, True, tg(front=0.9, mode='MANUAL_ACTIVE', direction=direction))
            assert tm.state == 'DRIVING', tm.events[-1]
            steer_.append(out[1])
    assert abs(out[0] - 0.5) < 1e-9                                     # ramped to the cap, no brake point
    assert abs(steer_[24]) < 1e-12 and abs(steer_[64] - 0.28) < 1e-9 and abs(steer_[104]) < 1e-9
    assert abs(steer_[-1] + 0.28) < 1e-9
    assert max(abs(b - a) for a, b in zip(steer_, steer_[1:])) <= TELEOP_STEER_RATE * 0.02 + 1e-9   # smooth
    out = tm.step(t_ + 0.02, None, False, tg(mode='MANUAL_ACTIVE'))        # release -> zero
    assert out[:3] == (0.0, 0.0, 0) and tm.events[-1]['detail'] == 'DEADMAN_RELEASED'
    tm.step(t_ + 2.0, None, False, tg())
    assert tm.state == 'IDLE' and 'teleop segment 1 ended' in tm.message, tm.message
    assert tm.step(t_ + 2.1, 'n', False, tg())[3] == 'MANUAL'              # next segment
    tm.step(t_ + 2.15, None, False, tg(mode='MANUAL_READY'))
    tm.step(t_ + 2.2, None, True, tg(mode='MANUAL_READY'))
    out = tm.step(t_ + 2.25, 'x', True, tg(mode='MANUAL_ACTIVE'))          # other key = KILL
    assert out[:3] == (0.0, 0.0, 0) and tm.killed
    out = tm.step(t_ + 2.30, None, True, tg(mode='STOP'))
    assert tm.state == 'STOPPING'
    tq = Machine(tp, p, live=False, ground_ok=True)
    tq.step(0.0, 'q', False, tg())
    assert tq.state == 'DONE'
    # teleop pre-check: table + mux cap + vesc clip
    smt = converter_steering(tab, 0.335)
    ti = teleop_precheck(tp, smt, 0.28, dict(servo_min=0.10, servo_max=0.68))
    assert abs(ti['servo_left'] - 0.11) < 1e-9 and abs(ti['servo_right'] - 0.66) < 1e-9, ti
    assert abs(conv.servo_command(0.335, -0.5, -0.5, 0.0, smt['tables']) - 0.335) < 1e-12
    for args_, why in (((converter_steering(gain, 0.335), 0.28, dict(servo_min=0.1, servo_max=0.68)), 'TABLE'),
                       ((smt, 0.10, dict(servo_min=0.1, servo_max=0.68)), 'max_steer_rad'),
                       ((smt, 0.28, dict(servo_min=0.27, servo_max=0.37)), 'clip')):
        try:
            teleop_precheck(tp, *args_)
            raise AssertionError(why)
        except SystemExit as exc:
            assert why in str(exc), (why, exc)
    # 28 Nov: PROVISIONAL teleop table (centre 0.36, linear to the full clip 0.10 / 0.68) with its servo_offset override
    import yaml as _yaml
    tpv = _yaml.safe_load((vp.CONFIG / 't1b_converter_teleop_provisional.yaml').read_text())[
        't1b_stop_converter']['ros__parameters']
    assert tpv['servo_offset'] == 0.36 and t1b_params_problem(dict(gain, **tpv)) is None
    smp = converter_steering(dict(gain, **tpv), 0.335)
    assert smp['mode'] == 'table' and smp['offset'] == 0.36 and smp['limits'] == (-0.28, 0.28), smp
    tip = teleop_precheck(tp, smp, 0.28, dict(servo_min=0.10, servo_max=0.68))      # vesc_teleop_erpm12620.yaml clip
    assert abs(tip['servo_left'] - 0.10) < 1e-9 and abs(tip['servo_right'] - 0.68) < 1e-9, tip
    assert abs(conv.servo_command(0.36, -0.5, -0.5, 0.0, smp['tables']) - 0.36) < 1e-12
    assert abs(conv.servo_command(0.36, -0.5, -0.5, 0.14, smp['tables']) - 0.23) < 1e-9          # linear halfway
    assert abs(conv.servo_command(0.36, -0.5, -0.5, -0.14, smp['tables']) - 0.52) < 1e-9
    try:                                                     # table without its offset override -> refused
        converter_steering(dict(gain, steer_table_left=tpv['steer_table_left'],
                                steer_table_right=tpv['steer_table_right']), 0.335)
        raise AssertionError('provisional table accepted with servo_offset 0.335')
    except SystemExit as exc:
        assert 'table invalid' in str(exc), exc
    try:                                                     # a tighter VESC clip than 0.10-0.68 -> refused
        teleop_precheck(tp, smp, 0.28, dict(servo_min=0.11, servo_max=0.66))
        raise AssertionError('provisional table accepted with the T3 0.11 clip')
    except SystemExit as exc:
        assert 'clip' in str(exc), exc
    print('PASS: 23 Nov: converter parameter read (gain mode = empty Humble batch -> per name, tables None; table')
    print('      checked; unreachable / missing named), auto-live, zstd bag command, --fast (no disk / visibility')
    print('      refusal, IMU / VESC / SPACE kept, hands-off 0.5 s), drive keys, teleop (no brake point, smooth')
    print('      steering 0.7 rad/s to +/-0.28, release = zero, KILL, segments until q, table / cap / clip pre-check);')
    print('      28 Nov provisional teleop table (0.36 centre, 0.10 / 0.68 ends, offset override needed, clip 0.10-0.68).')


PS4_REST = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]   # DS4 under joy_node (evdev): 8 axes, triggers rest at +1
PS4_NB = 13                                             # 13 buttons


def ps4_btn(*idx):
    return [1 if i in idx else 0 for i in range(PS4_NB)]


def ps4_axes(**kv):
    """axes with overrides by index: ps4_axes(a0=1.0, a4=-1.0)."""
    ax = list(PS4_REST)
    for k, v in kv.items():
        ax[int(k[1:])] = v
    return ax


def self_test_23nov_eve(p, world):
    """23 Nov eve (updated 28 Nov for the ps4_teleop.yaml layout): PS4 pad, controller check, teleop_ps4 Machine basics,
    low-visibility rule, keyboard KILL."""
    cfg = vp.load_ps4()
    assert (cfg['deadman_button'], cfg['brake_button'], cfg['arm_button'], cfg['steer_axis'], cfg['throttle_axis']) == (
        5, 4, 9, 0, 4) and 1 in cfg['kill_buttons'] and cfg['throttle_mode'] == 'fixed', cfg
    assert float(cfg['stick_deadzone']) == 0.10 and float(cfg['joy_node']['deadzone']) == 0.0
    btn, rest = ps4_btn, PS4_REST
    pad = Ps4Pad(cfg)
    assert pad.problem(0.0).startswith('PS4_STALE') and not pad.held(0.0)
    pad.update(0.00, btn(5, 9), rest, 0.005)       # first message with R1 + options down: no key from it
    assert pad.get_key() is None and pad.held(0.05) and not pad.held(0.21)   # joy_ttl 0.20 s
    assert pad.problem(0.25).startswith('PS4_STALE')
    pad.update(0.02, btn(), rest, 0.0)
    pad.update(0.04, btn(9), rest, 0.0)            # options rising edge = n
    pad.update(0.06, btn(9, 1), rest, 0.0)         # circle = KILL
    pad.update(0.08, btn(2), rest, 0.0)            # triangle = m
    assert [pad.get_key() for _ in range(4)] == ['n', 'PS4_KILL', 'm', None]
    pad.update(0.10, [0] * 4, rest, 0.0)
    assert pad.problem(0.11).startswith('PS4_TOO_FEW_BUTTONS') and not pad.held(0.11)
    pad.update(0.12, btn(5), rest[:3], 0.0)
    assert pad.problem(0.13).startswith('PS4_TOO_FEW_AXES') and not pad.held(0.13)
    for x, want in ((0.05, 0.0), (-0.05, 0.0), (1.0, 1.0), (-1.0, -1.0), (0.55, 0.5), (1.3, 1.0)):
        pad.update(0.14, btn(5), ps4_axes(a0=x), 0.0)
        assert abs(pad.steer_frac() - want) < 1e-9, (x, pad.steer_frac())
    pad.steer_sign = -1.0
    assert abs(pad.steer_frac() + 1.0) < 1e-9
    # controller check on a scripted controller (fake clock): rest, LEFT stick left 2..3 s, RIGHT stick up 3.5..4.5 s,
    # R1 5.0..5.3, L1 5.6..5.9, circle 6.2..6.5
    clk = [0.0]

    def script(t, up_axis=4, up_val=1.0, r1=5, l1=4, circ=1):
        ax = list(rest)
        if 2.0 <= t < 3.0:
            ax[0] = 1.0
        if 3.5 <= t < 4.5:
            ax[up_axis] = up_val
        b = (btn(r1) if 5.0 <= t < 5.3 else btn(l1) if 5.6 <= t < 5.9 else btn(circ) if 6.2 <= t < 6.5 else btn())
        return b, ax

    def make(gap=0.02, script_=script):
        pad_ = Ps4Pad(cfg)
        nxt = [0.0]

        def sleep(dt):
            t_end = clk[0] + dt
            while nxt[0] <= t_end:
                clk[0] = nxt[0]
                pad_.update(clk[0], *script_(clk[0]), lag=0.003)
                nxt[0] += gap
            clk[0] = t_end
        return pad_, sleep
    clk[0] = 0.0
    pad2, sl = make()
    info = ps4_controller_check(pad2, cfg, lambda: clk[0], sl, lambda s_: None)
    assert (info['steer_axis'], info['throttle_axis'], info['deadman_button'], info['brake_button']) == (0, 4, 5, 4), info
    assert pad2.get_key() is None                   # circle during the check never kills later
    for gap, script_, why in (
            (0.15, script, 'max gap'),
            (0.02, lambda t: (btn(5), rest), 'pressed at rest'),
            (0.02, lambda t: script(t, up_val=-1.0), 'set throttle_axis: 4 and throttle_sign: -1.0'),   # up reads -
            (0.02, lambda t: script(t, up_axis=3), 'set throttle_axis: 3'),             # HIDAPI order
            (0.02, lambda t: script(t, r1=10), 'set deadman_button: 10'),
            (0.02, lambda t: script(t, l1=9), 'set brake_button: 9')):
        clk[0] = 0.0
        pad3, sl3 = make(gap, script_)
        try:
            ps4_controller_check(pad3, dict(cfg, check_timeout_s=8.0), lambda: clk[0], sl3, lambda s_: None)
            raise AssertionError(why)
        except SystemExit as exc:
            assert why in str(exc), (why, exc)
    # teleop_ps4 Machine: arm from the pad, stale controller refuses arm / zeroes, R1 held at arm refused, steering
    tp = resolve_profile('teleop_ps4', 0.3, None, p)
    assert tp['ps4'] and tp['teleop'] and tp['converter'] == 't1b_stop_converter' and tp['stop_method'] == 'brake5A'
    assert tp['throttle_mode'] == 'fixed' and 'steps' not in tp
    tm = Machine(tp, p, live=False, ground_ok=True)
    gst = dict(mode='STOP', last_mode_request='MANUAL')

    def tg(mode='STOP', frac=0.0, prob=None, thr=0.0):
        return world(front_m=2.0, sector_min_m=1.2, scan_age=0.05, left_m=1.0, right_m=1.0,
                     st=dict(gst, mode=mode), drive_dir=frac, ps4_problem=prob, ps4_thr=thr)
    tm.step(0.0, 'n', False, tg(prob='PS4_STALE(0.30 s)'))
    assert tm.state == 'IDLE' and tm.events[-1]['detail'].startswith('PS4_STALE'), tm.events[-1]
    assert [e['event'] for e in tm.events] == ['CONTROLLER_LOST', 'ARM_REFUSED'], tm.events
    tm.step(0.05, 'n', True, tg())
    assert tm.events[-1]['detail'] == 'RELEASE_DEADMAN_FIRST(R1)'   # R1 held at arm: refused
    assert tm.events[-2]['event'] == 'CONTROLLER_OK'
    assert tm.step(0.1, 'n', False, tg())[3] == 'MANUAL'          # autonomy safety mode: plain MANUAL
    tm.step(0.15, None, False, tg('MANUAL_READY'))
    assert tm.events[-1]['event'] == 'ARMED' and tm.events[-2]['event'] == 'MANUAL_READY'
    tm.step(0.2, None, True, tg('MANUAL_READY'))
    assert tm.state == 'DRIVING'
    t_ = 0.2
    for _ in range(50):
        t_ += 0.02
        out = tm.step(t_, None, True, tg('MANUAL_ACTIVE', frac=0.5, thr=1.0))
    assert abs(out[1] - 0.14) < 1e-9 and abs(out[0] - 0.3) < 1e-9, out   # half stick = 0.14 rad, fixed 0.3 m/s
    out = tm.step(t_ + 0.02, None, True, tg('MANUAL_ACTIVE', prob='PS4_STALE(0.21 s)', thr=1.0))
    assert out[:3] == (0.0, 0.0, 0) and tm.events[-1]['detail'].startswith('PS4_STALE'), tm.events[-1]
    assert tm.events[-2]['event'] == 'CONTROLLER_LOST'
    tk = Machine(tp, p, live=False, ground_ok=True)
    tk.step(0.0, 'n', False, tg())
    tk.step(0.05, None, False, tg('MANUAL_READY'))
    tk.step(0.1, None, True, tg('MANUAL_READY'))
    out = tk.step(0.12, 'PS4_KILL', True, tg('MANUAL_ACTIVE'))
    assert out[:3] == (0.0, 0.0, 0) and tk.killed and 'PS4_KILL' in tk.events[-2]['detail'], tk.events[-2:]
    assert 'release R1 and press options' in tk.message and 'hold R1' in tk.events[1]['detail'], tk.message
    # keyboard in teleop_ps4: no drive key (SPACE is a pending key = KILL while armed)

    class KS:
        drive_keys, drive_key = {}, None
    assert not drive_key_event(Deadman(0.7, 0.25), KS(), 'space', 0.0)
    lv10 = dict(manual_low_visibility_enabled=True, manual_min_front_valid_fraction=0.10)
    assert lowvis_gate('teleop_ps4', lv10, True, 0.3) == (True, None)
    assert '<= 0.3' in lowvis_gate('teleop_ps4', lv10, True, 0.5)[1]
    assert '/ps4/joy' in BAG_TOPICS and not is_protected('/ps4/joy')
    print('PASS: 23 Nov eve (28 Nov layout): PS4 pad (rising edges only, first message never arms, TTL 0.20 s, too few')
    print('      buttons / axes, deadzone / sign / clip), controller check (configured steer / throttle axes + signs,')
    print('      R1 / L1 / circle verified; slow joy_node, button at rest, inverted or HIDAPI axis, wrong R1 / L1 index')
    print('      refused with the index seen), teleop_ps4 (stale pad refuses arm / zeroes, R1 held at arm refused, ARMED,')
    print('      half stick = 0.14 rad, PS4 KILL), keyboard SPACE = KILL, low-visibility <= 0.3 m/s.')


def self_test_28nov(p, world):
    """28 Nov: mux safety_mode gate, teleop speed (default / floor / ceiling, stack check), VESC yaml match, stopping
    distance, throttle mapping, L1 brake, no jump start, stale joy / old stamp, link heartbeat, events."""
    cfg = vp.load_ps4()
    kb_prof, ps4_prof = PROFILES['teleop'], PROFILES['teleop_ps4']
    # keyboard runner + mux safety_mode teleop is refused (the mux refuses its plain MANUAL too)
    assert safety_mode_gate(kb_prof, dict(safety_mode='teleop'))[1].startswith('mux safety_mode teleop')
    assert 'PS4 controller only' in safety_mode_gate(PROFILES['t4_straight'], dict(safety_mode='teleop'))[1]
    assert safety_mode_gate(ps4_prof, dict(safety_mode='teleop')) == ('teleop', None)
    assert safety_mode_gate(kb_prof, dict(safety_mode='autonomy')) == ('autonomy', None)
    assert safety_mode_gate(kb_prof, {}) == ('autonomy', None)              # old mux: autonomy (all gates)
    assert safety_mode_gate(ps4_prof, dict(safety_mode='fast'))[1]
    # --speed: default 0.3, 1.5 accepted, 1.6 refused (hard ceiling in code), below the 0.15 floor refused
    assert resolve_profile('teleop_ps4', None, None, p)['speed'] == 0.3
    assert resolve_profile('teleop_ps4', 1.5, None, p)['speed'] == 1.5
    for bad, why in ((1.6, 'hard ceiling of 1.50'), (1.5000001, 'hard ceiling'), (0.1, 'below 0.15'),
                     (float('nan'), 'hard ceiling')):
        try:
            resolve_profile('teleop_ps4', bad, None, p)
            raise AssertionError('--speed %r accepted' % bad)
        except SystemExit as exc:
            assert why in str(exc), (bad, exc)
    try:
        teleop_speed(1.4, dict(cfg, speed_ceiling_mps=9.0)) and teleop_speed(1.6, dict(cfg, speed_ceiling_mps=9.0))
        raise AssertionError('yaml ceiling above the code ceiling honoured')
    except SystemExit:
        pass
    assert resolve_profile('teleop_ps4', 0.5, None, p, throttle_mode='proportional')['throttle_mode'] == 'proportional'
    for kw in (dict(steer=0.1), dict(throttle_mode='turbo')):
        try:
            resolve_profile('teleop_ps4', 0.3, kw.get('steer'), p, throttle_mode=kw.get('throttle_mode'))
            raise AssertionError(kw)
        except SystemExit:
            pass
    try:
        resolve_profile('teleop', 0.3, None, p, throttle_mode='fixed')
        raise AssertionError('--throttle-mode accepted for the keyboard teleop')
    except SystemExit:
        pass
    g = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    assert teleop_speed_problem(1.5, 1.5, 12620.0, g) is None          # 12621 ERPM vs 12620 (rounding allowed)
    assert teleop_speed_problem(0.3, 0.3, 2780.0, g) is None           # the T3 yaml still fits 0.3
    assert 'vesc_teleop_erpm12620.yaml' in teleop_speed_problem(0.5, 0.5, 2780.0, g)
    assert 'launch max_speed_mps:=1.00' in teleop_speed_problem(1.0, 0.3, 12620.0, g)
    assert 'unreadable' in teleop_speed_problem(0.3, 0.3, None, g)
    # VESC yaml in use, matched from the vesc_driver parameters
    import yaml
    for name in ('vesc_teleop_erpm12620.yaml', 'vesc_t3_erpm2780.yaml'):
        rp = yaml.safe_load((vp.CONFIG / name).read_text())['vesc_driver']['ros__parameters']
        assert vesc_yaml_match({k: rp[k] for k in VESC_MATCH_KEYS}, vp.CONFIG) == [name], name
    tele = yaml.safe_load((vp.CONFIG / 'vesc_teleop_erpm12620.yaml').read_text())['vesc_driver']['ros__parameters']
    assert (tele['speed_max'], tele['servo_min'], tele['servo_max'], tele['brake_max']) == (12620.0, 0.10, 0.68, 10.0)
    assert abs(1.5 * g - tele['speed_max']) <= SPEED_MAX_ROUNDING
    assert vesc_yaml_match(dict(speed_max=1.0), vp.CONFIG) == []
    # floor brake-distance gate above 1.0 m/s
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        assert teleop_brake_gate(td, 1.0, 0.9)[0]
        assert 'brake-distance check' in teleop_brake_gate(td, 1.2, 0.9)[1]
        run = Path(td) / '20261201_120000_teleop_ps4_v1.00_brake5A_erpmmax12620_armed'
        run.mkdir()
        (run / 'teleop_onset.json').write_text(json.dumps([dict(event='BRAKE_BUTTON', speed_mps=0.98,
                                                                vesc_distance_m=0.80)]))
        ok, why = teleop_brake_gate(td, 1.5, 0.9)
        assert not ok and '0.80 m from 0.98 m/s' in why, why               # 0.80 > 1.25 x 0.534 + 0.05
        (run / 'teleop_onset.json').write_text(json.dumps([dict(event='BRAKE_BUTTON', speed_mps=0.98,
                                                                vesc_distance_m=0.55)]))
        assert teleop_brake_gate(td, 1.5, 0.9)[0]
        (run / 'teleop_onset.json').write_text(json.dumps([dict(event='BRAKE_BUTTON', speed_mps=0.5,
                                                                vesc_distance_m=0.10)]))
        assert not teleop_brake_gate(td, 1.5, 0.9)[0]                      # a 0.5 m/s stop does not count
    # stopping distance v*0.25 + v^2/(2*0.9)
    assert abs(stopping_distance(0.3, 0.25, 0.9) - 0.125) < 1e-9
    assert abs(stopping_distance(1.5, 0.25, 0.9) - 1.625) < 1e-9
    tp = resolve_profile('teleop_ps4', 1.0, None, p)
    tp['safety_mode'] = 'teleop'
    ti = teleop_start_info(tp, dict(brake_current_a=5.0), ['vesc_teleop_erpm12620.yaml'], {})
    for s in ('safety_mode=teleop', 'throttle_mode=fixed', 'teleop_max_speed_mps=1.00', 'brake_current_a=5.0',
              'vesc_yaml=vesc_teleop_erpm12620.yaml', 'stop_estimate_m=0.81', 'floor-verified only at 0.5 m/s'):
        assert s in ti['start_line'], (s, ti['start_line'])
    assert 'floor-verified only at 0.5 m/s' in teleop_start_text(tp, ti) and 'YOU are' in teleop_start_text(tp, ti)
    # throttle mapping: fixed vs proportional, the 0.15 m/s floor
    dz, ft, vmin = 0.10, 0.20, 0.15
    assert throttle_target(0.0, 'fixed', 0.3, dz, ft, vmin) == (0.0, 'CENTRE')
    assert throttle_target(0.09, 'fixed', 0.3, dz, ft, vmin) == (0.0, 'CENTRE')
    assert throttle_target(-0.5, 'fixed', 0.3, dz, ft, vmin) == (0.0, 'DOWN')
    assert throttle_target(0.19, 'fixed', 0.3, dz, ft, vmin) == (0.0, 'BELOW_FIXED')
    assert throttle_target(0.21, 'fixed', 0.3, dz, ft, vmin) == (0.3, None)
    assert throttle_target(1.0, 'fixed', 1.5, dz, ft, vmin) == (1.5, None)
    assert throttle_target(float('nan'), 'fixed', 0.3, dz, ft, vmin) == (0.0, 'NONFINITE')
    assert throttle_target(1.0, 'proportional', 1.0, dz, ft, vmin) == (1.0, None)
    v, why = throttle_target(0.55, 'proportional', 1.0, dz, ft, vmin)
    assert abs(v - 0.5) < 1e-9 and why is None
    v, why = throttle_target(0.235, 'proportional', 1.0, dz, ft, vmin)
    assert abs(v - 0.15) < 1e-9 and why is None                          # exactly the floor: sent
    assert throttle_target(0.2, 'proportional', 1.0, dz, ft, vmin) == (0.0, 'BELOW_MIN')   # 0.11 m/s: a stop
    assert throttle_target(0.5, 'proportional', 0.3, dz, ft, vmin) == (0.0, 'BELOW_MIN')   # 0.133 m/s at max 0.3
    assert abs(throttle_target(1.0, 'proportional', 0.3, dz, ft, vmin)[0] - 0.3) < 1e-9

    # Machine: teleop safety mode, MANUAL_PS4 handshake, L1, no jump start, stick stops, WARN, events
    tp = resolve_profile('teleop_ps4', 0.3, None, p)
    tp.update(safety_mode='teleop', manual_request='MANUAL_PS4')

    def tw(mode='MANUAL_ACTIVE', thr=0.0, brake=False, brake_at=None, centre_at=None, prob=None, req='MANUAL_PS4',
           **st):
        return world(front_m=float('nan'), sector_min_m=0.3, scan_age=0.05, left_m=1.0, right_m=1.0,
                     st=dict(dict(mode=mode, last_mode_request=req, safety_mode='teleop'), **st),
                     drive_dir=0.0, ps4_problem=prob, ps4_thr=thr, ps4_brake_down=brake, ps4_brake_at=brake_at,
                     ps4_centre_at=centre_at, ps4_joy_wall=1000.0, ps4_link_wall=1000.0)

    def armed(m, t0=0.0, thr=0.0):
        assert m.step(t0, 'n', False, tw('STOP', req=None))[3] == 'MANUAL_PS4'
        m.step(t0 + 0.02, None, False, tw('MANUAL_READY', req='MANUAL'))        # a plain MANUAL is not ours
        assert m.state == 'ARMING'
        m.step(t0 + 0.04, None, False, tw('MANUAL_READY'))
        assert m.state == 'READY' and m.events[-1]['event'] == 'ARMED'
        m.step(t0 + 0.06, None, True, tw('MANUAL_READY', thr=thr))
        assert m.state == 'DRIVING'
        return t0 + 0.06

    def drive(m, t, n, held=True, **kw):
        out = None
        for _ in range(n):
            t += 0.02
            out = m.step(t, None, held, tw(**kw))
        return t, out
    m = Machine(tp, p, live=False, ground_ok=True)
    # the mux sector object (0.3 m) and the lost front range do not refuse the arm or stop teleop
    t = armed(m, thr=1.0)                                  # R1 pressed with the stick already up: no jump start
    t, out = drive(m, t, 10, thr=1.0)
    assert out[0] == 0.0 and out[2] == 1 and m.events[-1]['event'] == 'NO_JUMP_START', (out, m.events[-1])
    t, out = drive(m, t, 1, thr=0.0)                       # centre
    t, out = drive(m, t, 50, thr=1.0)
    assert abs(out[0] - 0.3) < 1e-9 and m.state == 'DRIVING', out          # ramp 0.5 m/s^2 -> 0.3 in 0.6 s
    # L1 overrides a full stick; car stays armed (deadman 1, mux MANUAL_ACTIVE, Machine DRIVING)
    t, out = drive(m, t, 5, thr=1.0, brake=True, brake_at=t + 0.1)
    assert out[0] == 0.0 and out[2] == 1 and m.state == 'DRIVING', out
    bb = [e for e in m.events if e['event'] == 'BRAKE_BUTTON']
    assert len(bb) == 1 and 'L1 pressed' in bb[0]['detail'] and '0.30 m/s' in bb[0]['detail'], bb
    # L1 released, stick still up: no motion until the stick passed through the centre
    t_rel = t
    t, out = drive(m, t, 30, thr=1.0, brake_at=t_rel)
    assert out[0] == 0.0 and m.events[-1]['event'] == 'NO_JUMP_START', m.events[-1]
    assert [e['detail'] for e in m.events if e['event'] == 'BRAKE_BUTTON'][-1].startswith('L1 released')
    t, out = drive(m, t, 5, thr=1.0, brake_at=t_rel, centre_at=t_rel - 0.5)   # centre before the release: no
    assert out[0] == 0.0
    t, out = drive(m, t, 1, thr=1.0, brake_at=t_rel, centre_at=t)             # flick through the centre: yes
    t, out = drive(m, t, 40, thr=1.0, brake_at=t_rel, centre_at=t - 0.02)
    assert abs(out[0] - 0.3) < 1e-9, out
    # an L1 tap between two ticks (brake_at after the previous tick, button already up) still brakes
    t, out = drive(m, t, 1, thr=1.0, brake_at=t + 0.01)
    assert out[0] == 0.0 and m.events[-1]['event'] == 'BRAKE_BUTTON', m.events[-1]
    t, out = drive(m, t, 1, thr=0.0, brake_at=t - 0.02)
    t, out = drive(m, t, 40, thr=1.0)
    assert abs(out[0] - 0.3) < 1e-9
    # stick in the deadzone = brake (no block: the next push starts at once); stick down = brake + block
    t, out = drive(m, t, 1, thr=0.05)
    assert out[0] == 0.0 and m.events[-1]['event'] == 'STICK_STOP' and 'centre' in m.events[-1]['detail']
    t, out = drive(m, t, 1, thr=1.0)
    assert out[0] > 0.0, out
    t, out = drive(m, t, 40, thr=1.0)
    t, out = drive(m, t, 1, thr=-0.6)
    assert out[0] == 0.0 and 'down' in m.events[-1]['detail'], m.events[-1]
    t, out = drive(m, t, 10, thr=1.0)                      # down -> up without the centre: blocked
    assert out[0] == 0.0
    t, out = drive(m, t, 1, thr=0.0)
    t, out = drive(m, t, 2, thr=1.0)
    assert out[0] > 0.0
    # WARN events (mux teleop warn fields) while driving: logged, no stop
    t, out = drive(m, t, 1, thr=1.0, teleop_warn='FRONT_UNKNOWN', front_valid_fraction=0.17)
    assert m.state == 'DRIVING' and out[0] > 0 and m.events[-1]['event'] == 'WARN' and 'no stop' in m.events[-1][
        'detail'], m.events[-1]
    t, out = drive(m, t, 1, thr=1.0, teleop_warn='OBSTACLE_NEAR', nearest_front_m=0.5)
    assert m.events[-1]['event'] == 'WARN' and 'OBSTACLE_NEAR' in m.events[-1]['detail']
    t, out = drive(m, t, 1, thr=1.0)
    assert m.events[-1]['event'] == 'WARN_CLEAR'
    # R1 released -> DEADMAN_RELEASED, zero (converter brake), segment ends; options needed again
    t, out = drive(m, t, 1, held=False, thr=1.0)
    assert out[:3] == (0.0, 0.0, 0) and m.state == 'STOPPING'
    assert [e['event'] for e in m.events[-2:]] == ['DEADMAN_RELEASED', 'ZERO_COMMAND'], m.events[-2:]
    t, out = drive(m, t, 60, held=True, thr=1.0, mode='STOP')
    assert m.state == 'IDLE' and out[0] == 0.0
    # proportional: below the 0.15 m/s floor = stop and block
    pp = resolve_profile('teleop_ps4', 1.0, None, p, throttle_mode='proportional')
    pp.update(safety_mode='teleop', manual_request='MANUAL_PS4')
    mp = Machine(pp, p, live=False, ground_ok=True)
    t = armed(mp, 0.0)
    t, out = drive(mp, t, 200, thr=0.55)
    assert abs(out[0] - 0.5) < 1e-9, out
    t, out = drive(mp, t, 1, thr=0.2)
    assert out[0] == 0.0 and 'floor' in mp.events[-1]['detail'], mp.events[-1]
    t, out = drive(mp, t, 5, thr=0.55)
    assert out[0] == 0.0                                       # through 0.2 -> 0.55 without the centre: blocked
    # circle (KILL) latches until re-arm: R1 alone never drives again, options with R1 held refused
    mk = Machine(tp, p, live=False, ground_ok=True)
    t = armed(mk, 0.0)
    t, out = drive(mk, t, 1, thr=0.0)
    t, out = drive(mk, t, 30, thr=1.0)
    assert out[0] > 0
    out = mk.step(t + 0.02, 'PS4_KILL', True, tw(thr=1.0))
    assert out[:3] == (0.0, 0.0, 0) and mk.killed and mk.events[-1]['event'] == 'KILL'
    t, out = drive(mk, t + 0.02, 80, thr=1.0, mode='STOP')
    assert mk.state == 'IDLE' and out[:3] == (0.0, 0.0, 0) and mk.killed
    mk.step(t + 0.02, 'n', True, tw('STOP', thr=1.0))
    assert mk.state == 'IDLE' and mk.events[-1]['detail'] == 'RELEASE_DEADMAN_FIRST(R1)' and mk.killed
    assert mk.step(t + 0.04, 'n', False, tw('STOP'))[3] == 'MANUAL_PS4' and not mk.killed
    # stale joy: no new message -> zero within joy_ttl + one runner tick; old stamp -> zero at the next tick
    pad = Ps4Pad(cfg)
    ms = Machine(tp, p, live=False, ground_ok=True)
    t = armed(ms, 0.0)
    pad.update(t, ps4_btn(5), PS4_REST, 0.004)
    t, out = drive(ms, t, 1, thr=0.0)
    t_last = t
    pad.update(t_last, ps4_btn(5), ps4_axes(a4=1.0), 0.004)          # last message, R1 + stick up; then silence
    t_zero = None
    while t < t_last + 1.0:
        t += 0.02
        out = ms.step(t, None, pad.held(t), tw(thr=1.0, prob=pad.problem(t)))
        if out[0] == 0.0 and ms.state == 'STOPPING':
            t_zero = t
            break
    assert t_zero is not None and t_zero - t_last <= float(cfg['joy_ttl_s']) + 0.02 + 1e-9, (t_zero, t_last)
    assert [e['event'] for e in ms.events[-2:]] == ['CONTROLLER_LOST', 'ZERO_COMMAND'], ms.events[-2:]
    assert 'last_joy_wall=1000.0' in ms.events[-2]['detail']
    # brake onset bound: runner zero + mux tick (0.05 s) + converter tick (0.02 s) < 0.3 s
    assert (t_zero - t_last) + 0.05 + 0.02 < 0.3, t_zero - t_last
    pad.update(5.0, ps4_btn(5), PS4_REST, 0.26)                         # delivered 0.26 s late (old stamp)
    assert pad.problem(5.0).startswith('PS4_STAMP_OLD') and not pad.held(5.0)
    pad.update(5.1, ps4_btn(5), PS4_REST, 0.20)
    assert pad.held(5.1) and pad.problem(5.149) is None and pad.problem(5.15).startswith('PS4_STAMP_OLD')
    pad.update(5.2, ps4_btn(5), PS4_REST, None)
    assert pad.problem(5.2).startswith('PS4_NO_STAMP')
    pad.update(5.3, ps4_btn(5), PS4_REST, -0.3)                         # stamp from the future (clock mismatch)
    assert pad.problem(5.3).startswith('PS4_STAMP_OLD')
    # centre_at / brake_at from every message (flick, tap between ticks)
    pad.update(6.0, ps4_btn(5), ps4_axes(a4=0.02), 0.0)
    pad.update(6.004, ps4_btn(5, 4), ps4_axes(a4=0.9), 0.0)
    pad.update(6.008, ps4_btn(5), ps4_axes(a4=0.9), 0.0)
    mi = pad.machine_inputs()
    assert mi['ps4_centre_at'] == 6.0 and mi['ps4_brake_at'] == 6.004 and not mi['ps4_brake_down']
    assert abs(mi['ps4_thr'] - 0.9) < 1e-9
    pad.thr_sign = -1.0
    assert abs(pad.machine_inputs()['ps4_thr'] + 0.9) < 1e-9
    # link heartbeat: Bluetooth drop with joy_node still autorepeating (the hazard), device removal, real fd reader
    link = LinkWatch(0.20)
    pl = Ps4Pad(cfg, link)
    assert pl.problem(0.0).startswith('PS4_STALE')
    pl.update(0.0, ps4_btn(5), PS4_REST, 0.0)
    assert pl.problem(0.0).startswith('PS4_LINK_LOST(no motion-sensor event yet)')
    link.event(0.0)
    tt = 0.0
    while tt < 0.18:                                      # joy_node autorepeat keeps /ps4/joy fresh, link quiet
        tt += 0.02
        pl.update(tt, ps4_btn(5), PS4_REST, 0.002)
    assert pl.held(0.18) and pl.problem(0.199) is None
    assert pl.problem(0.20).startswith('PS4_LINK_LOST(no report') and not pl.held(0.20)
    link.set_lost('device closed')
    assert 'device closed' in pl.problem(0.2)
    r_fd, w_fd = os.pipe()
    lw = LinkWatch(0.2)
    th = threading.Thread(target=lw.read_loop, args=(r_fd,), daemon=True)
    th.start()
    for k in range(5):
        os.write(w_fd, INPUT_EVENT.pack(0, k, 4, 5, k) + INPUT_EVENT.pack(0, k, 0, 0, 0))   # MSC_TIMESTAMP + SYN
        time.sleep(0.01)
    t_w = time.time() + 1.0
    while lw.count < 1 and time.time() < t_w:
        time.sleep(0.01)
    assert lw.count >= 1 and lw.problem(time.monotonic()) is None, lw.stats()
    os.close(w_fd)
    th.join(1.0)
    assert lw.problem(time.monotonic()) == 'PS4_LINK_LOST(device closed)', lw.problem(time.monotonic())
    os.close(r_fd)
    assert INPUT_EVENT.size == 24
    devs = ('I: Bus=0005 Vendor=054c Product=09cc Version=8100\nN: Name="Wireless Controller"\nP: Phys=x\n'
            'H: Handlers=event5 js0 \n\n'
            'I: Bus=0005 Vendor=054c Product=09cc Version=8100\nN: Name="Wireless Controller Motion Sensors"\n'
            'H: Handlers=event6 \n\n'
            'I: Bus=0005 Vendor=054c Product=09cc Version=8100\nN: Name="Wireless Controller Touchpad"\n'
            'H: Handlers=mouse2 event7 \n\n'
            'I: Bus=0011 Vendor=0001 Product=0001 Version=ab41\nN: Name="AT Translated Set 2 keyboard"\n'
            'H: Handlers=sysrq kbd event3 \n')
    assert ps4_input_devices(devs, 'Motion Sensors', '054c') == dict(
        bus='0005', gamepad='Wireless Controller', motion='/dev/input/event6')
    usb = devs.replace('Bus=0005', 'Bus=0003').replace('Wireless Controller Motion Sensors', 'Other')
    assert ps4_input_devices(usb, 'Motion Sensors', '054c')['bus'] == '0003'
    assert ps4_input_devices('', 'Motion Sensors', '054c') == dict(bus=None, gamepad=None, motion=None)
    tmp = Path(os.environ.get('TMPDIR', '/tmp'))
    probe = tmp / ('ps4_devices_selftest_%d' % os.getpid())
    try:
        bt_nomotion = devs.split('\n\n')[0]
        for text, synth, why in ((bt_nomotion, False, 'Use the USB cable'), ('', False, 'no Sony controller'),
                                 (devs, True, 'real controller is connected')):
            probe.write_text(text)
            try:
                ps4_link_setup(Ps4Pad(cfg), cfg, synth, str(probe))
                raise AssertionError(why)
            except SystemExit as exc:
                assert why in str(exc), (why, exc)
        probe.write_text('')
        assert 'synthetic' in ps4_link_setup(Ps4Pad(cfg), cfg, True, str(probe))['link']
        probe.write_text(usb.split('\n\n')[0])
        assert 'USB' in ps4_link_setup(Ps4Pad(cfg), cfg, False, str(probe))['link']
    finally:
        if probe.exists():
            probe.unlink()
    print('PASS: 28 Nov: keyboard runner + mux safety_mode teleop refused (old mux = autonomy); --speed default 0.3,')
    print('      1.5 accepted, 1.6 / NaN / 0.1 refused (code ceiling over the yaml); mux cap + VESC speed_max check')
    print('      (vesc_teleop_erpm12620 for > 0.33 m/s); VESC yaml matched by parameters; stopping distance 0.125 /')
    print('      1.625 m + floor-verified-at-0.5 note; fixed vs proportional + 0.15 m/s floor; MANUAL_PS4 handshake;')
    print('      L1 overrides a full stick and stays armed; no jump start (arm, L1 release, stick down, below floor;')
    print('      flick through the centre counted); deadzone / down = brake; WARN events without a stop; R1 release;')
    print('      circle latched until re-arm; stale joy zero <= 0.22 s (+ mux + converter < 0.3 s), old / no / future')
    print('      stamp; link heartbeat (autorepeat hides the drop, the heartbeat does not; fd reader; device closed);')
    print('      /proc/bus/input/devices parse; Bluetooth without heartbeat / no controller / synthetic refusals;')
    print('      --speed > 1.0 only after an analysed L1 stop from >= 0.9 m/s within 1.25 x the estimate.')


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
    # per-side limits (21 Nov): left <= 0.27, right <= 0.325; exactly one of --steer / --full-lock
    for bad_steer, lock_ in ((None, None), (0.04, None), (0.28, None), (-0.33, None), (float('nan'), None),
                             (0.2, 'left'), (None, 'up')):
        try:
            resolve_profile('t3_circle', 0.5, bad_steer, p, lock_)
        except SystemExit:
            continue
        raise AssertionError('t3 resolved steer %s / full lock %s' % (bad_steer, lock_))
    for ok_steer in (0.27, -0.31, -0.325, 0.05):
        pr_ = resolve_profile('t3_circle', 0.5, ok_steer, p)
        assert pr_['steer'] == ok_steer == pr_['expect_steer'] and pr_['full_lock'] is None
    fl_l, fl_r = (resolve_profile('t3_circle', 0.3, None, p, s_) for s_ in ('left', 'right'))
    assert (fl_l['steer'], fl_r['steer']) == (0.27, -0.325), (fl_l['steer'], fl_r['steer'])
    assert (fl_l['expect_steer'], fl_r['expect_steer']) == (float(vp.get(p, 'steering.max_left_rad')),
                                                            float(vp.get(p, 'steering.max_right_rad')))
    for t_, st_, fl_ in (('t4_straight', 0.1, None), ('t4_straight', None, 'left')):
        try:
            resolve_profile(t_, 0.5, st_, p, fl_)
            raise AssertionError('t4 accepted --steer / --full-lock')
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
    g26f = t3_geometry(0.26, t3p, L, *body, extra_m=1.0)            # VESC-timeout fallback: disk only
    assert abs(g26f['diameter_m'] - g26['diameter_m'] - 2.0) < 1e-9 and g26f['r_max'] == g26['r_max']

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
    # 22 Nov rehearsal: own XT60 leads at 96-134 deg (0.04-0.17 m, y ~ +0.15 just outside the body box) refused
    # every LEFT circle; the 270 deg LakiBeam (blind 134.5..-136) made the visibility prediction 0.00.
    lid = vp.get(p, 'lidar')
    fov_ = (float(lid['fov_min_deg']), float(lid['fov_max_deg']))
    mask_ = [tuple(float(v) for v in m) for m in lid['self_mask']]
    assert masked(110.0, 0.15, mask_) and not masked(110.0, 0.35, mask_) and not masked(90.0, 0.15, mask_)

    def rehearsal_scan(blind=True, leads=True):
        sc = Scan3(6.0)
        if leads:
            sc.put(105.0, 0.15, 17.0).put(125.0, 0.08, 17.0)
        if blind:
            for i in range(1440):
                b = math.degrees(sc.angle_min + i * sc.angle_increment)
                if not fov_[0] <= b <= fov_[1]:
                    sc.ranges[i] = float('inf')
        return sc
    g18 = t3_geometry(0.18, t3p, L, *body)
    assert t3_clearance(rehearsal_scan(), g18, lx, *body)['min_m'] < 0            # the refusal, reproduced
    assert t3_clearance(rehearsal_scan(), g18, lx, *body, mask=mask_)['min_m'] > 1.0   # masked: clear
    box110 = rehearsal_scan().put(110.0, 0.8, 4.0)                                  # real box behind the mask range
    assert t3_clearance(box110, g18, lx, *body, mask=mask_)['min_m'] < 0
    assert t3_visibility(rehearsal_scan())['min_fraction'] == 0.0                   # old: blind = "no return"
    vb = t3_visibility(rehearsal_scan(), fov=fov_, mask=mask_)
    assert vb['min_fraction'] == 1.0 and vb['skipped'] > 0 and vb['predicted'] > 0, vb
    assert all(-115.75 <= h <= 114.25 for h in [vb['heading_deg']])
    door = rehearsal_scan().put(0.0, float('inf'), 60.0)                            # real gap inside the fov
    vd = t3_visibility(door, fov=fov_, mask=mask_)
    assert vd['min_fraction'] < 0.05 and abs(vd['heading_deg']) <= 10, vd
    assert t3_visibility(rehearsal_scan(), fov=(-10.0, 10.0)) is None               # nothing predictable
    st0 = (1.0, 2.0, math.pi / 2)                      # facing +y; left circle centre at (1 - r_max, 2)
    assert abs(t3_fence(st0, g26, (1.0 - 2 * g26['r_max'], 2.0, 0)) - 0.0) < 1e-9
    assert t3_fence(st0, g26, (1.0, 2.0, 0)) == 0.0 - 0.0 and t3_fence(st0, g26, (1.5, 2.0, 0)) > 0.4

    def c3world(pose, gz, mode='STOP', clear=2.0, vis=1.0, rf_age=0.05, sector=3.0, lowvis=False, **kw):
        st = dict(mode=mode, last_mode_request='MANUAL', manual_min_front_valid_fraction=0.35 if lowvis else 0.75,
                  manual_low_visibility_enabled=lowvis)
        return world(front_m=float('nan'), sector_min_m=sector, scan_age=0.05, left_m=2.0, right_m=2.0, st=st,
                     gz=gz, gz_mean_1s=0.003, gz_std_1s=0.0005, rf2o_pose=pose, rf2o_age=rf_age,
                     t3_clear=dict(min_m=clear, bearing_deg=45.0, n=1440),
                     t3_vis=dict(min_fraction=vis, heading_deg=10.0), **kw)

    def circle_run(steer, r_true_factor=1.0, turn_sign=1.0, fail_at=None, fail_kw=None, limit=80.0, lock=None):
        """Kinematic car on a circle (r_true = factor x nominal, turning turn_sign x commanded side);
        raw gz = -(yaw rate) + bias (IMU z down). Returns (machine, zero reason, elapsed)."""
        prof3 = resolve_profile('t3_circle', 0.5, steer, p, lock)
        steer = prof3['steer']
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
    for lk_ in ('left', 'right'):                     # full lock: geometry + turn check from the T2 angle
        m3, why, k = circle_run(None, lock=lk_)
        assert why.startswith('TURNS_DONE') and m3.steady_count == 1, (lk_, why)
        assert abs(m3.geom['r_nom'] - L / math.tan(abs(m3.expect_steer))) < 1e-12 and m3.expect_steer != m3.prof['steer']
        assert m3.repeats[-1]['full_lock'] == lk_
    m3, why, k = circle_run(None, lock='left', r_true_factor=1.6)                     # lock much weaker: fence
    assert why.startswith('GEOFENCE') and m3.steady_count == 0, why
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
    a3f = Machine(dict(pr3, clear_extra_m=1.0), p, live=False, ground_ok=True)
    a3f.step(0.0, 'm', False, c3world((0, 0, 0), 0.003))
    assert 'clear disk 5.31 m diameter incl. +1.0 m VESC-timeout FALLBACK' in a3f.events[-1]['detail']
    # 22 Nov: mux low-visibility mode (T3 only): the runner follows the mux threshold 0.35 (+0.05) and says so
    lv = Machine(pr3, p, live=False, ground_ok=True)
    lv.step(0.0, 'm', False, c3world((0, 0, 0), 0.003, lowvis=True, vis=0.44))
    assert lv.events[-1]['detail'].startswith('LOW-VISIBILITY MODE ACTIVE (T3 only: mux MANUAL front >= 0.35')
    assert 'needs >= 0.40' in lv.events[-1]['detail'], lv.events[-1]['detail']
    lv.step(0.1, 'n', False, c3world((0, 0, 0), 0.003, lowvis=True, vis=0.38))
    assert lv.state == 'IDLE' and lv.events[-1]['detail'].startswith('CIRCLE_LOW_VISIBILITY(0.38 < 0.40'), lv.events[-1]
    lv.step(0.2, 'n', False, c3world((0, 0, 0), 0.003, lowvis=True, vis=0.44))
    assert lv.state == 'ARMING', lv.events[-1]
    sd_ = Machine(pr3, p, live=False, ground_ok=True)                   # standard mode: 0.44 refused, no banner
    sd_.step(0.0, 'm', False, c3world((0, 0, 0), 0.003, vis=0.44))
    assert not sd_.events[-1]['detail'].startswith('LOW-VISIBILITY')
    sd_.step(0.1, 'n', False, c3world((0, 0, 0), 0.003, vis=0.44))
    assert sd_.events[-1]['detail'].startswith('CIRCLE_LOW_VISIBILITY(0.44 < 0.80'), sd_.events[-1]
    lvst = dict(manual_low_visibility_enabled=True, manual_min_front_valid_fraction=0.35)
    assert lowvis_gate('t3_circle', lvst, True) == (True, None)
    assert 'T3 and teleop' in lowvis_gate('t4_straight', lvst, True)[1] and lowvis_gate('t1b_stop', lvst, True)[1]
    lv10 = dict(manual_low_visibility_enabled=True, manual_min_front_valid_fraction=0.10)   # 23 Nov corridor teleop
    assert lowvis_gate('teleop', lv10, True, 0.3) == (True, None)
    assert '<= 0.3' in lowvis_gate('teleop', lv10, True, 0.5)[1]
    assert 'VESC timeout' in lowvis_gate('teleop', lv10, False, 0.3)[1]
    assert 'T3 keeps' in lowvis_gate('t3_circle', lv10, True, 0.3)[1]
    assert 'T3 and teleop' in lowvis_gate('t5b_stand_imu', lv10, True, 0.5)[1]
    assert lowvis_gate('teleop', dict(manual_low_visibility_enabled=False), False, 0.5) == (False, None)
    assert 'VESC timeout' in lowvis_gate('t3_circle', lvst, False)[1]
    assert lowvis_gate('t4_straight', dict(manual_low_visibility_enabled=False), False) == (False, None)
    a3l = Machine(resolve_profile('t3_circle', 0.5, None, p, 'right'), p, live=False, ground_ok=True)
    a3l.step(0.0, 'm', False, c3world((0, 0, 0), 0.003))
    assert 'FULL LOCK (command -0.325 rad, expected -0.321)' in a3l.events[-1]['detail'], a3l.events[-1]['detail']
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
        t3_precheck(resolve_profile('t3_circle', 0.5, 0.27, p), p, 0.33, t3c, 0.335, wide)  # needs 0.099 < 0.10
        raise AssertionError('servo below the converter clip accepted')
    except SystemExit as e:
        assert 'converter clip' in str(e) and '--full-lock' in str(e)
    ci = t3_precheck(resolve_profile('t3_circle', 0.5, -0.31, p), p, 0.33, t3c, 0.335, wide)   # right ~90 %
    assert abs(ci['servo_planned'] - 0.66422) < 1e-9 and ci['full_lock'] is None
    for lk_, clip_ in (('left', 0.10), ('right', 0.68)):          # full lock: servo exactly at the clip
        ci = t3_precheck(resolve_profile('t3_circle', 0.3, None, p, lk_), p, 0.33, t3c, 0.335, wide)
        assert ci['servo_planned'] == clip_ and ci['full_lock'] == lk_, ci['servo_planned']
        assert abs(ci['geometry']['r_nom'] - L / math.tan(abs(ci['expect_steer_rad']))) < 1e-12
    try:                                                           # lock command that does not reach the clip
        t3_precheck(dict(resolve_profile('t3_circle', 0.3, None, p, 'left'), steer=0.25), p, 0.33, t3c, 0.335, wide)
        raise AssertionError('full lock below the clip accepted')
    except SystemExit as e:
        assert 'not the clip' in str(e), str(e)
    try:
        t3_precheck(resolve_profile('t3_circle', 0.3, None, p, 'left'), p, 0.33, t3c, 0.335,
                    dict(servo_min=0.27, servo_max=0.37))
        raise AssertionError('full lock with the T4 VESC clip accepted')
    except SystemExit as e:
        assert 'vesc_driver clip' in str(e)
    # 22 Nov: steering TABLE in the converter (t1b_converter_t3_table.yaml) is accepted instead of the gains
    import yaml as _yaml
    tbl = _yaml.safe_load((vp.CONFIG / 't1b_converter_t3_table.yaml').read_text())['t1b_stop_converter']['ros__parameters']
    t3t = dict(servo_offset=0.335, servo_gain_left=-0.5, servo_gain_right=-0.5, **tbl)    # gains = baseline (unused)
    assert converter_steering(dict(servo_offset=0.335), 0.335)['mode'] == 'gain'
    smt = converter_steering(t3t, 0.335)
    assert smt['mode'] == 'table' and smt['limits'] == (-0.28, 0.28), smt
    for bad_t, why_ in ((dict(t3t, steer_table_right=[]), 'only one steering table'),
                        (dict(t3t, servo_offset=0.340), 'table invalid')):          # centre != table (0, 0.335)
        try:
            converter_steering(bad_t, 0.335)
            raise AssertionError('accepted %s' % why_)
        except SystemExit as e:
            assert why_ in str(e), str(e)
    ct = t3_precheck(resolve_profile('t3_circle', 0.3, 0.18, p), p, 0.33, t3t, 0.335, wide)
    assert ct['steer_mode'] == 'table' and ct['servo_gain_side'] is None
    assert abs(ct['servo_planned'] - (0.335 + (0.178 - 0.335) * 0.18 / 0.2042)) < 1e-9, ct['servo_planned']
    for lk_, end_, sv_ in (('left', 0.28, 0.11), ('right', -0.28, 0.660)):     # lock = the table's end point
        ct = t3_precheck(resolve_profile('t3_circle', 0.3, None, p, lk_), p, 0.33, t3t, 0.335, wide)
        assert ct['steer_rad'] == end_ == ct['expect_steer_rad'] and abs(ct['servo_planned'] - sv_) < 1e-9, ct
        assert abs(ct['geometry']['r_nom'] - L / math.tan(0.28)) < 1e-12
    for st_, why_ in ((-0.31, 'beyond the converter table end'), (0.27, None)):
        try:
            ct = t3_precheck(resolve_profile('t3_circle', 0.3, st_, p), p, 0.33, t3t, 0.335, wide)
            assert why_ is None, 'accepted %s' % st_
        except SystemExit as e:
            assert why_ and why_ in str(e), str(e)
    try:
        t3_precheck(resolve_profile('t3_circle', 0.3, None, p, 'left'), p, 0.33, t3t, 0.335,
                    dict(servo_min=0.27, servo_max=0.37))
        raise AssertionError('table lock with the T4 VESC clip accepted')
    except SystemExit as e:
        assert 'vesc_driver clip' in str(e)
    try:                                           # neither the T3 gains nor a table
        t3_precheck(pr3, p, 0.33, dict(servo_offset=0.335, servo_gain_left=-0.5, servo_gain_right=-0.5), 0.335, wide)
        raise AssertionError('baseline gains without a table accepted')
    except SystemExit as e:
        assert 'or a steering table' in str(e)
    # m readout: steer_mode from /t1b_stop_converter/status (T3 and T4)
    cv = dict(conv=dict(steer_mode='table', steer_limits=[-0.28, 0.28]), conv_age=0.05)
    mt = Machine(pr3, p, live=False, ground_ok=True)
    mt.step(0.0, 'm', False, dict(c3world((0, 0, 0), 0.003), **cv))
    assert mt.events[-1]['detail'].endswith('converter steer_mode table (limits -0.28 / +0.28 rad)'), mt.events[-1]
    m4 = Machine(resolve_profile('t4_straight', 0.5, None, p), p, live=False, ground_ok=True)
    m4.step(0.0, 'm', False, world(front_m=2.8, sector_min_m=2.8, left_m=1.0, right_m=1.0, scan_age=0.05,
                                   conv=dict(steer_mode='gain', steer_limits=None), conv_age=2.0))
    assert m4.events[-1]['detail'].endswith('converter steer_mode gain STALE 2.0 s'), m4.events[-1]
    assert steer_mode_text({}).endswith('no /t1b_stop_converter/status)')
    # VESC timeout evidence + fallback 0.3 m/s gate (folders as the runner / vesc_timeout_check.py write them)
    import tempfile
    with tempfile.TemporaryDirectory() as tdir:
        root = Path(tdir)
        assert not timeout_verified(root)[0]
        d = root / '20261122_100000_vesc_timeout_check_v0.50_brake5A_erpmmax4630_armed'
        d.mkdir()
        (d / 'timeout_check.json').write_text(json.dumps({'pass': False, 'verdict': 'FAIL onset 1.02 s'}))
        ok_, why_ = timeout_verified(root)
        assert not ok_ and 'FAIL onset 1.02 s' in why_, why_
        d2 = root / '20261122_101000_vesc_timeout_check_v0.50_brake5A_erpmmax4630_armed'
        d2.mkdir()
        (d2 / 'timeout_check.json').write_text(json.dumps({'pass': True}))
        assert timeout_verified(root)[0]
        p18 = resolve_profile('t3_circle', 0.5, 0.18, p)
        assert not t3_slow_gate(root, p18)[0] and t3_slow_gate(root, resolve_profile('t3_circle', 0.3, 0.18, p))[0]
        assert 'low-visibility' in t3_slow_gate(root, dict(p18, mux_low_visibility=True))[1]

        def t3run(name, steer, lock, reason, live=True):
            d_ = root / name
            d_.mkdir()
            (d_ / 'run_info.json').write_text(json.dumps(dict(live=live, profile=dict(steer=steer, full_lock=lock))))
            with open(d_ / 'runner_events.csv', 'w', newline='') as fp:
                wr = csv.DictWriter(fp, ['wall', 'mono', 'repeat', 'state', 'event', 'detail'])
                wr.writeheader()
                wr.writerow(dict(wall=1, mono=1, repeat=1, state='STOPPING', event='ZERO_COMMAND', detail=reason))
        t3run('1_t3_circle_v0.30_brake5A_erpmmax2780_armed', 0.18, None, 'GEOFENCE(0.3 m outside)')  # not counted
        t3run('2_t3_circle_v0.30_brake5A_erpmmax2780_armed', -0.18, None, 'TURNS_DONE(gyro 720 deg)')  # other side
        t3run('3_t3_circle_v0.30_brake5A_erpmmax2780_armed', 0.18, None, 'TURNS_DONE(gyro 720 deg)', live=False)
        assert not t3_slow_gate(root, p18)[0]
        t3run('4_t3_circle_v0.30_brake5A_erpmmax2780_armed', 0.18, None, 'TURNS_DONE(gyro 720 deg)')
        assert t3_slow_gate(root, p18)[0]
        assert not t3_slow_gate(root, resolve_profile('t3_circle', 0.5, None, p, 'left'))[0]
        t3run('5_t3_circle_v0.30_brake5A_erpmmax2780_armed', 0.27, 'left', 'TURNS_DONE(gyro 720 deg)')
        assert t3_slow_gate(root, resolve_profile('t3_circle', 0.5, None, p, 'left'))[0]
        assert not t3_slow_gate(root, resolve_profile('t3_circle', 0.5, 0.27, p))[0]     # --steer 0.27 != lock
    # T5b / VESC timeout check: stand profiles at the T1b-passed 0.5 m/s, brake 5 A
    for t_, st_ in (('t5b_stand_imu', 90.0), ('vesc_timeout_check', 20.0)):
        pr_ = resolve_profile(t_, 0.5, None, p)
        assert pr_['place'] == 'stand' and pr_['steady_s'] == st_ and pr_['stop_method'] == 'brake5A'
        m5 = Machine(pr_, p, live=False)
        assert abs(m5.hard_s - (1.0 + st_ + 0.5)) < 1e-9 and not m5.ground
    print('PASS: T3 circle: --steer limits, nested clear disk, clearance (box, own body), visibility gap,')
    print('      geofence; drives: 2 gyro turns counted (L/R, bias removed), wrong direction / straight ->')
    print('      GEOFENCE or TURN_CHECK_FAILED by 2.1 s, too large -> GEOFENCE, RF2O stale, IMU fault, mux STOP')
    print('      backstop; arm refusals (not clear, low visibility, RF2O stale, no gyro, mux sector object);')
    print('      pre-run refusals (steer cap, converter gains, converter clip, VESC clip / T4 yaml).')
    print('      21 Nov: per-side limits 0.27 / 0.325, --full-lock (servo exactly 0.10 / 0.68, geometry and turn')
    print('      check from the T2 angle), fallback disk +1 m, VESC timeout evidence, 0.3-first gate; T5b /')
    print('      vesc_timeout_check stand profiles.')
    print('      22 Nov: self_mask (own XT60 leads no longer refuse a left circle, a real box behind them still')
    print('      does), 270 deg FOV: blind / masked beams unknown, visibility only for headings fully in the FOV.')
    print('      Mux low-visibility mode: runner threshold follows the mux (0.35 + 0.05), m-line banner, T3 only,')
    print('      needs the verified VESC timeout, 0.3 m/s first.')
    print('      Converter steering TABLE accepted instead of the T3 gains (--steer within the table end, full lock')
    print('      = table end, bad / one-sided table refused); steer_mode in the m readout (T3 and T4).')

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
    self_test_23nov(p, world)
    self_test_23nov_eve(p, world)
    self_test_28nov(p, world)
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


def param_value(v):
    """rcl_interfaces ParameterValue -> Python (None for NOT_SET / unsupported types)."""
    return {1: v.bool_value, 2: v.integer_value, 3: v.double_value, 4: v.string_value,
            7: list(v.integer_array_value), 8: list(v.double_array_value)}.get(v.type)


def params_from_answers(names, batch, single):
    """{name: value} from a GetParameters batch answer (list of ParameterValue, or None when unreachable) and a
    per-name fallback single(name) -> [ParameterValue] or []. rclpy (Humble parameter_service.py) answers the WHOLE
    batch with an empty list as soon as one name is declared but unset (ParameterUninitializedException): in gain
    mode the converter's steer_table_left/right are typed and unset, so the 22 Nov batch read returned {} and the
    runner refused ("parameters unreadable: {}"). Short answer -> ask per name; an unset name maps to None."""
    if batch is None:
        return {}
    if len(batch) == len(names):
        return {n: param_value(v) for n, v in zip(names, batch)}
    out = {}
    for n in names:
        one = single(n)
        out[n] = param_value(one[0]) if one else None
    return out


def get_node_params(node, node_name, names):
    """{name: value} read from a running node (bool/int/double/string/arrays); {} if unreachable; a declared but
    unset name (e.g. the converter's steering tables in gain mode) -> None. Needs `node` spinning in another
    thread (RosSession)."""
    from rcl_interfaces.srv import GetParameters
    cli = node.create_client(GetParameters, '/%s/get_parameters' % node_name)

    def ask(req_names):
        fut = cli.call_async(GetParameters.Request(names=list(req_names)))
        t_end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        return None if not fut.done() or fut.result() is None else list(fut.result().values)

    try:
        if not cli.wait_for_service(timeout_sec=2.0):
            return {}
        return params_from_answers(list(names), ask(names), lambda n: ask([n]) or [])
    finally:
        node.destroy_client(cli)


# t1b converter parameters the runner reads; the steering tables are optional (unset = gain mode).
T1B_PARAMS_REQUIRED = ('method', 'brake_current_a', 'brake_hold_s', 'servo_offset', 'servo_gain_left',
                       'servo_gain_right')
T1B_PARAMS_OPTIONAL = ('steer_table_left', 'steer_table_right')


def t1b_params_problem(cp):
    """None if the t1b converter parameters are usable, else the refusal text (names what is missing)."""
    if not cp:
        return ('t1b_stop_converter parameters unreadable (no answer from /t1b_stop_converter/get_parameters '
                'in 2 s: is the converter running in this ROS graph / domain?)')
    missing = [n for n in T1B_PARAMS_REQUIRED if cp.get(n) is None]
    if missing:
        return 't1b_stop_converter parameters missing: %s (got %s)' % (', '.join(missing), cp)
    return None


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

def drive_key_event(deadman, state, key, now):
    """A drive key (state.drive_keys) presses the deadman. A CHANGE of drive key starts a new press (first-press
    window), because the terminal auto-repeats only the last key and needs its start delay again (teleop: roll
    from w to a by pressing a before releasing w). Returns False for any other key."""
    if key not in state.drive_keys:
        return False
    if key != state.drive_key:
        deadman.reset()
        state.drive_key = key
    deadman.press(now)
    return True


class Keyboard:
    """Single keys from the SSH terminal (no Enter) in a background thread."""

    def __init__(self, deadman, drive_keys=None):
        if not sys.stdin.isatty():
            raise SystemExit('stdin is not a terminal: the SPACE deadman needs a real TTY')
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        self.deadman = deadman
        # None: SPACE is the deadman; {} (teleop_ps4): no drive key, the controller is the deadman and SPACE kills.
        self.drive_keys = {'space': 0} if drive_keys is None else drive_keys
        self.drive_key = None
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
                if not drive_key_event(self.deadman, self, key, now):
                    self.pending.append(key)

    def held(self, now):
        with self.lock:
            return not self.eof and self.deadman.held(now)

    def direction(self):
        with self.lock:
            return self.drive_keys.get(self.drive_key)

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

def resolve_profile(test, speed, steer=None, p=None, full_lock=None, throttle_mode=None, ps4_cfg=None):
    """Profile with its speed fixed: stepped profiles need --speed equal to one step; circle profiles
    need --steer (+ = left, per-side limits t3.steer_max_left_rad / steer_max_right_rad) or --full-lock.
    teleop_ps4 (28 Nov): --speed = teleop_max_speed_mps (default / floor / hard ceiling: teleop_speed), throttle mode
    from --throttle-mode or ps4_teleop.yaml, stop method brake<brake_current_a>A."""
    prof = dict(PROFILES[test])
    if prof.get('ps4'):
        cfg = ps4_cfg or vp.load_ps4()
        mode = throttle_mode or str(cfg['throttle_mode'])
        if mode not in THROTTLE_MODES:
            raise SystemExit('teleop_ps4: throttle_mode %r must be one of %s' % (mode, THROTTLE_MODES))
        if steer is not None or full_lock is not None:
            raise SystemExit('teleop_ps4 steers with the left stick; do not pass --steer / --full-lock')
        prof.update(speed=teleop_speed(speed, cfg), ps4_cfg=cfg, throttle_mode=mode,
                    stop_method='brake%gA' % float(cfg['brake_current_a']))
        return prof
    if throttle_mode is not None:
        raise SystemExit('--throttle-mode is for teleop_ps4 only')
    if prof.get('circle'):
        p = p or vp.load()
        prof['steer'], prof['expect_steer'] = t3_command(vp.get(p, 't3'), vp.get(p, 'steering'), steer, full_lock)
        prof['full_lock'] = full_lock
    elif steer is not None or full_lock is not None:
        raise SystemExit('%s has a fixed steering angle; do not pass --steer / --full-lock' % test)
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
    prof = resolve_profile(args.test, args.speed, args.steer, p, args.full_lock, args.throttle_mode)
    if prof.get('circle'):
        # VESC timeout fallback (21 Nov): until a passed vesc_timeout_check, the clear disk grows by
        # t3.fallback_extra_m and 0.5 m/s needs a counted 0.3 m/s run of the same command (drive_session).
        ok, why = timeout_verified(args.out_root or vp.evidence_root())
        prof['vesc_timeout'] = dict(verified=ok, reason=why)
        prof['clear_extra_m'] = 0.0 if ok else float(vp.get(p, 't3.fallback_extra_m'))
        say(why + ('' if ok else ': FALLBACK rules: clear disk +%.1f m, 0.3 m/s first, spotter outside the disk'
                   % prof['clear_extra_m']))
    prof['fast'] = bool(args.fast)
    prof['hands_off_s'] = FAST_HANDS_OFF_S if args.fast else HANDS_OFF_S
    if args.fast:
        say('FAST MODE: no T3 clear-disk / visibility refusal, no 0.3-first gate, no --t1-passed-confirmed, hands '
            'off %.1f s. Kept: SPACE release = brake, brake converter, caps, IMU freeze, VESC status, geofence.'
            % FAST_HANDS_OFF_S)
    if args.live and not prof['live_allowed']:
        raise SystemExit('profile %s never runs with real actuation' % args.test)
    if prof['place'] == 'ground' and not (args.t1_passed_confirmed or args.fast):
        raise SystemExit('ground profile: needs --t1-passed-confirmed (Moatasem written T1 pass)')
    mism = vp.config_mismatches(p)
    if mism:
        raise SystemExit('config/real_car disagrees with vehicle_params.yaml:\n  ' + '\n  '.join(mism))
    r = vp.get(p, 'runner')
    mux_sector = float(vp.get(p, 't4.mux_sector_deg'))
    laser_x = float(vp.get(p, 'lidar.x_m'))
    lid = vp.get(p, 'lidar')
    fov = (float(lid['fov_min_deg']), float(lid['fov_max_deg']))
    self_mask = [tuple(float(v) for v in m) for m in (lid.get('self_mask') or [])]
    veh = vp.get(p, 'vehicle')
    body = (float(veh['body_length_m']), float(veh['body_width_m']), float(veh['front_bumper_x_m']))
    t3_geom = (t3_geometry(prof['expect_steer'], vp.get(p, 't3'), float(veh['wheelbase_m']), *body,
                           extra_m=prof['clear_extra_m'])
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
            self.t3_geom = t3_geom                     # replaced by drive_session after the T3 pre-check
            self.conv = self.conv_at = None
            if prof.get('converter') == 't1b_stop_converter':
                self.create_subscription(String, '/t1b_stop_converter/status', self.on_conv, 10)
            self.pad = None
            if prof.get('ps4'):
                self.pad = Ps4Pad(prof['ps4_cfg'])
                self.create_subscription(Joy, prof['ps4_cfg']['topic'], self.on_ps4, qos_profile_sensor_data)
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

        def on_conv(self, msg):
            try:
                c = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            with self.lock:
                self.conv, self.conv_at = c, time.monotonic()

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

        def on_ps4(self, msg):
            ns = int(msg.header.stamp.sec) * 1000000000 + int(msg.header.stamp.nanosec)
            lag = (self.get_clock().now().nanoseconds - ns) / 1e9 if ns > 0 else None
            self.pad.update(time.monotonic(), msg.buttons, msg.axes, lag, time.time())

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
            if self.pad is not None:
                topic = prof['ps4_cfg']['topic']
                n = len(self.get_publishers_info_by_topic(topic))
                if n != 1:
                    return 'PS4_JOY_PUBLISHERS(%s=%d, need exactly 1 joy_node)' % (topic, n)
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
                w['conv'], w['conv_age'] = self.conv, (now - self.conv_at) if self.conv_at else 1e9
                w['imu_problem'] = self.imu_watch.status(now, imu_publishers)
                if prof.get('circle'):
                    gz = [x for t_, x in self.gz_hist if now - t_ <= prof['hands_off_s']]
                    w.update(gz=self.gz, gz_mean_1s=statistics.mean(gz) if gz else None,
                             gz_std_1s=statistics.pstdev(gz) if len(gz) > 1 else None,
                             rf2o_pose=self.rf2o, rf2o_age=(now - self.rf2o_at) if self.rf2o_at else 1e9)
                    scan = self.scan
            if prof.get('circle'):
                # Clearance + visibility: ~1440 points, at most 5 Hz and only for a new scan (cached).
                if prof['fast']:
                    self.t3_cache = (None, None)   # --fast: clear disk + visibility not computed, not checked
                elif scan is not None and scan is not self.t3_scan and now - self.t3_cache_at >= 0.2:
                    self.t3_cache = (t3_clearance(scan, self.t3_geom, laser_x, *body, mask=self_mask),
                                     t3_visibility(scan, fov=fov, mask=self_mask))
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


def ps4_link_setup(pad, cfg, synthetic, devices_path='/proc/bus/input/devices'):
    """Bluetooth link heartbeat (LinkWatch) from the DS4 motion-sensor device. Bluetooth (Bus 0005) without it is
    refused (link_required_on_bluetooth); USB without it is allowed (a cable pull removes the device: joy_node goes
    silent). No DS4 in the kernel list: refused unless synthetic (laptop stub test, hidden --ps4-synthetic), and a
    real DS4 present refuses synthetic. -> dict for run_info.json."""
    try:
        text = Path(devices_path).read_text()
    except OSError:
        text = ''
    dev = ps4_input_devices(text, str(cfg['link_device_match']), str(cfg['link_vendor']))
    if synthetic:
        if dev['bus'] is not None:
            raise SystemExit('--ps4-synthetic refused: a real controller is connected (%s)' % dev['gamepad'])
        return dict(dev, link='synthetic (laptop stub test): no link heartbeat')
    if dev['bus'] is None:
        raise SystemExit('teleop_ps4: no Sony controller in %s (vendor %s): pair / plug it in (SESSION_HANDOFF)'
                         % (devices_path, cfg['link_vendor']))
    bluetooth = dev['bus'] == '0005'
    if dev['motion'] is None:
        if bluetooth and cfg['link_required_on_bluetooth']:
            raise SystemExit('teleop_ps4: controller on Bluetooth but no "%s" device: a link drop could not be seen '
                             '(joy_node repeats the last state). Use the USB cable.' % cfg['link_device_match'])
        return dict(dev, link='none (USB: a cable pull removes the device)')
    link = LinkWatch(float(cfg['link_ttl_s']))
    try:
        link.start(dev['motion'])
    except OSError as exc:
        if bluetooth and cfg['link_required_on_bluetooth']:
            raise SystemExit('teleop_ps4: cannot read %s (%s): add the user to group input (sudo usermod -aG input '
                             '$USER, log out and in), or use the USB cable' % (dev['motion'], exc))
        return dict(dev, link='none (%s unreadable: %s; USB)' % (dev['motion'], exc))
    pad.link = link
    return dict(dev, link='heartbeat %s, ttl %.2f s' % (dev['motion'], link.ttl))


def ps4_precheck(node, cfg, synthetic=False):
    """teleop_ps4 refusals before the run folder exists: exactly one /ps4/joy publisher (joy_node, remapped), the
    runner alone on /joy (graph_fault), joy_node parameters as ps4_teleop.yaml joy_node (sticky_buttons false: a
    sticky deadman never releases; autorepeat_rate; deadzone 0: one deadzone, the yaml's), the link heartbeat
    (ps4_link_setup), then the interactive controller check (ps4_controller_check). -> dict for run_info.json."""
    topic = cfg['topic']
    pubs = node.get_publishers_info_by_topic(topic)
    want = cfg['joy_node']
    cmd = ('ros2 run joy joy_node --ros-args -r joy:=%s -p device_id:=0 -p autorepeat_rate:=%s -p deadzone:=%s '
           '-p sticky_buttons:=false' % (topic, float(want['autorepeat_rate']), float(want['deadzone'])))
    if len(pubs) != 1:
        raise SystemExit('teleop_ps4: %d publishers on %s (need exactly 1): on the Jetson run `%s`'
                         % (len(pubs), topic, cmd))
    jn = (pubs[0].node_namespace.rstrip('/') + '/' + pubs[0].node_name).lstrip('/')
    jp = node.get_params(jn, ['sticky_buttons', 'autorepeat_rate', 'deadzone'])
    if jp.get('sticky_buttons') is not False:
        raise SystemExit('teleop_ps4: %s sticky_buttons = %s (must be false: a sticky deadman never releases)'
                         % (jn, jp.get('sticky_buttons')))
    for k in ('autorepeat_rate', 'deadzone'):
        if jp.get(k) is None or abs(float(jp[k]) - float(want[k])) > 1e-9:
            raise SystemExit('teleop_ps4: %s %s = %s, ps4_teleop.yaml needs %s: restart it as `%s`'
                             % (jn, k, jp.get(k), want[k], cmd))
    joy_pubs = [i.node_name for i in node.get_publishers_info_by_topic('/joy')]
    if joy_pubs != ['real_car_test_runner']:
        raise SystemExit('teleop_ps4: /joy publishers %s: only this runner may publish /joy (joy_node must be '
                         'remapped to %s)' % (joy_pubs, topic))
    link = ps4_link_setup(node.pad, cfg, synthetic)
    say('PS4 link: %s (controller %s, bus %s)' % (link['link'], link['gamepad'], link['bus']))
    info = ps4_controller_check(node.pad, cfg, time.monotonic, time.sleep, say)
    info.update(joy_node=jn, joy_params=jp, topic=topic, link=link)
    return info


def converter_steering(converter_params, base_off):
    """Steering mode of the running t1b converter from its parameters: 'table' (steer_table_left/right, checked
    with the converter's own parse_steer_table) or 'gain'. SystemExit on a bad or one-sided table (the converter
    refuses those too). -> dict(mode, tables, limits=(right end, left end) or None, offset)."""
    import t1b_stop_converter as conv
    off = float(converter_params.get('servo_offset') or base_off)
    tl, tr = converter_params.get('steer_table_left') or [], converter_params.get('steer_table_right') or []
    if not tl and not tr:
        return dict(mode='gain', tables=None, limits=None, offset=off)
    if not (tl and tr):
        raise SystemExit('t1b converter has only one steering table (left %s / right %s)' % (bool(tl), bool(tr)))
    try:
        tables = (conv.parse_steer_table(tl, 'left', off), conv.parse_steer_table(tr, 'right', off))
    except ValueError as exc:
        raise SystemExit('t1b converter steering table invalid: %s' % exc)
    return dict(mode='table', tables=tables, limits=(tables[1][-1][0], tables[0][-1][0]), offset=off)


def steer_mode_text(w):
    """Placement readout of /t1b_stop_converter/status steer_mode ('' if this stack has no t1b status)."""
    c, age = w.get('conv'), w.get('conv_age', 1e9)
    if c is None:
        return ' | converter steer_mode - (no /t1b_stop_converter/status)'
    lim = c.get('steer_limits')
    return ' | converter steer_mode %s%s%s' % (
        c.get('steer_mode', 'gain'),
        ' (limits %+.2f / %+.2f rad)' % (lim[0], lim[1]) if lim else '',
        ' STALE %.1f s' % age if age > 1.0 else '')


def teleop_precheck(prof, steering, steer_cap, clip):
    """Teleop refusals (SystemExit): the converter must steer with a TABLE (gain mode would give about half the angle,
    T2/T3), the table ends and the mux steer cap must reach the profile's +/-steer, and both servo ends must be inside
    the vesc_driver clip (vesc_t3_erpm<N>.yaml: 0.10-0.68). -> info for run_info.json."""
    import t1b_stop_converter as conv
    smax = abs(float(prof['steer']))
    if not steering or steering['mode'] != 'table':
        raise SystemExit('teleop needs the converter steering TABLE (launch converter_params_file:=/home/ftenth/'
                         'f1tenth_uj/config/real_car/t1b_converter_t3_table.yaml)')
    right_end, left_end = steering['limits']
    if left_end < smax - 1e-9 or -right_end < smax - 1e-9:
        raise SystemExit('teleop: converter table ends %+.2f / %+.2f rad do not reach +/-%.2f'
                         % (right_end, left_end, smax))
    if steer_cap < smax - 1e-9:
        raise SystemExit('teleop: mux max_steer_rad %.2f < %.2f (launch max_steer_rad:=%.2f)' % (steer_cap, smax, smax))
    sl = conv.servo_command(steering['offset'], -0.5, -0.5, smax, steering['tables'])
    sr = conv.servo_command(steering['offset'], -0.5, -0.5, -smax, steering['tables'])
    if (len(clip) < 2 or None in clip.values() or not all(
            float(clip['servo_min']) - 1e-9 <= s_ <= float(clip['servo_max']) + 1e-9 for s_ in (sl, sr))):
        raise SystemExit('teleop: servo %.3f / %.3f outside the vesc_driver clip %s (launch vesc_yaml:=/home/ftenth/'
                         'f1tenth_uj/config/real_car/vesc_t3_erpm<N>.yaml)' % (sl, sr, clip))
    return dict(steer_max_rad=smax, steer_rate_rad_s=TELEOP_STEER_RATE, servo_left=sl, servo_right=sr,
                vesc_servo_clip=clip, keys=TELEOP_KEYS)


def t3_precheck(prof, p, steer_cap, converter_params, base_off, clip):
    """T3 refusals before anything is armed (SystemExit with the fix): mux steer cap <= t3.mux_cap_max_rad; the
    converter steers either with the T3 per-side GAINS or with a valid steering TABLE (22 Nov). Gains: a --steer
    servo must not be clamped by the converter clip, --full-lock must land exactly on the clip. Table: a --steer
    must stay within the table's end angle on its side; --full-lock = the table's end point (its angle is then
    also the expected angle). The planned servo must be inside the vesc_driver clip.
    -> circle info for run_info.json (steer_rad / expect_steer_rad may differ from prof for a table full lock)."""
    import t1b_stop_converter as conv
    t3, veh = vp.get(p, 't3'), vp.get(p, 'vehicle')
    steer, lock = float(prof['steer']), prof.get('full_lock')
    expect = float(prof.get('expect_steer', steer))
    if steer_cap > float(t3['mux_cap_max_rad']) + 1e-9:
        raise SystemExit('t3_circle: mux max_steer_rad %.2f > t3.mux_cap_max_rad %.2f (launch max_steer_rad:=%.2f)'
                         % (steer_cap, float(t3['mux_cap_max_rad']), float(t3['mux_cap_max_rad'])))
    sm = converter_steering(converter_params, base_off)
    off = sm['offset']
    gl, gr = converter_params.get('servo_gain_left'), converter_params.get('servo_gain_right')
    if sm['mode'] == 'table':
        end = sm['limits'][1] if steer > 0 else sm['limits'][0]
        if lock:
            steer = expect = end                      # the table's own lock: its end point
        elif abs(steer) > abs(end) + 1e-9:
            raise SystemExit('t3_circle: steer %+.3f rad is beyond the converter table end %+.2f rad: choose a '
                             'smaller --steer, or --full-lock for the table end' % (steer, end))
        servo = conv.servo_command(off, -0.5, -0.5, steer, sm['tables'])
        gside = None
    else:
        if (gl is None or gr is None or abs(float(gl) - float(t3['servo_gain_left'])) > 1e-6
                or abs(float(gr) - float(t3['servo_gain_right'])) > 1e-6):
            raise SystemExit('t3_circle needs the T3 converter gains %.3f / %.3f or a steering table; the converter '
                             'has gains %s / %s and no table (launch converter_params_file:=/home/ftenth/f1tenth_uj/'
                             'config/real_car/t1b_converter_t3_t2gain.yaml or t1b_converter_t3_table.yaml)'
                             % (float(t3['servo_gain_left']), float(t3['servo_gain_right']), gl, gr))
        gside = float(gl) if steer > 0 else float(gr)
        servo = conv.servo_command(off, float(gl), float(gr), steer)
        linear = off + gside * steer
        if lock:
            clip_side = conv.SERVO_DRIVE_CLIP[0] if steer > 0 else conv.SERVO_DRIVE_CLIP[1]
            if abs(servo - clip_side) > 1e-9:
                raise SystemExit('t3_circle --full-lock %s: steer %+.3f rad gives servo %.4f, not the clip %.2f '
                                 '(t3.steer_max_*_rad / converter gains disagree)' % (lock, steer, servo, clip_side))
        elif abs(servo - linear) > 1e-9:
            raise SystemExit('t3_circle: steer %+.3f rad needs servo %.4f, outside the converter clip %s: '
                             'choose a smaller --steer, or --full-lock for the clip itself'
                             % (steer, linear, conv.SERVO_DRIVE_CLIP))
    if (len(clip) < 2 or None in clip.values()
            or not (float(clip['servo_min']) - 1e-9 <= servo <= float(clip['servo_max']) + 1e-9)):
        raise SystemExit('t3_circle: planned servo %.3f outside the vesc_driver clip %s (launch '
                         'vesc_yaml:=.../vesc_t3_erpm<N>.yaml)' % (servo, clip))
    return dict(steer_rad=steer, full_lock=lock, expect_steer_rad=expect, servo_planned=servo, servo_offset=off,
                servo_gain_side=gside, steer_mode=sm['mode'], steer_table=sm['tables'], steer_limits=sm['limits'],
                vesc_servo_clip=clip, t3=t3, vesc_timeout=prof.get('vesc_timeout'),
                geometry=t3_geometry(expect, t3, float(veh['wheelbase_m']), float(veh['body_length_m']),
                                     float(veh['body_width_m']), float(veh['front_bumper_x_m']),
                                     float(prof.get('clear_extra_m', 0.0))))


def teleop_start_info(prof, converter_params, vesc_yamls, vesc_params):
    """teleop_ps4 start-up record: safety mode, throttle mode, teleop_max_speed_mps, brake current, VESC yaml (matched
    from the running vesc_driver), stopping-distance estimate. -> dict (start_line = the runner_events START detail)."""
    cfg, v = prof['ps4_cfg'], float(prof['speed'])
    lat, dec = float(cfg['deadman_latency_s']), float(cfg['floor_decel_mps2'])
    d = stopping_distance(v, lat, dec)
    yaml_txt = ','.join(vesc_yamls) if vesc_yamls else 'UNKNOWN(speed_max %s, servo %s..%s)' % (
        vesc_params.get('speed_max'), vesc_params.get('servo_min'), vesc_params.get('servo_max'))
    line = ('safety_mode=%s throttle_mode=%s teleop_max_speed_mps=%.2f brake_current_a=%s vesc_yaml=%s '
            'stop_estimate_m=%.2f (v*%.2f + v^2/(2*%.1f); braking floor-verified only at %.1f m/s)'
            % (prof.get('safety_mode', 'autonomy'), prof['throttle_mode'], v, converter_params.get('brake_current_a'),
               yaml_txt, d, lat, dec, float(cfg['decel_verified_at_mps'])))
    return dict(safety_mode=prof.get('safety_mode', 'autonomy'), throttle_mode=prof['throttle_mode'],
                teleop_max_speed_mps=v, brake_current_a=converter_params.get('brake_current_a'),
                vesc_yaml=vesc_yamls, vesc_params=vesc_params, stop_estimate_m=d, start_line=line)


def teleop_start_text(prof, ti):
    cfg = prof['ps4_cfg']
    return ('!' * 72 + '\nPS4 TELEOP  safety_mode %s | throttle %s | teleop_max_speed_mps %.2f | brake %s A | VESC %s\n'
            'Estimated stopping distance at %.2f m/s: %.2f m (%.2f s deadman latency + 5 A tile decel %.1f m/s^2).\n'
            'NOTE: braking has been floor-verified only at %.1f m/s.%s\n' % (
                ti['safety_mode'], ti['throttle_mode'], ti['teleop_max_speed_mps'], ti['brake_current_a'],
                ','.join(ti['vesc_yaml']) or 'UNKNOWN', ti['teleop_max_speed_mps'], ti['stop_estimate_m'],
                float(cfg['deadman_latency_s']), float(cfg['floor_decel_mps2']), float(cfg['decel_verified_at_mps']),
                ('\nsafety_mode TELEOP: the mux front-visibility and 0.80 m obstacle gates only WARN: YOU are the '
                 'obstacle stop.' if ti['safety_mode'] == 'teleop' else '')) + '!' * 72)


def drive_session(args, p, prof, r, node):
    period = 1.0 / float(r['publish_hz'])
    t_end = time.monotonic() + 3.0
    while node.status is None and time.monotonic() < t_end:
        time.sleep(0.05)
    if node.status is None:
        raise SystemExit('No /control/status in 3 s: start the test stack first.')
    st = dict(node.status)
    safety, why_sm = safety_mode_gate(prof, st)
    if why_sm:
        raise SystemExit(why_sm)
    prof['safety_mode'] = safety
    if 'safety_mode' not in st:
        say('mux status has no safety_mode: the installed mux predates 28 Nov (colcon build --packages-select '
            'f1tenth_bringup on the Jetson); treated as autonomy (all gates).')
    if prof.get('ps4') and safety == 'teleop':
        prof['manual_request'] = 'MANUAL_PS4'        # physical_drive_mux TELEOP_MANUAL_REQUEST
    t_ok = (prof.get('vesc_timeout') or {}).get('verified')
    if t_ok is None and prof.get('teleop'):
        t_ok = timeout_verified(args.out_root or vp.evidence_root())[0]
    lowvis, why_lv = lowvis_gate(args.test, st, bool(t_ok), prof['speed'],
                                 float(vp.get(p, 't3.low_visibility_manual_min_front_valid')))
    if why_lv:
        raise SystemExit(why_lv)
    prof['mux_low_visibility'] = lowvis
    if lowvis and prof.get('teleop'):
        say('!' * 72 + '\nLOW-VISIBILITY TELEOP: mux MANUAL front >= %.2f (standard 0.75), --speed %.2f (max %.1f).\n'
            'Kept: mux 0.80 m obstacle stop (needs a finite return), release = brake 5 A, caps, IMU / VESC checks.\n'
            'The LiDAR sees little ahead: spotter ahead of the car, hand on the battery disconnect.\n' % (
                float(st.get('manual_min_front_valid_fraction', 0)), prof['speed'], TELEOP_LOWVIS_MAX_SPEED) + '!' * 72)
    elif lowvis:
        say('LOW-VISIBILITY MODE ACTIVE (mux MANUAL front >= %.2f; T3 only; AUTO and the 0.80 m obstacle gate '
            'unchanged). Conditions: clear disk on, look behind the car before every n, spotter outside the disk, '
            '0.3 m/s first for every command.' % float(st.get('manual_min_front_valid_fraction', 0)))
    real = bool(st.get('real_actuation_enabled'))
    real, args.live, why_live = live_mode(real, args.live, prof['live_allowed'])
    if why_live:
        say(why_live)
    cap = float(st.get('max_speed_mps', vp.get(p, 'caps.mux_default_max_speed_mps')))
    steer_cap = float(st.get('max_steer_rad', vp.get(p, 'caps.mux_default_max_steer_rad')))
    if prof['speed'] > cap + 1e-9 or abs(prof['steer']) > steer_cap + 1e-9:
        raise SystemExit('Profile %.2f m/s / %.2f rad exceeds mux cap %.2f m/s / %.2f rad.' % (
            prof['speed'], prof['steer'], cap, steer_cap))
    speed_max = node.vesc_speed_max()
    gain = float(vp.get(p, 'drivetrain.speed_to_erpm_gain'))
    need_erpm = prof['speed'] * gain
    if prof.get('ps4'):
        bad = teleop_speed_problem(prof['speed'], cap, speed_max, gain)
        if bad:
            raise SystemExit('teleop_ps4: ' + bad)
        ok, why = teleop_brake_gate(args.out_root or vp.evidence_root(), prof['speed'],
                                    float(prof['ps4_cfg']['floor_decel_mps2']))
        if not ok:
            raise SystemExit('teleop_ps4: ' + why)
        say('teleop brake gate: ' + why)
    elif speed_max is not None and need_erpm > speed_max:
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
        converter_params = node.get_params('t1b_stop_converter', T1B_PARAMS_REQUIRED + T1B_PARAMS_OPTIONAL)
        bad = t1b_params_problem(converter_params)
        if bad:
            raise SystemExit(bad)
        if abs(float(converter_params['servo_offset']) - base_off) > 1e-9:
            print('TEST servo_offset %.3f in the converter (baseline %.3f): recorded in run_info.json.'
                  % (float(converter_params['servo_offset']), base_off))
        method = converter_params['method']
        if method == 'brake':
            method = 'brake%gA' % converter_params['brake_current_a']
    steering = converter_steering(converter_params, base_off) if prof['converter'] == 't1b_stop_converter' else None
    if steering:
        say('Converter steering: %s%s' % (steering['mode'], ' %s / %s' % steering['tables'] if steering['tables'] else ''))
    teleop_info = None
    if prof.get('teleop'):
        teleop_info = teleop_precheck(prof, steering, steer_cap,
                                      node.get_params('vesc_driver', ['servo_min', 'servo_max']))
        say('TELEOP: steer +/-%.2f rad -> servo %.3f (left) / %.3f (right), table mode, vesc clip %s'
            % (teleop_info['steer_max_rad'], teleop_info['servo_left'], teleop_info['servo_right'],
               teleop_info['vesc_servo_clip']))
    ps4_info = None
    if prof.get('ps4'):
        if abs(steer_cap - abs(float(prof['steer']))) > 1e-9:
            raise SystemExit('teleop_ps4 keeps the steering cap at %.2f rad: launch max_steer_rad:=%.2f (mux has %.2f)'
                             % (abs(prof['steer']), abs(prof['steer']), steer_cap))
        ps4_info = ps4_precheck(node, prof['ps4_cfg'], bool(getattr(args, 'ps4_synthetic', False)))
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
        if circle_info['steer_rad'] != prof['steer']:      # table full lock: command and expectation = table end
            prof['steer'], prof['expect_steer'] = circle_info['steer_rad'], circle_info['expect_steer_rad']
        node.t3_geom = circle_info['geometry']           # the node's live clearance check uses the same disk
        g = circle_info['geometry']
        say('T3 circle %s %s%+.3f rad -> servo %.3f; R nominal %.2f m; clear disk %.2f m diameter%s, centre %.2f m '
            'to the %s of the start' % ('LEFT' if prof['steer'] > 0 else 'RIGHT',
                                        'FULL LOCK ' if prof.get('full_lock') else '', prof['steer'],
                                        circle_info['servo_planned'], g['r_nom'], g['diameter_m'],
                                        ' (incl. fallback +%.1f m)' % g['extra_m'] if g['extra_m'] else '',
                                        g['r_max'], 'left' if prof['steer'] > 0 else 'right'))
        if args.fast:
            say('T3 0.3-first gate: skipped (--fast)')
        elif args.live and (not prof['vesc_timeout']['verified'] or prof.get('mux_low_visibility')):
            ok, why = t3_slow_gate(args.out_root or vp.evidence_root(), prof)
            if not ok:
                raise SystemExit('T3 fallback gate: ' + why)
            say('T3 fallback gate: ' + why)
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
    teleop_ps4 = None
    if prof.get('ps4'):
        vparams = node.get_params('vesc_driver', list(VESC_MATCH_KEYS))
        teleop_ps4 = teleop_start_info(prof, converter_params, vesc_yaml_match(vparams, vp.CONFIG), vparams)
    info = dict(test=args.test, profile=prof, live=args.live, fast=bool(args.fast), started=stamp, git_commit=commit,
                teleop_ps4=teleop_ps4, safety_mode=prof.get('safety_mode'),
                stop_method=method, converter=prof['converter'], converter_params=converter_params,
                step_gate=gate, teleop=teleop_info, ps4=ps4_info,
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

    machine = Machine(prof, p, live=args.live, ground_ok=args.t1_passed_confirmed or args.fast)
    pad = node.pad
    if prof.get('ps4'):
        say(teleop_start_text(prof, info['teleop_ps4']))
        say('START ' + info['teleop_ps4']['start_line'])
        machine.log(time.monotonic(), 'START', info['teleop_ps4']['start_line'])
    kb = Keyboard(Deadman(float(r['deadman_first_press_s']), float(r['deadman_repeat_s'])),
                  {} if pad else TELEOP_KEYS if prof.get('teleop') else None)
    # Audit log only from the vesc_driver of THIS ROS graph (a process on another domain is not ours).
    in_graph = 'vesc_driver' in node.get_node_names()
    audit_pid, t_run0 = (vesc_driver_pid() if in_graph else None), time.time()
    bag_log = open(out / 'bag.log', 'w')
    # stdin DEVNULL: the bag must never share the terminal that carries the SPACE deadman.
    bag = subprocess.Popen(bag_command(out / 'bag', BAG_TOPICS),
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
        if machine.ground and not machine.teleop:
            need, top = machine.ground_window()
            say('Ground: arm only with %.2f..%.2f m free ahead (LiDAR); runner brakes at %.2f m '
                '(mux gate %.2f m = backstop). m = placement readout.' % (
                    need, top, machine.brake_range(prof['speed']), machine.gate))
        if pad:
            say('PS4 TELEOP: options = arm (R1 released) | HOLD R1 = deadman | right stick UP = speed (%s, max %.2f '
                'm/s) | centre / down = brake | left stick = steer (+/-%.2f rad, %.1f rad/s) | L1 held = brake %g A, '
                'stays armed | after any stop: centre the right stick first | release R1 = brake + segment end '
                '(options again) | circle = KILL (latched until options). Laptop keys: n = arm, m = placement, '
                'q = finish (idle); while armed EVERY key (SPACE too) = KILL.'
                % (prof['throttle_mode'], prof['speed'], abs(prof['steer']), TELEOP_STEER_RATE,
                   float(prof['ps4_cfg']['brake_current_a'])))
        elif machine.teleop:
            say('TELEOP keys: n = arm; HOLD w or SPACE = straight, a = left, d = right (%.2f m/s, steer +/-%.2f rad); '
                'change key by pressing the new one BEFORE releasing the old; release all = brake 5 A; q = finish '
                '(idle). While armed/driving any other key = KILL.' % (prof['speed'], abs(prof['steer'])))
        if not pad:
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
            w['drive_dir'] = pad.steer_frac() if pad else kb.direction()
            w['ps4_problem'] = pad.problem(now) if pad else None
            if pad:
                w.update(pad.machine_inputs())
            if w.get('imu_problem') != last_imu:
                last_imu = w.get('imu_problem')
                machine.log(now, 'IMU_STATUS', last_imu or 'OK')
                say('IMU: %s' % (last_imu or 'OK'))
            key = kb.get_key()
            if key is None and pad:
                key = pad.get_key()
            speed, steer, dead, req = machine.step(now, key, pad.held(now) if pad else kb.held(now), w)
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
                if machine.message.startswith('*** KILL ***'):
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
                          dict(vesc_speed_audit=audit, bag_returncode=rc), pad)
        except Exception as exc:  # noqa: BLE001
            say('Writing runner outputs failed: %s' % exc)
        say('VESC audit: %s' % audit)
        nxt = 'hall_loop_map.py' if prof.get('teleop') else 'analyze.py'
        say('Results: %s\nNext: python3 %s %s' % (out, Path(__file__).with_name(nxt), out))


def write_outputs(out, machine, intervals, keys, status_log, extra, pad=None):
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
    if pad is not None:
        with open(out / 'ps4_events.csv', 'w', newline='') as fp:
            wr = csv.writer(fp)
            wr.writerow(['wall', 'mono', 'event'])
            wr.writerows(pad.log)
        summary['ps4_msg_gaps'] = interval_stats(pad.gaps)
        summary['ps4_link'] = pad.link.stats() if pad.link is not None else None
    (out / 'runner_summary.json').write_text(json.dumps(summary, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--test', choices=sorted(PROFILES))
    ap.add_argument('--live', action='store_true',
                    help='Mux real actuation is enabled for an approved step. Never for dry_run.')
    ap.add_argument('--t1-passed-confirmed', action='store_true',
                    help='Ground profiles only, after Moatasem confirmed T1 in writing.')
    ap.add_argument('--fast', action='store_true',
                    help='Skip the T3 clear-disk / visibility refusals, the T3 0.3-first gate and '
                         '--t1-passed-confirmed; hands-off 0.5 s. Recorded as "fast": true in run_info.json.')
    ap.add_argument('--params', help='vehicle_params.yaml (default config/real_car/)')
    ap.add_argument('--out-root', help='results root (default ~/f1tenth_hw_evidence/real_car_tests)')
    ap.add_argument('--self-test', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--steer', type=float, help='t3_circle: constant steering angle, rad (+ = left)')
    ap.add_argument('--full-lock', choices=['left', 'right'],
                    help='t3_circle: servo exactly at the T2 driving clip of that side (minimum radius)')
    ap.add_argument('--speed', type=float, help='stepped profiles (t1_stop, t1b_stop, t4_straight): this step, m/s; '
                    'teleop_ps4: teleop_max_speed_mps (default 0.3, hard ceiling 1.5)')
    ap.add_argument('--throttle-mode', choices=THROTTLE_MODES,
                    help='teleop_ps4: fixed (stick up > 20%% = --speed) or proportional; default ps4_teleop.yaml')
    ap.add_argument('--ps4-synthetic', action='store_true', help=argparse.SUPPRESS)  # laptop stub test only
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
