#!/usr/bin/env python3
"""LAPTOP-ONLY end-to-end test of the PS4 teleop chain (28 Nov): the real physical_drive_mux (src copy), the real
t1b_stop_converter (table yaml, brake 5 A) and the real run_test.py --test teleop_ps4 (under a pty), with a STUB
VESC (node vesc_driver: records /commands/*, simulates ERPM on /sensors/core), a fake joy_node on /ps4/joy, a fake
LiDAR (/scan) and a fake IMU (/imu/data_raw). Nothing here can reach hardware: it refuses to start with a VESC
serial device present, a running vesc_driver_node, ROS_DOMAIN_ID 42 (the car) or without ROS_LOCALHOST_ONLY=1.

Phase A, mux safety_mode teleop: controller check, arm, drive; L1 brake + stays armed; L1 released with the stick
up = no motion until the centre; stick centre / down = brake; obstacle 0.5 m = WARN, no stop; joy silent and old
stamps = brake (onset < 0.3 s); circle latched until re-arm; R1 release; keyboard runner refused (runner + mux).
Phase B, mux safety_mode autonomy: the same runner latches STOP on the 0.80 m obstacle gate and on FRONT_UNKNOWN.

Needs ROS Humble + vesc_msgs (python) sourced, ros2 bag (zstd). Example (scratch overlay for vesc_msgs):
  export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=1; unset CYCLONEDDS_URI
  source /opt/ros/humble/setup.bash; source <vesc_msgs install>/setup.bash
  /usr/bin/python3 tools/real_car_tests/teleop_stub_test.py --work <scratch dir>
"""
import argparse
import glob
import json
import math
import os
import pty
import random
import re
import select
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
MUX = REPO / 'src' / 'f1tenth_bringup' / 'scripts' / 'physical_drive_mux.py'
CONV = HERE / 't1b_stop_converter.py'
RUNNER = HERE / 'run_test.py'
TABLE = REPO / 'config' / 'real_car' / 't1b_converter_t3_table.yaml'
VESC_YAML = REPO / 'config' / 'real_car' / 'vesc_teleop_erpm12620.yaml'
GAIN = 8414.0
R1, L1, CIRCLE, OPTIONS = 5, 4, 1, 9
NB, REST = 13, [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]


def guard():
    why = []
    if os.environ.get('ROS_DOMAIN_ID', '0') in ('42', ''):
        why.append('ROS_DOMAIN_ID is %r (car domain 42 / unset): export ROS_DOMAIN_ID=43' % os.environ.get(
            'ROS_DOMAIN_ID'))
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        why.append('ROS_LOCALHOST_ONLY must be 1')
    if glob.glob('/dev/serial/by-id/*ChibiOS*') or glob.glob('/dev/serial/by-id/*STMicro*'):
        why.append('a VESC serial device is present: this is not the laptop test setup')
    if subprocess.run(['pgrep', '-x', 'vesc_driver_nod'], capture_output=True).returncode == 0:
        why.append('a vesc_driver_node is running')
    if why:
        raise SystemExit('teleop_stub_test refused:\n  ' + '\n  '.join(why))


class World:
    """All fake nodes in this process (stub VESC, joy_node, LiDAR, IMU, status listener)."""

    def __init__(self):
        import rclpy
        import yaml
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Imu, Joy, LaserScan
        from std_msgs.msg import Float64, String
        from vesc_msgs.msg import VescStateStamped
        rclpy.init()
        self.rclpy, self.lock = rclpy, threading.Lock()
        self.cmds = []                       # (mono, kind, value)
        self.erpm, self.status, self.status_log = 0.0, None, []
        self.joy_on, self.buttons, self.axes, self.stamp_offset = True, [0] * NB, list(REST), 0.0
        self.front = 3.0                     # front +/-25 deg range (inf = no return)
        vparams = yaml.safe_load(VESC_YAML.read_text())['vesc_driver']['ros__parameters']
        w = self

        class Vesc(Node):
            def __init__(self):
                super().__init__('vesc_driver')
                for k, v in vparams.items():
                    self.declare_parameter(k, v)
                for kind in ('speed', 'brake', 'current'):
                    self.create_subscription(Float64, '/commands/motor/' + kind,
                                             lambda m, k_=kind: w.cmd(k_, m.data), 10)
                self.create_subscription(Float64, '/commands/servo/position', lambda m: w.cmd('servo', m.data), 10)
                self.pub = self.create_publisher(VescStateStamped, '/sensors/core', 10)
                self.create_timer(0.02, self.tick)
                self.t = time.monotonic()

            def tick(self):
                now = time.monotonic()
                dt, self.t = now - self.t, now
                with w.lock:
                    last = [c for c in w.cmds if c[1] != 'servo'][-1:] or [(0.0, 'current', 0.0)]
                t_c, kind, val = last[0]
                with w.lock:
                    if now - t_c > 0.25 or kind == 'brake':             # VESC timeout brake or commanded brake
                        w.erpm = max(0.0, w.erpm - GAIN * 0.9 * dt)
                    elif kind == 'speed':
                        w.erpm += (val - w.erpm) * min(1.0, dt / 0.1)
                    else:
                        w.erpm *= max(0.0, 1.0 - 0.5 * dt)
                    e = w.erpm
                m = VescStateStamped()
                m.header.stamp = self.get_clock().now().to_msg()
                m.state.speed = float(e)
                m.state.current_motor = 0.0
                m.state.fault_code = 0
                self.pub.publish(m)

        class JoyN(Node):
            def __init__(self):
                super().__init__('joy_node')
                self.declare_parameter('autorepeat_rate', 50.0)
                self.declare_parameter('deadzone', 0.0)
                self.declare_parameter('sticky_buttons', False)
                self.declare_parameter('device_id', 0)
                self.pub = self.create_publisher(Joy, '/ps4/joy', 10)
                self.create_timer(0.02, self.tick)

            def tick(self):
                with w.lock:
                    if not w.joy_on:
                        return
                    b, a, off = list(w.buttons), list(w.axes), w.stamp_offset
                m = Joy()
                ns = self.get_clock().now().nanoseconds + int(off * 1e9)
                m.header.stamp.sec, m.header.stamp.nanosec = ns // 1000000000, ns % 1000000000
                m.buttons, m.axes = b, a
                self.pub.publish(m)

        class Sensors(Node):
            def __init__(self):
                super().__init__('fake_sensors')
                self.scan_pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
                self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
                self.create_timer(0.1, self.scan)
                self.create_timer(0.01, self.imu)
                self.create_subscription(String, '/control/status', self.on_status, 10)

            def scan(self):
                s = LaserScan()
                s.header.stamp = self.get_clock().now().to_msg()
                s.header.frame_id = 'laser'
                s.angle_min, s.angle_increment = -math.pi, 2 * math.pi / 1440
                s.angle_max = s.angle_min + 1439 * s.angle_increment
                s.range_min, s.range_max = 0.05, 30.0
                with w.lock:
                    f = w.front
                s.ranges = [float(f) if abs(-math.pi + i * s.angle_increment) <= math.radians(25) else 3.0
                            for i in range(1440)]
                self.scan_pub.publish(s)

            def imu(self):
                m = Imu()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'imu_link'
                m.angular_velocity.z = random.gauss(0.0, 0.001)
                m.linear_acceleration.z = 9.81 + random.gauss(0.0, 0.01)
                self.imu_pub.publish(m)

            def on_status(self, msg):
                st = json.loads(msg.data)
                with w.lock:
                    w.status = st
                    w.status_log.append((time.monotonic(), st))

        self.nodes = [Vesc(), JoyN(), Sensors()]
        self.ex = MultiThreadedExecutor()
        for n in self.nodes:
            self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def cmd(self, kind, value):
        with self.lock:
            self.cmds.append((time.monotonic(), kind, float(value)))

    def set(self, buttons=(), **axes):
        with self.lock:
            self.buttons = [1 if i in buttons else 0 for i in range(NB)]
            self.axes = list(REST)
            for k, v in axes.items():
                self.axes[int(k[1:])] = v

    def first_after(self, t0, kind, pred=lambda v: True):
        with self.lock:
            for t, k, v in self.cmds:
                if t > t0 and k == kind and pred(v):
                    return t
        return None

    def moving_after(self, t0):
        return self.first_after(t0, 'speed', lambda v: v > 1.0)

    def close(self):
        self.ex.shutdown()
        for n in self.nodes:
            n.destroy_node()
        self.rclpy.try_shutdown()


class Proc:
    """A child process; a pty (runner) or a log file."""

    def __init__(self, cmd, env, log, tty=False):
        self.buf, self.lock = '', threading.Lock()
        self.logf = open(log, 'w')
        if tty:
            self.master, slave = pty.openpty()
            self.p = subprocess.Popen(cmd, stdin=slave, stdout=slave, stderr=slave, env=env, start_new_session=True)
            os.close(slave)
            threading.Thread(target=self.read, daemon=True).start()
        else:
            self.master = None
            self.p = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=self.logf, stderr=subprocess.STDOUT,
                                      env=env, start_new_session=True)

    def read(self):
        while True:
            try:
                r, _, _ = select.select([self.master], [], [], 0.1)
                if not r:
                    if self.p.poll() is not None:
                        return
                    continue
                data = os.read(self.master, 4096).decode(errors='replace')
            except OSError:
                return
            if not data:
                return
            with self.lock:
                self.buf += data
            self.logf.write(data)
            self.logf.flush()

    def wait_for(self, pattern, timeout=15.0, since=0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            with self.lock:
                m = re.search(pattern, self.buf[since:])
            if m:
                return since + m.end()
            if self.p.poll() is not None and self.master is None:
                break
            time.sleep(0.02)
        raise RuntimeError('timeout waiting for %r; tail:\n%s' % (pattern, self.buf[-1500:]))

    def mark(self):
        with self.lock:
            return len(self.buf)

    def key(self, k):
        os.write(self.master, k.encode())

    def stop(self, timeout=10.0):
        if self.p.poll() is None:
            os.killpg(self.p.pid, 2)
            try:
                self.p.wait(timeout)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, 9)
        self.logf.close()


class Checks:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=''):
        self.rows.append((name, bool(ok), detail))
        print('%s  %s  %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)


def press(w, button, hold=0.25, **axes):
    w.set((button,), **axes)
    time.sleep(hold)
    w.set((), **axes)
    time.sleep(0.15)


def controller_check(w, r):
    r.wait_for(r'PS4 check 2/4', 30)
    w.set((), a0=1.0)
    r.wait_for(r'steer = axis 0')
    w.set(())
    r.wait_for(r'PS4 check 3/4')
    w.set((), a4=1.0)
    r.wait_for(r'throttle = axis 4')
    w.set(())
    for name, b in (('R1', R1), ('L1', L1), ('circle', CIRCLE)):
        r.wait_for(r'press and release %s' % name)
        time.sleep(0.1)
        press(w, b)
    r.wait_for(r'PS4 check PASSED')


def arm_and_drive(w, r, label, c, since=None):
    """options -> ARMED, then R1 + centre, then R1 + stick up; returns when the stub sees speed commands."""
    w.set(())
    time.sleep(0.2)
    mk = r.mark() if since is None else since
    press(w, OPTIONS, 0.15)
    r.wait_for(r'ARMED', 5, mk)
    w.set((R1,))
    time.sleep(0.3)
    t0 = time.monotonic()
    w.set((R1,), a4=1.0)
    t_move = None
    while time.monotonic() < t0 + 3.0 and t_move is None:
        t_move = w.moving_after(t0)
        time.sleep(0.02)
    c.add('%s: drives after arm + R1 + stick up' % label, t_move is not None,
          '' if t_move is None else 'first speed command %.2f s after stick up' % (t_move - t0))
    time.sleep(1.0)                          # steady (ramp 0.6 s to 0.3 m/s), converter in DRIVE
    return t_move is not None


def onset(w, t0, timeout=1.5):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        tb = w.first_after(t0, 'brake', lambda v: v > 0)
        if tb is not None:
            return tb - t0
        time.sleep(0.01)
    return None


def no_motion(w, t0, dur):
    time.sleep(dur)
    return w.moving_after(t0) is None


def start_stack(work, env, mode, speed):
    mux = Proc(['/usr/bin/python3', '-u', str(MUX), '--ros-args', '-p', 'enable_real_actuation:=true',
                '-p', 'enable_autonomous_actuation:=false', '-p', 'manual_allow_reduced_visibility:=false',
                '-p', 'manual_min_front_valid:=0.75', '-p', 'safety_mode:=%s' % mode,
                '-p', 'teleop_ps4_topic:=/ps4/joy', '-p', 'teleop_ps4_deadman_button:=5',
                '-p', 'max_speed_mps:=%s' % speed, '-p', 'max_steer_rad:=0.28'], env, work / ('mux_%s.log' % mode))
    conv = Proc(['/usr/bin/python3', '-u', str(CONV), '--ros-args', '-p', 'method:=brake', '-p',
                 'brake_current_a:=5.0', '-p', 'brake_hold_s:=2.0', '--params-file', str(TABLE)], env,
                work / ('conv_%s.log' % mode))
    time.sleep(3.0)
    return mux, conv


def runner(work, env, out, test='teleop_ps4', speed='0.3', tag='a'):
    return Proc(['/usr/bin/python3', '-u', str(RUNNER), '--test', test, '--speed', speed, '--fast',
                 '--ps4-synthetic', '--out-root', str(out)], env, work / ('runner_%s.log' % tag), tty=True)


def phase_teleop(w, work, env, out, c):
    mux, conv = start_stack(work, env, 'teleop', '0.3')
    try:
        st = w.status or {}
        c.add('A: mux status safety_mode teleop', st.get('safety_mode') == 'teleop', str(st.get('safety_mode')))
        r = runner(work, env, out, tag='teleop')
        try:
            controller_check(w, r)
            r.wait_for(r'safety_mode=teleop throttle_mode=fixed teleop_max_speed_mps=0.30 brake_current_a=5.0 '
                       r'vesc_yaml=vesc_teleop_erpm12620.yaml', 10)
            c.add('A: START line (safety_mode, throttle_mode, speed, brake, VESC yaml, estimate)', True)
            r.wait_for(r'PS4 TELEOP: options', 10)
            arm_and_drive(w, r, 'A', c)
            # L1 overrides a full stick, stays armed
            t0 = time.monotonic()
            w.set((R1, L1), a4=1.0)
            d = onset(w, t0)
            time.sleep(1.0)
            st = w.status or {}
            c.add('A: L1 brake onset with the stick full up', d is not None and d < 0.3, 'onset %s s' % d)
            c.add('A: L1 held: still armed (mux MANUAL_ACTIVE)', st.get('mode') == 'MANUAL_ACTIVE', st.get('mode'))
            # L1 released with the stick still up: no motion until the centre
            t1 = time.monotonic()
            w.set((R1,), a4=1.0)
            c.add('A: L1 released, stick up: no motion for 1.0 s', no_motion(w, t1, 1.0))
            w.set((R1,))
            time.sleep(0.1)
            t2 = time.monotonic()
            w.set((R1,), a4=1.0)
            time.sleep(1.0)
            c.add('A: stick through the centre -> drives again', w.moving_after(t2) is not None)
            # stick centre = brake; then up again at once (no block for a centre stop)
            t3 = time.monotonic()
            w.set((R1,), a4=0.05)
            d = onset(w, t3)
            c.add('A: stick in the deadzone = brake', d is not None and d < 0.3, 'onset %s s' % d)
            time.sleep(0.5)
            t4 = time.monotonic()
            w.set((R1,), a4=1.0)
            time.sleep(0.8)
            c.add('A: after a centre stop, stick up drives at once', w.moving_after(t4) is not None)
            time.sleep(0.5)
            # stick down = brake + block
            t5 = time.monotonic()
            w.set((R1,), a4=-0.8)
            d = onset(w, t5)
            c.add('A: stick down = brake', d is not None and d < 0.3, 'onset %s s' % d)
            time.sleep(0.3)
            t6 = time.monotonic()
            w.set((R1,), a4=1.0)
            c.add('A: down -> up without the centre: no motion for 0.8 s', no_motion(w, t6, 0.8))
            w.set((R1,))
            time.sleep(0.1)
            w.set((R1,), a4=1.0)
            time.sleep(1.2)
            # obstacle at 0.5 m: WARN only
            t7 = time.monotonic()
            with w.lock:
                w.front = 0.5
            time.sleep(1.0)
            st = w.status or {}
            c.add('A: obstacle 0.5 m in teleop: no brake for 1.0 s', w.first_after(t7, 'brake') is None)
            c.add('A: obstacle 0.5 m in teleop: WARN fields', 'OBSTACLE_NEAR' in str(st.get('teleop_warn'))
                  and st.get('teleop_warn_obstacle_near') is True, str(st.get('teleop_warn')))
            with w.lock:
                w.front = float('inf')
            time.sleep(0.6)
            st = w.status or {}
            c.add('A: no front return in teleop: WARN FRONT_UNKNOWN, still moving',
                  'FRONT_UNKNOWN' in str(st.get('teleop_warn')) and w.first_after(t7, 'brake') is None,
                  str(st.get('teleop_warn')))
            with w.lock:
                w.front = 3.0
            time.sleep(0.4)
            # controller lost: joy silent
            t8 = time.monotonic()
            with w.lock:
                w.joy_on = False
            d = onset(w, t8)
            c.add('A: joy silent -> brake onset < 0.3 s', d is not None and d < 0.3, 'onset %s s' % d)
            r.wait_for(r'ZERO_COMMAND PS4_STALE', 5)          # CONTROLLER_LOST + zero in the same tick
            time.sleep(1.5)
            with w.lock:
                w.joy_on = True
            r.wait_for(r'teleop segment', 5)
            arm_and_drive(w, r, 'A re-arm after joy loss', c)
            # old stamps
            t9 = time.monotonic()
            with w.lock:
                w.stamp_offset = -0.3
            d = onset(w, t9)
            c.add('A: /ps4/joy stamp 0.3 s old -> brake onset < 0.3 s', d is not None and d < 0.3, 'onset %s s' % d)
            time.sleep(1.5)
            with w.lock:
                w.stamp_offset = 0.0
            r.wait_for(r'teleop segment', 5)
            arm_and_drive(w, r, 'A re-arm after old stamps', c)
            # circle = KILL, latched until re-arm
            t10 = time.monotonic()
            w.set((R1, CIRCLE), a4=1.0)
            d = onset(w, t10)
            c.add('A: circle KILL -> brake', d is not None and d < 0.3, 'onset %s s' % d)
            w.set((R1,), a4=1.0)
            mk = r.mark()
            r.wait_for(r'teleop segment', 5)
            c.add('A: after KILL, R1 + stick up: no motion for 1.5 s', no_motion(w, t10 + 0.5, 1.5))
            press(w, OPTIONS, 0.15, a4=0.0)          # R1 released by press(): options alone re-arms
            r.wait_for(r'ARMED', 5, mk)
            c.add('A: KILL latch cleared only by options (re-arm)', True)
            w.set((R1,))
            time.sleep(0.3)
            w.set((R1,), a4=1.0)
            time.sleep(1.2)
            # R1 release
            t11 = time.monotonic()
            w.set((), a4=1.0)
            d = onset(w, t11)
            c.add('A: R1 release -> brake onset < 0.3 s', d is not None and d < 0.3, 'onset %s s' % d)
            r.wait_for(r'teleop segment', 5)
            w.set(())
            r.key('q')
            r.wait_for(r'Results: ', 30)
            time.sleep(1.0)
        finally:
            r.stop()
        folder = max((p for p in out.glob('*_teleop_ps4_*')), key=lambda p: p.stat().st_mtime)
        ev = (folder / 'runner_events.csv').read_text()
        for e in ('START', 'ARMED', 'DEADMAN_RELEASED', 'BRAKE_BUTTON', 'STICK_STOP', 'KILL', 'CONTROLLER_LOST',
                  'NO_JUMP_START', 'WARN', 'WARN_CLEAR'):
            c.add('A: runner_events.csv has %s' % e, (',%s,' % e) in ev)
        c.add('A: bag written', (folder / 'bag' / 'metadata.yaml').exists(), folder.name)
        # keyboard runner refused against the teleop mux
        k = runner(work, env, out, test='teleop', tag='keyboard')
        try:
            k.wait_for(r'PS4 controller only', 20)
            c.add('A: keyboard runner refused (mux safety_mode teleop)', True)
        finally:
            k.stop()
        # mux refuses a plain MANUAL request itself
        from rclpy.node import Node
        from std_msgs.msg import String
        n = Node('manual_request_probe')
        w.ex.add_node(n)
        pub = n.create_publisher(String, '/control/request_mode', 10)
        time.sleep(1.0)
        for _ in range(5):
            pub.publish(String(data='MANUAL'))
            time.sleep(0.1)
        time.sleep(0.3)
        st = w.status or {}
        c.add('A: mux rejects a plain MANUAL in teleop', st.get('mode_request_result') ==
              'REJECT_TELEOP_SAFETY_MODE_NEEDS_PS4_RUNNER' and st.get('mode') == 'STOP', st.get('mode_request_result'))
        w.ex.remove_node(n)
        n.destroy_node()
        return folder
    finally:
        mux.stop()
        conv.stop()


def phase_autonomy(w, work, env, out, c):
    with w.lock:
        w.front = 3.0
    mux, conv = start_stack(work, env, 'autonomy', '0.3')
    try:
        st = w.status or {}
        c.add('B: mux status safety_mode autonomy', st.get('safety_mode') == 'autonomy', str(st.get('safety_mode')))
        r = runner(work, env, out, tag='autonomy')
        try:
            controller_check(w, r)
            r.wait_for(r'safety_mode=autonomy', 10)
            r.wait_for(r'PS4 TELEOP: options', 10)
            arm_and_drive(w, r, 'B', c)
            t0 = time.monotonic()
            with w.lock:
                w.front = 0.5
            d = onset(w, t0)
            time.sleep(0.3)
            st = w.status or {}
            c.add('B: autonomy: obstacle 0.5 m -> brake (latched)', d is not None and d < 0.5 and
                  st.get('last_stop_reason') == 'LATCH_STOP_FRONT_SCAN_NOT_CLEAR', 'onset %s s, %s' % (
                      d, st.get('last_stop_reason')))
            r.wait_for(r'teleop segment', 5)
            with w.lock:
                w.front = 3.0
            time.sleep(0.5)
            w.set(())
            time.sleep(0.3)
            arm_and_drive(w, r, 'B re-arm', c)
            t1 = time.monotonic()
            with w.lock:
                w.front = float('inf')
            d = onset(w, t1)
            time.sleep(0.3)
            st = w.status or {}
            c.add('B: autonomy: no front return (FRONT_UNKNOWN) -> brake (latched)', d is not None and
                  st.get('last_stop_reason') == 'LATCH_STOP_FRONT_SCAN_NOT_CLEAR' and st.get('scan_reason') ==
                  'FRONT_UNKNOWN', 'onset %s s, %s / %s' % (d, st.get('last_stop_reason'), st.get('scan_reason')))
            c.add('B: autonomy status has no teleop warning', st.get('teleop_warn') is None)
            r.wait_for(r'teleop segment', 5)
            w.set(())
            r.key('q')
            r.wait_for(r'Results: ', 30)
        finally:
            r.stop()
    finally:
        mux.stop()
        conv.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--work', required=True, help='scratch folder (logs, HOME of the children, run folders)')
    args = ap.parse_args()
    guard()
    work = Path(args.work).expanduser().resolve()
    (work / 'home').mkdir(parents=True, exist_ok=True)
    out = work / 'runs'
    out.mkdir(exist_ok=True)
    env = dict(os.environ, HOME=str(work / 'home'), PYTHONUNBUFFERED='1')
    w = World()
    c = Checks()
    try:
        phase_teleop(w, work, env, out, c)
        phase_autonomy(w, work, env, out, c)
    except RuntimeError as exc:
        c.add('harness', False, str(exc))
    finally:
        w.close()
    bad = [r for r in c.rows if not r[1]]
    print('\n%d checks, %d failed. Logs: %s' % (len(c.rows), len(bad), work))
    (work / 'checks.json').write_text(json.dumps(c.rows, indent=1))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
