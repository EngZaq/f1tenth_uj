#!/usr/bin/env python3
"""Phase F (21 Nov): replay T4 bags through the real-car SHADOW EKF variants, offline, and compare
them with independent LiDAR references.

For every T4 repeat that moved (> MIN_TOTAL_M) in the given run folders (analyze.py --t4 must have run):
1. Build an input bag (work dir) with ekf_shadow_inputs.py (the SAME functions as the live node), window
   1 s before the command .. end of the brake hold: /ekf_shadow/in/{vesc,vesc_dw,imu,rf2o}. Brake state
   from the bag's /t1b_stop_converter/status. Header stamps = sensor time; bag time = original receive
   time (RF2O stays ~51 ms late, as live). Still calibration (gyro bias + gravity): --calib pre (the 1 s
   before each command, default) or startup (first still 2 s of the session, as the live node does).
2. Run robot_localization ekf_node per variant (config/real_car/ekf_real_shadow.yaml) with use_sim_time,
   play the input bag with --clock, record /ekf_shadow/odom and the ekf_node CPU time. Isolated ROS
   domain DOMAIN; refuses if a car stack runs.
3. References (same code and windows as analyze.py --t4): LiDAR wall-fit perpendicular range and heading.
   EKF progress = displacement of the LASER point (base_link + lidar.x_m) along the start wall normal.
   Phases: start ramp (command .. start delay + v / ramp accel), brake (brake command .. standstill),
   total (still .. still); hold drift = EKF progress during the LiDAR still window (car not moving).
   Heading error vs RF2O; latency-free cruise gain.
--rf2o-cpu: play /scan + /tf_static of one bag into rf2o_laser_odometry_node (stack params) and report its
CPU on this machine.
Writes ekf_shadow_replay.json into each run folder; prints a per-variant summary. Publishes only in DOMAIN.
  ekf_shadow_replay.py [<folder> ...] [--variants A B C0 C1 C2] [--calib pre|startup] [--work DIR]
  ekf_shadow_replay.py --rf2o-cpu [<folder>]
  ekf_shadow_replay.py --live-check [<folder>]    live node (ekf_shadow_inputs.py) vs replay inputs
  ekf_shadow_replay.py --self-test
"""
import argparse
import bisect
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import ekf_shadow_inputs as inp  # noqa: E402
import params as vp  # noqa: E402

DOMAIN = 88
REPO = Path(__file__).resolve().parents[2]
YAML = REPO / 'config' / 'real_car' / 'ekf_real_shadow.yaml'
DEFAULT_RUNS = ['20261120_203053_t4_straight_v0.50_brake5A_erpmmax4630_armed',   # carpet, offset 0.358
                '20261120_210243_t4_straight_v0.50_brake5A_erpmmax4630_armed']   # tile, offset 0.358
# Not replayed: 200901 (carpet 0.320) and 211256 (tile 0.340): IMU frozen (F-18).
VARIANTS = {'A': ('ekf_real_shadow', 'VESC vx + vy=0 + gyro'),
            'B': ('ekf_real_shadow_rf2o', 'A + RF2O vx, vyaw'),
            'C0': ('ekf_real_shadow_c0', 'A, VESC de-weighted, no fallback (check)'),
            'C1': ('ekf_real_shadow_c1', 'A, VESC de-weighted + RF2O vx'),
            'C2': ('ekf_real_shadow_c2', 'A, VESC de-weighted + BMI160 forward accel')}
BIAS_S = 1.0
MIN_TOTAL_M = 0.3
CRUISE_PAD_S = 0.2     # cruise window starts this long after the ramp end
STACK_PATTERNS = ('vesc_driver_node', 'bmi160_node', 'lakibeam1_scan_node', 'rf2o_laser_odometry_node',
                  'async_slam_toolbox_node', 'physical_drive_mux', 't1b_stop_converter', 'ekf_node')
RF2O_PARAMS = dict(laser_scan_topic='/scan', odom_topic='/odom_rf2o', publish_tf=True, base_frame_id='base_link',
                   odom_frame_id='odom', init_pose_from_topic='', freq=20.0, use_sim_time=True)
# = src/f1tenth_bringup/launch/observe_only.launch.py (+ use_sim_time). init_pose_from_topic '' is required:
# otherwise rf2o waits for a ground-truth pose and never processes a scan.


# ---------------------------------------------------------------- pure logic

def laser_progress(psi_pre, pose0, pose, laser_x):
    """Progress of the laser toward the wall (m, + = closer) between two base_link poses (x, y, yaw)
    in the EKF odom frame. psi_pre = heading to the wall normal at pose0 (wall_fit, + = turned left):
    the normal, in the pose0 car frame, is (cos psi, -sin psi)."""
    x0, y0, h0 = pose0
    x1, y1, h1 = pose
    c, s = math.cos(-h0), math.sin(-h0)
    dx, dy = c * (x1 - x0) - s * (y1 - y0), s * (x1 - x0) + c * (y1 - y0)    # in the pose0 frame
    dh = h1 - h0
    lx, ly = dx + laser_x * math.cos(dh) - laser_x, dy + laser_x * math.sin(dh)
    return lx * math.cos(psi_pre) - ly * math.sin(psi_pre)


def unwrap_series(vals):
    out, off = [], 0.0
    for i, v in enumerate(vals):
        if i:
            d = v + off - out[-1]
            off -= 2 * math.pi * round(d / (2 * math.pi))
        out.append(v + off)
    return out


def state_at(times, states, t):
    """Latest state at or before t (step function); default False."""
    i = bisect.bisect_right(times, t) - 1
    return states[i] if i >= 0 else False


def summarize(vals):
    v = [x for x in vals if x is not None]
    if not v:
        return None
    return dict(mean=statistics.mean(v), min=min(v), max=max(v), n=len(v))


def self_test():
    import random
    rnd = random.Random(1)
    lx = 0.215
    # Wall x = 3 in the world; laser at base + R(h)(lx, 0); d = 3 - laser_x; psi = h (turned left = +).
    for _ in range(200):
        b0 = (rnd.uniform(-0.5, 0.5), rnd.uniform(-0.5, 0.5), rnd.uniform(-0.3, 0.3))
        b1 = (b0[0] + rnd.uniform(0, 1.5), b0[1] + rnd.uniform(-0.3, 0.3), b0[2] + rnd.uniform(-0.3, 0.3))
        d0 = 3 - (b0[0] + lx * math.cos(b0[2]))
        d1 = 3 - (b1[0] + lx * math.cos(b1[2]))
        a, ox, oy = rnd.uniform(-3, 3), rnd.uniform(-2, 2), rnd.uniform(-2, 2)

        def tf(p):
            return (ox + math.cos(a) * p[0] - math.sin(a) * p[1],
                    oy + math.sin(a) * p[0] + math.cos(a) * p[1], p[2] + a)
        got = laser_progress(b0[2], tf(b0), tf(b1), lx)
        assert abs(got - (d0 - d1)) < 1e-9, (got, d0 - d1)
    assert abs(laser_progress(0.0, (0, 0, 0), (1.0, 0, 0), lx) - 1.0) < 1e-12
    assert abs(laser_progress(0.0, (0, 0, 0), (0, 0, math.radians(10)), lx)
               - lx * (math.cos(math.radians(10)) - 1)) < 1e-12
    u = unwrap_series([3.1, -3.1, -3.0])
    assert abs(u[1] - (2 * math.pi - 3.1)) < 1e-9 and u[2] > u[1]
    ts, ss = [1.0, 2.0, 3.0], [False, True, False]
    assert state_at(ts, ss, 0.5) is False and state_at(ts, ss, 2.0) is True and state_at(ts, ss, 2.9) is True
    assert state_at(ts, ss, 3.5) is False
    assert summarize([None, 1.0, 3.0]) == dict(mean=2.0, min=1.0, max=3.0, n=2) and summarize([None]) is None
    assert set(VARIANTS) == {'A', 'B', 'C0', 'C1', 'C2'}
    import yaml
    cfg = yaml.safe_load(YAML.read_text())
    for key, (node, _) in VARIANTS.items():
        p = cfg[node]['ros__parameters']
        assert p['publish_tf'] is False and p['two_d_mode'] is True, node
        topics = [p[k] for k in ('odom0', 'odom1', 'imu0') if k in p]
        assert all(t in inp.TOPICS.values() for t in topics), (node, topics)
        assert ('/ekf_shadow/in/vesc_dw' in topics) == key.startswith('C'), node
        q = p['process_noise_covariance']
        assert q[6 * 15 + 6] == 0.05 and q[11 * 15 + 11] == 0.02, node
    print('PASS: laser progress along the wall normal (200 random poses, frame invariant, rotation in place),')
    print('      yaw unwrap, brake-state lookup, summary; yaml: 5 variants, publish_tf false, Q_vx 0.05,')
    print('      Q_vyaw 0.02, topics = ekf_shadow_inputs.TOPICS, de-weighted VESC only in C*.')


# ---------------------------------------------------------------- bag -> inputs -> EKF

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def calib_window(imu, a, b):
    c = inp.StillCalib()
    for t, m in imu:
        if a <= t < b:
            g, x = m.angular_velocity, m.linear_acceleration
            c.add((g.x, g.y, g.z), (x.x, x.y, x.z))
    return c.result()


def calib_startup(imu):
    """First still block of StillCalib.SAMPLES samples of the session, as the live node does at start-up
    (a moving block is discarded and the next one tried)."""
    n = inp.StillCalib.SAMPLES
    for i in range(0, len(imu) - n + 1, n):
        c = inp.StillCalib()
        for _, m in imu[i:i + n]:
            g, x = m.angular_velocity, m.linear_acceleration
            c.add((g.x, g.y, g.z), (x.x, x.y, x.z))
        r = c.result()
        if r and not r['moving']:
            return r, imu[i][0]
    return None, None


def build_inputs(data, w0, w1, cal, gain, out_dir):
    import rosbag2_py
    from rclpy.serialization import serialize_message
    st_t, st_b = [], []
    for t, m in data.get('/t1b_stop_converter/status', []):
        st_t.append(t)
        st_b.append(inp.brake_active(m.data))
    msgs = []
    for t, m in data['/sensors/core']:
        if w0 <= t <= w1:
            br = state_at(st_t, st_b, t)
            msgs.append((t, inp.TOPICS['vesc'], inp.vesc_msg(m.header.stamp, m.state.speed, br, gain, False)))
            msgs.append((t, inp.TOPICS['vesc_dw'], inp.vesc_msg(m.header.stamp, m.state.speed, br, gain, True)))
    for t, m in data['/imu/data_raw']:
        if w0 <= t <= w1:
            a = m.linear_acceleration
            msgs.append((t, inp.TOPICS['imu'], inp.imu_msg(m.header.stamp, m.angular_velocity.z, (a.x, a.y, a.z), cal)))
    for t, m in data['/odom_rf2o']:
        if w0 <= t <= w1:
            msgs.append((t, inp.TOPICS['rf2o'], inp.rf2o_msg(m)))
    msgs.sort(key=lambda x: x[0])
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(out_dir), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for key, name in inp.TOPICS.items():
        typ = 'sensor_msgs/msg/Imu' if key == 'imu' else 'nav_msgs/msg/Odometry'
        w.create_topic(rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr'))
    for t, topic, m in msgs:
        w.write(topic, serialize_message(m), int(t * 1e9))
    del w
    return len(msgs)


class Recorder:
    def __init__(self):
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('ekf_shadow_replay_recorder',
                                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock, self.samples = threading.Lock(), []
        self.node.create_subscription(Odometry, '/ekf_shadow/odom', self.on_odom, 50)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.samples.append((stamp(m), m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                                 m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))

    def take(self):
        with self.lock:
            out, self.samples = self.samples, []
        return out

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()


def exe(pkg, name):
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', pkg], capture_output=True, text=True, check=True).stdout.strip()
    return str(Path(prefix) / 'lib' / pkg / name)


def cpu_s(pid):
    with open('/proc/%d/stat' % pid) as fp:
        f = fp.read().rsplit(')', 1)[1].split()
    return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK')


def play(in_bag, env, log, topics=None):
    cmd = ['ros2', 'bag', 'play', str(in_bag), '--clock', '100', '-d', '1.0', '--disable-keyboard-controls']
    if topics:
        cmd += ['--topics'] + topics
    subprocess.run(cmd, env=env, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=600)


def run_node(cmd, in_bag, env, log_path, rec=None, topics=None):
    """Start a node, play the bag, stop the node. Returns (samples, cpu seconds, wall seconds of play)."""
    if rec:
        rec.take()
    with open(log_path, 'w') as log:
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(2.0)
            c0, w0 = cpu_s(proc.pid), time.monotonic()
            play(in_bag, env, log, topics)
            c1, w1 = cpu_s(proc.pid), time.monotonic()
            time.sleep(0.5)
        finally:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
    out = sorted(rec.take(), key=lambda s: s[0]) if rec else None
    return out, c1 - c0, w1 - w0


def ekf_cmd(node_name):
    return [exe('robot_localization', 'ekf_node'), '--ros-args', '-r', '__node:=%s' % node_name,
            '--params-file', str(YAML), '-p', 'use_sim_time:=true', '-r', 'odometry/filtered:=/ekf_shadow/odom']


# ---------------------------------------------------------------- references and comparison

def repeat_refs(A, x, scan_t, walls, hold_s, ramp_accel, speed):
    """LiDAR references for one repeat, same windows as analyze.t4_repeat (wall fit)."""
    tc, t0 = x['t_cmd'], x['t0']
    a = max(t0 + x['stop_time_s'] + 0.2, t0 + 0.5)
    b = t0 + hold_s - 0.1
    dts = [t for t, w in zip(scan_t, walls) if w]
    dvs = [w['d'] for w in walls if w]
    pvs = [w['psi'] for w in walls if w]
    r_pre = A.med(A.in_window(dts, dvs, tc - A.PRE_S, tc - 0.02))
    r_still = A.med(A.in_window(dts, dvs, a, b))
    psi_pre = A.med(A.in_window(dts, pvs, tc - A.PRE_S, tc))
    psi_still = A.med(A.in_window(dts, pvs, a, b))
    t_ramp = min(tc + (x.get('start_delay_erpm_s') or 0.0) + speed / ramp_accel, t0)
    d_ramp, d_brake = A.interp(dts, dvs, t_ramp), A.interp(dts, dvs, t0)
    return dict(tc=tc, t_ramp=t_ramp, t0=t0, a=a, b=b, psi_pre=psi_pre,
                ramp=r_pre - d_ramp, cruise=d_ramp - d_brake, brake=d_brake - r_still, total=r_pre - r_still,
                heading=math.degrees(psi_still - psi_pre))


def cruise_gain(A, ref, scan_t, walls, core, rf2o, gain):
    """Latency-free scale check at constant speed: LiDAR range slope (wall d / cos psi) vs the mean
    VESC (and RF2O) speed over the cruise window. gain_true = gain x v_vesc / v_lidar."""
    a, b = ref['t_ramp'] + CRUISE_PAD_S, ref['t0'] - 0.05
    pts = [(t, w['d'], w['psi']) for t, w in zip(scan_t, walls) if w and a <= t <= b]
    if len(pts) < 10:
        return None
    sl = A.slope([q[0] for q in pts], [q[1] for q in pts])
    v_lidar = -sl / math.cos(statistics.mean(q[2] for q in pts))
    vv = [inp.vesc_vx(m.state.speed, gain) for t, m in core if a <= t <= b]
    vr = [m.twist.twist.linear.x for t, m in rf2o if a <= stamp(m) <= b]
    return dict(window_s=b - a, v_lidar=v_lidar, v_vesc=statistics.mean(vv), v_rf2o=statistics.mean(vr),
                gain_cruise_lidar=gain * statistics.mean(vv) / v_lidar, rf2o_over_lidar=statistics.mean(vr) / v_lidar)


def ekf_metrics(A, ref, samples, laser_x, rf2o_heading):
    ts = [s[0] for s in samples]
    yaw = unwrap_series([s[3] for s in samples])

    def pose(t):
        return tuple(A.interp(ts, v, t) for v in ([s[1] for s in samples], [s[2] for s in samples], yaw))
    tm = (ref['a'] + ref['b']) / 2          # standstill: middle of the LiDAR still window
    p = {k: pose(ref[k]) for k in ('tc', 't_ramp', 't0', 'a', 'b')}
    p['still'] = pose(tm)
    if any(None in v for v in p.values()):
        return None
    prog = {k: laser_progress(ref['psi_pre'], p['tc'], v, laser_x) for k, v in p.items()}
    heading = math.degrees(p['still'][2] - p['tc'][2])
    out = dict(ramp=prog['t_ramp'], cruise=prog['t0'] - prog['t_ramp'], brake=prog['still'] - prog['t0'],
               total=prog['still'], heading=heading, hold_drift=prog['b'] - prog['a'],
               vx_end_hold=A.interp(ts, [s[4] for s in samples], ref['b']))
    for k in ('ramp', 'brake', 'total'):
        out[k + '_err'] = out[k] - ref[k]
    out['heading_err_rf2o'] = None if rf2o_heading is None else heading - rf2o_heading
    return out


def stack_running():
    out = subprocess.run(['pgrep', '-af', '|'.join(STACK_PATTERNS)], capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if 'pgrep' not in ln and 'ekf_shadow_replay' not in ln]


def rf2o_cpu(folder, work, env):
    import yaml
    in_bag = folder / 'bag'
    pfile = work / 'rf2o_params.yaml'
    pfile.write_text(yaml.safe_dump({'rf2o_laser_odometry': {'ros__parameters': RF2O_PARAMS}}))
    cmd = [exe('rf2o_laser_odometry', 'rf2o_laser_odometry_node'), '--ros-args', '-r',
           '__node:=rf2o_laser_odometry', '--params-file', str(pfile)]
    _, cpu, wall = run_node(cmd, in_bag, env, work / 'rf2o_cpu.log', topics=['/scan', '/tf_static'])
    log = (work / 'rf2o_cpu.log').read_text()
    waits = log.count('Waiting for laser_scans')
    ncpu = os.cpu_count()
    print('rf2o "Waiting for laser_scans" lines: %d (timer ticks without a new scan; all ticks = no scans)' % waits)
    if waits > 0.5 * 20 * wall:
        print('INVALID: rf2o did not process scans (see %s)' % (work / 'rf2o_cpu.log'))
    print('rf2o_laser_odometry_node on %s (%d cores): %.1f s CPU over %.1f s of /scan replay = %.0f %% of one core '
          '(%.1f %% of the machine)' % (os.uname().nodename, ncpu, cpu, wall, 100 * cpu / wall, 100 * cpu / wall / ncpu))
    return dict(cpu_s=cpu, wall_s=wall, core_pct=100 * cpu / wall, machine_pct=100 * cpu / wall / ncpu, cores=ncpu)


def live_check(folder, work, env, gain):
    """Run the LIVE node (ekf_shadow_inputs.py) against the bag in DOMAIN, record /ekf_shadow/in/*, and
    compare every message with the replay inputs (start-up calibration, same stamps)."""
    import analyze as A
    out_bag = work / 'live_check_out'
    if out_bag.exists():
        import shutil
        shutil.rmtree(out_bag)
    log = open(work / 'live_check.log', 'w')
    rec = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(out_bag)] + list(inp.TOPICS.values()),
                           env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    node = subprocess.Popen([sys.executable, str(Path(__file__).with_name('ekf_shadow_inputs.py'))],
                            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(3.0)
        play(folder / 'bag', env, log, ['/sensors/core', '/imu/data_raw', '/odom_rf2o', '/t1b_stop_converter/status'])
        time.sleep(1.0)
    finally:
        for pr in (node, rec):
            os.killpg(pr.pid, signal.SIGINT)
            try:
                pr.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(pr.pid, signal.SIGKILL)
        log.close()
    live = A.read_bag(out_bag)
    data = A.read_bag(folder / 'bag')
    # The live node calibrates on the first SAMPLES messages it RECEIVES; the first few published before DDS
    # discovery finished are lost, so use the same samples here (still-only, as calib_startup).
    imu = data['/imu/data_raw']
    skip = max(0, len(imu) - len(live.get(inp.TOPICS['imu'], [])) - inp.StillCalib.SAMPLES)
    cal, _ = calib_startup(imu[skip:])
    print('live check: live node lost the first %d IMU messages (discovery); reference calibrated on the same samples'
          % skip)
    st_t = [t for t, _ in data['/t1b_stop_converter/status']]
    st_b = [inp.brake_active(m.data) for _, m in data['/t1b_stop_converter/status']]
    ref = {}
    for t, m in data['/sensors/core']:
        br = state_at(st_t, st_b, t)
        ref[('vesc', stamp(m))] = inp.vesc_msg(m.header.stamp, m.state.speed, br, gain, False)
        ref[('vesc_dw', stamp(m))] = inp.vesc_msg(m.header.stamp, m.state.speed, br, gain, True)
    for t, m in data['/imu/data_raw']:
        a = m.linear_acceleration
        ref[('imu', stamp(m))] = inp.imu_msg(m.header.stamp, m.angular_velocity.z, (a.x, a.y, a.z), cal)
    for t, m in data['/odom_rf2o']:
        ref[('rf2o', stamp(m))] = inp.rf2o_msg(m)
    names = {v: k for k, v in inp.TOPICS.items()}
    res = {}
    for topic, msgs in live.items():
        k = names.get(topic)
        if not k:
            continue
        n = same = cov_diff = 0
        worst = 0.0
        for _, m in msgs:
            r = ref.get((k, stamp(m)))
            if r is None:
                continue
            n += 1
            if k == 'imu':
                d = max(abs(m.angular_velocity.z - r.angular_velocity.z), abs(m.linear_acceleration.x - r.linear_acceleration.x))
                cd = m.linear_acceleration_covariance[0] != r.linear_acceleration_covariance[0]
            else:
                d = max(abs(m.twist.twist.linear.x - r.twist.twist.linear.x), abs(m.twist.twist.angular.z - r.twist.twist.angular.z))
                cd = m.twist.covariance[0] != r.twist.covariance[0]
            worst = max(worst, d)
            same += int(bool(d < 1e-6) and not bool(cd))
            cov_diff += int(bool(cd))
        res[k] = dict(live=len(msgs), matched=n, identical=same, cov_differs=cov_diff, worst_abs_diff=float(worst))
    print('live check (%s): live node vs replay inputs' % folder.name[:15])
    for k, r in sorted(res.items()):
        print('  %-8s live %5d, matched by stamp %5d, identical %5d, covariance differs %3d, worst value diff %.2e'
              % (k, r['live'], r['matched'], r['identical'], r['cov_differs'], r['worst_abs_diff']))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--gain', type=float, default=None, help='default: vehicle_params speed_to_erpm_gain')
    ap.add_argument('--variants', nargs='+', default=sorted(VARIANTS), choices=sorted(VARIANTS))
    ap.add_argument('--calib', choices=('pre', 'startup'), default='pre')
    ap.add_argument('--work', default=None, help='input bags + logs (default: a new temp dir)')
    ap.add_argument('--rf2o-cpu', action='store_true')
    ap.add_argument('--live-check', action='store_true', help='live node vs replay inputs on one bag')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    busy = stack_running()
    if busy:
        print('REFUSED: a car stack (or another EKF) is running; offline replays only with the stack down:')
        print('\n'.join('  ' + b for b in busy))
        return 2
    os.environ['ROS_DOMAIN_ID'] = str(DOMAIN)       # before rclpy.init; also for the children
    env = dict(os.environ)
    gain = args.gain or inp.gain_from_params()
    p = vp.load()
    folders = [Path(f).expanduser() for f in args.folders] or [vp.evidence_root() / r for r in DEFAULT_RUNS]
    work = Path(args.work or tempfile.mkdtemp(prefix='ekf_shadow_replay_')).expanduser()
    work.mkdir(parents=True, exist_ok=True)
    if args.live_check:
        r = live_check(folders[-1], work, env, gain)
        (work / 'live_check.json').write_text(json.dumps(r, indent=1))
        return 0
    if args.rf2o_cpu:
        r = rf2o_cpu(folders[-1], work, env)
        (work / 'rf2o_cpu.json').write_text(json.dumps(r, indent=1))
        return 0
    import analyze as A
    laser_x = float(vp.get(p, 'lidar.x_m'))
    wall_deg = float(vp.get(p, 't4.wall_fit_deg'))
    print('domain %d, gain %.0f, calib %s, work %s' % (DOMAIN, gain, args.calib, work))
    rec = Recorder()
    rows = []
    try:
        for folder in folders:
            summ = json.loads((folder / 'summary.json').read_text())
            info = json.loads((folder / 'run_info.json').read_text())
            hold = float((info.get('converter_params') or {}).get('brake_hold_s') or 2.0)
            ramp = float(info.get('ramp_accel_mps2') or vp.get(p, 'runner.ramp_accel_mps2'))
            speed = float(info.get('speed_mps') or info.get('mux_max_speed_mps') or 0.5)
            data = A.read_bag(folder / 'bag')
            scans = data['/scan']
            scan_t = [t for t, _ in scans]
            walls = [A.wall_fit(m, wall_deg) for _, m in scans]
            imu = data['/imu/data_raw']
            cal0, cal0_t = calib_startup(imu) if args.calib == 'startup' else (None, None)
            results = []
            for x in summ['t4']['items']:
                if not (x.get('moved') and x.get('stopped')) or (x.get('total_lidar_m') or 0) < MIN_TOTAL_M:
                    continue
                ref = repeat_refs(A, x, scan_t, walls, hold, ramp, speed)
                # Same reference as the results doc (analyze.py --t4), to the mm / 0.1 deg.
                assert abs(ref['total'] - x['total_lidar_m']) < 1e-3, (ref['total'], x['total_lidar_m'])
                assert abs(ref['heading'] - x['heading_change_wall_deg']) < 0.1
                cal = cal0 or calib_window(imu, ref['tc'] - BIAS_S, ref['tc'] - 0.02)
                assert cal and not cal['moving'], 'car not still in the calibration window'
                name = '%s_r%d_%s_g%.0f' % (folder.name[:15], x['index'], args.calib, gain)
                in_bag = work / (name + '_in')
                if not in_bag.exists():
                    build_inputs(data, ref['tc'] - BIAS_S - 0.2, ref['b'] + 0.3, cal, gain, in_bag)
                res = dict(index=x['index'], counted=x.get('counted'), runner_reason=x.get('runner_reason'),
                           calib=dict(source=args.calib, t=cal0_t, bias_gz=cal['bias_gz'], tilt_deg=cal['tilt_deg'],
                                      g_norm=cal['g_norm']),
                           lidar=ref, rf2o_heading_deg=x.get('heading_change_rf2o_deg'),
                           cruise=cruise_gain(A, ref, scan_t, walls, data['/sensors/core'], data['/odom_rf2o'], gain))
                for v in args.variants:
                    samples, cpu, wall = run_node(ekf_cmd(VARIANTS[v][0]), in_bag, env,
                                                  work / ('%s_%s.log' % (name, v)), rec)
                    m = ekf_metrics(A, ref, samples, laser_x, x.get('heading_change_rf2o_deg')) if samples else None
                    if m:
                        m.update(n=len(samples), ekf_cpu_s=cpu, play_s=wall)
                    res[v] = m
                print('  %s: %s' % (name, ' '.join('%s %+.3f' % (v, res[v]['total_err']) for v in args.variants
                                                   if res.get(v))))
                results.append(res)
                rows.append((folder.name[:15], res))
            (folder / ('ekf_shadow_replay%s.json' % ('' if args.calib == 'pre' else '_startup'))).write_text(
                json.dumps(dict(gain=gain, yaml=str(YAML), domain=DOMAIN, calib=args.calib,
                                variants={k: VARIANTS[k] for k in args.variants},
                                R=dict(vesc_vx=inp.R_VESC_VX, vy=inp.R_VY, imu_wz=inp.R_IMU_WZ, rf2o_vx=inp.R_RF_VX,
                                       rf2o_wz=inp.R_RF_WZ, ax=inp.R_AX, deweight=inp.DEWEIGHT,
                                       sensorless_erpm=inp.SENSORLESS_ERPM),
                                repeats=results), indent=1))
    finally:
        rec.close()
    print_tables(rows, args.variants)
    return 0


def print_tables(rows, variants):
    def rng(s, nd=3, unit=''):
        return '-' if s is None else '%+.*f (%+.*f..%+.*f)%s' % (nd, s['mean'], nd, s['min'], nd, s['max'], unit)
    print('\nPer variant over %d repeats: mean (min..max), errors = EKF - LiDAR (heading: EKF - RF2O)' % len(rows))
    print('| Variant | total err m | ramp err m | brake err m | heading err vs RF2O deg | hold drift m | ekf CPU % core |')
    print('|---|---|---|---|---|---|---|')
    for v in variants:
        got = [r[v] for _, r in rows if r.get(v)]
        cpu = sum(g['ekf_cpu_s'] for g in got) / max(1e-9, sum(g['play_s'] for g in got))
        print('| %s %s | %s | %s | %s | %s | %s | %.1f |' % (
            v, VARIANTS[v][1], rng(summarize([g['total_err'] for g in got])),
            rng(summarize([g['ramp_err'] for g in got])), rng(summarize([g['brake_err'] for g in got])),
            rng(summarize([g['heading_err_rf2o'] for g in got]), 1), rng(summarize([g['hold_drift'] for g in got])),
            100 * cpu))
    print('\nLiDAR per repeat: total / ramp / brake m, cruise gain')
    for run, r in rows:
        L, c = r['lidar'], r.get('cruise')
        print('  %s r%d %s: %.3f / %.3f / %.3f, %s' % (run, r['index'], 'counted' if r['counted'] else 'not counted',
                                                      L['total'], L['ramp'], L['brake'],
                                                      '%.0f' % c['gain_cruise_lidar'] if c else '-'))


if __name__ == '__main__':
    sys.exit(main())
