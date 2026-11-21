#!/usr/bin/env python3
"""Phase F (21 Nov): replay T4 bags through the real-car SHADOW EKF, offline, and compare it with
independent LiDAR references.

For every T4 repeat that moved (> MIN_TOTAL_M) in the given run folders (analyze.py --t4 must have run):
1. Build an input bag (work dir) from the T4 bag, window = 1 s before the command .. standstill + hold:
   /ekf_shadow/in/vesc  vx = ERPM / --gain (default 8700, CANDIDATE), vy = 0 constraint
   /ekf_shadow/in/imu   base_link yaw rate = -(gz - bias) (IMU upside down), bias = 1 s before the command
   /ekf_shadow/in/rf2o  /odom_rf2o twist vx, vyaw
   Header stamps = sensor time; bag time = original receive time (RF2O stays ~51 ms late, as live).
   R values: config/real_car/ekf_real_shadow.yaml header.
2. Run robot_localization ekf_node (variant A, then B) with use_sim_time, play the input bag with
   --clock, record /ekf_shadow/odom. Isolated ROS domain DOMAIN; refuses if a car stack runs.
3. References (same code and windows as analyze.py --t4): LiDAR wall-fit perpendicular range and
   heading to the wall. EKF progress = displacement of the LASER point (base_link + lidar.x_m) along
   the wall normal seen at the start. Phases: start ramp (command .. start delay + v / ramp accel),
   cruise (.. brake command), brake (.. standstill), total.
Writes ekf_shadow_replay.json into each run folder; prints a table. Publishes only inside DOMAIN.
  ekf_shadow_replay.py [<folder> ...] [--gain 8700] [--variants A B] [--work DIR]
  ekf_shadow_replay.py --self-test
"""
import argparse
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
import params as vp  # noqa: E402

DOMAIN = 88
REPO = Path(__file__).resolve().parents[2]
YAML = REPO / 'config' / 'real_car' / 'ekf_real_shadow.yaml'
DEFAULT_RUNS = ['20261120_203053_t4_straight_v0.50_brake5A_erpmmax4630_armed',   # carpet, offset 0.358
                '20261120_210243_t4_straight_v0.50_brake5A_erpmmax4630_armed']   # tile, offset 0.358
# Not replayed: 200901 (carpet 0.320) and 211256 (tile 0.340): IMU frozen (F-18).
VARIANTS = {'A': ('ekf_real_shadow', 'VESC vx + vy=0 + gyro'),
            'B': ('ekf_real_shadow_rf2o', 'A + RF2O vx, vyaw')}
GAIN = 8700.0
BIAS_S = 1.0
MIN_TOTAL_M = 0.3
CRUISE_PAD_S = 0.2     # cruise window starts this long after the ramp end
# R (variances) inside the input messages; source: ekf_real_shadow.yaml header.
R_VESC_VX, R_VY, R_IMU_WZ, R_RF_VX, R_RF_WZ = 5.0e-3, 4.0e-4, 8.0e-3, 1.0e-2, 1.2e-2
BIG = 1.0e6            # covariance of fields that are not fused
STACK_PATTERNS = ('vesc_driver_node', 'bmi160_node', 'lakibeam1_scan_node', 'rf2o_laser_odometry_node',
                  'async_slam_toolbox_node', 'physical_drive_mux', 't1b_stop_converter', 'ekf_node')


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


def vesc_vx(erpm, gain):
    return erpm / gain       # ERPM > 0 = forward on this car (bag: 4251 ERPM <-> /odom_vesc_observe +0.505)


def imu_yaw_rate(gz, bias):
    return -(gz - bias)      # base_link -> imu_link roll pi


def unwrap_series(vals):
    out, off = [], 0.0
    for i, v in enumerate(vals):
        if i:
            d = v + off - out[-1]
            off -= 2 * math.pi * round(d / (2 * math.pi))
        out.append(v + off)
    return out


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
        # EKF frame: rotated and shifted arbitrarily (only relative motion matters)
        a, ox, oy = rnd.uniform(-3, 3), rnd.uniform(-2, 2), rnd.uniform(-2, 2)

        def tf(p):
            return (ox + math.cos(a) * p[0] - math.sin(a) * p[1],
                    oy + math.sin(a) * p[0] + math.cos(a) * p[1], p[2] + a)
        got = laser_progress(b0[2], tf(b0), tf(b1), lx)
        assert abs(got - (d0 - d1)) < 1e-9, (got, d0 - d1)
    assert abs(laser_progress(0.0, (0, 0, 0), (1.0, 0, 0), lx) - 1.0) < 1e-12
    # Pure rotation in place by +10 deg facing the wall: laser moves back by lx(1 - cos).
    assert abs(laser_progress(0.0, (0, 0, 0), (0, 0, math.radians(10)), lx)
               - lx * (math.cos(math.radians(10)) - 1)) < 1e-12
    assert abs(vesc_vx(4350.0, 8700.0) - 0.5) < 1e-12
    assert imu_yaw_rate(-0.1 + 0.003, 0.003) == 0.1        # gz negative (upside down) = turning left
    u = unwrap_series([3.1, -3.1, -3.0])
    assert abs(u[1] - (2 * math.pi - 3.1)) < 1e-9 and u[2] > u[1]
    print('PASS: laser progress along the wall normal (200 random poses, frame invariant, rotation in place),')
    print('      VESC vx sign/gain, IMU sign/bias, yaw unwrap.')


# ---------------------------------------------------------------- bag -> inputs -> EKF

def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def build_inputs(data, w0, w1, bias, gain, out_dir):
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu
    msgs = []
    for t, m in data['/sensors/core']:
        if w0 <= t <= w1:
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, 'odom', 'base_link'
            o.twist.twist.linear.x = vesc_vx(m.state.speed, gain)
            o.twist.covariance = [0.0] * 36
            for i in range(6):
                o.twist.covariance[7 * i] = BIG
                o.pose.covariance[7 * i] = BIG
            o.twist.covariance[0], o.twist.covariance[7] = R_VESC_VX, R_VY
            msgs.append((t, '/ekf_shadow/in/vesc', o))
    for t, m in data['/imu/data_raw']:
        if w0 <= t <= w1:
            o = Imu()
            o.header.stamp, o.header.frame_id = m.header.stamp, 'base_link'
            o.angular_velocity.z = imu_yaw_rate(m.angular_velocity.z, bias)
            o.orientation_covariance[0] = -1.0
            o.linear_acceleration_covariance[0] = -1.0
            o.angular_velocity_covariance = [BIG, 0.0, 0.0, 0.0, BIG, 0.0, 0.0, 0.0, R_IMU_WZ]
            msgs.append((t, '/ekf_shadow/in/imu', o))
    for t, m in data['/odom_rf2o']:
        if w0 <= t <= w1:
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = m.header.stamp, 'odom', 'base_link'
            o.twist.twist.linear.x = m.twist.twist.linear.x
            o.twist.twist.angular.z = m.twist.twist.angular.z
            for i in range(6):
                o.twist.covariance[7 * i] = BIG
                o.pose.covariance[7 * i] = BIG
            o.twist.covariance[0], o.twist.covariance[35] = R_RF_VX, R_RF_WZ
            msgs.append((t, '/ekf_shadow/in/rf2o', o))
    msgs.sort(key=lambda x: x[0])
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(out_dir), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for name, typ in (('/ekf_shadow/in/vesc', 'nav_msgs/msg/Odometry'), ('/ekf_shadow/in/imu', 'sensor_msgs/msg/Imu'),
                      ('/ekf_shadow/in/rf2o', 'nav_msgs/msg/Odometry')):
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


def run_ekf(in_bag, node_name, env, rec, log_path):
    rec.take()
    with open(log_path, 'w') as log:
        ekf = subprocess.Popen(
            ['ros2', 'run', 'robot_localization', 'ekf_node', '--ros-args', '-r', '__node:=%s' % node_name,
             '--params-file', str(YAML), '-p', 'use_sim_time:=true', '-r', 'odometry/filtered:=/ekf_shadow/odom'],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(2.0)
            subprocess.run(['ros2', 'bag', 'play', str(in_bag), '--clock', '100', '-d', '1.0',
                            '--disable-keyboard-controls'], env=env, check=True,
                           stdout=log, stderr=subprocess.STDOUT, timeout=120)
            time.sleep(0.5)
        finally:
            os.killpg(ekf.pid, signal.SIGINT)
            try:
                ekf.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(ekf.pid, signal.SIGKILL)
    out = rec.take()
    out.sort(key=lambda s: s[0])
    return out


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
    vv = [vesc_vx(m.state.speed, gain) for t, m in core if a <= t <= b]
    vr = [m.twist.twist.linear.x for t, m in rf2o if a <= stamp(m) <= b]
    return dict(window_s=b - a, v_lidar=v_lidar, v_vesc=statistics.mean(vv), v_rf2o=statistics.mean(vr),
                gain_cruise_lidar=gain * statistics.mean(vv) / v_lidar, rf2o_over_lidar=statistics.mean(vr) / v_lidar)


def ekf_metrics(A, ref, samples, laser_x):
    ts = [s[0] for s in samples]
    yaw = unwrap_series([s[3] for s in samples])

    def pose(t):
        return tuple(A.interp(ts, v, t) for v in ([s[1] for s in samples], [s[2] for s in samples], yaw))
    tm = (ref['a'] + ref['b']) / 2          # standstill: middle of the LiDAR still window
    p = {k: pose(ref[k]) for k in ('tc', 't_ramp', 't0')}
    p['still'] = pose(tm)
    if any(None in v for v in p.values()):
        return None
    prog = {k: laser_progress(ref['psi_pre'], p['tc'], v, laser_x) for k, v in p.items()}
    vy = [abs(s[5]) for s in samples if ref['tc'] <= s[0] <= tm]
    vx_still = [s[4] for s in samples if ref['a'] <= s[0] <= ref['b']]
    return dict(ramp=prog['t_ramp'], cruise=prog['t0'] - prog['t_ramp'], brake=prog['still'] - prog['t0'],
                total=prog['still'], heading=math.degrees(p['still'][2] - p['tc'][2]),
                max_abs_vy=max(vy) if vy else None, vx_still_max=max(map(abs, vx_still)) if vx_still else None)


def stack_running():
    out = subprocess.run(['pgrep', '-af', '|'.join(STACK_PATTERNS)], capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if 'pgrep' not in ln and 'ekf_shadow_replay' not in ln]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folders', nargs='*')
    ap.add_argument('--gain', type=float, default=GAIN)
    ap.add_argument('--variants', nargs='+', default=['A', 'B'], choices=sorted(VARIANTS))
    ap.add_argument('--work', default=None, help='input bags + ekf logs (default: a new temp dir)')
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
    import analyze as A
    p = vp.load()
    laser_x = float(vp.get(p, 'lidar.x_m'))
    wall_deg = float(vp.get(p, 't4.wall_fit_deg'))
    folders = [Path(f).expanduser() for f in args.folders] or [vp.evidence_root() / r for r in DEFAULT_RUNS]
    work = Path(args.work or tempfile.mkdtemp(prefix='ekf_shadow_replay_')).expanduser()
    work.mkdir(parents=True, exist_ok=True)
    print('domain %d, gain %.0f, work %s' % (DOMAIN, args.gain, work))
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
            it = [t for t, _ in data['/imu/data_raw']]
            gz = [m.angular_velocity.z for _, m in data['/imu/data_raw']]
            results = []
            for x in summ['t4']['items']:
                if not (x.get('moved') and x.get('stopped')) or (x.get('total_lidar_m') or 0) < MIN_TOTAL_M:
                    continue
                ref = repeat_refs(A, x, scan_t, walls, hold, ramp, speed)
                # Same reference as the results doc (analyze.py --t4), to the mm / 0.1 deg.
                assert abs(ref['total'] - x['total_lidar_m']) < 1e-3, (ref['total'], x['total_lidar_m'])
                assert abs(ref['heading'] - x['heading_change_wall_deg']) < 0.1
                bias = statistics.mean(g for t, g in zip(it, gz) if ref['tc'] - BIAS_S <= t < ref['tc'] - 0.02)
                name = '%s_r%d' % (folder.name[:15], x['index'])
                in_bag = work / (name + '_in')
                if not in_bag.exists():
                    build_inputs(data, ref['tc'] - BIAS_S - 0.2, ref['b'] + 0.3, bias, args.gain, in_bag)
                res = dict(index=x['index'], counted=x.get('counted'), runner_reason=x.get('runner_reason'),
                           bias_rad_s=bias, lidar=ref, rf2o_heading_deg=x.get('heading_change_rf2o_deg'),
                           rf2o_total_m=x.get('total_rf2o_m'), vesc8414_total_m=x.get('total_vesc_m'),
                           cruise=cruise_gain(A, ref, scan_t, walls, data['/sensors/core'], data['/odom_rf2o'],
                                              args.gain))
                for v in args.variants:
                    samples = run_ekf(in_bag, VARIANTS[v][0], env, rec, work / ('%s_%s.log' % (name, v)))
                    res[v] = ekf_metrics(A, ref, samples, laser_x) if samples else None
                    res[v + '_n'] = len(samples)
                    print('  %s %s: %d EKF samples' % (name, v, len(samples)))
                results.append(res)
                rows.append((folder.name[:15], res))
            (folder / 'ekf_shadow_replay.json').write_text(json.dumps(dict(
                gain=args.gain, yaml=str(YAML), domain=DOMAIN, variants={k: VARIANTS[k] for k in args.variants},
                R=dict(vesc_vx=R_VESC_VX, vy=R_VY, imu_wz=R_IMU_WZ, rf2o_vx=R_RF_VX, rf2o_wz=R_RF_WZ),
                repeats=results), indent=1))
    finally:
        rec.close()
    print_table(rows, args.variants)
    return 0


def print_table(rows, variants):
    def f(v, nd=3):
        return '-' if v is None else ('%+.*f' % (nd, v))
    hdr = ['run', 'cnt', 'LiDAR total m'] + ['%s err' % v for v in variants] + \
          ['ramp LiDAR'] + ['%s err' % v for v in variants] + ['brake LiDAR'] + ['%s err' % v for v in variants] + \
          ['heading wall deg', 'RF2O'] + ['%s' % v for v in variants] + ['cruise gain']
    print('| ' + ' | '.join(hdr) + ' |')
    print('|' + '---|' * len(hdr))
    for run, r in rows:
        L = r['lidar']
        cells = ['%s r%d' % (run, r['index']), 'y' if r['counted'] else 'n', '%.3f' % L['total']]
        for k in ('total', 'ramp', 'brake'):
            if k != 'total':
                cells.append('%.3f' % L[k])
            cells += [f(r[v][k] - L[k]) if r.get(v) else '-' for v in variants]
        cells += ['%+.1f' % L['heading'], f(r['rf2o_heading_deg'], 1)]
        cells += [f(r[v]['heading'], 1) if r.get(v) else '-' for v in variants]
        cells.append('%.0f' % r['cruise']['gain_cruise_lidar'] if r.get('cruise') else '-')
        print('| ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    sys.exit(main())
