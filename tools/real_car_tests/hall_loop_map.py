#!/usr/bin/env python3
"""Hall-loop map, OFFLINE (23 Nov): build a slam_toolbox map from a hall-loop bag twice, loop closing OFF then ON,
save both maps and the lap-closure error at the start tape mark. Reads bags only; publishes only in its own
isolated ROS domain (default 89; refuses if a car stack runs).

Input: a hall_loop_record.sh folder (hand push) or a run_test.py teleop folder (both: <folder>/bag), or a bag dir.
Per variant:
1. slam_toolbox async node (repo config/slam_toolbox.yaml of f1tenth_bringup, mode mapping, use_sim_time,
   do_loop_closing off / on, map_update_interval 1 s).
2. ros2 bag play --clock of /scan + /tf (remapped to /tf_bag). A relay republishes /tf WITHOUT the recorded
   map -> odom (the live SLAM's), so the offline SLAM is the only map authority; /tf_static comes from the bag
   through a static broadcaster. After the bag ends the clock keeps running a few seconds for the last map update.
3. Samples map -> base_link (pose) and map -> odom (the SLAM correction; a jump > 5 cm = a loop closure / re-fit).
4. Saves map_lc_<off|on>.pgm/.yaml (map_saver trinary convention) and the trajectory CSV.
Lap closure at the tape mark: the car is still at the start and at the end (on the same mark), found from RF2O
(odom) motion: start pose = median map pose over the 1 s before motion, end pose = median after motion stopped.
Error = distance and heading between them (includes how precisely the car was put back on the mark). RF2O alone
(odom frame) gives the raw odometry closure for comparison. Writes hall_loop_map.json.
  hall_loop_map.py <folder> [--out DIR] [--rate 1.0] [--domain 89] [--variants off on]
  hall_loop_map.py --sensor-check [optional topics...]     (used by hall_loop_record.sh; prints rates)
  hall_loop_map.py --self-test
"""
import argparse
import csv
import json
import math
import os
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

DOMAIN = 89
REPO = Path(__file__).resolve().parents[2]
SLAM_YAML = REPO / 'src' / 'f1tenth_bringup' / 'config' / 'slam_toolbox.yaml'
MOVE_M = 0.05            # RF2O displacement that counts as "moving" (start / end still detection)
STILL_PAD_S = 0.2
JUMP_M = 0.05            # map -> odom change between samples counted as a SLAM correction jump
STACK_PATTERNS = ('vesc_driver_node', 'bmi160_node', 'lakibeam1_scan_node', 'rf2o_laser_odometry_node',
                  'async_slam_toolbox_node', 'ekf_node')
# Sensor check (hall_loop_record.sh): topic -> (minimum Hz, what to do when it fails)
SENSOR_RULES = {
    '/scan': (25.0, 'LiDAR not publishing (~30 Hz expected). 22 Nov the LiDAR driver crashed once (exit -6): '
                    'stop the launch (Ctrl+C), preflight.sh clean, launch again.'),
    '/imu/data_raw': (90.0, 'IMU not publishing (~100 Hz expected). observe_only.launch.py does NOT start the IMU: '
                            'run `ros2 launch f1tenth_imu imu.launch.py` in its own terminal and keep the car STILL '
                            '10 s (gyro calibration; the node exits if the car moves; Errno 121 at start: retry).'),
    '/odom_rf2o': (10.0, 'RF2O not publishing (~20 Hz expected): it needs /scan; observe_only.launch.py starts it.'),
}
FROZEN_SHARE = 0.5       # IMU: more than this share of identical consecutive samples = frozen (F-18)


# ---------------------------------------------------------------- pure helpers (self-tested)

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def still_bounds(track, move_m=MOVE_M):
    """track = [(t, x, y, yaw)] (odom). -> (t_motion_start, t_motion_end): first sample farther than move_m from
    the first pose, last sample farther than move_m from the last pose. None if it never moved."""
    if len(track) < 3:
        return None
    x0, y0 = track[0][1], track[0][2]
    x1, y1 = track[-1][1], track[-1][2]
    ts = next((p[0] for p in track if math.hypot(p[1] - x0, p[2] - y0) > move_m), None)
    te = next((p[0] for p in reversed(track) if math.hypot(p[1] - x1, p[2] - y1) > move_m), None)
    if ts is None or te is None or te <= ts:
        return None
    return ts, te


def median_pose(track, a, b):
    sel = [p for p in track if a <= p[0] <= b]
    if not sel:
        return None
    c = statistics.median(math.cos(p[3]) for p in sel)
    s = statistics.median(math.sin(p[3]) for p in sel)
    return (statistics.median(p[1] for p in sel), statistics.median(p[2] for p in sel), math.atan2(s, c), len(sel))


def closure(track, bounds, pad=STILL_PAD_S):
    """Start / end still poses of `track` around the motion bounds -> dict(distance, heading error) or None."""
    if bounds is None or not track:
        return None
    ts, te = bounds
    p0 = median_pose(track, ts - 1.0 - pad, ts - pad) or median_pose(track, track[0][0], ts - pad)
    p1 = median_pose(track, te + pad, track[-1][0])
    if p0 is None or p1 is None:
        return None
    return dict(start=p0[:3], end=p1[:3], n_start=p0[3], n_end=p1[3],
                distance_m=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                heading_deg=math.degrees(wrap(p1[2] - p0[2])))


def path_stats(track, bounds):
    """Path length and total turned heading (laps) of an odom track between the motion bounds."""
    sel = [p for p in track if bounds[0] - 0.5 <= p[0] <= bounds[1] + 0.5] if bounds else track
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(sel, sel[1:]))
    turned = sum(wrap(b[3] - a[3]) for a, b in zip(sel, sel[1:]))
    return dict(path_m=length, turned_deg=math.degrees(turned), laps=abs(turned) / (2 * math.pi))


def correction_jumps(odom_corr, jump_m=JUMP_M):
    """map -> odom samples [(t, x, y, yaw)] -> [(t, step m)] where the correction moved more than jump_m."""
    return [(b[0], math.hypot(b[1] - a[1], b[2] - a[2])) for a, b in zip(odom_corr, odom_corr[1:])
            if math.hypot(b[1] - a[1], b[2] - a[2]) > jump_m]


def grid_to_pgm(width, height, data):
    """OccupancyGrid data (row 0 = lowest y) -> PGM bytes, map_saver trinary: occupied (>= 65) 0, free (0..25)
    254, unknown 205; image row 0 = highest y."""
    out = bytearray(b'P5\n# hall_loop_map.py\n%d %d\n255\n' % (width, height))
    for row in range(height - 1, -1, -1):
        for v in data[row * width:(row + 1) * width]:
            out.append(205 if v < 0 else 0 if v >= 65 else 254 if v <= 25 else 205)
    return bytes(out)


def map_yaml(image, resolution, origin):
    return ('image: %s\nmode: trinary\nresolution: %.4f\norigin: [%.4f, %.4f, %.4f]\nnegate: 0\n'
            'occupied_thresh: 0.65\nfree_thresh: 0.25\n' % (image, resolution, origin[0], origin[1], origin[2]))


def self_test():
    # still detection + closure: still 2 s at (0, 0), a square lap, back at (0.03, -0.02), still 2 s
    tr = [(t * 0.1, 0.0, 0.0, 0.0) for t in range(20)]
    t = 2.0
    for k, (dx, dy) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
        x0, y0 = tr[-1][1], tr[-1][2]
        for i in range(1, 21):
            t += 0.1
            tr.append((t, x0 + 2.0 * dx * i / 20, y0 + 2.0 * dy * i / 20, wrap(k * math.pi / 2 + (i / 20) * 0.0)))
    tr[-1] = (tr[-1][0], 0.03, -0.02, 2 * math.pi - 0.01)
    end = tr[-1]
    tr += [(end[0] + 0.1 * (i + 1), 0.03, -0.02, end[3]) for i in range(20)]
    b = still_bounds(tr)
    assert b and 1.9 < b[0] < 2.2 and b[1] < tr[-21][0] + 1e-9, b
    c = closure(tr, b)
    assert abs(c['distance_m'] - math.hypot(0.03, 0.02)) < 1e-9 and abs(c['heading_deg'] + math.degrees(0.01)) < 1e-6, c
    assert still_bounds([(0.0, 0, 0, 0), (1.0, 0.01, 0, 0), (2.0, 0.0, 0.01, 0)]) is None
    ps = path_stats(tr, b)
    assert abs(ps['path_m'] - 8.0) < 0.1, ps
    # heading wrap in the median pose (start yaw ~ +pi / -pi)
    mp = median_pose([(0.0, 0, 0, math.pi - 0.01), (0.1, 0, 0, -math.pi + 0.01), (0.2, 0, 0, math.pi - 0.02)], 0, 1)
    assert abs(abs(mp[2]) - math.pi) < 0.03, mp
    assert correction_jumps([(0, 0, 0, 0), (1, 0.01, 0, 0), (2, 0.2, 0, 0)]) == [(2, 0.19)]
    pgm = grid_to_pgm(3, 2, [0, 100, -1, 50, 0, 0])
    assert pgm.endswith(bytes([205, 254, 254, 254, 0, 205])), pgm[-6:]
    assert 'origin: [-1.0000, 2.0000, 0.0000]' in map_yaml('m.pgm', 0.05, (-1, 2, 0))
    for t_, (hz, why) in SENSOR_RULES.items():
        assert hz > 0 and why
    print('PASS: hall_loop_map: still detection, lap closure (distance, heading wrap), path / laps, SLAM correction')
    print('      jumps, PGM (map_saver trinary, row flip) + yaml, sensor rules.')


# ---------------------------------------------------------------- sensor check (hall_loop_record.sh)

def sensor_check(optional, seconds=5.0):
    """Count messages for `seconds`; 0 if all SENSOR_RULES pass, else 2. Prints one line per topic and
    'OPTIONAL_PRESENT <topics>' for the optional topics that published at least once."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    rclpy.init()
    node = rclpy.create_node('hall_loop_sensor_check')
    try:
        t_end = time.monotonic() + 1.5
        types = {}
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
            types = dict(node.get_topic_names_and_types())
        counts, imu_last, imu_same = {}, [None], [0]

        def cb(topic):
            def f(msg):
                counts[topic] = counts.get(topic, 0) + 1
                if topic == '/imu/data_raw':
                    g, a = msg.angular_velocity, msg.linear_acceleration
                    s = (g.x, g.y, g.z, a.x, a.y, a.z)
                    imu_same[0] += s == imu_last[0]
                    imu_last[0] = s
            return f
        subs = []
        for topic in list(SENSOR_RULES) + list(optional):
            if topic in types:
                raw = topic != '/imu/data_raw'
                subs.append(node.create_subscription(get_message(types[topic][0]), topic, cb(topic),
                                                     qos_profile_sensor_data, raw=raw))
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
        dt = time.monotonic() - t0
        ok = True
        for topic, (hz_min, why) in SENSOR_RULES.items():
            hz = counts.get(topic, 0) / dt
            bad = hz < hz_min
            extra = ''
            if topic == '/imu/data_raw' and counts.get(topic, 0) > 10:
                share = imu_same[0] / max(1, counts[topic] - 1)
                extra = ', identical samples %.0f %%' % (100 * share)
                if share > FROZEN_SHARE:
                    bad, why = True, ('IMU FROZEN (F-18: one sample repeated): stop the IMU node, start it again with '
                                      'the car still 10 s.')
            status = 'OK' if not bad else 'MISSING: ' + why
            print('%-16s %6.1f Hz (needs >= %.0f)%s  %s' % (topic, hz, hz_min, extra, status))
            ok &= not bad
        present = [t for t in optional if counts.get(t, 0) > 0]
        for t in optional:
            print('%-28s %s' % (t, '%.1f Hz (recorded)' % (counts.get(t, 0) / dt) if t in present
                                else 'absent (not recorded)'))
        print('OPTIONAL_PRESENT ' + ' '.join(present))
        return 0 if ok else 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# ---------------------------------------------------------------- offline SLAM

class Relay:
    """Sim-time node: /tf_bag -> /tf without map->odom, static TFs from the bag, map->base_link / map->odom
    sampling, last /map, clock continuation after the bag."""

    def __init__(self, static_msgs):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import OccupancyGrid
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        import tf2_ros
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node('hall_loop_map_relay',
                                      parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.pose, self.corr, self.map_msg, self.map_at, self.last_clock = [], [], None, None, None
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 100)
        self.static = tf2_ros.StaticTransformBroadcaster(self.node)
        transforms = [tr for m in static_msgs for tr in m.transforms]
        if transforms:
            self.static.sendTransform(transforms)
        self.node.create_subscription(TFMessage, '/tf_bag', self.on_tf, 100)
        self.node.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)  # bag: best effort
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.Clock = Clock
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self.node)
        self.node.create_timer(0.05, self.sample)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def on_tf(self, msg):
        keep = [t for t in msg.transforms if not (t.child_frame_id.lstrip('/') == 'odom'
                                                  and t.header.frame_id.lstrip('/') == 'map')]
        if keep:
            msg.transforms = keep
            self.tf_pub.publish(msg)

    def on_map(self, msg):
        with self.lock:
            self.map_msg, self.map_at = msg, time.monotonic()

    def on_clock(self, msg):
        with self.lock:
            self.last_clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def sample(self):
        from rclpy.time import Time
        now = self.node.get_clock().now().nanoseconds * 1e-9
        for parent, child, store in (('map', 'base_link', self.pose), ('map', 'odom', self.corr)):
            try:
                tr = self.buf.lookup_transform(parent, child, Time())
            except Exception:  # noqa: BLE001 (not yet available)
                continue
            t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
            with self.lock:
                if not store or store[-1][0] != t:
                    store.append((t if child == 'base_link' else now, tr.transform.translation.x,
                                  tr.transform.translation.y, yaw_of(tr.transform.rotation)))

    def run_clock_on(self, seconds, step=0.1):
        """Keep the sim clock going after the bag (slam_toolbox's map timer runs on sim time)."""
        with self.lock:
            t = self.last_clock
        if t is None:
            return
        for _ in range(int(seconds / step)):
            t += step
            m = self.Clock()
            m.clock.sec, m.clock.nanosec = int(t), int((t % 1) * 1e9)
            self.clock_pub.publish(m)
            time.sleep(step)

    def close(self):
        self.ex.shutdown()
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def stack_running():
    out = subprocess.run(['pgrep', '-af', '|'.join(STACK_PATTERNS)], capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if 'pgrep' not in ln and 'hall_loop_map' not in ln]


def stop_group(proc, timeout=10.0):
    """SIGINT to the process group, SIGKILL after timeout; never leaves the node running."""
    import signal
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            continue


def save_map(msg, out, stem):
    info = msg.info
    (out / (stem + '.pgm')).write_bytes(grid_to_pgm(info.width, info.height, list(msg.data)))
    o = info.origin
    (out / (stem + '.yaml')).write_text(map_yaml(stem + '.pgm', info.resolution,
                                                 (o.position.x, o.position.y, yaw_of(o.orientation))))
    known = sum(1 for v in msg.data if v >= 0)
    return dict(width=info.width, height=info.height, resolution=info.resolution, known_cells=known,
                occupied_cells=sum(1 for v in msg.data if v >= 65), area_known_m2=known * info.resolution ** 2)


def build_variant(bag, static_msgs, lc, out, work, env, rate, log_name):
    stem = 'map_lc_%s' % ('on' if lc else 'off')
    relay = Relay(static_msgs)
    slam_log = open(work / ('%s_slam.log' % log_name), 'w')
    prefix = subprocess.run(['ros2', 'pkg', 'prefix', 'slam_toolbox'], capture_output=True, text=True,
                            check=True).stdout.strip()
    # the node itself (not `ros2 run`, whose wrapper does not pass SIGINT on), in its own process group
    slam = subprocess.Popen([str(Path(prefix) / 'lib' / 'slam_toolbox' / 'async_slam_toolbox_node'), '--ros-args',
                             '--params-file', str(SLAM_YAML), '-p', 'use_sim_time:=true',
                             '-p', 'do_loop_closing:=%s' % ('true' if lc else 'false'), '-p', 'mode:=mapping',
                             '-p', 'map_update_interval:=1.0'],
                            env=env, stdout=slam_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(3.0)
        with open(work / ('%s_play.log' % log_name), 'w') as log:
            subprocess.run(['ros2', 'bag', 'play', str(bag), '--clock', '100', '-d', '1.0', '--rate', str(rate),
                            '--disable-keyboard-controls', '--topics', '/scan', '/tf', '--remap', '/tf:=/tf_bag'],
                           env=env, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=7200)
        t_play = time.monotonic()
        relay.run_clock_on(4.0)
        while time.monotonic() - t_play < 15.0:
            with relay.lock:
                if relay.map_at is not None and relay.map_at > t_play + 1.0:
                    break
            time.sleep(0.2)
        with relay.lock:
            msg, pose, corr = relay.map_msg, list(relay.pose), list(relay.corr)
    finally:
        stop_group(slam)
        slam_log.close()
        relay.close()
    if msg is None:
        raise SystemExit('slam_toolbox published no /map (see %s)' % (work / ('%s_slam.log' % log_name)))
    res = save_map(msg, out, stem)
    with open(out / (stem + '_trajectory.csv'), 'w', newline='') as fp:
        w = csv.writer(fp)
        w.writerow(['t', 'x', 'y', 'yaw'])
        w.writerows(pose)
    res.update(map=str(out / (stem + '.yaml')), pose_samples=len(pose), correction_jumps=correction_jumps(corr))
    return res, pose


def build(folder, out, variants, rate, domain):
    import analyze as A
    bag = folder / 'bag' if (folder / 'bag' / 'metadata.yaml').exists() else folder
    busy = stack_running()
    if busy:
        print('REFUSED: a car stack / SLAM / EKF runs on this machine; offline maps only with the stack down:')
        print('\n'.join('  ' + b for b in busy))
        return 2
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    env = dict(os.environ)
    out.mkdir(parents=True, exist_ok=True)
    work = out / 'work'
    work.mkdir(exist_ok=True)
    data = A.read_bag(bag)
    for need in ('/scan', '/tf', '/odom_rf2o'):
        if not data.get(need):
            raise SystemExit('bag has no %s: not a hall-loop recording' % need)
    rf = sorted(((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
                  m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)) for _, m in data['/odom_rf2o']))
    bounds = still_bounds(rf)
    static_msgs = [m for _, m in data.get('/tf_static', [])]
    summary = dict(folder=str(folder), bag=str(bag), domain=domain, rate=rate, slam_yaml=str(SLAM_YAML),
                   motion=None if bounds is None else dict(start=bounds[0], end=bounds[1],
                                                           duration_s=bounds[1] - bounds[0]),
                   rf2o=dict(closure=closure(rf, bounds), **(path_stats(rf, bounds) if bounds else {})))
    moved = ('no motion found' if bounds is None else 'moving %.1f..%.1f s (%.1f s), path %.1f m, %.2f laps' % (
        bounds[0] - rf[0][0], bounds[1] - rf[0][0], bounds[1] - bounds[0], summary['rf2o']['path_m'],
        summary['rf2o']['laps']))
    print('bag %s: %d scans, RF2O %s' % (bag, len(data['/scan']), moved))
    del data
    for v in variants:
        lc = v == 'on'
        print('slam_toolbox loop closing %s (domain %d, rate %.1f) ...' % (v.upper(), domain, rate))
        res, pose = build_variant(bag, static_msgs, lc, out, work, env, rate, 'lc_' + v)
        res['closure'] = closure(pose, bounds)
        summary['lc_' + v] = res
    (out / 'hall_loop_map.json').write_text(json.dumps(summary, indent=1, default=str))
    print('\n| source | lap-closure distance m | heading deg | path m | laps | map known m^2 | SLAM jumps > 5 cm |')
    print('|---|---|---|---|---|---|---|')
    rc = summary['rf2o']['closure']
    print('| RF2O odom only | %s | %s | %.1f | %.2f | - | - |' % (
        '-' if not rc else '%.3f' % rc['distance_m'], '-' if not rc else '%+.1f' % rc['heading_deg'],
        summary['rf2o'].get('path_m', float('nan')), summary['rf2o'].get('laps', float('nan'))))
    for v in variants:
        r = summary['lc_' + v]
        c = r['closure']
        print('| SLAM loop closing %s | %s | %s | - | - | %.1f | %d |' % (
            v, '-' if not c else '%.3f' % c['distance_m'], '-' if not c else '%+.1f' % c['heading_deg'],
            r['area_known_m2'], len(r['correction_jumps'])))
    print('Maps: %s (work logs %s)' % (out, work))
    print('The closure includes how precisely the car was put back on the tape mark (same heading).')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?')
    ap.add_argument('--out', help='default <folder>/maps')
    ap.add_argument('--rate', type=float, default=1.0, help='bag play rate (slam_toolbox async drops scans if > 1)')
    ap.add_argument('--domain', type=int, default=DOMAIN)
    ap.add_argument('--variants', nargs='+', default=['off', 'on'], choices=['off', 'on'])
    ap.add_argument('--sensor-check', nargs='*', metavar='OPTIONAL_TOPIC')
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.sensor_check is not None:
        return sensor_check(args.sensor_check)
    if not args.folder:
        ap.error('folder, --sensor-check or --self-test')
    folder = Path(args.folder).expanduser()
    return build(folder, Path(args.out).expanduser() if args.out else folder / 'maps', args.variants, args.rate,
                 args.domain)


if __name__ == '__main__':
    sys.exit(main())
