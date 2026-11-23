#!/usr/bin/env python3
"""Real-car SHADOW EKF inputs (Phase F, 21 Nov). ONE definition for the offline replay
(ekf_shadow_replay.py builds its input bags with these functions) and the live node (main() here).

Observation only: subscribes /sensors/core, /imu/data_raw, /odom_rf2o, /t1b_stop_converter/status and
publishes ONLY /ekf_shadow/in/{vesc,vesc_dw,imu,rf2o}. Never /drive, /commands/*, TF or parameters.
Not a replacement for Zaid's tools/shadow_covariance_adapter.py (his sim-port topics /shadow_inputs/*).

Inputs (all already in base_link, so the EKF needs no TF; header stamp = sensor time):
- vesc     vx = ERPM / speed_to_erpm_gain (vehicle_params, 8414), vy = 0 constraint (R_VY).
- vesc_dw  same, vx variance x DEWEIGHT while the brake is active (converter phase STOP_BRAKE) or
           0 < |ERPM| < SENSORLESS_ERPM (T4 bags: below ~2000 ERPM the VESC reads +0.08..+0.12 m/s ahead
           in the start ramp; in braking it drops to 0 while the car still moves 0.2-0.4 m/s).
           Exactly 0 ERPM outside braking stays trusted: it is the standstill (zero-velocity) cue.
- imu      yaw rate = gyro (bias removed) projected on the still gravity direction (= base +z in the IMU frame;
           23 Nov eve, YAW_MODE 'tilt'): the mount is tilted ~9 deg, so -gz alone reads cos(9.2 deg) = 0.987 of
           the yaw rate
           (the 1.2 % C1-vs-RF2O turn deficit in the T3 circles). YAW_MODE 'gz' = the old -(gz - bias).
           Forward accel = specific force projected on the
           IMU -x axis made perpendicular to the still gravity vector, scaled by 9.80665 / |g_still|
           (21 Nov: IMU x points BACKWARD - forward accel reads negative on +x - and the mount is
           tilted ~9-10 deg; 23 Nov eve TF in vehicle_params: pitch pi + tilt). Gravity is removed by the projection.
- rf2o     /odom_rf2o twist vx, vyaw (its covariance fields are 0).
Still calibration (gyro bias + gravity): replay = the 1 s before each drive command; live = at start-up,
StillCalib.SECONDS of samples with gyro std <= GYRO_STILL_STD; retried with a log line until the car is
still (F-18 lesson: never exit on motion).
  ekf_shadow_inputs.py [--no-rf2o]            live node (stack running; shadow only)
  ekf_shadow_inputs.py --self-test
"""
import argparse
import json
import math
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

SENSORLESS_ERPM = 2000.0   # T4 bags 21 Nov: |ERPM| bins 500-1999 read +0.08..+0.12 m/s; >= 2000 within 0.03
DEWEIGHT = 100.0           # vx variance factor while de-weighted
BRAKE_PHASES = ('STOP_BRAKE',)
G0 = 9.80665
IMU_FORWARD = (-1.0, 0.0, 0.0)   # IMU axis that points forward (measured 21 Nov, T4 ramps and brakes)
# Variances inside the messages; source and method: config/real_car/ekf_real_shadow.yaml header.
# R_IMU_WZ 1.2e-3 (23 Nov eve, applied; was 8.0e-3): T3 steady-turn gyro white noise var 4.2e-5 x ~30 for the 100 Hz
# samples' correlated vibration and the residual scale / bias (results doc "T3 gyro noise in turns").
R_VESC_VX, R_VY, R_IMU_WZ, R_RF_VX, R_RF_WZ, R_AX = 5.0e-3, 4.0e-4, 1.2e-3, 1.0e-2, 1.2e-2, 0.32
YAW_MODE = 'tilt'          # 'tilt' (23 Nov eve): gyro . gravity direction; 'gz': -(gz - bias) (21 Nov .. 23 Nov day)
BIG = 1.0e6
GYRO_STILL_STD = math.radians(0.5)    # rad/s, same limit as the bmi160 driver's own calibration
RF2O_STALE_S = 0.2         # = EKF sensor_timeout: older than this, the EKF stops fusing RF2O (C1 -> C0)
TOPICS = dict(vesc='/ekf_shadow/in/vesc', vesc_dw='/ekf_shadow/in/vesc_dw', imu='/ekf_shadow/in/imu',
              rf2o='/ekf_shadow/in/rf2o')


def gain_from_params():
    return float(vp.get(vp.load(), 'drivetrain.speed_to_erpm_gain'))


# ---------------------------------------------------------------- pure logic

def vesc_vx(erpm, gain):
    return erpm / gain       # ERPM > 0 = forward on this car (T4 bags)


def brake_active(status_text):
    """t1b converter status JSON -> True while it commands the brake. Unknown text -> False."""
    try:
        return json.loads(status_text).get('phase') in BRAKE_PHASES
    except (ValueError, AttributeError):
        return False


def vesc_deweighted(erpm, braking):
    return bool(braking) or 0.0 < abs(erpm) < SENSORLESS_ERPM


def vesc_vx_variance(erpm, braking, deweight):
    return R_VESC_VX * (DEWEIGHT if deweight and vesc_deweighted(erpm, braking) else 1.0)


def yaw_rate(gz, bias):
    return -(gz - bias)      # IMU z points down (YAW_MODE 'gz')


def yaw_rate_vec(gyro, cal, mode=None):
    """Base yaw rate from the 3-axis gyro: (gyro - bias) . up, up = unit still specific force (= base +z in the IMU
    frame, tilt included). mode 'gz' (or a calibration without 'up'): the old -(gz - bias_gz)."""
    if (mode or YAW_MODE) == 'gz' or 'up' not in cal:
        return yaw_rate(gyro[2], cal['bias_gz'])
    return sum(u * (g - b) for u, g, b in zip(cal['up'], gyro, cal['bias']))


class StillCalib:
    """Gyro z bias and gravity vector from still samples (SAMPLES = 2 s at 100 Hz; count-based so the
    live node and the replay use exactly the same samples)."""
    SECONDS = 2.0
    SAMPLES = 200

    def __init__(self):
        self.g, self.a = [], []

    def add(self, gyro, accel):
        self.g.append(tuple(gyro))
        self.a.append(tuple(accel))

    def result(self):
        """dict(bias_gz, gravity, forward, g_norm, gyro_std) or None if too few samples / moving."""
        if len(self.g) < 20:
            return None
        gstd = max(statistics.pstdev(v[i] for v in self.g) for i in range(3))
        if gstd > GYRO_STILL_STD:
            return dict(moving=True, gyro_std=gstd)
        grav = [statistics.mean(v[i] for v in self.a) for i in range(3)]
        n = math.sqrt(sum(x * x for x in grav))
        gh = [x / n for x in grav]
        dot = sum(f * g for f, g in zip(IMU_FORWARD, gh))
        fwd = [f - dot * g for f, g in zip(IMU_FORWARD, gh)]
        fn = math.sqrt(sum(x * x for x in fwd))
        bias = [statistics.mean(v[i] for v in self.g) for i in range(3)]
        return dict(moving=False, bias_gz=bias[2], bias=bias, up=gh, gravity=grav, g_norm=n,
                    forward=[x / fn for x in fwd], gyro_std=gstd,
                    tilt_deg=math.degrees(math.asin(min(1.0, abs(dot)))))


def forward_accel(accel, cal):
    return sum(a * f for a, f in zip(accel, cal['forward'])) * G0 / cal['g_norm']


class Rf2oFreshness:
    """RF2O freshness: receive gaps (stale = no message for > RF2O_STALE_S) and latency (receive - header
    stamp; RF2O stamps with the scan time). Times in seconds, wall clock live."""

    def __init__(self):
        self.last = None
        self.n = 0
        self.lat = []
        self.max_gap = 0.0
        self.stale_events = 0
        self.stale = False
        self.t0 = None

    def add(self, t_recv, t_stamp):
        if self.last is not None:
            self.max_gap = max(self.max_gap, t_recv - self.last)
        self.t0 = t_recv if self.t0 is None else self.t0
        self.last, self.stale = t_recv, False
        self.n += 1
        self.lat.append(t_recv - t_stamp)

    def check(self, now):
        """-> age in s (None before the first message); counts a stale event once per gap."""
        if self.last is None:
            return None
        age = now - self.last
        if age > RF2O_STALE_S and not self.stale:
            self.stale, self.stale_events = True, self.stale_events + 1
        return age

    def report(self, now):
        """Summary since the last report; resets rate, latency and max gap."""
        span = (now - self.t0) if self.t0 is not None else 0.0
        lat = sorted(self.lat)
        out = dict(hz=self.n / span if span > 0 else 0.0, n=self.n, max_gap_s=self.max_gap,
                   lat_med_ms=1e3 * lat[len(lat) // 2] if lat else None, lat_max_ms=1e3 * lat[-1] if lat else None,
                   stale_events=self.stale_events)
        self.n, self.lat, self.max_gap, self.t0 = 0, [], 0.0, now
        return out


# ---------------------------------------------------------------- messages (ROS imported lazily)

def _diag(cov, n, big=BIG):
    for i in range(n):
        cov[(n + 1) * i] = big


def vesc_msg(header_stamp, erpm, braking, gain, deweight):
    from nav_msgs.msg import Odometry
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = header_stamp, 'odom', 'base_link'
    o.twist.twist.linear.x = vesc_vx(erpm, gain)
    _diag(o.pose.covariance, 6)
    _diag(o.twist.covariance, 6)
    o.twist.covariance[0] = vesc_vx_variance(erpm, braking, deweight)
    o.twist.covariance[7] = R_VY
    return o


def imu_msg(header_stamp, gyro, accel, cal):
    from sensor_msgs.msg import Imu
    o = Imu()
    o.header.stamp, o.header.frame_id = header_stamp, 'base_link'
    o.angular_velocity.z = yaw_rate_vec(gyro, cal)
    o.angular_velocity_covariance = [BIG, 0.0, 0.0, 0.0, BIG, 0.0, 0.0, 0.0, R_IMU_WZ]
    o.linear_acceleration.x = forward_accel(accel, cal)
    o.linear_acceleration_covariance = [R_AX, 0.0, 0.0, 0.0, BIG, 0.0, 0.0, 0.0, BIG]
    o.orientation_covariance[0] = -1.0
    return o


def rf2o_msg(src):
    from nav_msgs.msg import Odometry
    o = Odometry()
    o.header.stamp, o.header.frame_id, o.child_frame_id = src.header.stamp, 'odom', 'base_link'
    o.twist.twist.linear.x = src.twist.twist.linear.x
    o.twist.twist.angular.z = src.twist.twist.angular.z
    _diag(o.pose.covariance, 6)
    _diag(o.twist.covariance, 6)
    o.twist.covariance[0], o.twist.covariance[35] = R_RF_VX, R_RF_WZ
    return o


def self_test():
    assert abs(vesc_vx(4207.0, 8414.0) - 0.5) < 1e-12
    assert brake_active('{"phase":"STOP_BRAKE","out":"brake"}') and not brake_active('{"phase":"DRIVE"}')
    assert not brake_active('garbage') and not brake_active('{"phase":"RELEASED"}')
    assert vesc_deweighted(1500.0, False) and vesc_deweighted(-1999.0, False) and vesc_deweighted(4200.0, True)
    assert not vesc_deweighted(0.0, False) and not vesc_deweighted(2000.0, False) and not vesc_deweighted(4200.0, False)
    assert vesc_vx_variance(1000.0, False, True) == R_VESC_VX * DEWEIGHT
    assert vesc_vx_variance(1000.0, False, False) == R_VESC_VX
    assert yaw_rate(-0.1 + 0.003, 0.003) == 0.1
    # Tilted, upside-down IMU with x backward: still specific force (1.8, -0.55, -10.02) (T4 tile bag).
    c = StillCalib()
    for i in range(200):
        c.add((1e-4 * (i % 3), 0.0, 0.003 + 1e-4 * (i % 2)), (1.8, -0.55, -10.02))
    cal = c.result()
    assert not cal['moving'] and abs(cal['bias_gz'] - 0.00305) < 1e-6
    assert abs(forward_accel((1.8, -0.55, -10.02), cal)) < 1e-9                  # gravity removed
    # 0.5 m/s^2 forward = along -x_imu (perpendicular to gravity): reads +0.5 after the g scaling.
    f = cal['forward']
    a = [g + 0.5 * cal['g_norm'] / G0 * fi for g, fi in zip((1.8, -0.55, -10.02), f)]
    assert abs(forward_accel(a, cal) - 0.5) < 1e-9
    a = [g + 0.3 * gi / cal['g_norm'] for g, gi in zip((1.8, -0.55, -10.02), (1.8, -0.55, -10.02))]
    assert abs(forward_accel(a, cal)) < 1e-9                                       # vertical bump: no leak
    assert 9.0 < cal['tilt_deg'] < 11.0, cal['tilt_deg']
    # 23 Nov eve: yaw rate on the tilted mount. Still specific force of 23 Nov (1.54, -0.50, -10.05): a base yaw
    # rate w seen by the IMU is w * up; -gz alone gives w * 0.987 (the C1 T3 turn deficit), the projection gives w back.
    c2 = StillCalib()
    for i in range(200):
        c2.add((0.001, -0.002, 0.003 + 1e-4 * (i % 2)), (1.54, -0.50, -10.05))
    cal2 = c2.result()
    up = cal2['up']
    w = 0.5
    gyro = [w * u + b for u, b in zip(up, cal2['bias'])]
    assert abs(yaw_rate_vec(gyro, cal2) - w) < 1e-9
    assert abs(yaw_rate_vec(gyro, cal2, 'gz') / w - 0.9871) < 2e-3, yaw_rate_vec(gyro, cal2, 'gz') / w
    assert abs(yaw_rate_vec(cal2['bias'], cal2)) < 1e-12                         # still: zero
    gy = [b for b in cal2['bias']]
    gy[1] += 0.3                                                                   # pure pitch-axis rate leaks
    assert abs(yaw_rate_vec(gy, cal2)) < 0.3 * 0.05      # only through the 2.8 deg roll
    m = StillCalib()
    for i in range(200):
        m.add((0.0, 0.0, 0.05 * (-1) ** i), (1.8, -0.55, -10.02))                  # 2.9 deg/s: moving
    assert m.result()['moving']
    fr = Rf2oFreshness()
    assert fr.check(0.0) is None
    for i in range(20):                                    # 20 Hz, 50 ms latency
        fr.add(1.0 + 0.05 * i, 1.0 + 0.05 * i - 0.05)
    assert fr.check(1.0 + 0.05 * 19 + 0.1) < RF2O_STALE_S and fr.stale_events == 0
    assert fr.check(1.0 + 0.05 * 19 + 0.5) > RF2O_STALE_S and fr.stale_events == 1
    fr.check(1.0 + 0.05 * 19 + 0.9)                        # same gap: counted once
    fr.add(2.5, 2.45)
    r = fr.report(2.5)
    assert fr.stale_events == 1 and abs(r['max_gap_s'] - 0.55) < 1e-9 and abs(r['lat_med_ms'] - 50) < 1e-6
    assert 13.0 < r['hz'] < 15.0, r
    print('PASS: RF2O freshness: stale after %.1f s (once per gap), max gap, latency, rate.' % RF2O_STALE_S)
    print('PASS: VESC vx/gain, brake phase parse, de-weight rule (brake, 0 < |ERPM| < %.0f, 0 trusted),'
          % SENSORLESS_ERPM)
    print('      yaw from the gyro projected on the still gravity (tilt 9.2 deg: -gz alone 0.987 x), R_IMU_WZ %.1e,'
          % R_IMU_WZ)
    print('      yaw sign/bias, still calibration (moving -> retry), forward accel: gravity and vertical')
    print('      removed on a tilted, x-backward IMU.')


# ---------------------------------------------------------------- live node

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--no-rf2o', action='store_true', help='do not forward /odom_rf2o')
    ap.add_argument('--self-test', action='store_true')
    args, _ = ap.parse_known_args()
    if args.self_test:
        self_test()
        return 0
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from vesc_msgs.msg import VescStateStamped
    gain = gain_from_params()

    class Inputs(Node):
        def __init__(self):
            super().__init__('ekf_shadow_inputs')
            self.pub = {k: self.create_publisher(Imu if k == 'imu' else Odometry, t, 20)
                        for k, t in TOPICS.items() if not (k == 'rf2o' and args.no_rf2o)}
            self.calib, self.cal, self.braking, self.status_at = StillCalib(), None, False, None
            self.create_subscription(VescStateStamped, '/sensors/core', self.on_core, qos_profile_sensor_data)
            self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
            self.create_subscription(String, '/t1b_stop_converter/status', self.on_status, 10)
            if not args.no_rf2o:
                self.create_subscription(Odometry, '/odom_rf2o', self.on_rf2o, qos_profile_sensor_data)
            self.t_start = self.get_clock().now()
            self.create_timer(5.0, self.check_status)
            self.fresh = Rf2oFreshness()
            if not args.no_rf2o:
                self.create_timer(0.1, self.check_rf2o)
                self.create_timer(10.0, self.report_rf2o)
            self.get_logger().warning('SHADOW ONLY: publishes /ekf_shadow/in/* only. gain %.0f. '
                                      'Keep the car still: IMU still calibration %.0f s.' % (gain, StillCalib.SECONDS))

        def on_status(self, msg):
            self.braking, self.status_at = brake_active(msg.data), self.get_clock().now()

        def check_status(self):
            if self.status_at is None or (self.get_clock().now() - self.status_at).nanoseconds > 1e9:
                self.get_logger().warning('No /t1b_stop_converter/status: brake de-weighting OFF '
                                          '(|ERPM| < %.0f rule only).' % SENSORLESS_ERPM)
                self.braking = False

        def on_core(self, m):
            for key, dw in (('vesc', False), ('vesc_dw', True)):
                self.pub[key].publish(vesc_msg(m.header.stamp, m.state.speed, self.braking, gain, dw))

        def on_imu(self, m):
            g, a = m.angular_velocity, m.linear_acceleration
            if self.cal is None:
                self.calib.add((g.x, g.y, g.z), (a.x, a.y, a.z))
                if len(self.calib.g) >= StillCalib.SAMPLES:
                    r = self.calib.result()
                    if r and not r['moving']:
                        self.cal = r
                        self.get_logger().info('IMU still calibration OK: gz bias %.5f rad/s, tilt %.1f deg, '
                                               '|g| %.3f' % (r['bias_gz'], r['tilt_deg'], r['g_norm']))
                    else:
                        self.get_logger().warning('IMU still calibration: MOTION (gyro std %.2f deg/s). Keep '
                                                  'the car still; retrying.' % math.degrees((r or {}).get('gyro_std', 0.0)))
                        self.calib = StillCalib()
                return
            self.pub['imu'].publish(imu_msg(m.header.stamp, (g.x, g.y, g.z), (a.x, a.y, a.z), self.cal))

        def on_rf2o(self, m):
            self.fresh.add(self.now_s(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            self.pub['rf2o'].publish(rf2o_msg(m))

        def now_s(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def check_rf2o(self):
            n0 = self.fresh.stale_events
            age = self.fresh.check(self.now_s())
            if age is None:
                if (self.get_clock().now() - self.t_start).nanoseconds > 5e9:
                    self.get_logger().warning('RF2O MISSING: no /odom_rf2o yet: C1 has no fallback speed.',
                                              throttle_duration_sec=5.0)
            elif self.fresh.stale_events > n0:
                self.get_logger().warning('RF2O STALE: no /odom_rf2o for %.2f s (> %.1f s = EKF sensor_timeout): '
                                          'C1 has no fallback while the VESC is de-weighted.' % (age, RF2O_STALE_S))

        def report_rf2o(self):
            r = self.fresh.report(self.now_s())
            lat = '-' if r['lat_med_ms'] is None else '%.0f / %.0f ms' % (r['lat_med_ms'], r['lat_max_ms'])
            line = ('RF2O freshness (10 s): %.1f Hz, max gap %.2f s, latency median / max %s, stale events total %d'
                    % (r['hz'], r['max_gap_s'], lat, r['stale_events']))
            ok = r['hz'] >= 15.0 and r['max_gap_s'] <= RF2O_STALE_S
            (self.get_logger().info if ok else self.get_logger().warning)(('OK   ' if ok else 'FAIL ') + line)

    rclpy.init()
    node = Inputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
