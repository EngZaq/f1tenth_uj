#!/usr/bin/env python3
"""
real_car_tests.py - one script that runs the real-car characterisation tests in sequence.

WHAT IT DOES
  Runs a list of open-loop manoeuvres (straight lines at 3 speeds, 4 circles) one after another.
  For each manoeuvre it measures, from /odom + /scan + the servo command:
    straight: odom distance vs LiDAR distance (-> speed_to_erpm_gain correction),
              sideways drift vs a side wall (-> steering offset check),
              start delay (command -> wheels moving), stopping distance + deceleration.
    circle:   the steering angle that was really commanded to the servo (after clamping)
              and the radius it SHOULD give. You measure the real diameter on the floor.
  Results are printed and appended to ~/real_car_tests/<time>/results.csv.

SAFETY (why it is built this way)
  - Keyboard over SSH, no PS4. A terminal does not report "key released", so the deadman uses
    KEY REPEAT: while you hold SPACE the laptop sends ~30 spaces/s. When they stop arriving
    for `repeat_timeout` (0.25 s) the car stops. Before the repeat starts (first ~0.5 s of
    holding) the limit is `first_press_timeout` (0.7 s).
    -> An SSH/Wi-Fi drop also stops the car (no more spaces arrive).
  - On stop, speed 0 is sent and kept for >= 1 s (we do NOT just stop publishing: the VESC
    could keep the last command).
  - No /odom or /scan -> stop. Max time per manoeuvre.
  - Straight runs stop automatically before the wall in front:
      stop when front range <= stop_margin + v^2 / (2 * assumed_decel).
  - Run it ON THE STAND first (wheels off the ground) to check the stop-on-release.
  - Have a second person ready to pick up the car / unplug the VESC battery.

HOW TO USE (keyboard, in the SSH terminal that runs this script)
  n           = arm the next manoeuvre (printed in the terminal)
  hold SPACE  = drive; release = stop
  s           = skip this manoeuvre
  q / Ctrl+C  = stop and quit
  Do not run the PS4/teleop drive at the same time.

NEEDS RUNNING: vesc driver + ackermann_to_vesc + vesc_to_odom (/odom), LiDAR (/scan).
"""
import csv
import math
import os
import queue
import select
import statistics
import sys
import termios
import threading
import time
import tty
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


class Keyboard:
    """Reads single keys from the terminal (no Enter needed) in a background thread."""

    def __init__(self, first_press_timeout, repeat_timeout):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # keys arrive one by one; Ctrl+C still works
        self.first_to, self.repeat_to = first_press_timeout, repeat_timeout
        self.keys = queue.Queue()
        self.space_t = 0.0
        self.space_count = 0
        self.eof = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self.eof:
            ready, _, _ = select.select([self.fd], [], [], 0.1)
            if not ready:
                continue
            ch = os.read(self.fd, 1)
            if not ch:  # terminal closed (SSH dropped)
                self.eof = True
                break
            now = time.monotonic()
            if ch == b" ":
                if now - self.space_t > self.first_to:
                    self.space_count = 0  # a new press, not a repeat
                self.space_t = now
                self.space_count += 1
            else:
                self.keys.put(ch.decode(errors="ignore").lower())

    def space_held(self):
        if self.eof:
            return False
        limit = self.repeat_to if self.space_count >= 2 else self.first_to
        return time.monotonic() - self.space_t < limit

    def get_key(self):
        try:
            return self.keys.get_nowait()
        except queue.Empty:
            return None

    def restore(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

# name, kind, speed [m/s], steering [rad, + = left], max distance [m] (straight only), max time [s]
# Circles: +/-0.60 rad is more than the servo limits allow -> the VESC driver clamps it,
# so this gives the REAL full lock on each side. Release SPACE after one full circle.
MANEUVERS = [
    ("straight_0.5", "straight", 0.5, 0.0, 3.0, 10.0),
    ("straight_1.0", "straight", 1.0, 0.0, 3.0, 8.0),
    ("straight_1.5", "straight", 1.5, 0.0, 3.0, 6.0),
    ("circle_left_full", "circle", 0.5, +0.60, None, 20.0),
    ("circle_right_full", "circle", 0.5, -0.60, None, 20.0),
    ("circle_left_half", "circle", 0.5, +0.25, None, 25.0),
    ("circle_right_half", "circle", 0.5, -0.25, None, 25.0),
]


def scan_range(scan, angle, half_width_deg=3.0, use_min=False):
    """Median (or min) LiDAR range in a small window around `angle` (rad, 0 = car front)."""
    if scan is None or scan.angle_increment == 0.0:
        return float("nan")
    n = len(scan.ranges)
    full_circle = (scan.angle_max - scan.angle_min) > 6.2
    hw = math.radians(half_width_deg)
    lo = int(math.floor((angle - hw - scan.angle_min) / scan.angle_increment))
    hi = int(math.ceil((angle + hw - scan.angle_min) / scan.angle_increment))
    vals = []
    for i in range(lo, hi + 1):
        if full_circle:
            i %= n
        elif not 0 <= i < n:
            continue
        r = scan.ranges[i]
        if math.isfinite(r) and scan.range_min < r < scan.range_max:
            vals.append(r)
    if not vals:
        return float("nan")
    return min(vals) if use_min else statistics.median(vals)


class RealCarTests(Node):
    def __init__(self):
        super().__init__("real_car_tests")
        p = self.declare_parameter
        self.drive_topic = p("drive_topic", "/drive").value
        self.wheelbase = p("wheelbase", 0.25).value            # MEASURE IT and pass the real value
        self.speed_gain = p("speed_to_erpm_gain", 8414.0).value  # only used to print the suggestion
        self.servo_offset = p("steering_angle_to_servo_offset", 0.361).value
        self.servo_gain = p("steering_angle_to_servo_gain", -0.50).value
        self.kb = Keyboard(p("first_press_timeout", 0.7).value, p("repeat_timeout", 0.25).value)
        self.stop_margin = p("stop_margin", 0.6).value          # m kept free in front of the car
        self.assumed_decel = p("assumed_decel", 1.5).value      # m/s^2, conservative on purpose

        self.pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Float64, "/sensors/servo_position_command", self.on_servo, 10)

        self.scan = None; self.scan_t = 0.0
        self.v = 0.0; self.odom_t = 0.0; self.odom_dist = 0.0
        self.servo = float("nan")

        self.idx = 0
        self.state = "WAIT"
        self.r = {}  # results of the current manoeuvre
        self.out_dir = os.path.expanduser(
            "~/real_car_tests/" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, "results.csv")

        self.create_timer(0.02, self.tick)  # 50 Hz command loop
        self.get_logger().info(f"Publishing to {self.drive_topic}. Results -> {self.csv_path}")
        self.prompt()

    # ---------------- callbacks ----------------
    def on_scan(self, msg):
        self.scan = msg; self.scan_t = time.monotonic()

    def on_servo(self, msg):
        self.servo = msg.data

    def on_odom(self, msg):
        now = time.monotonic()
        if self.odom_t > 0.0:
            dt = min(now - self.odom_t, 0.2)
            self.odom_dist += abs(self.v) * dt  # distance from wheel speed, independent of steering
        self.v = msg.twist.twist.linear.x
        self.odom_t = now

    # ---------------- helpers ----------------
    def deadman_held(self):
        return self.kb.space_held()

    def send(self, speed, steer):
        m = AckermannDriveStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.drive.speed = float(speed)
        m.drive.steering_angle = float(steer)
        self.pub.publish(m)

    def prompt(self):
        if self.idx >= len(MANEUVERS):
            self.get_logger().info("All manoeuvres done. Ctrl+C to exit.")
            self.state = "DONE"
            return
        name, kind, v, steer, dmax, tmax = MANEUVERS[self.idx]
        self.get_logger().info(
            f"\n[{self.idx + 1}/{len(MANEUVERS)}] NEXT: {name}  v={v} m/s steer={steer:+.2f} rad. "
            f"Place the car ({'straight at a wall in front' if kind == 'straight' else 'free space ~2.5 m'}). "
            f"n = arm, s = skip, q = quit.")

    # ---------------- main loop ----------------
    def tick(self):
        now = time.monotonic()
        name, kind, v_cmd, steer, dmax, tmax = (MANEUVERS[self.idx] if self.idx < len(MANEUVERS)
                                                  else (None,) * 6)

        key = self.kb.get_key()
        if key == "q" or self.kb.eof:
            self.send(0.0, 0.0)
            raise KeyboardInterrupt

        if self.state == "WAIT":
            if key == "s":
                self.get_logger().info(f"Skipped {name}."); self.idx += 1; self.prompt()
            elif key == "n":
                self.state = "ARMED"
                self.get_logger().info("Armed. HOLD SPACE to drive, release to stop.")

        elif self.state == "ARMED":
            if self.deadman_held():
                if now - self.scan_t > 0.5 or now - self.odom_t > 0.5:
                    self.get_logger().error("No fresh /scan or /odom - not starting.")
                    self.state = "WAIT"; return
                self.r = dict(name=name, kind=kind, v_cmd=v_cmd, steer_cmd=steer,
                              t0=now, t_move=None, v_peak=0.0, servo_seen=[],
                              front0=scan_range(self.scan, 0.0),
                              left0=scan_range(self.scan, math.pi / 2),
                              right0=scan_range(self.scan, -math.pi / 2))
                self.odom_dist = 0.0
                self.state = "RUN"

        elif self.state == "RUN":
            r = self.r
            if r["t_move"] is None and abs(self.v) > 0.1:
                r["t_move"] = now
            r["v_peak"] = max(r["v_peak"], abs(self.v))
            if math.isfinite(self.servo):
                r["servo_seen"].append(self.servo)

            front_min = scan_range(self.scan, 0.0, half_width_deg=10.0, use_min=True)
            stop_range = self.stop_margin + abs(self.v) ** 2 / (2 * self.assumed_decel)
            reason = None
            if not self.deadman_held():
                reason = "deadman released"
            elif now - self.scan_t > 0.3 or now - self.odom_t > 0.3:
                reason = "sensor timeout"
            elif now - r["t0"] > tmax:
                reason = "max time"
            elif kind == "straight" and self.odom_dist >= dmax:
                reason = "distance reached"
            elif kind == "straight" and math.isfinite(front_min) and front_min <= stop_range:
                reason = "wall ahead"

            if reason:
                r.update(stop_reason=reason, t_stop=now, v_at_stop=abs(self.v),
                         dist_at_stop=self.odom_dist, front_at_stop=scan_range(self.scan, 0.0))
                self.still_since = None
                self.state = "STOP"
                self.send(0.0, 0.0)
            else:
                self.send(v_cmd, steer)

        elif self.state == "STOP":
            self.send(0.0, 0.0)  # keep sending zero, do not go silent
            if abs(self.v) < 0.03:
                self.still_since = self.still_since or now
            else:
                self.still_since = None
            stopped = self.still_since is not None and now - self.still_since > 0.5
            if (stopped and now - self.r["t_stop"] > 1.0) or now - self.r["t_stop"] > 5.0:
                self.finish(now)
                self.idx += 1
                self.state = "WAIT"
                self.prompt()


    # ---------------- results ----------------
    def finish(self, now):
        r = self.r
        nan = float("nan")
        out = dict(name=r["name"], stop_reason=r["stop_reason"], v_cmd=r["v_cmd"],
                   steer_cmd=r["steer_cmd"], v_peak=round(r["v_peak"], 3))
        out["start_delay_s"] = round(r["t_move"] - r["t0"], 3) if r["t_move"] else nan

        if r["kind"] == "straight":
            front_end = scan_range(self.scan, 0.0)
            lidar_dist = r["front0"] - front_end
            stop_d = r["front_at_stop"] - front_end
            out.update(
                odom_dist_m=round(self.odom_dist, 3),
                lidar_dist_m=round(lidar_dist, 3),
                odom_over_lidar=round(self.odom_dist / lidar_dist, 4) if lidar_dist > 0.5 else nan,
                left_drift_m=round(scan_range(self.scan, math.pi / 2) - r["left0"], 3),
                right_drift_m=round(scan_range(self.scan, -math.pi / 2) - r["right0"], 3),
                v_at_stop=round(r["v_at_stop"], 3),
                stop_dist_lidar_m=round(stop_d, 3),
                stop_dist_odom_m=round(self.odom_dist - r["dist_at_stop"], 3),
                decel_mps2=round(r["v_at_stop"] ** 2 / (2 * stop_d), 2) if stop_d > 0.02 else nan,
            )
            if math.isfinite(out["odom_over_lidar"]):
                out["suggested_speed_gain"] = round(self.speed_gain * out["odom_over_lidar"], 0)
        else:
            servo = statistics.median(r["servo_seen"]) if r["servo_seen"] else nan
            delta = (servo - self.servo_offset) / self.servo_gain  # angle actually sent to the servo
            out.update(servo_cmd=round(servo, 4), steer_after_clamp_rad=round(delta, 3),
                       expected_radius_rear_axle_m=round(self.wheelbase / math.tan(abs(delta)), 3)
                       if abs(delta) > 1e-3 else nan,
                       measured_diameter_m="FILL IN")

        for k, v in out.items():
            self.get_logger().info(f"  {k:28s} {v}")
        new = not os.path.exists(self.csv_path)
        keys = ["name", "stop_reason", "v_cmd", "steer_cmd", "v_peak", "start_delay_s",
                "odom_dist_m", "lidar_dist_m", "odom_over_lidar", "suggested_speed_gain",
                "left_drift_m", "right_drift_m", "v_at_stop", "stop_dist_lidar_m",
                "stop_dist_odom_m", "decel_mps2", "servo_cmd", "steer_after_clamp_rad",
                "expected_radius_rear_axle_m", "measured_diameter_m"]
        with open(self.csv_path, "a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            if new:
                w.writeheader()
            w.writerow({k: out.get(k, "") for k in keys})


def main():
    rclpy.init()
    node = RealCarTests()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(25):  # leave the car commanded to zero
            node.send(0.0, 0.0)
            time.sleep(0.02)
        node.kb.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
