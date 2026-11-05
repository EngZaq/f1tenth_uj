#!/usr/bin/env python3
"""Read-only F1TENTH front LiDAR angular profile.

Distinguishes +Inf/no-return rays from finite distances by bearing.
It never publishes messages or declares +Inf to be free space.
Requires only rclpy + sensor_msgs from ROS 2 Humble.
"""

import argparse
import json
import math
import statistics
import time
from pathlib import Path


def classify_ray(value, rmin, rmax):
    if math.isnan(value):
        return "nan"
    if math.isinf(value):
        return "pos_inf" if value > 0 else "neg_inf"
    if value < rmin:
        return "below_min"
    if value > rmax:
        return "above_max"
    return "valid"


def longest_run(flags):
    best = curr = 0
    for flag in flags:
        curr = curr + 1 if flag else 0
        best = max(best, curr)
    return best


def percent(values, p):
    if not values:
        return None
    vals = sorted(values)
    return vals[min(len(vals) - 1, math.ceil(len(vals) * p / 100) - 1)]


def self_test():
    assert classify_ray(float("inf"), 0.05, 25.) == "pos_inf"
    assert classify_ray(float("-inf"), 0.05, 25.) == "neg_inf"
    assert classify_ray(float("nan"), 0.05, 25.) == "nan"
    assert classify_ray(0., 0.05, 25.) == "below_min"
    assert classify_ray(30., 0.05, 25.) == "above_max"
    assert classify_ray(2., 0.05, 25.) == "valid"
    assert longest_run([False, True, True, False, True]) == 2
    assert longest_run([True, True, True]) == 3
    assert longest_run([]) == 0
    assert percent([1, 2, 3, 4, 5], 50) == 3
    print("PASS: ray classification, unknown run and percentile tests")
    print("No ROS messages or actuator commands were published.")


class AngularProfile:
    def __init__(self, half_fov=60., bin_width=5.):
        self.half = half_fov
        self.width = bin_width
        self.bin_count = math.ceil(2 * half_fov / bin_width)
        self.classes = (
            "valid", "pos_inf", "neg_inf", "nan", "below_min", "above_max"
        )
        self.bins = [
            {name: 0 for name in self.classes}
            for _ in range(self.bin_count)
        ]
        self.bins_min = [None] * self.bin_count
        self.frames = 0
        self.total_scan_beams = None
        self.range_min = None
        self.range_max = None
        self.frame_id = None
        self.scan_stamps_zero = 0
        self.gaps_forward20 = []
        self.invalid_fraction_forward20 = []
        self.center10_valid_fraction = []

    def record(self, scan):
        self.frames += 1
        self.total_scan_beams = len(scan.ranges)
        self.range_min = float(scan.range_min)
        self.range_max = float(scan.range_max)
        self.frame_id = scan.header.frame_id
        if scan.header.stamp.sec == 0 and scan.header.stamp.nanosec == 0:
            self.scan_stamps_zero += 1
        invalid_20 = []
        center_total, center_valid = 0, 0
        full_20, valid_20 = 0, 0
        step_deg = abs(math.degrees(scan.angle_increment))
        for i, rv in enumerate(scan.ranges):
            angle = math.degrees(scan.angle_min + i * scan.angle_increment)
            if not -self.half <= angle < self.half:
                continue
            category = classify_ray(float(rv), self.range_min, self.range_max)
            bin_idx = min(
                self.bin_count - 1,
                int((angle + self.half) / self.width)
            )
            self.bins[bin_idx][category] += 1
            if category == "valid":
                v = float(rv)
                old = self.bins_min[bin_idx]
                if old is None or v < old:
                    self.bins_min[bin_idx] = v
            if abs(angle) <= 20.:
                full_20 += 1
                is_valid = category == "valid"
                valid_20 += int(is_valid)
                invalid_20.append(not is_valid)
            if abs(angle) <= 5.:
                center_total += 1
                center_valid += int(category == "valid")
        if full_20:
            self.invalid_fraction_forward20.append(
                1. - valid_20 / full_20
            )
            self.gaps_forward20.append(longest_run(invalid_20) * step_deg)
        if center_total:
            self.center10_valid_fraction.append(
                center_valid / center_total
            )

    def report(self):
        output = []
        for i, counter in enumerate(self.bins):
            n = sum(counter.values())
            left = -self.half + i * self.width
            right = min(self.half, left + self.width)
            output.append({
                "bearing_deg": [round(left, 2), round(right, 2)],
                "readings": n,
                "valid_fraction": round(counter["valid"] / n, 4) if n else None,
                "pos_inf_fraction": round(counter["pos_inf"] / n, 4) if n else None,
                "minimum_finite_m": round(self.bins_min[i], 3)
                                    if self.bins_min[i] is not None else None,
                "classification_counts": counter,
            })
        return {
            "test": "Read-only physical LiDAR angular classification",
            "scans": self.frames,
            "scan_beams": self.total_scan_beams,
            "laser_frame": self.frame_id,
            "range_min_m": self.range_min,
            "range_max_m": self.range_max,
            "zero_scan_timestamps": self.scan_stamps_zero,
            "front_40deg_invalid_fraction_median": (
                round(statistics.median(self.invalid_fraction_forward20), 4)
                if self.invalid_fraction_forward20 else None
            ),
            "central_10deg_valid_fraction_median": (
                round(statistics.median(self.center10_valid_fraction), 4)
                if self.center10_valid_fraction else None
            ),
            "longest_consecutive_invalid_arc_forward40deg_p50_deg": (
                round(statistics.median(self.gaps_forward20), 2)
                if self.gaps_forward20 else None
            ),
            "longest_consecutive_invalid_arc_forward40deg_p95_deg": (
                round(percent(self.gaps_forward20, 95), 2)
                if self.gaps_forward20 else None
            ),
            "bearing_bins": output,
            "caution": (
                "Positive infinity means no finite return was measured. "
                "It is NOT automatically proof of free space or an obstacle. "
                "No motor commands are published and no safe driving corridor "
                "is certified by this tool."
            ),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--duration", type=float, default=20.)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not 5. <= args.duration <= 180.:
        parser.error("duration must be 5..180 seconds")
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("f1tenth_lidar_angle_profile")
    profile = AngularProfile()
    subscriber = node.create_subscription(
        LaserScan, "/scan", profile.record, qos_profile_sensor_data)
    print("READ-ONLY LiDAR profile: collecting for", args.duration, "seconds")
    try:
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(subscriber)
        node.destroy_node()
        rclpy.shutdown()
    result = profile.report()
    print("=== FRONT LiDAR ANGULAR PROFILE ===")
    print("scans:", result["scans"],
          "rays/scan:", result["scan_beams"],
          "frame:", result["laser_frame"])
    print("sensor range min/max (m):", result["range_min_m"],
          "/", result["range_max_m"])
    print("median invalid ±20deg:", result["front_40deg_invalid_fraction_median"])
    print("median valid ±5deg:", result["central_10deg_valid_fraction_median"])
    print("longest missing arc forward ±20deg (p50,p95) degrees:",
          result["longest_consecutive_invalid_arc_forward40deg_p50_deg"],
          result["longest_consecutive_invalid_arc_forward40deg_p95_deg"])
    print("\nBearing       Valid %   +Inf %   Closest finite (m)")
    for b in result["bearing_bins"]:
        a0, a1 = b["bearing_deg"]
        valid = b["valid_fraction"]
        inf = b["pos_inf_fraction"]
        print(f"{a0:+6.1f}..{a1:+6.1f}  "
              f"{valid*100:6.1f}    {inf*100:6.1f}    {b['minimum_finite_m']}"
              if valid is not None and inf is not None
              else f"{a0:+6.1f}..{a1:+6.1f}  no readings")
    if args.output:
        out = Path(args.output).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print("Saved:", out)
    if result["scans"] < 20:
        raise SystemExit("FAILED: fewer than 20 scans received")


if __name__ == "__main__":
    main()
