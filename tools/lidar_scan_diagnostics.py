#!/usr/bin/env python3

import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarDiagnostics(Node):
    def __init__(self):
        super().__init__('lidar_scan_diagnostics')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback,
            qos_profile_sensor_data
        )

        self.done = False


    def angle_deg(self, msg, index):
        angle = msg.angle_min + index * msg.angle_increment
        deg = math.degrees(angle)

        while deg >= 180.0:
            deg -= 360.0
        while deg < -180.0:
            deg += 360.0

        return deg


    def longest_invalid_sector(self, valid):
        n = len(valid)

        invalid = [not x for x in valid]

        if all(invalid):
            return 0, n

        # Invalid samples from beginning
        prefix = 0
        while prefix < n and invalid[prefix]:
            prefix += 1

        # Invalid samples from end
        suffix = 0
        while suffix < n and invalid[n - 1 - suffix]:
            suffix += 1

        best_start = 0
        best_length = 0

        # Circular run crossing -180/+180
        if prefix + suffix > 0:
            best_start = n - suffix
            best_length = prefix + suffix

        # Internal invalid runs
        current_start = None
        current_length = 0

        for i in range(prefix, n - suffix):
            if invalid[i]:
                if current_start is None:
                    current_start = i
                    current_length = 1
                else:
                    current_length += 1
            else:
                if current_length > best_length:
                    best_start = current_start
                    best_length = current_length

                current_start = None
                current_length = 0

        if current_length > best_length:
            best_start = current_start
            best_length = current_length

        return best_start, best_length


    def distance_near_angle(self, msg, target_deg, window=3):
        n = len(msg.ranges)

        target_rad = math.radians(target_deg)

        index = round(
            (target_rad - msg.angle_min) /
            msg.angle_increment
        )

        index = max(0, min(n - 1, index))

        values = []

        for offset in range(-window, window + 1):
            i = index + offset

            if 0 <= i < n:
                r = msg.ranges[i]

                if math.isfinite(r) and r > 0:
                    values.append(r)

        if not values:
            return None

        return statistics.median(values)


    def callback(self, msg):
        if self.done:
            return

        self.done = True

        ranges = list(msg.ranges)
        n = len(ranges)

        valid = [
            math.isfinite(r) and r > 0.0
            for r in ranges
        ]

        finite_ranges = [
            r for r in ranges
            if math.isfinite(r) and r > 0.0
        ]

        blind_start, blind_length = \
            self.longest_invalid_sector(valid)

        blind_end = \
            (blind_start + blind_length - 1) % n

        active_start = \
            (blind_end + 1) % n

        active_length = \
            n - blind_length

        active_end = \
            (active_start + active_length - 1) % n

        increment_deg = \
            math.degrees(msg.angle_increment)

        blind_width = \
            blind_length * abs(increment_deg)

        active_width = \
            active_length * abs(increment_deg)

        # Count valid returns only inside active sector
        active_indices = [
            (active_start + i) % n
            for i in range(active_length)
        ]

        active_valid = sum(
            1 for i in active_indices if valid[i]
        )

        active_invalid = \
            active_length - active_valid

        print()
        print("========== LAKIBEAM DIAGNOSTICS ==========")

        print(f"Frame: {msg.header.frame_id}")
        print()

        print(f"Total samples:       {n}")
        print(f"Angular increment:   {increment_deg:.4f} deg")

        print(f"Scan time:           {msg.scan_time:.6f} s")

        if msg.scan_time > 0:
            print(
                f"Frequency:           "
                f"{1.0 / msg.scan_time:.3f} Hz"
            )

        print()

        print(
            f"Main blind sector:   "
            f"{blind_length} samples"
        )

        print(
            f"Blind sector width:  "
            f"~{blind_width:.2f} deg"
        )

        print(
            f"Blind sector angles: "
            f"{self.angle_deg(msg, blind_start):.2f} deg "
            f"to {self.angle_deg(msg, blind_end):.2f} deg"
        )

        print()

        print(
            f"Active sector:       "
            f"{active_length} samples"
        )

        print(
            f"Active FOV:          "
            f"~{active_width:.2f} deg"
        )

        print(
            f"Active angles:       "
            f"{self.angle_deg(msg, active_start):.2f} deg "
            f"to {self.angle_deg(msg, active_end):.2f} deg"
        )

        print()

        print(
            f"Valid active returns:   "
            f"{active_valid}"
        )

        print(
            f"Invalid active returns: "
            f"{active_invalid}"
        )

        if active_length:
            print(
                f"Active valid ratio:     "
                f"{100.0 * active_valid / active_length:.1f}%"
            )

        print()

        if finite_ranges:
            print(
                f"Closest return:       "
                f"{min(finite_ranges):.3f} m"
            )

            print(
                f"Farthest return now:  "
                f"{max(finite_ranges):.3f} m"
            )

        print()
        print("Distances around key ROS angles:")

        for angle in [-135, -90, -45, 0, 45, 90, 135]:
            value = self.distance_near_angle(
                msg, angle
            )

            if value is None:
                print(
                    f"{angle:+4d} deg : no valid return"
                )
            else:
                print(
                    f"{angle:+4d} deg : {value:.3f} m"
                )

        print()
        print("==========================================")
        print()

        rclpy.shutdown()


def main():
    rclpy.init()

    node = LidarDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
