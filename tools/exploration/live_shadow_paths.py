#!/usr/bin/env python3
"""LIVE LiDAR candidate-path visualization in physical SHADOW mode.

Reads the real /scan, publishes five nav_msgs/Path previews plus a JSON
std_msgs/String diagnostic. No AckermannDriveStamped, motor, servo, VESC,
arm/enable, or TF publishers; NOT a driver or a validated safety system.

Paths are geometry previews even when the rays are unknown or obstructed.
They must NOT be connected to a physical vehicle command conversion bridge.
This diagnostic cannot determine complete 3-D collision-free motion.
"""

import argparse
import json
import math
import time

from offline_swept_footprint import (
    CANDIDATES, analyze_candidate, bicycle_pose, samples_for_path,
)


TOPIC_SUFFIX = {
    -0.150: 'right_sharp',
    -0.075: 'right_soft',
    0.0: 'straight',
    0.075: 'left_soft',
    0.150: 'left_sharp',
}


def self_test():
    assert len(TOPIC_SUFFIX) == len(CANDIDATES)
    for delta in CANDIDATES:
        assert delta in TOPIC_SUFFIX
        x, y, yaw = bicycle_pose(.5, delta)
        assert all(math.isfinite(v) for v in (x, y, yaw))
        assert x > 0
    print('PASS: path names and 2D bicycle geometry.')
    print('No actuator topics or ROS commands published by self-test.')


def run(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    class LivePreview(Node):
        def __init__(self):
            super().__init__('f1tenth_live_shadow_paths')
            self.latest = None
            self.latest_received = None
            self.readings = 0
            self.current_status = None
            self.probe_points = {
                delta: list(samples_for_path(
                    delta, args.preview_m, args.path_step_m, args.xy_step_m
                )) for delta in CANDIDATES
            }
            self.paths = {
                delta: self.create_publisher(
                    Path, '/exploration/shadow_path_' + TOPIC_SUFFIX[delta], 10
                ) for delta in CANDIDATES
            }
            self.diagnostics = self.create_publisher(
                String, '/exploration/shadow_path_diagnostics', 10
            )
            self.subscription = self.create_subscription(
                LaserScan, '/scan', self.on_scan, qos_profile_sensor_data
            )
            self.timer = self.create_timer(0.2, self.on_tick)

        def on_scan(self, msg):
            self.latest = msg
            self.latest_received = time.monotonic()
            self.readings += 1

        def render_paths(self, now_msg):
            for steer in CANDIDATES:
                path = Path()
                path.header.frame_id = 'base_link'
                path.header.stamp = now_msg
                for i in range(11):
                    x, y, yaw = bicycle_pose(
                        args.preview_m * i / 10, steer
                    )
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.position.z = 0.0
                    pose.pose.orientation.z = math.sin(yaw / 2)
                    pose.pose.orientation.w = math.cos(yaw / 2)
                    path.poses.append(pose)
                self.paths[steer].publish(path)

        def on_tick(self):
            if self.latest_received is None:
                self.emit({'status': 'WAITING_FOR_LIDAR',
                           'actuation': 'DISABLED'})
                return

            age_s = time.monotonic() - self.latest_received
            if age_s > .25 or age_s < 0:
                self.emit({'status': 'STALE_LIDAR',
                           'age_seconds': round(age_s, 3),
                           'actuation': 'DISABLED'})
                return

            scan = self.latest
            if scan.header.frame_id != 'laser':
                self.emit({'status': 'UNEXPECTED_SCAN_FRAME',
                           'frame': scan.header.frame_id,
                           'actuation': 'DISABLED'})
                return

            now_msg = self.get_clock().now().to_msg()
            self.render_paths(now_msg)
            result = {}
            for steer in CANDIDATES:
                sample = analyze_candidate(scan, self.probe_points[steer])
                result[TOPIC_SUFFIX[steer]] = {
                    'steering_rad_geometry_only': steer,
                    'measured_fraction': (
                        round(sample['fraction_observed'], 4)
                        if sample['fraction_observed'] is not None else None
                    ),
                    'unknown_fraction': (
                        round(sample['fraction_unknown'], 4)
                        if sample['fraction_unknown'] is not None else None
                    ),
                    'near_or_occluded_fraction': (
                        round(sample['fraction_hit_or_occluded'], 4)
                        if sample['fraction_hit_or_occluded'] is not None else None
                    ),
                    'new_sampled_points': sample['newly_sampled_points'],
                    'all_sampled_points_measured': sample[
                        'no_unmeasured_sampled_points'],
                    'no_close_observations_in_probes': sample[
                        'no_observed_close_samples'],
                }

            self.emit({
                'status': 'LIVE_SHADOW_PATHS_ONLY',
                'laser_frame': scan.header.frame_id,
                'scan_age_s': round(age_s, 4),
                'scan_count': self.readings,
                'preview_m': args.preview_m,
                'candidate_results': result,
                'actuation': 'DISABLED',
                'warning': (
                    'Paths are visualization regardless of unknown/near rays. '
                    'Not a collision-free trajectory or motion permission. '
                    'No Ackermann/VESC/servo commands are published.'
                )
            })

        def emit(self, d):
            self.diagnostics.publish(String(data=json.dumps(d, separators=(',',':'))))
            status = d['status']
            if status != self.current_status:
                self.get_logger().info('SHADOW PREVIEW: ' + status)
                self.current_status = status

    rclpy.init()
    node = LivePreview()
    print('Starting non-actuating LiDAR path preview: /exploration/shadow_path_*')
    print('Diagnostics: /exploration/shadow_path_diagnostics')
    print('WARNING: visualization only; DO NOT CONNECT TO MOTOR CONTROL')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--preview-m', type=float, default=0.50)
    parser.add_argument('--path-step-m', type=float, default=0.05)
    parser.add_argument('--xy-step-m', type=float, default=0.05)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not 0.15 <= args.preview_m <= 0.7:
        parser.error('preview distance must be 0.15–0.70 m')
    if not 0.025 <= args.path_step_m <= 0.10:
        parser.error('path step must be 0.025–0.10 m')
    if not 0.025 <= args.xy_step_m <= 0.10:
        parser.error('xy step must be 0.025–0.10 m')
    run(args)


if __name__ == '__main__':
    main()
