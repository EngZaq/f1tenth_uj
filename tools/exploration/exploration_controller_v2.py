#!/usr/bin/env python3
"""Portable LiDAR explorer: simulator driving or physical shadow evaluation.

Hardware driving is intentionally blocked in this prototype.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDriveStamped

class ExplorationController(Node):
    def __init__(self):
        super().__init__('uj_exploration_controller')
        self.declare_parameter('enable_drive', False)
        self.declare_parameter('platform', 'sim')  # sim or physical
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('proposal_topic', '/exploration/drive')
        self.declare_parameter('speed', 0.30)
        self.declare_parameter('max_steer', 0.40)
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('lookahead', 0.85)
        self.declare_parameter('stop_distance', 0.60)
        self.declare_parameter('scan_timeout', 0.25)
        self.declare_parameter('front_fov_deg', 160.0)
        self.declare_parameter('safety_bubble_m', 0.34)
        self.scan = None
        self.scan_received = None
        self.collision = False
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self.scan_cb, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.collision_cb, 10)
        self.proposed = self.create_publisher(AckermannDriveStamped, str(self.get_parameter('proposal_topic').value), 10)
        self.platform = str(self.get_parameter('platform').value)
        if self.platform not in ('sim', 'physical'):
            raise ValueError('platform must be sim or physical')
        if self.platform == 'physical' and bool(self.get_parameter('enable_drive').value):
            raise ValueError('Physical autonomous driving is BLOCKED pending steering calibration, independent e-stop and watchdog')
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10) if self.platform == 'sim' else None
        self.create_timer(0.05, self.tick)
        self.last_log_ns = 0
        self.get_logger().info('Explorer started; drive=%s' % self.get_parameter('enable_drive').value)

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_received = self.get_clock().now()

    def collision_cb(self, msg):
        self.collision = bool(msg.data)

    def compute(self):
        m = self.scan
        a = m.angle_min + np.arange(len(m.ranges)) * m.angle_increment
        r = np.asarray(m.ranges, dtype=float)
        valid = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        # Invalid returns are treated as blocked, NOT as open space.
        r = np.where(valid, r, 0.0)
        fov = math.radians(float(self.get_parameter('front_fov_deg').value)) / 2
        mask = (np.abs(a) <= fov)
        if not np.any(mask):
            return 0.0, 0.0, 'no forward beams'
        a, r = a[mask], r[mask]
        # Stop if nearest return in +/-20 degrees is too close.
        front = r[np.abs(a) < math.radians(20)]
        front = front[front > 0]
        if len(front) < 3:
            return 0.0, 0.0, 'front scan invalid'
        front_clearance = float(np.percentile(front, 15))
        stop = float(self.get_parameter('stop_distance').value)
        if front_clearance < stop:
            return 0.0, 0.0, 'front obstacle'
        # Disparity-aware bubble: exclude angles within a vehicle-width safety
        # bubble around short obstacle returns. Filtered range is conservative.
        inflated = r.copy()
        spacing = abs(m.angle_increment)
        for idx in np.where((r > 0.05) & (r < 1.75))[0]:
            width = math.atan2(float(self.get_parameter('safety_bubble_m').value), max(r[idx], 0.05))
            n = min(int(width / max(spacing, 1e-4)), 90)
            lo, hi = max(0,idx-n),min(len(r),idx+n+1)
            inflated[lo:hi] = np.minimum(inflated[lo:hi], r[idx])
        # Pick largest contiguous run with >0.75m measured clearance.
        feasible = inflated > max(stop + 0.15, 0.75)
        indices = np.where(feasible)[0]
        if indices.size == 0:
            return 0.0, 0.0, 'no traversable gap'
        groups = np.split(indices, np.where(np.diff(indices) != 1)[0] + 1)
        # Prefer a wide gap ahead, with enough room for vehicle turning.
        chosen = max(groups, key=lambda g: (len(g)*spacing, -abs(a[g[len(g)//2]])))
        if len(chosen) * spacing < math.radians(10):
            return 0.0, 0.0, 'gap too narrow'
        target_idx = chosen[len(chosen)//2]
        angle = float(a[target_idx])
        lookahead = float(self.get_parameter('lookahead').value)
        steer = math.atan2(2*float(self.get_parameter('wheelbase').value)*math.sin(angle), lookahead)
        limit = float(self.get_parameter('max_steer').value)
        steer = max(-limit, min(limit, steer))
        speed = float(self.get_parameter('speed').value)
        # Approach turns cautiously. This is a starting heuristic, not braking verification.
        speed *= max(0.45, 1.0-0.5*abs(steer)/max(limit,0.01))
        if front_clearance < 1.0:
            speed = min(speed, 0.15)
        return steer, speed, 'ok'

    def tick(self):
        now = self.get_clock().now()
        steer, speed, reason = 0.0, 0.0, 'waiting for scan'
        if self.scan is not None and self.scan_received is not None:
            age = (now - self.scan_received).nanoseconds / 1e9
            if age > float(self.get_parameter('scan_timeout').value) or age < 0:
                reason = 'stale scan'
            elif self.collision:
                reason = 'collision'
            else:
                steer, speed, reason = self.compute()
        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.drive.steering_angle = float(steer)
        cmd.drive.speed = float(speed)
        self.proposed.publish(cmd)
        if self.platform == 'sim' and bool(self.get_parameter('enable_drive').value):
            self.drive.publish(cmd)
        if now.nanoseconds - self.last_log_ns >= 1_000_000_000:
            self.get_logger().info('speed=%.2f m/s steer=%.3f rad status=%s mode=%s' % (speed,steer,reason,'DRIVE' if self.get_parameter('enable_drive').value else 'SHADOW'))
            self.last_log_ns = now.nanoseconds


def main():
    rclpy.init()
    node = ExplorationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # In driving mode send explicit stop before exiting.
        if node.platform == 'sim' and bool(node.get_parameter('enable_drive').value):
            cmd = AckermannDriveStamped()
            cmd.drive.speed = 0.0
            for _ in range(5):
                node.drive.publish(cmd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
