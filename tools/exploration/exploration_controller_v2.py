#!/usr/bin/env python3
"""Portable LiDAR explorer: simulator driving or physical shadow evaluation.

Hardware driving is intentionally blocked in this prototype.
"""
import math
import json
import numpy as np
import rclpy
from rclpy.node import Node
from front_scan_guard import proposal_front_guard
from physical_candidate_planner import select_steering

# Candidate planning depends on an installed sibling module. If a stale
# f1tenth_bringup install omits it, fail CLOSED without crashing the explorer
# or silently disabling the actuator mux's swept-path gate.
try:
    from physical_path_guard import swept_path_guard
    SWEPT_GUARD_IMPORT_ERROR = None
except ImportError as exc:
    swept_path_guard = None
    SWEPT_GUARD_IMPORT_ERROR = str(exc)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
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
        # Narrow finite evidence for NON-ACTUATING hardware proposal only.
        # The real actuator mux retains its own wider conservative stop gate.
        self.declare_parameter('min_front_valid_fraction', 0.90)
        self.declare_parameter('enable_candidate_planning', False)
        self.declare_parameter('candidate_lookahead_m', 1.05)
        self.scan = None
        self.scan_received = None
        self.collision = False
        self.create_subscription(LaserScan, str(self.get_parameter('scan_topic').value), self.scan_cb, 10)
        self.create_subscription(Bool, '/ego_racecar/collision', self.collision_cb, 10)
        self.proposed = self.create_publisher(AckermannDriveStamped, str(self.get_parameter('proposal_topic').value), 10)
        self.diagnostics_pub = self.create_publisher(String, '/exploration/diagnostics', 10)
        self.last_plan_report = {}
        self.last_candidate_steer = 0.0
        self.last_plan_at_ns = None
        self.cached_plan = (0.0, 0.0, 'not_evaluated')
        self.last_diag_ns = 0
        self.platform = str(self.get_parameter('platform').value)
        if self.platform not in ('sim', 'physical'):
            raise ValueError('platform must be sim or physical')
        if self.platform == 'physical' and bool(self.get_parameter('enable_drive').value):
            raise ValueError('Physical autonomous driving is BLOCKED pending steering calibration, independent e-stop and watchdog')
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10) if self.platform == 'sim' else None
        self.create_timer(0.05, self.tick)
        self.last_log_ns = 0
        self.get_logger().info('Explorer started; drive=%s' % self.get_parameter('enable_drive').value)
        if (self.platform == 'physical'
                and bool(self.get_parameter('enable_candidate_planning').value)
                and swept_path_guard is None):
            self.get_logger().error(
                'Physical candidate planner DISABLED (fail-closed): %s. '
                'Rebuild f1tenth_bringup and check python_nodes/physical_path_guard.py'
                % SWEPT_GUARD_IMPORT_ERROR)

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_received = self.get_clock().now()

    def collision_cb(self, msg):
        self.collision = bool(msg.data)

    def compute(self):
        self.last_plan_report = {}
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
        # The physical proposal stage checks reliable central observations
        # and rejects ANY measured near obstacle across +/-20 degrees.
        # This is NOT a full vehicle swept-path authorization.
        stop = float(self.get_parameter('stop_distance').value)
        if self.platform == 'physical':
            gate = proposal_front_guard(
                m.angle_min, m.angle_increment, m.ranges,
                m.range_min, m.range_max,
                stop_distance=stop,
                core_half_angle_deg=5.0, outer_half_angle_deg=20.0,
                core_valid_fraction_min=float(
                    self.get_parameter('min_front_valid_fraction').value),
            )
            if not gate['ready']:
                # An open or poorly reflective straight-ahead sector can
                # invalidate the narrow front-gate even though a different
                # swept steering path might be fully observed. Only the
                # explicit, opt-in candidate mode may consider recovery.
                # Never override a measured near obstacle or bad geometry.
                if (bool(self.get_parameter('enable_candidate_planning').value)
                        and gate['reason'] == 'central_front_insufficient_valid_returns'):
                    if swept_path_guard is None:
                        self.last_plan_report = {
                            'guard_import_error': SWEPT_GUARD_IMPORT_ERROR}
                        return 0.0, 0.0, 'physical_path_guard_missing'
                    limit = float(self.get_parameter('max_steer').value)
                    requested_speed = min(
                        float(self.get_parameter('speed').value), 0.10)
                    candidate_steer, report = select_steering(
                        m, 0.0, self.last_candidate_steer, limit,
                        requested_speed, swept_path_guard,
                        lookahead_m=float(
                            self.get_parameter('candidate_lookahead_m').value))
                    self.last_plan_report = {
                        'front_gate': gate,
                        'candidate_planner': report,
                        'recovery_speed_cap_mps': 0.10,
                    }
                    if report['ready']:
                        self.last_candidate_steer = candidate_steer
                        return candidate_steer, requested_speed, 'swept_only_observed_candidate'
                    return 0.0, 0.0, 'central_front_invalid_and_no_observed_candidate'
                return 0.0, 0.0, gate['reason']
            front_clearance = gate['nearest_m']
        else:
            # Preserve existing SIM behavior until a separate regression run.
            front = r[np.abs(a) < math.radians(20)]
            front = front[front > 0]
            if len(front) < 3:
                return 0.0, 0.0, 'front scan invalid'
            front_clearance = float(np.percentile(front, 15))
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
        raw_steer = math.atan2(2*float(self.get_parameter('wheelbase').value)*math.sin(angle), lookahead)
        limit = float(self.get_parameter('max_steer').value)
        steer = max(-limit, min(limit, raw_steer))
        self.last_plan_report = {
            'target_gap_angle_deg': round(math.degrees(angle), 2),
            'raw_target_steer_rad': round(raw_steer, 4),
            'steering_saturated': abs(raw_steer) > limit+1e-5,
            'max_steer_rad': round(limit, 4),
            'ideal_min_radius_m': round(float(self.get_parameter('wheelbase').value)/math.tan(limit), 3),
            'front_clearance_m': round(front_clearance, 3),
        }
        speed = float(self.get_parameter('speed').value)
        # Approach turns cautiously. This is a starting heuristic, not braking verification.
        speed *= max(0.45, 1.0-0.5*abs(steer)/max(limit,0.01))
        if self.platform == 'physical':
            # Start reducing approach speed BEFORE the hard 0.80 m stop.
            # This is conservative planning, not validated braking distance.
            if front_clearance < 1.50:
                speed = min(speed, 0.20)
            if front_clearance < 1.10:
                speed = min(speed, 0.15)
        elif front_clearance < 1.0:
            speed = min(speed, 0.15)
        if self.platform == 'physical' and bool(self.get_parameter('enable_candidate_planning').value):
            # Lazy import keeps simulation-only direct script use independent
            # of the hardware package's installed physical_path_guard module.
            if swept_path_guard is None:
                self.last_plan_report['guard_import_error'] = SWEPT_GUARD_IMPORT_ERROR
                return 0.0, 0.0, 'physical_path_guard_missing'
            # Advisory candidate selection only: the mux independently re-checks
            # the REAL proposed turn against its 0.8 m swept vehicle envelope.
            # Never increase the calibrated steering limit or bypass obstacles.
            candidate_steer, report = select_steering(
                m, raw_steer, self.last_candidate_steer, limit,
                speed, swept_path_guard,
                lookahead_m=float(self.get_parameter('candidate_lookahead_m').value))
            self.last_plan_report['candidate_planner'] = report
            if not report['ready']:
                return 0.0, 0.0, 'no_feasible_physical_turn'
            steer = candidate_steer
            self.last_candidate_steer = steer
            if abs(steer) > 0.75*limit:
                speed = min(speed, 0.18)
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
                if self.platform == 'physical' and bool(self.get_parameter('enable_candidate_planning').value):
                    # Expensive swept-car candidate search at 10 Hz, command
                    # proposals still published at 20 Hz. The mux stays 20 Hz.
                    if (self.last_plan_at_ns is None or
                            now.nanoseconds-self.last_plan_at_ns >= 100_000_000):
                        self.cached_plan = self.compute()
                        self.last_plan_at_ns = now.nanoseconds
                    steer, speed, reason = self.cached_plan
                else:
                    steer, speed, reason = self.compute()
        cmd = AckermannDriveStamped()
        cmd.header.stamp = now.to_msg()
        cmd.drive.steering_angle = float(steer)
        cmd.drive.speed = float(speed)
        self.proposed.publish(cmd)
        if self.platform == 'physical' and now.nanoseconds-self.last_diag_ns >= 500_000_000:
            report = {'status':reason, 'proposal_speed_mps':round(speed,4),
                      'proposal_steer_rad':round(steer,4),
                      'candidate_planning_enabled':bool(self.get_parameter('enable_candidate_planning').value),
                      **self.last_plan_report}
            self.diagnostics_pub.publish(String(data=json.dumps(report,separators=(',',':'))))
            self.last_diag_ns = now.nanoseconds
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
