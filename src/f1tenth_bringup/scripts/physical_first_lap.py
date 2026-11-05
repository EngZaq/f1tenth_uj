#!/usr/bin/env python3
"""Physical first-lap monitor: RF2O + observed map closure; no motor commands."""
import json
import math
import os
import time
from pathlib import Path

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def main():
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import Bool, String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from map_coverage import corridor_coverage

    class PhysicalFirstLap(Node):
        def __init__(self):
            super().__init__('physical_first_lap')
            self.declare_parameter('output', '~/f1tenth_hw_evidence/physical_first_lap.json')
            self.declare_parameter('min_travel_m', 22.0)
            self.declare_parameter('min_departure_m', 2.5)
            self.declare_parameter('min_duration_s', 40.0)
            self.declare_parameter('return_radius_m', .85)
            self.declare_parameter('heading_tolerance_deg', 48.0)
            self.thresholds={
                'min_travel_m':float(self.get_parameter('min_travel_m').value),
                'min_departure_m':float(self.get_parameter('min_departure_m').value),
                'min_duration_s':float(self.get_parameter('min_duration_s').value),
                'return_radius_m':float(self.get_parameter('return_radius_m').value),
                'heading_tolerance_deg':float(self.get_parameter('heading_tolerance_deg').value),
            }
            if (self.thresholds['min_travel_m']<=0 or self.thresholds['min_departure_m']<=0
                    or self.thresholds['min_duration_s']<=0
                    or self.thresholds['return_radius_m']<=0
                    or not 0<self.thresholds['heading_tolerance_deg']<=180):
                raise ValueError('Invalid physical first-lap thresholds')
            self.file = Path(str(self.get_parameter('output').value)).expanduser()
            self.progress_file=self.file.with_name('physical_first_lap_progress.json')
            self.last_progress_at=0.0
            self.first = None
            self.previous = None
            self.travel = 0.0
            self.start_time = None
            self.left_start = False
            self.done = False
            self.last_map = 0.0
            self.coverage = {'closed': False}
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, self)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
            self.done_pub = self.create_publisher(Bool, '/mission/first_lap_complete', qos)
            self.progress_pub = self.create_publisher(String, '/mission/physical_lap_progress', 10)
            self.create_subscription(Odometry, '/odom_rf2o', self.on_odom, 10)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)

        def on_map(self, msg):
            if msg.header.frame_id != 'map':
                return
            try:
                self.coverage = corridor_coverage(msg.data, msg.info.width,
                                                  msg.info.height, msg.info.resolution)
                self.last_map = time.monotonic()
            except ValueError as e:
                self.coverage = {'closed': False}
                self.get_logger().warn('Map rejected: '+str(e))

        def on_odom(self, msg):
            if self.done:
                return
            pose = msg.pose.pose
            state = (float(pose.position.x), float(pose.position.y),
                     yaw(pose.orientation))
            t = time.monotonic()
            if not all(math.isfinite(v) for v in state):
                return
            if self.first is None:
                self.first = state
                self.previous = state
                self.start_time = t
                return
            jump = math.hypot(state[0]-self.previous[0], state[1]-self.previous[1])
            if jump > .70:
                self.get_logger().warn('RF2O jump rejected')
                self.previous = state
                return
            self.previous = state
            self.travel += jump
            radius = math.hypot(state[0]-self.first[0], state[1]-self.first[1])
            self.left_start |= radius > self.thresholds['min_departure_m']
            heading_error=abs(math.degrees(wrap(state[2]-self.first[2])))
            elapsed=t-self.start_time
            progress={
                'phase':'AWAITING_MAP_CLOSURE' if self.left_start else 'AWAITING_DEPARTURE',
                'motion_source':'RF2O odometry; no wheel command',
                'travel_m':round(self.travel,2),
                'return_distance_m':round(radius,2),
                'heading_return_error_deg':round(heading_error,1),
                'elapsed_s':round(elapsed,1),
                'left_start_area':self.left_start,
                'observed_corridor_closed':bool(self.coverage.get('closed',False)),
                'observed_map':self.coverage,
                'thresholds':self.thresholds,
                'map_age_s':round(t-self.last_map,1) if self.last_map>0 else None,
                'eligible_for_completion':bool(
                    self.left_start and self.travel>=self.thresholds['min_travel_m']
                    and elapsed>=self.thresholds['min_duration_s']
                    and radius<=self.thresholds['return_radius_m']
                    and heading_error<=self.thresholds['heading_tolerance_deg']
                    and self.coverage.get('closed',False)
                    and 0<=t-self.last_map<=8.0),
            }
            if t-self.last_progress_at>=1.0:
                self.last_progress_at=t
                self.progress_pub.publish(String(data=json.dumps(progress,separators=(',',':'))))
                self.progress_file.parent.mkdir(parents=True,exist_ok=True)
                self.progress_file.write_text(json.dumps(progress,indent=2)+'\n')
            if not (self.left_start and self.travel >= self.thresholds['min_travel_m']
                    and elapsed >= self.thresholds['min_duration_s']
                    and radius <= self.thresholds['return_radius_m']
                    and heading_error <= self.thresholds['heading_tolerance_deg']
                    and self.coverage.get('closed', False)
                    and 0 <= t-self.last_map <= 8.0):
                return
            try:
                tf = self.tf.lookup_transform('map', 'base_link', Time())
            except TransformException:
                return
            record = {
                'completed': True, 'source': 'RF2O odometry plus actual observed map',
                'distance_odom_m': self.travel, 'return_distance_odom_m': radius,
                'duration_s': t-self.start_time, 'observed_corridor': self.coverage,
                'map_to_base_link_translation': {
                    'x': tf.transform.translation.x, 'y': tf.transform.translation.y},
            }
            self.file.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.file.with_suffix('.tmp')
            tmp.write_text(json.dumps(record, indent=2)+'\n')
            os.replace(tmp, self.file)
            self.done = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().warn('PHYSICAL FIRST LAP COMPLETE: map corridor closed')

    rclpy.init()
    node = PhysicalFirstLap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
