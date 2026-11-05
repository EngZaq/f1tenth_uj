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
    from std_msgs.msg import Bool
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener, TransformException
    from map_coverage import corridor_coverage

    class PhysicalFirstLap(Node):
        def __init__(self):
            super().__init__('physical_first_lap')
            self.declare_parameter('output', '~/f1tenth_hw_evidence/physical_first_lap.json')
            self.file = Path(str(self.get_parameter('output').value)).expanduser()
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
            self.left_start |= radius > 2.5
            if not (self.left_start and self.travel >= 22.0
                    and t-self.start_time >= 40.0 and radius <= .85
                    and abs(math.degrees(wrap(state[2]-self.first[2]))) <= 48.0
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
