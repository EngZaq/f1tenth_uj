#!/usr/bin/env python3
"""Read-only live LiDAR self-return vs nearby obstacle diagnostic.

Only subscribes to /scan and /exploration/drive. Runs the physical
footprint guard against snapshots without publishing ANY ROS messages.
No E-stop or actuator changes; verify suspect LiDAR beam by inspection.
"""
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/
                      'src/f1tenth_bringup/scripts'))
from physical_path_guard import swept_path_guard


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from ackermann_msgs.msg import AckermannDriveStamped

    class Probe(Node):
        def __init__(self):
            super().__init__('read_only_lidar_self_return_probe')
            self.scans=[]
            self.proposal=None
            self.subs=[
                self.create_subscription(LaserScan,'/scan',self.scan_cb,
                                         qos_profile_sensor_data),
                self.create_subscription(AckermannDriveStamped,
                                         '/exploration/drive',self.proposal_cb,10),
            ]
        def scan_cb(self,msg):
            # Store six snapshots ~0.3s apart, avoiding any motor command.
            now=time.monotonic()
            if not self.scans or now-self.scans[-1][0]>.30:
                self.scans.append((now,msg))
            if len(self.scans)>8:
                self.scans.pop(0)
        def proposal_cb(self,msg):
            self.proposal=(float(msg.drive.speed),
                           float(msg.drive.steering_angle))

    rclpy.init()
    node=Probe()
    try:
        end=time.monotonic()+2.6
        while rclpy.ok() and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.1)
        if not node.scans:
            raise RuntimeError('NO /scan received')
        if node.proposal is None:
            raise RuntimeError('NO /exploration/drive received')
        observations=[]
        for _,scan in node.scans:
            res=swept_path_guard(scan,*node.proposal)
            observations.append({
                'reason':res['reason'],'ready':res['ready'],
                'nearest_obstacle_m':res['nearest_obstacle_m'],
                'nearest_obstacle_detail':res.get('nearest_obstacle_detail'),
                'blocking_return_count':res.get('blocking_return_count'),
                'closest_blocking_returns':res.get('closest_blocking_returns'),
                'swept_known_fraction':res['swept_known_fraction'],
                'max_unknown_run_deg':res['max_unknown_run_deg'],
            })
        candidate=[
            x['nearest_obstacle_detail']['inside_uninflated_body_rectangle']
            for x in observations if x.get('nearest_obstacle_detail')]
        report={
            'mode':'READ_ONLY_NO_COMMAND_PUBLISHERS',
            'proposal_speed_mps':round(node.proposal[0],3),
            'proposal_steer_rad':round(node.proposal[1],3),
            'scans_analyzed':len(observations),
            'nearest_hits_inside_nominal_body':sum(candidate),
            'nearest_hits_total':len(candidate),
            'observations':observations,
            'interpretation':(
                'near inside body = self-return candidate, not verified; '
                'near outside body = potential real obstacle, inspect angle. '
                'Never mask unverified close returns before drive tests.')
        }
        print(json.dumps(report,indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
