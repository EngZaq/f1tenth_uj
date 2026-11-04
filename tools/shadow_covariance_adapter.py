#!/usr/bin/env python3
"""F1TENTH shadow EKF covariance adapter — observation only.

Republishes RF2O and VESC odometry on separate shadow-only topics with
PROVISIONAL nonzero covariance for the fields the shadow EKF will fuse.
Does not publish TF or hardware commands and does not change original topics.
The defaults ARE NOT measured sensor noise or approved driving parameters.
"""

import argparse
import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def positive(value):
    x = float(value)
    if not math.isfinite(x) or x <= 0:
        raise argparse.ArgumentTypeError('must be a finite positive number')
    return x


class CovarianceAdapter(Node):
    def __init__(self, xy_sigma, yaw_sigma_deg, vx_sigma):
        super().__init__('shadow_covariance_adapter')
        self.xy_var = xy_sigma ** 2
        self.yaw_var = math.radians(yaw_sigma_deg) ** 2
        self.vx_var = vx_sigma ** 2
        self.num_rf2o = 0
        self.num_vesc = 0
        self.rejected = 0

        self.rf_pub = self.create_publisher(
            Odometry, '/shadow_inputs/odom_rf2o', 10)
        self.ve_pub = self.create_publisher(
            Odometry, '/shadow_inputs/odom_vesc', 10)

        self.rf_sub = self.create_subscription(
            Odometry, '/odom_rf2o',
            self.on_rf2o, qos_profile_sensor_data)
        self.ve_sub = self.create_subscription(
            Odometry, '/odom_vesc_observe',
            self.on_vesc, qos_profile_sensor_data)

        self.get_logger().warning(
            'SHADOW ONLY: covariance values are provisional, NOT calibrated. '
            'No TF or actuator commands are published.'
        )
        self.get_logger().info(
            'Provisional standard deviations: RF2O XY=%.3f m, '
            'yaw=%.3f deg; VESC forward speed=%.3f m/s'
            % (xy_sigma, yaw_sigma_deg, vx_sigma)
        )
        self.create_timer(10.0, self.log_counts)

    def valid(self, msg):
        if (msg.header.frame_id.lstrip('/') != 'odom'
                or msg.child_frame_id.lstrip('/') != 'base_link'
                or (msg.header.stamp.sec == 0
                    and msg.header.stamp.nanosec == 0)):
            self.rejected += 1
            return False
        return True

    def on_rf2o(self, msg):
        if not self.valid(msg):
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        if not all(math.isfinite(x) for x in
                   (p.x, p.y, q.x, q.y, q.z, q.w)):
            self.rejected += 1
            return
        out = copy.deepcopy(msg)
        # 6x6 row-major covariance for (x,y,z,roll,pitch,yaw)
        out.pose.covariance[0] = self.xy_var
        out.pose.covariance[7] = self.xy_var
        out.pose.covariance[35] = self.yaw_var
        # Other fields are not selected for fusion; leave untouched.
        self.rf_pub.publish(out)
        self.num_rf2o += 1

    def on_vesc(self, msg):
        if not self.valid(msg):
            return
        if not math.isfinite(msg.twist.twist.linear.x):
            self.rejected += 1
            return
        out = copy.deepcopy(msg)
        # 6x6 row-major covariance for (vx,vy,vz,roll-rate,pitch-rate,yaw-rate).
        out.twist.covariance[0] = self.vx_var
        # Only vx will be selected for fusion; other fields are unchanged.
        self.ve_pub.publish(out)
        self.num_vesc += 1

    def log_counts(self):
        self.get_logger().info(
            'Shadow forwarded: RF2O=%d, VESC=%d, rejected=%d'
            % (self.num_rf2o, self.num_vesc, self.rejected))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rf2o-xy-sigma-m', type=positive, default=0.25)
    parser.add_argument('--rf2o-yaw-sigma-deg', type=positive, default=15.0)
    parser.add_argument('--vesc-vx-sigma-mps', type=positive, default=0.4)
    args = parser.parse_args()

    rclpy.init()
    node = CovarianceAdapter(
        args.rf2o_xy_sigma_m,
        args.rf2o_yaw_sigma_deg,
        args.vesc_vx_sigma_mps,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
