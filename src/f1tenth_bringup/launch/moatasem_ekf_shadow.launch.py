"""Moatasem VESC+IMU EKF on REAL sensor topics in read-only shadow mode.

Run alongside observe_only.launch.py and shadow_covariance_adapter.py.
This launch does not start VESC, LiDAR, RF2O, SLAM, IMU, or any actuator.
RF2O continues to own odom->base_link; SLAM continues to own map->odom.
The EKF publishes odometry only to a dedicated comparison topic.
Do not use as the controller's pose before measured dynamic calibration.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('f1tenth_bringup')
    config = os.path.join(share, 'config',
                          'ekf_moatasem_vesc_imu_shadow.yaml')
    imu_topic = LaunchConfiguration('imu_topic')
    vesc_topic = LaunchConfiguration('vesc_topic')

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_moatasem_shadow',
        output='screen',
        parameters=[config, {'publish_tf': False}],
        remappings=[
            ('/imu/data_raw', imu_topic),
            ('/shadow_inputs/odom_vesc', vesc_topic),
            ('/odometry/filtered', '/odometry/filtered_moatasem_shadow'),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_topic', default_value='/imu/data_raw',
            description='Real IMU sensor_msgs/Imu topic; verify yaw-rate sign, frames and covariance'),
        DeclareLaunchArgument(
            'vesc_topic', default_value='/shadow_inputs/odom_vesc',
            description='Real VESC nav_msgs/Odometry after provisional covariance adapter'),
        LogInfo(msg='SHADOW ONLY: EKF does not broadcast TF or command motors. '
                    'A real IMU and VESC data source must already be running. '
                    'Covariances/Q are not yet physically calibrated.'),
        ekf,
    ])
