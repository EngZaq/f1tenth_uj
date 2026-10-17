from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # base_link origin: center of the rear axle (x forward, y left, z up).
    # Values below are measured by hand, in meters.
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '--x', LaunchConfiguration('lidar_x'),
            '--y', LaunchConfiguration('lidar_y'),
            '--z', LaunchConfiguration('lidar_z'),
            '--roll', LaunchConfiguration('lidar_roll'),
            '--pitch', LaunchConfiguration('lidar_pitch'),
            '--yaw', LaunchConfiguration('lidar_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_x', default_value='0.195'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        # yaw unverified: box test pending
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        lidar_tf,
    ])