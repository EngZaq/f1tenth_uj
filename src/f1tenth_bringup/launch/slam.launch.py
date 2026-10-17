import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config', 'slam_toolbox.yaml')

    # Placeholder odometry: the car is treated as not moving.
    # Remove this when a real odom -> base_link source is added.
    odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link',
        ],
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[cfg, {'use_sim_time': False}],
    )

    return LaunchDescription([odom_tf, slam])