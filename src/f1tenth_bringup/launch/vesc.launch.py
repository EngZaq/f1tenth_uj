from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('f1tenth_bringup'),
        'config',
        'vesc.yaml'
    )

    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([
        vesc_driver,
    ])
