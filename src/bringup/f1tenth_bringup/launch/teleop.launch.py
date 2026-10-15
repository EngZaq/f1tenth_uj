import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_bringup')
    cfg = os.path.join(pkg, 'config')

    vesc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'vesc.launch.py')))

    joy_node = Node(
        package='joy', executable='joy_node', name='joy',
        parameters=[os.path.join(cfg, 'joy.yaml')])

    joy_teleop = Node(
        package='joy_teleop', executable='joy_teleop', name='joy_teleop',
        parameters=[os.path.join(cfg, 'joy_teleop.yaml')])

    ackermann_to_vesc = Node(
        package='vesc_ackermann', executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[os.path.join(cfg, 'ackermann_to_vesc.yaml')],
        remappings=[('ackermann_cmd', 'drive')])

    return LaunchDescription([vesc, joy_node, joy_teleop, ackermann_to_vesc])
