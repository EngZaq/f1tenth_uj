import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    host_ip = LaunchConfiguration('host_ip')
    sensor_ip = LaunchConfiguration('sensor_ip')
    lidar_port = LaunchConfiguration('lidar_port')

    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')

    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    lakibeam_launch = os.path.join(
        get_package_share_directory('lakibeam1'),
        'launch',
        'lakibeam1_scan.launch.py'
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lakibeam_launch),
        launch_arguments={
            'hostip': host_ip,
            'sensorip': sensor_ip,
            'port0': lidar_port,
            'frame_id': 'laser',
            'output_topic0': 'scan',
        }.items()
    )

    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '--x', lidar_x,
            '--y', lidar_y,
            '--z', lidar_z,

            '--roll', lidar_roll,
            '--pitch', lidar_pitch,
            '--yaw', lidar_yaw,

            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'host_ip',
            default_value='192.168.198.1'
        ),

        DeclareLaunchArgument(
            'sensor_ip',
            default_value='192.168.198.2'
        ),

        DeclareLaunchArgument(
            'lidar_port',
            default_value='"2368"'
        ),

        # Temporary until LiDAR is mounted and measured.
        DeclareLaunchArgument(
            'lidar_x',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'lidar_y',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'lidar_z',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'lidar_roll',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'lidar_pitch',
            default_value='0.0'
        ),

        DeclareLaunchArgument(
            'lidar_yaw',
            default_value='0.0'
        ),

        lidar_driver,
        lidar_tf,
        ])
