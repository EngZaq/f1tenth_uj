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


    # ---------------------------------------------------------
    # LakiBeam LiDAR
    # ---------------------------------------------------------

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


    # ---------------------------------------------------------
    # BMI160 IMU
    # ---------------------------------------------------------

    # imu_node = Node(
    #     package='f1tenth_imu',
    #     executable='bmi160_node',
    #     name='bmi160',
    #     output='screen',
    #     parameters=[{
    #         'i2c_bus': 7,
    #         'i2c_address': 0x69,
    #         'frame_id': 'imu_link',
    #         'topic_name': '/imu/data_raw',
    #         'publish_rate_hz': 100.0,
    #         'warmup_seconds': 2.0,
    #         'gyro_calibration_samples': 500,
    #         'gyro_motion_std_limit_dps': 0.5,
    #     }]
    # )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('f1tenth_bringup'),
                'launch', 'tf.launch.py')))

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


        lidar_driver,
        tf_launch,
        # imu_node,
    ])
