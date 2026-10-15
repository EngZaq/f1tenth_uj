from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='f1tenth_imu',
            executable='bmi160_node',
            name='bmi160',

            output='screen',

            parameters=[{
                'i2c_bus': 7,
                'i2c_address': 0x69,

                'frame_id': 'imu_link',
                'topic_name': '/imu/data_raw',

                'publish_rate_hz': 100.0,

                'warmup_seconds': 2.0,

                # 500 samples / 100 Hz
                # ≈ 5 seconds calibration
                'gyro_calibration_samples': 500,

                'gyro_motion_std_limit_dps': 0.5,
            }]
        )

    ])
