"""Read-only F1TENTH physical bringup (not an actuator safety system).

Publishes LiDAR, RF2O, SLAM, and optionally VESC telemetry + wheel odometry.
The VESC driver command inputs are remapped away from the standard actuator
topics. Never publish to /observe_only_unused/*; do not use teleop concurrently.

RF2O is the sole odom -> base_link TF authority. VESC odometry publishes
/odom_vesc_observe only, with TF explicitly disabled.
SLAM loop closing is disabled for this initial real-world integration audit.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VESC_BY_ID = (
    '/dev/serial/by-id/'
    'usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00'
)


def generate_launch_description():
    bringup_share = get_package_share_directory('f1tenth_bringup')
    vesc_enabled = LaunchConfiguration('enable_vesc')
    vesc_port = LaunchConfiguration('vesc_port')

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'sensors.launch.py')
        ),
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0,
        }],
    )

    slam_config = os.path.join(
        bringup_share, 'config', 'slam_toolbox.yaml'
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': False, 'do_loop_closing': False},
        ],
    )

    vesc_config = os.path.join(bringup_share, 'config', 'vesc.yaml')
    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        condition=IfCondition(vesc_enabled),
        parameters=[vesc_config, {'port': vesc_port}],
        remappings=[
            ('commands/motor/speed', '/observe_only_unused/motor_speed'),
            ('commands/motor/current', '/observe_only_unused/motor_current'),
            ('commands/motor/brake', '/observe_only_unused/motor_brake'),
            ('commands/motor/duty_cycle', '/observe_only_unused/motor_duty'),
            ('commands/motor/position', '/observe_only_unused/motor_position'),
            ('commands/servo/position', '/observe_only_unused/servo_position'),
        ],
    )

    vesc_odom_config = os.path.join(
        bringup_share, 'config', 'vesc_to_odom.yaml'
    )
    vesc_odom = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        condition=IfCondition(vesc_enabled),
        parameters=[
            vesc_odom_config,
            {
                'use_servo_cmd_to_calc_angular_velocity': False,
                'publish_tf': False,
            },
        ],
        remappings=[
            ('odom', '/odom_vesc_observe'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_vesc',
            default_value='false',
            description='Opt-in read-only VESC telemetry: true or false',
        ),
        DeclareLaunchArgument(
            'vesc_port',
            default_value=VESC_BY_ID,
            description='Stable USB serial path to VESC',
        ),
        LogInfo(
            msg='OBSERVE ONLY: No drive, teleop or actuator nodes are launched. '
                'VESC command subscriptions are isolated, not a hardware E-stop.'
        ),
        sensors,
        rf2o,
        slam,
        vesc_driver,
        vesc_odom,
    ])
