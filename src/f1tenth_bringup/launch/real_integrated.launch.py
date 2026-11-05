"""Integrated PHYSICAL F1TENTH ROS 2 system with the REAL VESC connected.

Sensors + RF2O + SLAM come from proven observe_only bringup.
The VESC driver and Ackermann converter use real actuator topics, unlike shadow.
ONLY physical_drive_mux is allowed to publish /drive.
Default mux is STOP and enable_real_actuation:=false.
This is NOT an emergency-stop system, safety certification, or ready-to-race.

First powered tests: secured wheel-off-ground stand, independent cutoff,
supervised operator; verify VESC firmware motor/battery/regen limits and timeout.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('f1tenth_bringup')
    cfg = os.path.join(share, 'config')
    enable_real = LaunchConfiguration('enable_real_actuation')
    allow_auto = LaunchConfiguration('enable_autonomous_actuation')
    enable_joy = LaunchConfiguration('enable_joystick')

    # No second VESC driver from this launch; obs-only provides sensors,
    # RF2O -> TF and SLAM. It is forbidden to launch another instance.
    observation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'observe_only.launch.py')),
        launch_arguments={'enable_vesc': 'false'}.items(),
    )

    # LIVE, physically connected motor/servo driver.
    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[os.path.join(cfg, 'vesc_real_integration.yaml')],
    )

    # The same converter previously used by RC -> Jetson -> VESC.
    # Its only Ackermann input is /drive from the command selector.
    ackermann_to_vesc = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[os.path.join(cfg, 'ackermann_to_vesc.yaml')],
        remappings=[('ackermann_cmd', '/drive')],
    )

    joystick = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        condition=IfCondition(enable_joy),
        parameters=[os.path.join(cfg, 'joy.yaml')],
    )

    manual_teleop = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        output='screen',
        condition=IfCondition(enable_joy),
        parameters=[os.path.join(cfg, 'joy_teleop_integrated.yaml')],
    )

    # The LiDAR proposal is NEVER published directly to the VESC. The mux
    # enforces a separate AUTO mode, sensor freshness, heartbeat and bounds.
    explorer = Node(
        package='f1tenth_bringup',
        executable='exploration_controller_v2.py',
        name='uj_exploration_controller',
        output='screen',
        parameters=[{
            'platform': 'physical',
            'enable_drive': False,
            'scan_topic': '/scan',
            'proposal_topic': '/exploration/drive',
            'wheelbase': 0.260,
            'speed': 0.15,
            'max_steer': 0.10,
            'stop_distance': 0.80,
            'scan_timeout': 0.22,
            'min_front_valid_fraction': 0.90,
        }],
    )

    mux = Node(
        package='f1tenth_bringup',
        executable='physical_drive_mux.py',
        name='physical_drive_mux',
        output='screen',
        parameters=[{
            'enable_real_actuation': ParameterValue(enable_real, value_type=bool),
            'enable_autonomous_actuation': ParameterValue(allow_auto, value_type=bool),
            'manual_min_front_valid': 0.75,
            'auto_min_front_valid': 0.90,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_real_actuation', default_value='false',
            description='ACTUAL motor+servo commands may be issued only if true.'
        ),
        DeclareLaunchArgument(
            'enable_autonomous_actuation', default_value='false',
            description='Experimental AUTO mode; dangerous without hardware validation.'
        ),
        DeclareLaunchArgument(
            'enable_joystick', default_value='true',
            description='Start existing joystick/joy_teleop with button-5 deadman.'
        ),
        LogInfo(
            msg='REAL VESC PATH CONNECTED. Command mux starts STOP. '
                'With actuation disabled it publishes zero. '
                'No software-only mechanism substitutes for physical cutoff.'
        ),
        observation, vesc_driver, ackermann_to_vesc,
        joystick, manual_teleop, explorer, mux
    ])
