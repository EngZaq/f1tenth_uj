"""Integrated PHYSICAL F1TENTH ROS 2 system with the REAL VESC connected.

Sensors + RF2O + SLAM come from proven observe_only bringup.
The VESC driver and Ackermann converter use real actuator topics, unlike shadow.
Physical first-lap monitor and mapping planner ported from simulator are included.
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
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess,
    RegisterEventHandler, EmitEvent
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Fail closed if the actual serial VESC driver crashes/disconnects.
    # An absent motor-speed subscriber was observed on the Jetson with
    # one Ackermann publisher and no /vesc_driver node. Continuing a
    # ROS mission graph would be misleading and unsafe.
    vesc_exit_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=vesc_driver,
            on_exit=[
                LogInfo(msg='FATAL: Real VESC driver exited. '
                            'Stopping integrated mission; inspect the launch log '
                            'and serial port before restarting.'),
                EmitEvent(event=Shutdown(reason='Real VESC driver exited')),
            ],
        ),
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
    # Start GitHub-created mode-100644 Python scripts with interpreter; this
    # works even under colcon --symlink-install (no chmod dependency).
    scripts_dir = os.path.join(share, 'python_nodes')
    explorer = ExecuteProcess(
        cmd=[
            'python3', '-u',
            os.path.join(scripts_dir, 'exploration_controller_v2.py'),
            '--ros-args',
            '-p', 'platform:=physical',
            '-p', 'enable_drive:=false',
            '-p', 'scan_topic:=/scan',
            '-p', 'proposal_topic:=/exploration/drive',
            '-p', 'wheelbase:=0.260',
            '-p', 'speed:=0.15',
            '-p', 'max_steer:=0.10',
            '-p', 'stop_distance:=0.80',
            '-p', 'scan_timeout:=0.22',
            '-p', 'min_front_valid_fraction:=0.90',
        ],
        output='screen',
    )

    mux = ExecuteProcess(
        cmd=[
            'python3', '-u',
            os.path.join(scripts_dir, 'physical_drive_mux.py'),
            '--ros-args',
            '-p', ['enable_real_actuation:=', enable_real],
            '-p', ['enable_autonomous_actuation:=', allow_auto],
            '-p', 'manual_min_front_valid:=0.75',
            '-p', 'auto_min_front_valid:=0.90',
        ],
        output='screen',
    )

    # Ports the simulator's first-lap map-closure gate to real RF2O /map.
    # It publishes completion only, never direct actuator commands.
    lap_watcher = ExecuteProcess(
        cmd=['python3', '-u',
             os.path.join(scripts_dir, 'physical_first_lap.py')],
        output='screen',
    )

    # Upon first-lap completion save the actual SLAM map and apply the
    # existing physical-geometry route preparation. Not a racing controller.
    map_finalizer = ExecuteProcess(
        cmd=['python3', '-u',
             os.path.join(scripts_dir, 'physical_map_finalizer.py')],
        output='screen',
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
            'enable_joystick', default_value='false',
            description='Optional original RC joystick path; default false (no remote connected).'
        ),
        LogInfo(
            msg='REAL VESC PATH CONNECTED. Command mux starts STOP. '
                'With actuation disabled it publishes zero. '
                'No software-only mechanism substitutes for physical cutoff.'
        ),
        vesc_exit_shutdown,
        observation, vesc_driver, ackermann_to_vesc,
        joystick, manual_teleop, explorer, mux, lap_watcher, map_finalizer
    ])
