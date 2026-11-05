"""Real-track map -> first-lap evidence -> centerline/route REHEARSAL.

MOVE THE CAR MANUALLY. No VESC driver, converter, /drive mux or explorer is
launched. Physically isolate propulsion before pushing the robot.
This tests physical RF2O /map topology and downstream planning only; it is
not an autonomous lap or a safety-rated motor interlock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('f1tenth_bringup')
    scripts=os.path.join(share,'python_nodes')
    min_travel=LaunchConfiguration('minimum_travel_m')
    min_depart=LaunchConfiguration('minimum_departure_m')
    min_duration=LaunchConfiguration('minimum_duration_s')

    observe=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share,'launch','observe_only.launch.py')),
        launch_arguments={
            'enable_vesc':'false',
            'enable_slam_loop_closing':'true',
        }.items(),
    )
    first_lap=ExecuteProcess(
        cmd=[
            'python3','-u',os.path.join(scripts,'physical_first_lap.py'),
            '--ros-args',
            '-p',['min_travel_m:=',min_travel],
            '-p',['min_departure_m:=',min_depart],
            '-p',['min_duration_s:=',min_duration],
        ],
        output='screen',
    )
    finalizer=ExecuteProcess(
        cmd=['python3','-u',os.path.join(scripts,'physical_map_finalizer.py')],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'minimum_travel_m',default_value='12.0',
            description='Rehearsal-only minimum RF2O distance, alongside observed closed-map and return gates.'),
        DeclareLaunchArgument(
            'minimum_departure_m',default_value='1.5',
            description='Minimum RF2O radius from start before detecting a returned lap.'),
        DeclareLaunchArgument(
            'minimum_duration_s',default_value='20.0',
            description='Minimum seconds before a physical push lap can complete.'),
        LogInfo(msg='PHYSICAL MAP REHEARSAL ONLY: VESC and all actuator nodes are absent. '
                    'Propulsion must be independently disconnected before moving car manually. '
                    'SLAM loop closing enabled for mapping.'),
        observe,first_lap,finalizer,
    ])
