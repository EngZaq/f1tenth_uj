import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Teleoperated mapping: drive with the PS4 pad while SLAM builds the map.
# STARTS THE MOTOR DRIVER: car on the stand (or a clear area) before launching.
#   sensors.launch.py -> LiDAR + tf.launch.py (base_link -> laser)
#   slam.launch.py    -> rf2o (odom -> base_link) + slam_toolbox (map -> odom)
#   teleop.launch.py  -> vesc.launch.py + joy + ackermann_to_vesc + vesc_to_odom
# vesc and tf are not included here again: that would start them twice.
def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('f1tenth_bringup'), 'launch')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)))

    # The Jetson has no display, so RViz is off by default; run it on the laptop.
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        include('sensors.launch.py'),
        include('slam.launch.py'),
        include('teleop.launch.py'),
        rviz,
    ])
