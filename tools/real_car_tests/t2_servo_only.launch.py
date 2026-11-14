"""T2 servo-only stack: vesc_driver alone. Launched by the operator only, never by Claude.

Motor phase wires PHYSICALLY DISCONNECTED (Moatasem) and, in addition, every motor command
clips to 0 in config/real_car/vesc_t2_servo_only.yaml (t2_servo_sweep.py reads them back).
No mux, no converter, no sensors: t2_servo_sweep.py is the only /commands/servo/position source.
The servo moves only when t2_servo_sweep.py runs.

servo_min / servo_max = the hard clip, just outside the last range Moatasem approved
(vehicle_params.yaml t2.clip_margin). vesc_driver reads them at start, so a wider range
means a relaunch with new values: that relaunch is the approval.

  ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py servo_min:=0.215 servo_max:=0.46
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402

SANITY_MIN, SANITY_MAX = 0.15, 0.55  # far outside anything approved; catches typos only


def driver(context):
    p = vp.load()
    centre = float(vp.get(p, 'steering.servo_offset'))
    lo = float(LaunchConfiguration('servo_min').perform(context))
    hi = float(LaunchConfiguration('servo_max').perform(context))
    if not SANITY_MIN <= lo < centre < hi <= SANITY_MAX:
        raise RuntimeError('servo clip %.3f..%.3f rejected: needs %.2f <= min < %.3f < max <= %.2f'
                           % (lo, hi, SANITY_MIN, centre, SANITY_MAX))
    node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[str(vp.CONFIG / 'vesc_t2_servo_only.yaml'), {'servo_min': lo, 'servo_max': hi}],
    )
    stop = RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[LogInfo(msg='vesc_driver exited; stopping T2 stack.'),
                 EmitEvent(event=Shutdown(reason='vesc_driver exited'))]))
    return [LogInfo(msg='T2 SERVO-ONLY: motor commands clip to 0; servo clip %.3f..%.3f. '
                        'Motor phases must be disconnected.' % (lo, hi)), node, stop]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('servo_min', default_value='0.215',
                              description='Hard servo clip, just below the last approved value.'),
        DeclareLaunchArgument('servo_max', default_value='0.46',
                              description='Hard servo clip, just above the last approved value.'),
        OpaqueFunction(function=driver),
    ])
