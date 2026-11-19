"""T2 servo-only stack: vesc_driver alone. Launched by the operator only, never by Claude.

Motor phase wires PHYSICALLY DISCONNECTED (Moatasem) and, in addition, every motor command
clips to 0 in config/real_car/vesc_t2_servo_only.yaml (t2_servo_sweep.py reads them back).
No mux, no converter, no sensors: t2_servo_sweep.py is the only /commands/servo/position source.
The servo moves only when t2_servo_sweep.py runs.

servo_min / servo_max = the hard clip, just outside the last range Moatasem approved
(vehicle_params.yaml t2.clip_margin). Normal T2: servo_min:=0.10 servo_max:=0.60 (the approved
range). Right-side extension (--extend-right): servo_max up to 1.0; min stays 0.10. vesc_driver reads them at start, so a wider range
means a relaunch with new values: that relaunch is the approval.

  ros2 launch ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py servo_min:=0.215 servo_max:=0.46
  python3 ~/f1tenth_uj/tools/real_car_tests/t2_servo_only.launch.py --self-test   (clip check only)
"""
import math
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

# Hard ceiling for the clip. VESC firmware maps servo 0..1 to 1000..2000 us (conf_general.h
# SERVO_OUT_PULSE_MIN_US / MAX_US). Min 0.10 = left last safe point (Moatasem, 19 Nov: 17 deg,
# mechanical stop ~0.08). Max 1.0 = the VESC maximum, for the right-side extension
# (t2_servo_sweep.py --extend-right); the sweep still visits every point briefly first.
SANITY_MIN, SANITY_MAX = 0.10, 1.00


def check_clip(lo, hi, centre):
    """Raise unless SANITY_MIN <= lo < centre < hi <= SANITY_MAX (all finite)."""
    if not all(math.isfinite(v) for v in (lo, hi, centre)) or \
            not SANITY_MIN <= lo < centre < hi <= SANITY_MAX:
        raise RuntimeError('servo clip %.3f..%.3f rejected: needs %.2f <= min < %.3f < max <= %.2f'
                           % (lo, hi, SANITY_MIN, centre, SANITY_MAX))


def self_test():
    centre = 0.320
    for lo, hi in ((0.215, 0.46), (0.15, 0.55), (0.10, 0.60), (0.10, 0.80), (0.10, 1.00)):
        check_clip(lo, hi, centre)
    for lo, hi in ((0.09, 0.46), (0.10, 1.01), (0.33, 0.46), (0.215, 0.31), (float('nan'), 0.46),
                   (0.10, float('inf'))):
        try:
            check_clip(lo, hi, centre)
        except RuntimeError:
            continue
        raise AssertionError('clip %r..%r accepted' % (lo, hi))
    # The approved range in vehicle_params.yaml must itself be a valid clip (one launch runs it all)
    p = vp.load()
    check_clip(float(vp.get(p, 't2.approved_servo_min')), float(vp.get(p, 't2.approved_servo_max')),
               float(vp.get(p, 'steering.servo_offset')))
    print('PASS: clip ceiling 0.10..1.00 accepted at its edges, 0.09 / 1.01 / centre outside the')
    print('      clip / NaN rejected; vehicle_params t2 approved range is a valid clip.')
    print('      No ROS node started; nothing published.')


def driver(context):
    p = vp.load()
    centre = float(vp.get(p, 'steering.servo_offset'))
    lo = float(LaunchConfiguration('servo_min').perform(context))
    hi = float(LaunchConfiguration('servo_max').perform(context))
    check_clip(lo, hi, centre)
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


if __name__ == '__main__' and '--self-test' in sys.argv:
    self_test()
