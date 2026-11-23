"""Real-car TEST stack: the single launch for T0, T1, T3, T4 and the dry run.

REAL VESC CONNECTED. The servo moves to the converter centre as soon as this runs,
even with enable_real_actuation:=false. Launched by the operator only, never by Claude.
Not an emergency stop: the battery disconnect / kill switch stays in the operator's hand.

Contents (each actuator topic has exactly one publisher and one subscriber):
  observe_only.launch.py (LiDAR, RF2O odom->base_link, SLAM map->odom, base_link->laser; VESC off)
  f1tenth_imu imu.launch.py (BMI160 -> /imu/data_raw) + static base_link->imu_link (24 Nov: pitch pi + measured
  tilt, from vehicle_params.yaml; position PROVISIONAL)
  vesc_driver        <- vesc_yaml (speed_max per speed step; one file per step)
  vesc_to_odom       -> /odom_vesc_observe, publish_tf false
  ackermann_to_vesc  <- /drive, config/real_car/ackermann_to_vesc_real.yaml (offset 0.335 provisional, gain -0.50)
    OR (converter:=t1b, T1b only) t1b_stop_converter.py: same conversion, test stop method
  physical_drive_mux -> /drive, starts STOP; caps from max_speed_mps / max_steer_rad
No explorer, lap watcher, joystick or trial adapter. Exactly one converter: an unknown
`converter` value starts none, and the mux then refuses (ownership check fails closed).

  ros2 launch ~/f1tenth_uj/tools/real_car_tests/real_car_test.launch.py enable_real_actuation:=false
"""
import math
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import params as vp  # noqa: E402


def converter_cmd(script, method, brake_a, hold_s, params_file=''):
    """t1b_stop_converter command line; params_file (test-only yaml) must exist when given."""
    cmd = ['python3', '-u', script, '--ros-args', '-p', 'method:=%s' % method,
           '-p', 'brake_current_a:=%s' % brake_a, '-p', 'brake_hold_s:=%s' % hold_s]
    if params_file:
        path = os.path.expanduser(params_file)
        if not os.path.isfile(path):
            raise RuntimeError('converter_params_file not found: %s' % path)
        cmd += ['--params-file', path]
    return cmd


def mux_visibility_params(flag, p, manual='0.75'):
    """Mux MANUAL visibility parameters. t3_low_visibility:=true (T3 ONLY, Moatasem 22 Nov) turns on Zaid's
    manual-only reduced-visibility option with t3.low_visibility_manual_min_front_valid. manual_min_front_valid:=<f>
    (23 Nov, corridor teleop; run_test.py allows a lowered value for teleop at <= 0.3 m/s only) sets the MANUAL
    threshold directly, 0.10..0.75; below 0.75 it turns the reduced-visibility option on. Default 0.75 = standard.
    Not both. AUTO (0.90) and the 0.80 m obstacle gate are not touched. -> ['-p', ..., '-p', ...]"""
    flag = str(flag).strip().lower()
    if flag not in ('true', 'false'):
        raise RuntimeError('t3_low_visibility must be true or false, got %r' % flag)
    try:
        m = float(str(manual).strip())
    except ValueError:
        raise RuntimeError('manual_min_front_valid must be a number, got %r' % manual)
    if not (math.isfinite(m) and 0.10 - 1e-9 <= m <= 0.75 + 1e-9):
        raise RuntimeError('manual_min_front_valid %r outside 0.10..0.75' % manual)
    if flag == 'true' and m < 0.75 - 1e-9:
        raise RuntimeError('use t3_low_visibility:=true OR manual_min_front_valid:=<f>, not both')
    if flag == 'false':
        if m >= 0.75 - 1e-9:
            return ['-p', 'manual_allow_reduced_visibility:=false', '-p', 'manual_min_front_valid:=0.75']
        return ['-p', 'manual_allow_reduced_visibility:=true', '-p', 'manual_min_front_valid:=%.2f' % m]
    frac = float(vp.get(p, 't3.low_visibility_manual_min_front_valid'))
    if not 0.35 <= frac < 0.75:
        raise RuntimeError('t3.low_visibility_manual_min_front_valid %.2f outside 0.35..0.75' % frac)
    return ['-p', 'manual_allow_reduced_visibility:=true', '-p', 'manual_min_front_valid:=%.2f' % frac]


def self_test():
    a = converter_cmd('c.py', 'brake', '5.0', '2.0')
    assert a == ['python3', '-u', 'c.py', '--ros-args', '-p', 'method:=brake', '-p', 'brake_current_a:=5.0',
                 '-p', 'brake_hold_s:=2.0'], a
    f = str(vp.CONFIG / 't1b_converter_t4_offset0358.yaml')
    b = converter_cmd('c.py', 'brake', '5.0', '2.0', f)
    assert b[:len(a)] == a and b[len(a):] == ['--params-file', f], b
    try:
        converter_cmd('c.py', 'brake', '5.0', '2.0', '/nonexistent.yaml')
        raise AssertionError('missing params file accepted')
    except RuntimeError:
        pass
    p = vp.load()
    assert mux_visibility_params('false', p) == ['-p', 'manual_allow_reduced_visibility:=false',
                                                 '-p', 'manual_min_front_valid:=0.75']
    assert mux_visibility_params('True', p) == ['-p', 'manual_allow_reduced_visibility:=true',
                                                '-p', 'manual_min_front_valid:=0.35']
    assert mux_visibility_params('false', p, '0.10') == ['-p', 'manual_allow_reduced_visibility:=true',
                                                         '-p', 'manual_min_front_valid:=0.10']
    assert mux_visibility_params('false', p, '0.75')[1] == 'manual_allow_reduced_visibility:=false'
    for bad_m, flag_ in (('0.05', 'false'), ('0.9', 'false'), ('nan', 'false'), ('x', 'false'), ('0.10', 'true')):
        try:
            mux_visibility_params(flag_, p, bad_m)
            raise AssertionError('manual_min_front_valid %r (t3 %s) accepted' % (bad_m, flag_))
        except RuntimeError:
            pass
    for bad in ('yes', '', '1'):
        try:
            mux_visibility_params(bad, p)
            raise AssertionError('t3_low_visibility %r accepted' % bad)
        except RuntimeError:
            pass
    print('PASS: t1b converter command line, with and without the test params file; missing file refused.')
    print('      Mux visibility: standard 0.75, t3_low_visibility:=true -> reduced 0.35 (vehicle_params), bad flag refused;')
    print('      manual_min_front_valid:=0.10 -> reduced 0.10, outside 0.10..0.75 / with t3_low_visibility refused.')
    print('      No ROS node started; nothing published.')


def generate_launch_description():
    p = vp.load()
    bringup = get_package_share_directory('f1tenth_bringup')
    imu_share = get_package_share_directory('f1tenth_imu')
    cfg = str(vp.CONFIG)
    enable_real = LaunchConfiguration('enable_real_actuation')
    max_speed = LaunchConfiguration('max_speed_mps')
    max_steer = LaunchConfiguration('max_steer_rad')
    vesc_yaml = LaunchConfiguration('vesc_yaml')
    converter = LaunchConfiguration('converter')

    def converter_is(name):
        return IfCondition(PythonExpression(["'", converter, "' == '%s'" % name]))

    observation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'observe_only.launch.py')),
        launch_arguments={'enable_vesc': 'false', 'enable_slam_loop_closing': 'false'}.items(),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(imu_share, 'launch', 'imu.launch.py')))

    # PROVISIONAL pose (vehicle_params.yaml imu.status); sole owner of this TF edge.
    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link',
        arguments=[
            '--x', str(vp.get(p, 'imu.x_m')), '--y', str(vp.get(p, 'imu.y_m')),
            '--z', str(vp.get(p, 'imu.z_m')), '--roll', str(vp.get(p, 'imu.roll_rad')),
            '--pitch', str(vp.get(p, 'imu.pitch_rad')), '--yaw', str(vp.get(p, 'imu.yaw_rad')),
            '--frame-id', vp.get(p, 'frames.base_link'),
            '--child-frame-id', vp.get(p, 'frames.imu'),
        ],
    )

    vesc_driver = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver',
        output='screen',
        parameters=[vesc_yaml],
    )

    # Fail closed if the serial VESC driver exits (same rule as real_integrated.launch.py).
    vesc_exit_shutdown = RegisterEventHandler(OnProcessExit(
        target_action=vesc_driver,
        on_exit=[LogInfo(msg='FATAL: vesc_driver exited; stopping the test stack.'),
                 EmitEvent(event=Shutdown(reason='vesc_driver exited'))],
    ))

    vesc_odom = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[os.path.join(cfg, 'vesc_to_odom_real.yaml')],
        remappings=[('odom', '/odom_vesc_observe')],
    )

    ackermann_to_vesc = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        output='screen',
        parameters=[os.path.join(cfg, 'ackermann_to_vesc_real.yaml')],
        remappings=[('ackermann_cmd', '/drive')],
        condition=converter_is('ackermann_to_vesc'),
    )

    # T1b only: replaces ackermann_to_vesc (never both). Use with vesc_t1b_erpm*.yaml.
    def t1b_converter_actions(context):
        if LaunchConfiguration('converter').perform(context) != 't1b':
            return []
        return [ExecuteProcess(cmd=converter_cmd(
            os.path.join(os.path.dirname(os.path.realpath(__file__)), 't1b_stop_converter.py'),
            *[LaunchConfiguration(k).perform(context) for k in (
                'stop_method', 'brake_current_a', 'brake_hold_s', 'converter_params_file')]),
            output='screen')]
    t1b_converter = OpaqueFunction(function=t1b_converter_actions)

    def mux_actions(context):
        vis = mux_visibility_params(LaunchConfiguration('t3_low_visibility').perform(context), p,
                                    LaunchConfiguration('manual_min_front_valid').perform(context))
        acts = [ExecuteProcess(
            cmd=['python3', '-u', os.path.join(bringup, 'python_nodes', 'physical_drive_mux.py'),
                 '--ros-args',
                 '-p', 'enable_real_actuation:=%s' % enable_real.perform(context),
                 '-p', 'enable_autonomous_actuation:=false'] + vis + [
                 '-p', 'max_speed_mps:=%s' % max_speed.perform(context),
                 '-p', 'max_steer_rad:=%s' % max_steer.perform(context)],
            output='screen')]
        if 'manual_allow_reduced_visibility:=true' in vis:
            acts.insert(0, LogInfo(msg='LOW-VISIBILITY MODE: mux MANUAL front threshold %s (AUTO and the 0.80 m '
                                       'obstacle gate unchanged). Runner: T3 (t3_low_visibility) or teleop <= 0.3 m/s '
                                       'only; spotter.' % vis[-1]))
        return acts
    mux = OpaqueFunction(function=mux_actions)

    return LaunchDescription([
        DeclareLaunchArgument('enable_real_actuation', default_value='false',
                              description='true only for an approved powered test step.'),
        DeclareLaunchArgument('max_speed_mps', default_value=str(vp.get(p, 'caps.mux_default_max_speed_mps')),
                              description='Mux speed cap for this test (ceiling 1.5).'),
        DeclareLaunchArgument('max_steer_rad', default_value=str(vp.get(p, 'caps.mux_default_max_steer_rad')),
                              description='Mux steering cap for this test (ceiling 0.40).'),
        DeclareLaunchArgument('vesc_yaml', default_value=os.path.join(cfg, 'vesc_test_erpm3500.yaml'),
                              description='vesc_driver yaml; speed_max per speed step.'),
        DeclareLaunchArgument('converter', default_value='ackermann_to_vesc',
                              description='ackermann_to_vesc (standard) or t1b (T1b stop-method test only).'),
        DeclareLaunchArgument('stop_method', default_value='speed0',
                              description='converter:=t1b only: speed0, brake or current0.'),
        DeclareLaunchArgument('brake_current_a', default_value='5.0',
                              description='converter:=t1b, stop_method:=brake: amps, (0, 10].'),
        DeclareLaunchArgument('brake_hold_s', default_value='2.0',
                              description='converter:=t1b, stop_method:=brake: seconds, (0, 5].'),
        DeclareLaunchArgument('manual_min_front_valid', default_value='0.75',
                              description='Mux MANUAL front valid fraction 0.10..0.75; below 0.75 = reduced-'
                                          'visibility mode (runner: teleop <= 0.3 m/s only).'),
        DeclareLaunchArgument('t3_low_visibility', default_value='false',
                              description='T3 ONLY (Moatasem 22 Nov): Zaid\'s manual-only reduced-visibility mux option '
                                          'with t3.low_visibility_manual_min_front_valid; AUTO and the obstacle gate '
                                          'unchanged. The runner allows it only for T3 and teleop <= 0.3 m/s.'),
        DeclareLaunchArgument('converter_params_file', default_value='',
                              description='converter:=t1b only: TEST-ONLY params yaml for t1b_stop_converter '
                                          '(e.g. config/real_car/t1b_converter_t4_offset0358.yaml). Empty = none.'),
        LogInfo(msg='REAL-CAR TEST STACK: real VESC connected, mux starts STOP. '
                    'Servo centres now. Not an emergency stop.'),
        vesc_exit_shutdown,
        observation, imu, imu_tf, vesc_driver, vesc_odom, ackermann_to_vesc, t1b_converter, mux,
    ])


if __name__ == '__main__' and '--self-test' in sys.argv:
    self_test()
