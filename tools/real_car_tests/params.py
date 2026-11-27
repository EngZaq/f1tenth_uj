"""Loads config/real_car/vehicle_params.yaml: the only source of real-car numbers.

No ROS import; safe to use from the launch file, the runner and offline analysis.
"""
import os
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
CONFIG = REPO / 'config' / 'real_car'
DEFAULT_PATH = CONFIG / 'vehicle_params.yaml'


def load(path=None):
    with open(path or DEFAULT_PATH) as fp:
        return yaml.safe_load(fp)


PS4_PATH = CONFIG / 'ps4_teleop.yaml'


def load_ps4(path=None):
    """PS4 teleop layout (config/real_car/ps4_teleop.yaml, 28 Nov) -> dict."""
    with open(path or PS4_PATH) as fp:
        return yaml.safe_load(fp)['ps4_teleop']


def get(params, dotted):
    """get(p, 'steering.servo_offset'); raises KeyError on a missing key."""
    value = params
    for key in dotted.split('.'):
        value = value[key]
    return value


def decel_for_checks(params):
    """Measured decel if T1/T4 wrote one, else the assumed 1.0 m/s^2."""
    measured = get(params, 'dynamics.decel_mps2')
    return float(measured) if measured else float(get(params, 'dynamics.decel_assumed_mps2'))


def bumper_ahead_of_lidar(params):
    """LiDAR ranges start at the laser; the car ends at the front bumper."""
    return float(get(params, 'vehicle.front_bumper_x_m')) - float(get(params, 'lidar.x_m'))


def config_mismatches(params):
    """Compare the official-node yamls in config/real_car/ against vehicle_params.yaml."""
    out = []

    def node_params(name, key):
        with open(CONFIG / name) as fp:
            return yaml.safe_load(fp)[key]['ros__parameters']

    expect = {
        'speed_to_erpm_gain': float(get(params, 'drivetrain.speed_to_erpm_gain')),
        'speed_to_erpm_offset': float(get(params, 'drivetrain.speed_to_erpm_offset')),
        'steering_angle_to_servo_gain': float(get(params, 'steering.servo_gain')),
        'steering_angle_to_servo_offset': float(get(params, 'steering.servo_offset')),
    }
    a2v = node_params('ackermann_to_vesc_real.yaml', '/ackermann_to_vesc_node')
    v2o = node_params('vesc_to_odom_real.yaml', '/vesc_to_odom_node')
    for key, value in expect.items():
        if abs(float(a2v[key]) - value) > 1e-9:
            out.append('ackermann_to_vesc_real.yaml %s=%s, expected %s' % (key, a2v[key], value))
        # vesc_to_odom uses speed = -erpm/gain, so its gain carries the opposite sign.
        odom_value = -value if key == 'speed_to_erpm_gain' else value
        if abs(float(v2o[key]) - odom_value) > 1e-9:
            out.append('vesc_to_odom_real.yaml %s=%s, expected %s' % (key, v2o[key], odom_value))
    if abs(float(v2o['wheelbase']) - float(get(params, 'vehicle.wheelbase_m'))) > 1e-9:
        out.append('vesc_to_odom_real.yaml wheelbase=%s, expected %s' % (
            v2o['wheelbase'], get(params, 'vehicle.wheelbase_m')))
    return out


def evidence_root():
    return Path(os.path.expanduser('~/f1tenth_hw_evidence/real_car_tests'))
