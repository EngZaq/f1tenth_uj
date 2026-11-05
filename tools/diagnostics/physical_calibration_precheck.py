#!/usr/bin/env python3
"""Read-only source calibration review for the F1TENTH physical car.

Reads YAML files only, never publishes ROS commands or connects to VESC.
Expected electronic conversions are NOT evidence of physical calibration.
"""
import argparse
import json
import math
from pathlib import Path


def report(root):
    try:
        import yaml
    except ImportError as exc:
        raise SystemExit('PyYAML required: use source /opt/ros/humble/setup.bash') from exc

    geom=yaml.safe_load((root/'src/f1tenth_bringup/config/physical_vehicle.yaml').read_text())
    ack=yaml.safe_load((root/'src/f1tenth_bringup/config/ackermann_to_vesc.yaml').read_text())
    vesc=yaml.safe_load((root/'src/f1tenth_bringup/config/vesc_real_integration.yaml').read_text())
    conv=ack['/ackermann_to_vesc_node']['ros__parameters']
    cfg=vesc['vesc_driver']['ros__parameters']
    wheelbase=float(geom['vehicle']['wheelbase_m'])
    gain=float(conv['speed_to_erpm_gain'])
    motor_min=float(cfg['speed_min'])
    motor_max=float(cfg['speed_max'])
    sgain=float(conv['steering_angle_to_servo_gain'])
    soffset=float(conv['steering_angle_to_servo_offset'])
    servomin=float(cfg['servo_min'])
    servomax=float(cfg['servo_max'])
    speeds=(0.,.075,.15)
    steering=(-.10,0.,.10)

    def check_speed(speed):
        erpm=speed*gain+float(conv['speed_to_erpm_offset'])
        return {'requested_mps':speed,'expected_erpm':round(erpm,1),
                'software_clamp_violated':not motor_min<=erpm<=motor_max}

    def check_steer(angle):
        pulse=angle*sgain+soffset
        radius=wheelbase/abs(math.tan(angle)) if abs(angle)>1e-8 else None
        return {'requested_road_wheel_angle_rad':angle,
                'servo_command':round(pulse,5),
                'software_servo_clamp_violated':not servomin<=pulse<=servomax,
                'kinematic_turn_radius_m':round(radius,3) if radius else None}

    checks={
        'wheelbase_m':wheelbase,
        'body_half_width_m':geom['vehicle']['body_half_width_m'],
        'lidar_x_m':geom['lidar']['x_m'],
        'front_body_x_m':geom['vehicle']['body_front_x_m'],
        'expected_erpm_per_mps':gain,
        'vesc_speed_clamp_erpm':[motor_min,motor_max],
        'speed_examples':[check_speed(x) for x in speeds],
        'steering_examples':[check_steer(x) for x in steering],
        'motor_direction_verified_in_yaml':geom['calibration']['motor_direction_verified'],
        'steering_conversion_verified_in_yaml':geom['calibration']['steering_conversion_verified'],
        'wheel_speed_scale_verified_in_yaml':geom['calibration']['wheel_speed_scale_verified'],
        'firmware_phase_current_limit':'NOT READ: verify in VESC Tool',
        'firmware_battery_current_limit':'NOT READ: verify in VESC Tool',
        'firmware_regen_current_limit':'NOT READ: verify in VESC Tool',
        'firmware_command_timeout':'NOT READ: verify in VESC Tool',
        'independent_motor_cutoff':'NOT VERIFIED',
        'physical_steering_response':'NOT MEASURED',
        'physical_braking_distance':'NOT MEASURED',
        'driving_clearance':'NOT GRANTED',
    }
    if any(x['software_clamp_violated'] for x in checks['speed_examples']):
        checks['software_calibration_warning']='One of the tested speeds would exceed driver ERPM clamps'
    if any(x['software_servo_clamp_violated'] for x in checks['steering_examples']):
        checks['software_steering_warning']='One of the tested steering commands would exceed servo clamps'
    return checks


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[2])
    a=p.parse_args()
    print(json.dumps(report(a.repo_root.expanduser()),indent=2))


if __name__=='__main__':
    main()
