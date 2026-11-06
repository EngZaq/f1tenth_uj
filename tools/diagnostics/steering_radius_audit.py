#!/usr/bin/env python3
"""Read-only Ackermann steering feasibility audit for the UJ F1TENTH car.

Uses NOMINAL software conversion only; mechanical steering angle is not yet
verified. Does not import ROS or publish hardware commands.
"""
import argparse
import math

WHEELBASE_M = .260
SOFT_STEERING_LIMIT_RAD = .10
SERVO_GAIN = -.50
SERVO_CENTER = .361
SERVO_MIN = .30
SERVO_MAX = .42


def required_angle(radius_m):
    if not math.isfinite(radius_m) or radius_m <= 0:
        raise ValueError('Turn radius must be positive and finite')
    return math.atan(WHEELBASE_M / radius_m)


def audit(radius_m):
    angle = required_angle(radius_m)
    plus_servo = SERVO_CENTER + SERVO_GAIN * angle
    minus_servo = SERVO_CENTER - SERVO_GAIN * angle
    return {
        'radius_m': radius_m,
        'steer_deg': math.degrees(angle),
        'steer_rad': angle,
        'positive_servo': plus_servo,
        'negative_servo': minus_servo,
        'within_software_angle_limit': angle <= SOFT_STEERING_LIMIT_RAD,
        'within_servo_range': (SERVO_MIN <= plus_servo <= SERVO_MAX and
                               SERVO_MIN <= minus_servo <= SERVO_MAX),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radius', type=float, default=None,
                        help='Optional actual measured CENTERLINE minimum turn radius (m)')
    args = parser.parse_args()
    assert abs(required_angle(WHEELBASE_M / math.tan(.1)) - .1) < 1e-10
    radii = ([args.radius] if args.radius is not None
             else [1., 1.5, 2., 2.5, 2.6, 3., 4.])
    print('Nominal model only: wheelbase 0.260m, servo=0.361-0.5*steering')
    print('radius_m | steer_deg | servo_left | servo_right | within current limits')
    for r in radii:
        result = audit(r)
        allowed = result['within_software_angle_limit'] and result['within_servo_range']
        print('%7.2f | %9.2f | %10.3f | %11.3f | %s' % (
            r, result['steer_deg'], result['positive_servo'],
            result['negative_servo'], 'YES (nominal)' if allowed else 'NO'))
    print('Ideal min turning radius at current +/-0.10rad limit: %.3fm' %
          (WHEELBASE_M/math.tan(SOFT_STEERING_LIMIT_RAD)))
    print('Not a physical steering calibration or safety certification.')


if __name__ == '__main__':
    main()
