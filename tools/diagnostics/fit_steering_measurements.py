#!/usr/bin/env python3
"""Read-only front steering characterization for physical F1TENTH.

Input CSV: servo,left_deg,right_deg
Angle sign: left wheel points LEFT => positive; RIGHT => negative.
The two road-wheel angles must be measured against the vehicle centerline,
not the servo horn. The calibration reported is nominal and requires a
separate physical validation; this script NEVER writes active ROS configs.
"""
import argparse
import csv
import math
from pathlib import Path

WHEELBASE_M = .260
CURRENT_SERVO_CENTER = .361
CURRENT_SERVO_GAIN = -.50  # servo = gain * virtual_steering_rad + center
MIN_EXCITATION = .018      # servo command span
MAX_WHEEL_ANGLE_DEG = 45.


def equivalent_steering(left_deg, right_deg):
    """Kinematic equivalent center-wheel angle from left and right tire angles."""
    if not all(math.isfinite(x) and abs(x) <= MAX_WHEEL_ANGLE_DEG
               for x in (left_deg, right_deg)):
        raise ValueError('Nonfinite or unrealistic front road-wheel angle')
    return math.atan(.5 * (
        math.tan(math.radians(left_deg)) + math.tan(math.radians(right_deg))))


def fit(rows):
    if len(rows) < 3:
        raise ValueError('Need >=3 measured servo positions, including neutral')
    positions = [float(r[0]) for r in rows]
    angles = [equivalent_steering(float(r[1]), float(r[2])) for r in rows]
    if not all(math.isfinite(x) and .25 <= x <= .5 for x in positions):
        raise ValueError('Invalid servo command values')
    span = max(positions) - min(positions)
    if span < MIN_EXCITATION:
        raise ValueError('Servo span too small to estimate gain reliably')
    sx = sum(positions)/len(positions)
    sy = sum(angles)/len(angles)
    sxx = sum((x-sx)**2 for x in positions)
    slope = sum((x-sx)*(y-sy) for x,y in zip(positions, angles)) / sxx
    if abs(slope) < .1:
        raise ValueError('Insufficient measured steering response')
    intercept = sy - slope*sx
    neutral_servo = -intercept/slope
    servo_gain = 1./slope
    residuals = [math.degrees(y-(intercept+slope*x))
                 for x,y in zip(positions, angles)]
    rmse_deg = math.sqrt(sum(e*e for e in residuals)/len(residuals))
    measured_max_deg = max(abs(math.degrees(x)) for x in angles)
    smallest_radius_m = (WHEELBASE_M / math.tan(math.radians(measured_max_deg))
                         if measured_max_deg > 0 else float('inf'))
    turn_directions = [math.copysign(1., y) for y in angles if abs(y)>math.radians(1.)]
    return {
        'samples': len(rows),
        'servo_span': span,
        'equivalent_deg': [round(math.degrees(x), 3) for x in angles],
        'fit_neutral_servo': neutral_servo,
        'fit_servo_gain_per_radian': servo_gain,
        'fit_rmse_deg': rmse_deg,
        'max_measured_roadwheel_equivalent_deg': measured_max_deg,
        'ideal_min_radius_from_measured_range_m': smallest_radius_m,
        'both_turn_directions_measured': -1. in turn_directions and 1. in turn_directions,
        'same_sign_as_current_config': servo_gain*CURRENT_SERVO_GAIN > 0.,
    }


def self_test():
    samples = []
    for position in (.311, .341, .361, .381, .411):
        delta_deg = math.degrees((position-CURRENT_SERVO_CENTER)/CURRENT_SERVO_GAIN)
        samples.append((position, delta_deg, delta_deg))
    r = fit(samples)
    assert abs(r['fit_neutral_servo']-.361) < 1e-8, r
    assert abs(r['fit_servo_gain_per_radian']+.5) < 1e-8, r
    assert r['both_turn_directions_measured'] and r['fit_rmse_deg']<1e-6, r
    print('PASS: virtual Ackermann fit recovers 0.361 offset and -0.50 gain')
    print('No ROS commands or active configuration changes.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('csv_file', nargs='?', help='CSV with servo,left_deg,right_deg')
    p.add_argument('--self-test', action='store_true')
    p.add_argument('--template', action='store_true', help='Print CSV template')
    args = p.parse_args()
    if args.template:
        print('servo,left_deg,right_deg')
        for pos in (.361,.341,.381,.311,.411):
            print('%.3f,,' % pos)
        return
    if args.self_test:
        self_test()
        return
    if not args.csv_file:
        p.error('Provide CSV or --template or --self-test')
    with Path(args.csv_file).expanduser().open(newline='') as fd:
        reader = csv.DictReader(fd)
        if not {'servo','left_deg','right_deg'}.issubset(set(reader.fieldnames or [])):
            raise ValueError('CSV needs servo,left_deg,right_deg')
        data = [(float(row['servo']), float(row['left_deg']),float(row['right_deg']))
                for row in reader]
    r = fit(data)
    print('STEERING FIT: observations, NOT verified actuator command calibration')
    for key,v in r.items():
        print('%s: %s' % (key, round(v,5) if isinstance(v,float) else v))
    if not r['both_turn_directions_measured']:
        print('WARNING: both steering directions not measured')
    if not r['same_sign_as_current_config']:
        print('WARNING: measured steering direction disagrees with configured servo gain')
    if r['fit_rmse_deg'] > 1.5:
        print('WARNING: angle fit residual >1.5 deg; inspect backlash and measurement quality')
    if not .30 <= r['fit_neutral_servo'] <= .42:
        print('WARNING: fitted neutral falls outside current VESC servo software window')
    print('Do NOT copy these values to the controller until repeatable linkage and stop tests pass.')


if __name__ == '__main__':
    main()
