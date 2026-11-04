#!/usr/bin/env python3
"""Audit a ROS map-frame racing line against the WHOLE F1TENTH vehicle.
Unknown cells are obstacles. This is an offline geometrical preflight, not
proof of safe autonomy: localization, tracking, and map uncertainties remain.
"""
from pathlib import Path
import argparse
import json
import math
import sys

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from physical_geometry import LASER, BODY_FRONT, BODY_REAR, BODY_HALFWIDTH, WHEELBASE


def audit(map_yaml, route_file, body_length=BODY_FRONT-BODY_REAR, body_width=2*BODY_HALFWIDTH,
          rear_axle_to_body_center=LASER['rear_axle_to_body_center_x_m'], wheelbase=WHEELBASE,
          max_steer=0.4189, body_margin=0.10):
    meta = yaml.safe_load(Path(map_yaml).read_text())
    image = Path(meta['image'])
    if not image.is_absolute():
        image = Path(map_yaml).parent / image
    pixels = np.asarray(Image.open(image).convert('L'))
    free = pixels >= 250
    res = float(meta['resolution'])
    ox, oy, angle = map(float, meta['origin'])
    path = np.loadtxt(route_file, comments='#', delimiter=';')
    if path.ndim != 2 or path.shape[1] < 5 or len(path) < 30:
        raise ValueError('Expected s;x;y;heading;curvature;... route CSV')
    if not np.all(np.diff(path[:, 0]) > 0):
        raise ValueError('Waypoint arc length must increase')
    xy, heading = path[:, 1:3], path[:, 3]
    steering = np.arctan(wheelbase * np.abs(path[:, 4]))

    # Simulator rear-axle origin, body center offset and 0.58 x 0.31 m body.
    lon, lat = np.meshgrid(
        np.linspace(rear_axle_to_body_center - body_length / 2,
                    rear_axle_to_body_center + body_length / 2, 30),
        np.linspace(-body_width / 2, body_width / 2, 17))
    lon, lat = lon.ravel(), lat.ravel()
    c, s = np.cos(heading)[:, None], np.sin(heading)[:, None]
    bx = xy[:, 0, None] + c * lon - s * lat
    by = xy[:, 1, None] + s * lon + c * lat
    mx = np.cos(angle) * (bx - ox) + np.sin(angle) * (by - oy)
    my = -np.sin(angle) * (bx - ox) + np.cos(angle) * (by - oy)
    col = np.floor(mx / res).astype(int)
    row = pixels.shape[0] - 1 - np.floor(my / res).astype(int)
    valid = ((row >= 0) & (row < pixels.shape[0]) &
             (col >= 0) & (col < pixels.shape[1]))
    rr = np.clip(row, 0, pixels.shape[0] - 1)
    cc = np.clip(col, 0, pixels.shape[1] - 1)
    distance = distance_transform_edt(free) * res
    clearance = np.where(valid, distance[rr, cc], 0.0)
    worst = clearance.min(axis=1)
    result = {
        'map': str(map_yaml), 'route': str(route_file),
        'waypoints': int(len(path)),
        'minimum_body_clearance_m': float(worst.min()),
        'limiting_waypoint': int(np.argmin(worst)),
        'max_required_steering_deg': float(np.degrees(steering.max())),
        'steering_limit_deg': float(math.degrees(max_steer)),
        'body_margin_requirement_m': float(body_margin),
        'occupied_or_unknown_body_samples': int(np.count_nonzero(clearance <= 0)),
        'passes_geometry': bool(np.all(worst >= body_margin) and
                                np.max(steering) <= max_steer),
        'caveat': 'Must also budget localization/tracking errors and scan uncertainty'
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('map_yaml')
    parser.add_argument('route_csv')
    parser.add_argument('--required-body-margin', type=float, default=0.10)
    parser.add_argument('--out')
    args = parser.parse_args()
    result = audit(args.map_yaml, args.route_csv,
                   body_margin=args.required_body_margin)
    output = json.dumps(result, indent=2) + '\n'
    print(output)
    if args.out:
        Path(args.out).write_text(output)
    return 0 if result['passes_geometry'] else 2


if __name__ == '__main__':
    sys.exit(main())
