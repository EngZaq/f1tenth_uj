#!/usr/bin/env python3
"""Optimize a closed map-frame route against occupied/unknown cells for the full car.

Offline physical geometry preflight only. Writes output only when requested geometry margin
and steering limit are satisfied. Nothing here licenses physical driving.
"""
from pathlib import Path
import argparse
import math
import sys
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import differential_evolution
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from physical_geometry import BODY_REAR, BODY_FRONT, BODY_HALFWIDTH, WHEELBASE


def build_problem(map_yaml, route_file):
    meta = yaml.safe_load(Path(map_yaml).read_text())
    image = Path(meta['image'])
    if not image.is_absolute():
        image = Path(map_yaml).parent / image
    raster = np.asarray(Image.open(image).convert('L'))
    res = float(meta['resolution'])
    origin = np.asarray(meta['origin'], dtype=float)
    distance = distance_transform_edt(raster >= 250) * res
    path = np.loadtxt(route_file, comments='#', delimiter=';')
    if path.ndim != 2 or path.shape[1] < 7 or len(path) < 100:
        raise ValueError('Input must have >=100 s;x;y;yaw;kappa;v;a rows')
    if not np.all(np.diff(path[:, 0]) > 0):
        raise ValueError('Route arc length must strictly increase')
    lon, lat = np.meshgrid(np.linspace(BODY_REAR, BODY_FRONT, 30),
                           np.linspace(-BODY_HALFWIDTH, BODY_HALFWIDTH, 17))
    return path, distance, origin, res, lon.ravel(), lat.ravel()


def geometry(xy, arc_step, wheelbase):
    vx = (np.roll(xy[:, 0], -1) - np.roll(xy[:, 0], 1)) / (2 * arc_step)
    vy = (np.roll(xy[:, 1], -1) - np.roll(xy[:, 1], 1)) / (2 * arc_step)
    ax = (np.roll(xy[:, 0], -1) - 2*xy[:, 0] + np.roll(xy[:, 0], 1)) / arc_step**2
    ay = (np.roll(xy[:, 1], -1) - 2*xy[:, 1] + np.roll(xy[:, 1], 1)) / arc_step**2
    yaw = np.arctan2(vy, vx)
    curvature = (vx * ay - vy * ax) / np.maximum((vx*vx + vy*vy)**1.5, 1e-9)
    steer = np.arctan(wheelbase * np.abs(curvature))
    return yaw, curvature, steer


def clearance(xy, yaw, distance, origin, resolution, lon, lat, selected=None):
    if selected is not None:
        xy, yaw = xy[selected], yaw[selected]
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    xx = xy[:, 0, None] + c * lon - s * lat
    yy = xy[:, 1, None] + s * lon + c * lat
    a = float(origin[2])
    ca, sa = math.cos(a), math.sin(a)
    u = ca * (xx - origin[0]) + sa * (yy - origin[1])
    v = -sa * (xx - origin[0]) + ca * (yy - origin[1])
    row = distance.shape[0] - v / resolution - 0.5
    col = u / resolution - 0.5
    # The nearest-cell audit catches raster aliasing that bilinear interpolation
    # alone can smooth away. Both occupied and unknown pixels are blocked.
    ri = distance.shape[0] - 1 - np.floor(v / resolution).astype(int)
    ci = np.floor(u / resolution).astype(int)
    good = ((ri >= 0) & (ri < distance.shape[0]) &
            (ci >= 0) & (ci < distance.shape[1]))
    rr = np.clip(ri, 0, distance.shape[0]-1)
    cc = np.clip(ci, 0, distance.shape[1]-1)
    cell = np.where(good, distance[rr, cc], 0.0).min(axis=1)
    smooth = map_coordinates(distance, [row, col], order=1,
                             mode='constant', cval=0.0).min(axis=1)
    return np.minimum(cell, smooth), cell, smooth


def improve(map_yaml, route_csv, out, min_margin, max_steer_deg, iters=40):
    path, distance, origin, res, lon, lat = build_problem(map_yaml, route_csv)
    xy = path[:, 1:3].copy()
    n = len(path)
    step = float(np.median(np.diff(path[:, 0])))
    wheelbase = WHEELBASE
    old, _, _ = clearance(xy, path[:, 3], distance, origin, res, lon, lat)
    worst = int(np.argmin(old))
    idx = np.array([(worst + d) % n for d in range(-15, 20)], dtype=int)
    base_normals = np.column_stack((-np.sin(path[:, 3]), np.cos(path[:, 3])))
    arange = np.arange(n)

    def candidate(z):
        offsets = np.zeros(n)
        for amp, relcenter, width in (z[:3], z[3:]):
            center = (worst + relcenter) % n
            dr = np.abs(arange - center)
            dr = np.minimum(dr, n - dr)
            offsets += amp * np.exp(-0.5 * (dr / width)**2)
        changed = xy + offsets[:, None] * base_normals
        yaw, kappa, steer = geometry(changed, step, wheelbase)
        return changed, yaw, kappa, steer, offsets

    def objective(z):
        changed, yaw, _, steer, _ = candidate(z)
        local, _, _ = clearance(changed, yaw, distance, origin, res,
                                lon, lat, idx)
        excess = max(0., math.degrees(float(steer.max())) - max_steer_deg)
        return -float(local.min()) + 2.0 * excess**2 + 0.001 * (abs(z[0]) + abs(z[3]))

    result = differential_evolution(
        objective, [(-0.12, 0.12), (-6, 2), (2, 11),
                    (-0.12, 0.12), (-2, 10), (2, 11)],
        maxiter=iters, popsize=6, polish=False, seed=29)
    changed, yaw, kappa, steer, offsets = candidate(result.x)
    margins, cell, smooth = clearance(changed, yaw, distance,
                                       origin, res, lon, lat)
    original_min = float(old.min())
    new_min = float(margins.min())
    peak_steer = math.degrees(float(steer.max()))
    # Also sample between each waypoint. A pointwise-only test misses a
    # corner of the body sweeping across an unsafe map cell.
    subdivisions = 8
    ii = np.repeat(np.arange(n), subdivisions)
    jj = (ii + 1) % n
    fraction = np.tile(np.arange(subdivisions)/subdivisions, n)
    dense_xy = ((1 - fraction[:, None]) * changed[ii] +
                fraction[:, None] * changed[jj])
    angle_delta = np.arctan2(np.sin(yaw[jj]-yaw[ii]),
                             np.cos(yaw[jj]-yaw[ii]))
    dense_yaw = yaw[ii] + fraction * angle_delta
    dense, _, _ = clearance(dense_xy, dense_yaw, distance,
                             origin, res, lon, lat)
    dense_min = float(dense.min())
    accepted = (new_min >= min_margin and dense_min >= min_margin and
                peak_steer <= max_steer_deg)
    print(f'Original min body clearance: {original_min:.3f}m at index {worst}')
    print(f'Candidate min at waypoints {new_min:.3f}m; '
          f'between waypoints {dense_min:.3f}m; '
          f'peak steering {peak_steer:.2f}deg; '
          f'peak lateral shift {float(np.max(np.abs(offsets))):.3f}m')
    print(f'Conservative cell min {float(cell.min()):.3f}m; '
          f'interpolated min {float(smooth.min()):.3f}m')
    if not accepted:
        print(f'REJECTED: nominal geometry requires at least {min_margin:.3f}m; '
              'do not drive. Improve mapping or trajectory planning.')
        return 2
    segments = np.linalg.norm(np.roll(changed, -1, axis=0) - changed, axis=1)
    arc = np.r_[0., np.cumsum(segments[:-1])]
    lat_accel, accel, decel, v_max = 0.75, 0.60, 0.80, 0.8
    curve_limit = np.sqrt(lat_accel / np.maximum(np.abs(kappa), 1e-4))
    width_limit = np.clip(0.2 + 2.0*(margins-min_margin), 0.20, v_max)
    speed = np.minimum(curve_limit, width_limit)
    for _ in range(20):
        for i in range(n-1, -1, -1):
            j = (i+1) % n
            speed[i] = min(speed[i], math.sqrt(speed[j]**2 + 2*decel*segments[i]))
        for i in range(n):
            j = (i+1) % n
            speed[j] = min(speed[j], math.sqrt(speed[i]**2 + 2*accel*segments[i]))
    ax = (np.roll(speed, -1)**2-speed**2)/(2*segments+1e-8)
    output = np.column_stack((arc, changed, yaw, kappa, speed, ax))
    target = Path(out)
    target.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(target, output, delimiter=';', fmt='%.6f',
               header='slam_map frame; nominal full-body-audited route; OFFLINE PHYSICAL GEOMETRY - NOT APPROVED FOR DRIVING\n'
                      's_m;x_m;y_m;psi_rad;kappa_1pm;vx_mps;ax_mps2',
               comments='# ')
    print('PASS nominal map geometry, NOT validated for localization/tracking:',
          target)
    print('Planned speed envelope range:', float(speed.min()), float(speed.max()))
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('map_yaml')
    p.add_argument('route_csv')
    p.add_argument('--out', required=True)
    p.add_argument('--margin', type=float, default=0.15)
    p.add_argument('--max-steer-deg', type=float, default=24.0)
    p.add_argument('--iterations', type=int, default=40)
    args = p.parse_args()
    if not (0 < args.margin < 0.5):
        p.error('body margin must be between 0 and 0.5m')
    raise SystemExit(improve(args.map_yaml, args.route_csv, args.out,
                             args.margin, args.max_steer_deg, args.iterations))


if __name__ == '__main__':
    main()
