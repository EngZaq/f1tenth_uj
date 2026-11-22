#!/usr/bin/env python3
"""LiDAR self-mask + field of view from a STILL scan recording (22 Nov). Reads a bag only; writes nothing.

Prints (laser frame, degrees):
- the field of view: the beams that returned anything in at least one scan (LakiBeam driver scan_range
  45..315 = 270 deg; the rear is blind);
- self-return sectors: beams with a return closer than --max-range (0.30 m) in >= --share of the scans
  (the car itself: cables, connectors, mounts), padded by 0.5 deg, as a `lidar.self_mask` yaml snippet.
The car must stand still and nobody may be near it during the recording (a hand or a leg becomes a sector).
The values go into config/real_car/vehicle_params.yaml `lidar` only with a results-doc row (rule).
  lidar_self_mask.py <run folder with bag/ | bag folder> [--max-range 0.30] [--share 0.05]
  lidar_self_mask.py --self-test
"""
import argparse
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

PAD_DEG = 0.5
JOIN_DEG = 2.0           # near sectors closer than this are one sector


def analyse(angles_deg, scans, max_range, share):
    """angles_deg = beam bearings; scans = [[range per beam]] -> (fov (min, max) or None, [(from, to, rmax_seen)])."""
    n = len(scans)
    seen = [False] * len(angles_deg)
    near = [0] * len(angles_deg)
    near_max = [0.0] * len(angles_deg)
    for ranges in scans:
        for i, r in enumerate(ranges):
            if math.isfinite(r) and r > 0.0:
                seen[i] = True
                if r < max_range:
                    near[i] += 1
                    near_max[i] = max(near_max[i], r)
    vis = [a for a, s in zip(angles_deg, seen) if s]
    fov = (min(vis), max(vis)) if vis else None
    sectors = []
    for a, k, rm in zip(angles_deg, near, near_max):
        if k < share * n:
            continue
        if sectors and a - sectors[-1][1] <= JOIN_DEG:
            sectors[-1] = (sectors[-1][0], a, max(sectors[-1][2], rm))
        else:
            sectors.append((a, a, rm))
    return fov, [(a - PAD_DEG, b + PAD_DEG, rm) for a, b, rm in sectors]


def self_test():
    inc = 0.25
    ang = [-180.0 + k * inc for k in range(1440)]

    def scan(k):
        out = []
        for a in ang:
            if not -135.75 <= a <= 134.25:
                out.append(float('inf'))                 # blind rear
            elif 96.5 <= a <= 134.25:
                out.append(0.15 if k % 10 else 6.0)       # own cables, seen in 90 % of scans
            elif -100.0 <= a <= -99.0 and k == 3:
                out.append(0.2)                          # a hand once: not a sector
            else:
                out.append(6.0)
        return out
    fov, sec = analyse(ang, [scan(k) for k in range(40)], 0.30, 0.05)
    assert fov == (-135.75, 134.25), fov
    assert len(sec) == 1 and abs(sec[0][0] - 96.0) < 1e-9 and abs(sec[0][1] - 134.75) < 1e-9 and sec[0][2] == 0.15, sec
    print('PASS: field of view edges, self-return sector (padded, joined), a one-off hand ignored. No ROS.')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('folder', nargs='?')
    ap.add_argument('--max-range', type=float, default=0.30)
    ap.add_argument('--share', type=float, default=0.05)
    ap.add_argument('--self-test', action='store_true')
    args = ap.parse_args()
    if args.self_test:
        self_test()
        return 0
    if not args.folder:
        ap.error('folder or --self-test')
    import analyze as A
    folder = Path(args.folder).expanduser()
    bag = folder / 'bag' if (folder / 'bag' / 'metadata.yaml').exists() else folder
    scans = A.read_bag(bag).get('/scan', [])
    if not scans:
        raise SystemExit('no /scan in %s' % bag)
    m0 = scans[0][1]
    ang = [round(math.degrees(m0.angle_min + i * m0.angle_increment), 4) for i in range(len(m0.ranges))]
    fov, sec = analyse(ang, [list(m.ranges) for _, m in scans], args.max_range, args.share)
    print('%s: %d scans' % (folder.name, len(scans)))
    if fov:
        print('field of view %.2f .. %.2f deg (%.1f deg); blind outside' % (fov[0], fov[1], fov[1] - fov[0]))
    print('self-return sectors (< %.2f m in >= %.0f %% of scans):' % (args.max_range, 100 * args.share))
    for a, b, rm in sec:
        print('  %.2f .. %.2f deg, closest-return max %.3f m' % (a, b, rm))
    print('yaml (lidar):\n  fov_min_deg: %.2f\n  fov_max_deg: %.2f\n  self_mask:' % fov if fov else 'yaml: -')
    for a, b, rm in sec:
        print('    - [%.1f, %.1f, %.2f]' % (math.floor(a * 2) / 2, math.ceil(b * 2) / 2, args.max_range))
    if not sec:
        print('    []   # nothing of the car is seen any more')
    return 0


if __name__ == '__main__':
    sys.exit(main())
