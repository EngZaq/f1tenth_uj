#!/usr/bin/env python3
"""Offline re-analysis of previously saved LakiBeam wall JSON (no ROS/actuation).

Uses per-half-degree aggregate mean returns, not individual beam raw samples.
Calibration is APPROXIMATE and can be contaminated by background returns.
"""
import argparse
import json
import math
import statistics
from pathlib import Path


def analyze(raw, distance, width):
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Measured wall distance must be positive and finite')
    if width is not None and (not math.isfinite(width) or width <= 0):
        raise ValueError('Board width must be positive and finite')
    if width is None:
        half_angle = 5.0
        scope = 'Assumed central +/-5deg; board width unknown, so coverage unverified'
    else:
        board_half_angle = math.degrees(math.atan(width/(2*distance)))
        half_angle = min(10., board_half_angle-2.)
        if half_angle < 2.:
            raise ValueError('Board is too narrow for a confident central ROI')
        scope = 'Calculated board ROI with 2deg margin on each edge; requires centered board'

    bins = [a for a in raw.get('angle_data', [])
            if abs(float(a['angle_deg'])) <= half_angle and a.get('observations',0)>0]
    if not bins:
        raise ValueError('Missing per-angle observations in saved JSON')
    n = sum(a['observations'] for a in bins)
    measured = sum(a['observations']*a['measured_fraction'] for a in bins)
    residuals = [
        float(a['mean_measured_m'])*math.cos(math.radians(float(a['angle_deg']))) - distance
        for a in bins if a.get('mean_measured_m') is not None
    ]
    # 10cm inlier classification is ONLY to exclude mixed background returns;
    # it is NOT a manufacturer's accuracy tolerance or a safety limit.
    inliers = [e for e in residuals if abs(e) <= .10]
    rms = lambda values: round(math.sqrt(sum(e*e for e in values)/len(values)),4) if values else None
    median = lambda values: round(statistics.median(values),4) if values else None
    return {
        'analysis': 'OFFLINE, NO DEVICE CONNECTION OR ACTUATION',
        'measured_wall_distance_m': distance,
        'board_width_m': width,
        'selected_half_angle_deg': round(half_angle,3),
        'roi_assumptions': scope,
        'angle_bin_count': len(bins),
        'finite_return_fraction_in_roi': round(measured/n,4),
        'bins_with_mean_returns': len(residuals),
        'all_bin_median_distance_error_m': median(residuals),
        'all_bin_rmse_m': rms(residuals),
        'wall_inlier_bins_within_10cm': len(inliers),
        'wall_inlier_fraction_of_bins_with_returns':
            round(len(inliers)/len(residuals),4) if residuals else None,
        'inlier_median_distance_error_m': median(inliers),
        'inlier_rmse_m': rms(inliers),
        'caveat': 'Binned averages can mix target and background; no per-echo certainty. '
                  'Confirm with broad wall covering ROI at known range and matte-dark obstacle.',
    }


def self_test():
    rows = [
        {'angle_deg':a, 'observations':30,'measured_fraction':1.,
         'mean_measured_m':round(1.95/math.cos(math.radians(a)),4)}
        for a in range(-8,9)
    ]
    r=analyze({'angle_data':rows},1.95,1.0)
    assert r['finite_return_fraction_in_roi']==1.0
    assert abs(r['all_bin_median_distance_error_m']) <= .001
    rows[0]['mean_measured_m']=3.0
    r=analyze({'angle_data':rows},1.95,1.0)
    assert r['all_bin_rmse_m'] > r['inlier_rmse_m']
    print('PASS: distance correction, angular ROI and background rejection (offline)')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--self-test',action='store_true')
    p.add_argument('--file',type=Path)
    p.add_argument('--distance-m',type=float)
    p.add_argument('--board-width-m',type=float,default=None)
    p.add_argument('--out',type=Path)
    a=p.parse_args()
    if a.self_test:
        self_test()
        return
    if a.file is None or a.distance_m is None:
        p.error('--file and --distance-m required')
    result=analyze(json.loads(a.file.expanduser().read_text()),
                   a.distance_m,a.board_width_m)
    print(json.dumps(result,indent=2))
    if a.out:
        dest=a.out.expanduser()
        dest.parent.mkdir(parents=True,exist_ok=True)
        dest.write_text(json.dumps(result,indent=2)+'\n')
        print('Saved:',dest)


if __name__=='__main__':
    main()
