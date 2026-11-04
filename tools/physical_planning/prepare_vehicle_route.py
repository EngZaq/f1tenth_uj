#!/usr/bin/env python3
"""Automatically smooth and body-audit an extracted closed route. Offline only."""
import argparse
import contextlib
import io
import json
import tempfile
from pathlib import Path
import numpy as np
from scipy.ndimage import gaussian_filter1d
from optimize_vehicle_route import improve, geometry
from physical_geometry import WHEELBASE


def prepare(map_yaml, source, output, report):
    data = np.loadtxt(source, comments='#', delimiter=';')
    attempts = []
    with tempfile.TemporaryDirectory() as directory:
        for sigma in (0., .5, 1., 1.5, 2., 3., 4.):
            xy = data[:,1:3].copy()
            if sigma:
                xy = gaussian_filter1d(xy, sigma, axis=0, mode='wrap')
            segments = np.linalg.norm(np.roll(xy,-1,axis=0)-xy,axis=1)
            if np.any(segments <= 1e-7):
                continue
            arc = np.r_[0.,np.cumsum(segments[:-1])]
            yaw, kappa, _ = geometry(xy, float(np.mean(segments)), WHEELBASE)
            candidate = Path(directory)/'candidate.csv'
            np.savetxt(candidate,np.column_stack((arc,xy,yaw,kappa,data[:,5:7])),delimiter=';')
            log = io.StringIO()
            with contextlib.redirect_stdout(log):
                code = improve(map_yaml,candidate,output,.15,24.,40)
            attempts.append({'smoothing_sigma_points':sigma,'accepted':code==0,'audit':log.getvalue()})
            print(f'Route smoothing sigma={sigma:g}: '+('accepted' if code==0 else 'rejected'),flush=True)
            if code == 0:
                Path(report).write_text(json.dumps({'accepted':True,'attempts':attempts},indent=2)+'\n')
                return
    Path(report).write_text(json.dumps({'accepted':False,'attempts':attempts},indent=2)+'\n')
    raise RuntimeError('No automatically prepared route passes body clearance and steering checks')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml')
    parser.add_argument('route_csv')
    parser.add_argument('--out',required=True)
    parser.add_argument('--report',required=True)
    args = parser.parse_args()
    prepare(args.map_yaml,args.route_csv,args.out,args.report)


if __name__ == '__main__':
    main()
