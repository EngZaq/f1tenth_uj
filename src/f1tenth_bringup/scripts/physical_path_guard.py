#!/usr/bin/env python3
"""Read-only geometry math for physical F1TENTH swept-path preconditions.

A PASS is *not* proof of safety: +inf is unknown, even when its measured
angular strip appears narrow. Firmware limits, braking, hardware cutoff and
obstacle-detection validation are independent requirements for motion.
"""
import math
import numpy as np

WHEELBASE_M=.260
LIDAR_X_M=.215
LIDAR_Y_M=0.
REAR_BODY_X_M=-.040
FRONT_BODY_X_M=.305
BODY_HALF_WIDTH_M=.0985
EXTRA_MARGIN_M=.080
LOOKAHEAD_TRAVEL_M=.80
MAX_UNKNOWN_RUN_DEG=1.5     # historical; now informational
MIN_SWEPT_KNOWN_FRACTION=.72
MAX_UNKNOWN_INTERSECTION_WIDTH_M=.020  # experimental software precondition, not safety standard
UNKNOWN_START_M=.18
UNKNOWN_END_M=1.20


def _trajectory(steer, travel, spacing=.04):
    k=math.tan(steer)/WHEELBASE_M
    n=max(2,math.ceil(travel/spacing)+1)
    s=np.linspace(0.,travel,n)
    t=k*s
    if abs(k)<1e-9:
        x=s
        y=np.zeros_like(s)
    else:
        x=np.sin(t)/k
        y=(1.-np.cos(t))/k
    return x[:,None],y[:,None],np.cos(t)[:,None],np.sin(t)[:,None]


def _slab_interval(origin,direction,lo,hi):
    # Avoid invalid numpy arithmetic when ray nearly parallel to axis.
    parallel=np.abs(direction)<1.e-10
    with np.errstate(divide='ignore',invalid='ignore'):
        t1=np.where(parallel,-np.inf,(lo-origin)/direction)
        t2=np.where(parallel, np.inf,(hi-origin)/direction)
    near=np.minimum(t1,t2)
    far=np.maximum(t1,t2)
    outside=parallel & ((origin<lo)|(origin>hi))
    near=np.where(outside,np.inf,near)
    far=np.where(outside,-np.inf,far)
    return near,far


def swept_path_guard(scan,speed_mps,steer_rad,
                     lookahead_travel_m=LOOKAHEAD_TRAVEL_M,
                     margin_m=EXTRA_MARGIN_M,
                     max_unknown_run_deg=MAX_UNKNOWN_RUN_DEG,
                     min_known_fraction=MIN_SWEPT_KNOWN_FRACTION):
    bad={'ready':False,'reason':'BAD_SCAN_GEOMETRY','nearest_obstacle_m':None,
         'swept_known_fraction':0.,'max_unknown_run_deg':None}
    try:
        amin=float(scan.angle_min)
        da=float(scan.angle_increment)
        rmin=float(scan.range_min)
        rmax=float(scan.range_max)
        rr=np.asarray(scan.ranges,dtype=np.float64)
        nums=(amin,da,rmin,rmax,float(speed_mps),float(steer_rad),
              lookahead_travel_m,margin_m,max_unknown_run_deg,min_known_fraction)
        if (rr.ndim!=1 or rr.size<1000 or not all(math.isfinite(v) for v in nums)
            or da<=0 or rmin<0 or rmax<=rmin or speed_mps<0
            or abs(steer_rad)>.5 or lookahead_travel_m<=0 or margin_m<0
            or max_unknown_run_deg<=0 or not 0<min_known_fraction<=1):
            return bad
    except (AttributeError,ValueError,TypeError):
        return bad

    aa=amin+np.arange(rr.size)*da
    sector=np.abs(aa)<=np.pi/2
    angles=aa[sector]
    ranges=rr[sector]
    if angles.size<40:
        return dict(bad,reason='NO_FORWARD_SCAN_SECTOR')

    cx,cy,co,si=_trajectory(steer_rad,lookahead_travel_m)
    # All rays transformed into each candidate rear-axle body frame.
    ox=co*(LIDAR_X_M-cx)+si*(LIDAR_Y_M-cy)
    oy=-si*(LIDAR_X_M-cx)+co*(LIDAR_Y_M-cy)
    dx=co*np.cos(angles)[None,:]+si*np.sin(angles)[None,:]
    dy=-si*np.cos(angles)[None,:]+co*np.sin(angles)[None,:]

    x0,x1=REAR_BODY_X_M-margin_m,FRONT_BODY_X_M+margin_m
    y0,y1=-BODY_HALF_WIDTH_M-margin_m,BODY_HALF_WIDTH_M+margin_m
    nx,fx=_slab_interval(ox,dx,x0,x1)
    ny,fy=_slab_interval(oy,dy,y0,y1)
    entry=np.maximum(np.maximum(nx,ny),0.)
    ray_exit=np.minimum(fx,fy)
    intersect=ray_exit>=entry

    # A ray can intersect the swept body while remaining UNKNOWN (+inf).
    # Track its farthest possible path intersection, not a fake max-range.
    possible=intersect & (ray_exit>=UNKNOWN_START_M) & (entry<=UNKNOWN_END_M)
    farthest=np.max(np.where(possible,np.minimum(ray_exit,UNKNOWN_END_M),0.),axis=0)
    relevant=farthest>0
    used=int(np.count_nonzero(relevant))
    if used<30:
        return dict(bad,reason='NO_RELEVANT_FORWARD_RAYS')

    valid=np.isfinite(ranges)&(ranges>rmin)&(ranges<=rmax)
    unknown=relevant&~valid
    missing=int(np.count_nonzero(unknown))
    known_fraction=(used-missing)/used
    # Distances used only when measurement exists, no-echo never obstacle-free.
    colliding=valid & np.any(intersect & (ranges[None,:]>=entry) & (ranges[None,:]<=ray_exit),axis=0)
    hit_idx=np.flatnonzero(colliding)
    obstacle_count=int(hit_idx.size)
    hits=[]
    for i in hit_idx[np.argsort(ranges[hit_idx])[:5]]:
        d=float(ranges[i]); a=float(angles[i]); x=LIDAR_X_M+d*math.cos(a);y=LIDAR_Y_M+d*math.sin(a)
        hits.append({'distance_m':round(d,4),'angle_deg':round(math.degrees(a),2),
            'rear_axle_frame_x_m':round(x,4),'rear_axle_frame_y_m':round(y,4),
            'inside_uninflated_body_rectangle':REAR_BODY_X_M<=x<=FRONT_BODY_X_M and abs(y)<=BODY_HALF_WIDTH_M,
            'inside_vehicle_body_plus_2cm':REAR_BODY_X_M-.02<=x<=FRONT_BODY_X_M+.02 and abs(y)<=BODY_HALF_WIDTH_M+.02})
    sectors=[]
    active=np.flatnonzero(unknown)
    if active.size:
        groups=np.split(active,np.flatnonzero(np.diff(active)>1)+1)
        for g in groups:
            r_far=float(np.max(farthest[g]));n=len(g);width=2*r_far*math.sin(n*da/2)
            sectors.append({'start_angle_deg':round(math.degrees(angles[g[0]]),2),
                'end_angle_deg':round(math.degrees(angles[g[-1]]),2),
                'angle_span_deg':round(math.degrees(n*da),3),'ray_count':n,
                'farthest_path_intersection_sample_m':round(r_far,3),
                'unobserved_width_at_farthest_sample_m':round(width,4)})
    sectors.sort(key=lambda q:q['unobserved_width_at_farthest_sample_m'],reverse=True)
    maxdeg=max((s['angle_span_deg'] for s in sectors),default=0.)
    maxwidth=max((s['unobserved_width_at_farthest_sample_m'] for s in sectors),default=0.)
    observed=ranges[relevant&valid]
    nearest=float(np.min(observed)) if observed.size else None
    details={
      'nearest_obstacle_m':round(float(ranges[hit_idx[np.argmin(ranges[hit_idx])]]),3) if obstacle_count else None,
      'nearest_obstacle_detail':hits[0] if hits else None,
      'blocking_return_count':obstacle_count,'closest_blocking_returns':hits,
      'nearest_measured_m':round(nearest,3) if nearest is not None else None,
      'swept_known_fraction':round(known_fraction,4),
      'max_unknown_run_deg':round(maxdeg,3),
      'max_unknown_path_width_m':round(maxwidth,4),
      'max_unknown_path_width_allowed_m':MAX_UNKNOWN_INTERSECTION_WIDTH_M,
      'largest_unknown_sectors':sectors[:5],
      'relevant_ray_count':used,'unknown_relevant_ray_count':missing,
      'steer_rad':round(float(steer_rad),4),'speed_mps':round(float(speed_mps),4),
      'path_lookahead_m':lookahead_travel_m,'body_margin_m':margin_m}
    if obstacle_count:
        return dict(details,ready=False,reason='MEASURED_OBSTACLE_IN_SWEPT_PATH')
    if known_fraction<min_known_fraction:
        return dict(details,ready=False,reason='SWEEP_INSUFFICIENT_MEASURED_COVERAGE')
    if maxwidth>MAX_UNKNOWN_INTERSECTION_WIDTH_M:
        return dict(details,ready=False,reason='SWEEP_UNKNOWN_CORRIDOR_TOO_WIDE')
    if nearest is None:
        return dict(details,ready=False,reason='NO_MEASURED_FORWARD_RANGES')
    return dict(details,ready=True,reason='SWEEP_PRECONDITION_PASS_NOT_SAFETY_CERTIFIED')
