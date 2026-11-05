#!/usr/bin/env python3
"""Conservative, ROS-free swept-vehicle LiDAR precondition.

Physical F1TENTH Ackermann steering, front LiDAR at x=0.215m from rear axle.
Measured obstacles inside an inflated swept footprint => STOP.
Missing/no-echo returns are UNKNOWN, never silently max-range free.
A small number of individually missing rays can be tolerated ONLY as an
unvalidated commissioning heuristic; a dark narrow obstacle may be missed.
Passing this SOFTWARE check is NOT permission for physical autonomous motion:
firmware current limits, measured braking, independent cutoff and dark target
obstacle response must all be commissioned separately.
"""
import math

WHEELBASE_M = .260
LIDAR_X_M = .215
LIDAR_Y_M = 0.0
REAR_BODY_X_M = -.040
FRONT_BODY_X_M = .305
BODY_HALF_WIDTH_M = .0985
EXTRA_MARGIN_M = .080
LOOKAHEAD_TRAVEL_M = .80
UNKNOWN_SAMPLE_RADIUS_M = (.18, .30, .45, .60, .80, 1.00, 1.20)
MAX_UNKNOWN_RUN_DEG = 1.5
MIN_SWEPT_KNOWN_FRACTION = .72


def _in_body_envelope(x, y, pose, margin):
    cx,cy,co,si = pose
    dx,dy=x-cx,y-cy
    local_x=co*dx+si*dy
    local_y=-si*dx+co*dy
    return (REAR_BODY_X_M-margin <= local_x <= FRONT_BODY_X_M+margin
            and abs(local_y) <= BODY_HALF_WIDTH_M+margin)


def _trajectory(steer, travel, spacing=.04):
    k=math.tan(steer)/WHEELBASE_M
    count=max(2,math.ceil(travel/spacing)+1)
    result=[]
    for i in range(count):
        s=travel*i/(count-1)
        if abs(k)<1e-8:
            result.append((s,0.,1.,0.))
        else:
            theta=k*s
            co,si=math.cos(theta),math.sin(theta)
            result.append((si/k,(1.-co)/k,co,si))
    return result


def swept_path_guard(scan, speed_mps, steer_rad, lookahead_travel_m=LOOKAHEAD_TRAVEL_M,
                     margin_m=EXTRA_MARGIN_M, max_unknown_run_deg=MAX_UNKNOWN_RUN_DEG,
                     min_known_fraction=MIN_SWEPT_KNOWN_FRACTION):
    """Return readiness + informative evidence. Works on ROS LaserScan or fixture.

    This tests only the *proposed* path, using the physical rectangle swept by
    a bicycle-model rear-axle trajectory. There is no claim that no-hit rays
    imply a clear path. The small-gap unknown allowance is a commissioning
    policy, NOT a proof of obstacle absence or braking performance.
    """
    failure={'ready':False,'reason':'BAD_SCAN_GEOMETRY','nearest_obstacle_m':None,
             'swept_known_fraction':0.,'max_unknown_run_deg':None}
    try:
        amin=float(scan.angle_min)
        da=float(scan.angle_increment)
        rmin=float(scan.range_min)
        rmax=float(scan.range_max)
        ranges=scan.ranges
        values=[amin,da,rmin,rmax,float(speed_mps),float(steer_rad),
                lookahead_travel_m,margin_m,max_unknown_run_deg,min_known_fraction]
        if (not ranges or len(ranges)<1000 or
            not all(math.isfinite(x) for x in values) or
            da<=0 or rmin<0 or rmax<=rmin or
            speed_mps<0 or lookahead_travel_m<=0 or margin_m<0 or
            not 0<min_known_fraction<=1 or max_unknown_run_deg<=0 or
            abs(steer_rad)>.50):
            return failure
    except (AttributeError,TypeError,ValueError):
        return failure

    poses=_trajectory(steer_rad,lookahead_travel_m)
    # Only the area ahead of the LiDAR is relevant to a forward-drive path;
    # sampled from beyond the closest sensor-origin point so we do not treat
    # an emitter's own origin inside the car as proof of an unknown obstacle.
    used=0
    unknown=0
    max_run=0
    run=0
    near=1e6
    nearest_obstacle=None
    nearest_hit=None
    obstacle_count=0
    obstacle_examples=[]

    for i, raw in enumerate(ranges):
        a=amin+i*da
        # Sensor rear-side scan is excluded from forward navigation.
        if abs(a)>math.pi/2:
            run=0
            continue
        co,si=math.cos(a),math.sin(a)
        # Do not consider the sensor's own origin a swept-path obstacle.
        # A missing beam is relevant if it could intersect the future swept
        # footprint in the first ~1.2m, not just if it lies in a fixed 20deg.
        ray_relevant=any(
            any(_in_body_envelope(LIDAR_X_M+r*co,LIDAR_Y_M+r*si,
                                  p,margin_m) for p in poses)
            for r in UNKNOWN_SAMPLE_RADIUS_M
        )
        if not ray_relevant:
            run=0
            continue
        used+=1
        d=float(raw)
        valid=math.isfinite(d) and rmin<d<=rmax
        if not valid:
            unknown+=1
            run+=1
            max_run=max(max_run,run)
            continue
        run=0
        near=min(near,d)
        x,y=LIDAR_X_M+d*co,LIDAR_Y_M+d*si
        if any(_in_body_envelope(x,y,p,margin_m) for p in poses):
            obstacle_count+=1
            hit={
                'distance_m':round(d,4),
                'angle_deg':round(math.degrees(a),2),
                'rear_axle_frame_x_m':round(x,4),
                'rear_axle_frame_y_m':round(y,4),
                'inside_uninflated_body_rectangle': (
                    REAR_BODY_X_M<=x<=FRONT_BODY_X_M and
                    abs(y)<=BODY_HALF_WIDTH_M),
                'inside_vehicle_body_plus_2cm': (
                    REAR_BODY_X_M-.02<=x<=FRONT_BODY_X_M+.02 and
                    abs(y)<=BODY_HALF_WIDTH_M+.02),
            }
            # Display the five closest blocking returns, not merely the
            # farthest obstacle or the average; no points are filtered out.
            obstacle_examples.append(hit)
            obstacle_examples.sort(key=lambda h:h['distance_m'])
            del obstacle_examples[5:]
            if nearest_obstacle is None or d<nearest_obstacle:
                nearest_obstacle=d
                nearest_hit=hit

    if used<30:
        return dict(failure,reason='NO_RELEVANT_FORWARD_RAYS')
    known_fraction=(used-unknown)/used
    missing_deg=max_run*math.degrees(da)
    details={
        'nearest_obstacle_m':round(nearest_obstacle,3) if nearest_obstacle is not None else None,
        'nearest_obstacle_detail': nearest_hit,
        'blocking_return_count': obstacle_count,
        'closest_blocking_returns': obstacle_examples,
        'nearest_measured_m':round(near,3) if near<1e6 else None,
        'swept_known_fraction':round(known_fraction,4),
        'max_unknown_run_deg':round(missing_deg,3),
        'relevant_ray_count':used,
        'unknown_relevant_ray_count':unknown,
        'steer_rad':round(float(steer_rad),4),
        'speed_mps':round(float(speed_mps),4),
        'path_lookahead_m':lookahead_travel_m,
        'body_margin_m':margin_m,
    }
    if nearest_obstacle is not None:
        return dict(details,ready=False,reason='MEASURED_OBSTACLE_IN_SWEPT_PATH')
    if known_fraction<min_known_fraction:
        return dict(details,ready=False,reason='SWEEP_INSUFFICIENT_MEASURED_COVERAGE')
    if missing_deg>max_unknown_run_deg:
        return dict(details,ready=False,reason='SWEEP_CONTIGUOUS_UNKNOWN_SECTOR')
    if near==1e6:
        return dict(details,ready=False,reason='NO_MEASURED_FORWARD_RANGES')
    return dict(details,ready=True,reason='SWEEP_PRECONDITION_PASS_NOT_SAFETY_CERTIFIED')
