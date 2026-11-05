#!/usr/bin/env python3
"""ROS-free swept vehicle guard tests. Does NOT drive a robot."""
import math
import time
from physical_path_guard import swept_path_guard


class Scan:
    angle_min = -math.pi
    angle_increment = 2*math.pi/1440
    range_min = 0.
    range_max = 100.
    def __init__(self, distance=3.):
        self.ranges=[distance]*1440
    def i(self, angle):
        return round((math.radians(angle)-self.angle_min)/self.angle_increment)


def check():
    scan=Scan(3.)
    baseline=swept_path_guard(scan,.075,-.1)
    assert baseline['ready'],baseline

    scan=Scan(float('inf'))
    assert not swept_path_guard(scan,.075,0.)['ready']

    scan=Scan(3.)
    for i in range(0,1440,4):
        scan.ranges[i]=float('inf')
    sparse=swept_path_guard(scan,.075,0.)
    assert sparse['ready'],sparse

    scan=Scan(3.)
    for angle in range(-3,4):
        for j in range(4):
            scan.ranges[scan.i(angle+j*.25)]=float('inf')
    gap=swept_path_guard(scan,.075,0.)
    assert not gap['ready'] and gap['reason']=='SWEEP_CONTIGUOUS_UNKNOWN_SECTOR',gap
    assert gap['largest_unknown_sectors'],gap
    biggest=gap['largest_unknown_sectors'][0]
    assert biggest['start_angle_deg']<=-2.75,biggest
    assert biggest['end_angle_deg']>=2.75,biggest
    assert biggest['ray_count']>=20,biggest
    assert biggest['unobserved_width_at_farthest_sample_m']>0,biggest

    scan=Scan(3.)
    scan.ranges[scan.i(0.)]=.36
    obs=swept_path_guard(scan,.075,0.)
    assert not obs['ready'] and obs['reason']=='MEASURED_OBSTACLE_IN_SWEPT_PATH',obs

    scan=Scan(3.)
    scan.ranges[scan.i(-15.)]=.40
    obs=swept_path_guard(scan,.075,0.)
    assert not obs['ready'],obs

    # Real Jetson evidence: a 0.111m return in a lateral direction may
    # be the bumper, tire or an external obstacle. Surface this geometry
    # without automatically treating it as safe or dropping the reading.
    scan=Scan(3.)
    scan.ranges[scan.i(60.)]=.111
    self_candidate=swept_path_guard(scan,.075,-.1)
    assert not self_candidate['ready'],self_candidate
    assert self_candidate['nearest_obstacle_detail']['angle_deg']==60.0,self_candidate
    assert self_candidate['nearest_obstacle_detail']['inside_uninflated_body_rectangle'],self_candidate

    scan=Scan(3.)
    scan.ranges[scan.i(0.)]=.111
    external=swept_path_guard(scan,.075,0.)
    assert not external['ready'],external
    assert not external['nearest_obstacle_detail']['inside_uninflated_body_rectangle'],external

    scan=Scan(3.)
    scan.ranges=[3.]*900  # incomplete frame: conservative
    assert not swept_path_guard(scan,.075,0.)['ready']

    print('PASS: clear, entirely unknown, intermittent no-return,')
    print('      contiguous unknown gap, forward + side obstacles,')
    print('      incomplete scan. Software-only; NO actuation.')
    print('Sparse no-return sample:',sparse)
    scan=Scan(3.)
    started=time.perf_counter()
    for _ in range(20):
        swept_path_guard(scan,.075,-.1)
    mean_ms=1000*(time.perf_counter()-started)/20
    print('Mean one-guard compute time: %.1f ms (20Hz mux target is 50ms)' % mean_ms)
    if mean_ms>=40:
        print('WARNING: CPU time approaches 20Hz deadline; do NOT enable motor actuation')


if __name__=='__main__':
    check()
