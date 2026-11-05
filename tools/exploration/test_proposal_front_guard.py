#!/usr/bin/env python3
"""ROS-free safety regression for physical LiDAR exploration PROPOSALS only."""
import math
from front_scan_guard import front_scan_guard, proposal_front_guard


def main():
    angles = [-math.pi + i*(2*math.pi/1440) for i in range(1440)]

    # Approximate supplied 1.95m test: every central +/-5deg hit wall,
    # but 25% of the broader forward rays have no echo.
    r = [float('inf')]*1440
    for i, a in enumerate(angles):
        if abs(math.degrees(a)) <= 20:
            r[i] = 1.95
            if abs(math.degrees(a)) > 5 and i % 3 == 0:
                r[i] = float('inf')
    legacy = front_scan_guard(-math.pi,2*math.pi/1440,r,0.0,100.0,
                              stop_distance=.8,valid_fraction_min=.9)
    proposal = proposal_front_guard(-math.pi,2*math.pi/1440,r,0.0,100.0)
    assert not legacy['ready'], legacy
    assert proposal['ready'], proposal
    assert proposal['outer_valid_fraction'] < .9, proposal

    # Real measured obstacle at a side angle must VETO the proposal.
    n = min(range(len(r)), key=lambda i:abs(math.degrees(angles[i])-16))
    r[n] = .36
    blocked = proposal_front_guard(-math.pi,2*math.pi/1440,r,0.0,100.0)
    assert not blocked['ready'] and blocked['reason']=='measured_forward_obstacle', blocked

    # Missing central evidence is always a hard rejection.
    r[n] = 1.95
    for i, a in enumerate(angles):
        if abs(math.degrees(a)) <= 5:
            r[i] = float('inf')
    unknown = proposal_front_guard(-math.pi,2*math.pi/1440,r,0.0,100.0)
    assert not unknown['ready'], unknown

    # Fully unknown scan is never accepted as a free corridor.
    all_unknown=proposal_front_guard(-math.pi,2*math.pi/1440,
                                    [float('inf')]*1440,0.0,100.0)
    assert not all_unknown['ready'], all_unknown

    print('PASS: rejects wide 90% quota when center measured,')
    print('      respects measured side obstacle, and fails closed on missing')
    print('      central observations and fully unknown LiDAR scans.')
    print('This validates NON-ACTUATING proposal logic only.')


if __name__=='__main__':
    main()
