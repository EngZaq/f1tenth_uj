# Port of EngZaq/f1tenth_uj_sim main: tools/mission/map_coverage.py
# Exact observed-corridor topology gate; uses real /map data, never simulator truth.
"""Observed occupancy-only topology gate before ending an exploration lap.

This is a necessary map-coverage check, not a body/steering feasibility audit.
Unknown cells stay blocked. The planner performs the full subsequent audit.
"""
import math
import numpy as np
from scipy import ndimage as ndi


def corridor_coverage(data, width, height, resolution, clearance=.30,
                      minimum_interior_area=.5):
    if (width<=0 or height<=0 or not math.isfinite(resolution) or resolution<=0 or
            not math.isfinite(clearance) or clearance<=0):
        raise ValueError('Invalid occupancy grid dimensions or resolution')
    values=np.asarray(data)
    if (values.size!=width*height or not np.isfinite(values).all() or
            np.any(values < -1) or np.any(values > 100)):
        raise ValueError('Invalid occupancy grid data')
    # Match map_saver's 0.25 free threshold; unknown (-1) remains blocked.
    free=(values.reshape(height,width)>=0)&(values.reshape(height,width)<=25)
    labels,count=ndi.label(free,np.ones((3,3),bool))
    result={'closed':False,'known_free_cells':int(free.sum()),'safe_cells':0,
            'largest_interior_area_m2':0.,'clearance_m':clearance}
    if not count:
        return result
    sizes=np.bincount(labels.ravel());sizes[0]=0
    region=labels==int(sizes.argmax())
    safe=ndi.distance_transform_edt(region)*resolution >= clearance-1e-8
    result['safe_cells']=int(safe.sum())
    holes=ndi.binary_fill_holes(safe)&~safe
    hole_labels,hole_count=ndi.label(holes,np.ones((3,3),bool))
    if hole_count:
        hole_sizes=np.bincount(hole_labels.ravel());hole_sizes[0]=0
        result['largest_interior_area_m2']=float(hole_sizes.max()*resolution**2)
        result['closed']=result['largest_interior_area_m2']>=minimum_interior_area
    return result
