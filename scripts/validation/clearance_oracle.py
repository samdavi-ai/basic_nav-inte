"""Independent configuration-space oracle for a rectangular body and a box."""
import argparse
import gzip
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

CORNERS = np.array([[-.36,-.35], [-.36,.35], [.36,.35], [.36,-.35]])
RADIUS = math.hypot(.36, .35)


def clearance(pose, bounds):
    """Signed distance to the boundary of wall minus rotated footprint."""
    x, y, h = pose
    lo_x, lo_y, hi_x, hi_y = bounds
    wall = np.array([[lo_x,lo_y],[lo_x,hi_y],[hi_x,hi_y],[hi_x,lo_y]])
    c, s = math.cos(h), math.sin(h)
    offsets = CORNERS @ np.array([[c,s],[-s,c]])
    cloud = (wall[:,None,:] - offsets).reshape(-1,2)
    hull = ConvexHull(cloud)
    boundary = cloud[hull.vertices]
    edges = np.roll(boundary,-1,axis=0) - boundary
    d = np.array([x,y]) - boundary
    u = np.clip(np.sum(d*edges,axis=1)/np.sum(edges*edges,axis=1),0,1)
    distance = np.linalg.norm(d-u[:,None]*edges,axis=1).min()
    inside = np.max(hull.equations[:,:2] @ [x,y] + hull.equations[:,2]) < 0
    return float(-distance if inside else distance)


def sweep(samples, bounds, precision=.00002):
    """Bound the minimum over piecewise-linear position and shortest-arc heading."""
    if len(samples)<2 or precision<=0 or np.any(np.diff(samples[:,0])<=0):
        raise ValueError("Need strictly increasing samples and positive precision")
    poses = samples[:,1:].copy()
    poses[:,2] = np.unwrap(poses[:,2])
    values = np.array([clearance(p,bounds) for p in poses])
    i = int(np.argmin(values))
    best = values[i]
    witness = samples[i].tolist()
    raw = float(best)
    lower = float('inf')
    for i,(a,b) in enumerate(zip(poses,poses[1:])):
        t0,t1 = samples[i:i+2,0]
        pending = [(a,b,t0,t1,values[i],values[i+1])]
        while pending:
            start,end,ts,te,ds,de = pending.pop()
            dx,dy,dh = end-start
            travel = math.hypot(dx,dy)+RADIUS*abs(dh)
            bound = min(ds,de)-travel/2
            if bound >= best-precision:
                lower = min(lower,bound)
                continue
            mid = (start+end)/2
            tm = (ts+te)/2
            dm = clearance(mid,bounds)
            if dm < best:
                best,witness = dm,[tm,*mid]
            pending.extend([(start,mid,ts,tm,ds,dm),(mid,end,tm,te,dm,de)])
    return dict(raw_min_m=raw,lower_m=lower,upper_m=best,witness=witness,
                interpolation_tolerance_m=precision, obstacle_bounds=bounds)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    args=parser.parse_args()
    with gzip.open(args.trace,'rt') as f:
        if args.trace.name == 'truth.jsonl.gz':
            poses = [json.loads(line) for line in f]
        else:
            poses = [r['truth'] for l in f if (r:=json.loads(l))['kind']=='motion' and r['truth']]
    samples=np.array(poses)
    samples=samples[np.r_[True,np.any(np.diff(samples,axis=0)!=0,axis=1)]]
    print(json.dumps(sweep(samples,[5.6,.9,9.,1.1]),indent=2))
