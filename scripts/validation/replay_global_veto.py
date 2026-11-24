"""Find the first swept rectangle/cell intersection in each recorded global plan."""
import argparse
import base64
import gzip
import json
import zlib

import numpy as np
from scipy.spatial import cKDTree
from rectangular_guard import RectangularGuard, cell_intersects


def witness(plan, grid):
    path = np.asarray(plan['points'])
    res = grid['resolution']
    cells = np.frombuffer(zlib.decompress(base64.b64decode(grid['cells_zlib_b64'])),
                          np.uint8).reshape(grid['size'][::-1])
    iy, ix = np.where(cells >= 254)
    centers = grid['origin'][:2] + (np.column_stack((ix,iy))+.5)*res
    tree = cKDTree(centers)
    tangent = path.copy()
    dx, dy = np.diff(path[:,:2],axis=0).T
    tangent[:-1,2] = np.where(np.hypot(dx,dy)>1e-7,np.arctan2(dy,dx),path[:-1,2])
    result = dict(plan=plan['t'], grid=grid['published'])
    for label, poses in [('published',path), ('tangent',tangent)]:
        for pose in RectangularGuard.interpolate(poses,.36,res/2):
            for i in tree.query_ball_point(pose[:2], np.hypot(.36,.35)+res/2**.5+res/4):
                if cell_intersects(centers[i:i+1],pose,(.36,.35),res,res/4):
                    return dict(**result,orientation=label,pose=pose.tolist(),
                                cell_center=centers[i].tolist(),cost=int(cells[iy[i],ix[i]]))
    return dict(**result,cell_center=None)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace')
    args = parser.parse_args()
    results, grid = [], None
    with gzip.open(args.trace,'rt') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'global_costmap':
                grid = row
            elif row['kind'] == 'plan' and grid and row['points']:
                results.append(witness(row,grid))
    print(json.dumps(results,indent=2))
