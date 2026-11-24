"""Read the RTAB-Map optimized graph and compare matching keyframes to Gazebo truth."""
import json
from pathlib import Path
import sqlite3
import sys
import zlib

import numpy as np
from scipy.spatial.transform import Rotation
from summarize_relocalization import load, nearest


def transform(values):
    mat=np.eye(4)
    mat[:3,:]=values.reshape(3,4)
    return mat


def evaluate(folder):
    observations=load(folder)
    truth=observations['truth']
    events = {(r['ref'], r['loop']): r for r in observations['graph'] if r['loop']}
    database=(folder/'map.db').resolve()
    with sqlite3.connect(database.as_uri()+'?mode=ro',uri=True) as db:
        nodes={n:(t,transform(np.frombuffer(p,'<f4'))) for n,t,p in db.execute('SELECT id,stamp,pose FROM Node')}
        ids,values=db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
        links=list(db.execute('SELECT from_id,to_id,type,transform FROM Link'))
        metric_features, = db.execute('SELECT count(*) FROM Feature WHERE depth_z > 0').fetchone()
    optimized={}
    if ids and values:
        indices=np.frombuffer(zlib.decompress(ids),'<i4')
        poses=np.frombuffer(zlib.decompress(values),'<f4').reshape(-1,12)
        optimized={int(i):transform(p) for i,p in zip(indices,poses,strict=True)}
    common=sorted((i for i in nodes.keys() & optimized.keys() if nearest(truth,nodes[i][0]) is not None),
                  key=lambda i:nodes[i][0])
    coverage = None
    if common:
        path = np.asarray([nearest(truth, nodes[i][0])[:3, 3] for i in common])
        length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
        coverage = dict(truth_bbox_min_m=path.min(axis=0).tolist(), truth_bbox_max_m=path.max(axis=0).tolist(),
                        matched_keyframe_path_length_m=length,
                        keyframes_per_meter=len(common)/length if length else None)
    neighbors = {i: set() for i in optimized}
    for a,b,_,_ in links:
        if a in neighbors and b in neighbors:
            neighbors[a].add(b)
            neighbors[b].add(a)
    remaining = set(neighbors)
    components = []
    while remaining:
        pending = [remaining.pop()]
        count = 0
        while pending:
            node = pending.pop()
            count += 1
            unseen = neighbors[node] & remaining
            remaining.difference_update(unseen)
            pending.extend(unseen)
        components.append(count)
    def errors(poses):
        if not common:
            return None
        start=common[0]
        align=nearest(truth,nodes[start][0])@np.linalg.inv(poses[start])
        residual=[np.linalg.inv(nearest(truth,nodes[i][0]))@align@poses[i] for i in common]
        return dict(translation_rmse_m=float(np.sqrt(np.mean([np.sum(m[:3,3]**2) for m in residual]))),
                    rotation_rmse_deg=float(np.sqrt(np.mean([np.degrees(Rotation.from_matrix(m[:3,:3]).magnitude())**2 for m in residual]))))
    candidates=[]
    for a,b,k,measurement in links:
        if k!=1 or a not in nodes or b not in nodes:
            continue
        pa,pb=[nearest(truth,nodes[i][0]) for i in (a,b)]
        earlier, later = sorted((nodes[a][0], nodes[b][0]))
        excursion = []
        if pb is not None:
            excursion = [np.linalg.norm(np.asarray(row['xyz']) - pb[:3, 3])
                         for row in truth if earlier <= row['t'] <= later]
        edge_error = None
        if pa is not None and pb is not None and measurement:
            measured = transform(np.frombuffer(measurement, '<f4'))
            residual = np.linalg.inv(np.linalg.inv(pa) @ pb) @ measured
            edge_error = dict(translation_m=float(np.linalg.norm(residual[:3, 3])),
                              rotation_deg=float(np.degrees(Rotation.from_matrix(residual[:3, :3]).magnitude())))
        stats = events.get((a,b), {}).get('stats', {})
        candidates.append(dict(source=a,target=b,dt=abs(nodes[a][0]-nodes[b][0]),
                 visual_inliers=stats.get('Loop/Visual_inliers/'),
                 optimization_iterations=stats.get('Loop/Optimization_iterations/'),
                 relative_constraint_error=edge_error,
                 max_excursion_from_target_m=float(max(excursion)) if excursion else None,
                 truth_separation_m=float(np.linalg.norm(pa[:3,3]-pb[:3,3])) if pa is not None and pb is not None else None))
    return dict(common_keyframes=common, optimized_nodes=len(optimized),total_nodes=len(nodes),
                observations_with_positive_depth=metric_features, truth_coverage=coverage,
                optimized_component_sizes=sorted(components, reverse=True),
                raw=errors({i:p for i,(_,p) in nodes.items()}),optimized=errors(optimized),
                type1_constraints=candidates)


if __name__=='__main__':
    print(json.dumps(evaluate(Path(sys.argv[1])),indent=2))
