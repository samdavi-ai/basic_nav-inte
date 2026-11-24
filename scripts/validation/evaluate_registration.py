"""Compare base-frame visual odometry to separately recorded Gazebo model poses."""
from collections import defaultdict
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation


def stats(values):
    a=np.asarray(values)
    if not a.size:
        return None
    return dict(median=float(np.median(a)),p95=float(np.percentile(a,95)),max=float(max(a)))


def matrix(row):
    m=np.eye(4)
    m[:3,:3]=Rotation.from_quat(row['q']).as_matrix()
    m[:3,3]=row['xyz']
    return m


def evaluate(path):
    groups=defaultdict(list)
    with path.open() as f:
        for line in f:
            r=json.loads(line)
            groups[r['kind']].append(r)
    truth=sorted(groups['truth'],key=lambda r:r['t'])
    times=np.array([r['t'] for r in truth])
    poses={r['t']:r for r in groups['odom']}
    info={r['t']:r for r in groups['info']}
    stamps=[r['t'] for r in groups['rgb']]
    depth={r['t'] for r in groups['depth']}
    unique=sorted(set(stamps))
    # The bridge publishes RGB and CameraInfo in the same image callback. The
    # small calibration message can survive a transport loss of the RGB payload.
    frame_stamps = set(unique) | {r['t'] for r in groups['/camera/camera_info']}
    valid=[t in info and not info[t]['lost'] and t in poses for t in unique]
    align=None
    errors=[]
    angles=[]
    for t,ok in zip(unique,valid):
        if not ok or not times.size:
            continue
        i=np.argmin(abs(times-t))
        if abs(times[i]-t)>.05:
            continue
        g,e=matrix(truth[i]),matrix(poses[t])
        if align is None:
            align=g@np.linalg.inv(e)
        residual=np.linalg.inv(g)@align@e
        errors.append(np.linalg.norm(residual[:3,3]))
        angles.append(np.degrees(Rotation.from_matrix(residual[:3,:3]).magnitude()))
    depth_times=np.asarray(sorted(depth))
    nearest_skew=[float(np.min(abs(depth_times-t))*1000) for t in unique] if depth else []
    depth_summary=[r['quantiles'][2] for r in groups['depth'] if r['quantiles']]
    graph=groups.get('graph',[])
    from collections import Counter
    import re
    logpath=path.parent/'1_gracemo_rgbd_slam.log'
    failures=Counter()
    if logpath.exists():
        for message in re.findall(r'Registration failed: "([^"]*)"',logpath.read_text()):
            failures[next((k for k in ('Missing correspondences','Not enough inliers','Too low inliers after bundle adjustment','Not enough features') if message.startswith(k)),'other')]+=1
    return dict(rgb_sim_hz=(len(unique)-1)/(unique[-1]-unique[0]) if len(unique)>1 else None,
                depth_median_m=stats(depth_summary),nearest_depth_skew_ms=stats(nearest_skew),
                image_mean_abs_change=stats([r['delta'] for r in groups['rgb'] if r['delta'] is not None]),
                exactly_repeated_images=sum(r['delta']==0 for r in groups['rgb']),
                rejection_log_counts=dict(failures),graph_events=len(graph),
                keyframe_decisions=sum(r['key_frame_added'] for r in info.values()),
                graph_matches=[dict(t=r['t'],ref=r['ref'],loop=r['loop'],proximity=r['proximity'],
                    inliers=r['stats'].get('Loop/Visual_inliers/'),iterations=r['stats'].get('Loop/Optimization_iterations/'))
                    for r in graph if r['loop'] or r['proximity']],
                interval=[min(unique),max(unique)],counts={k:len(v) for k,v in groups.items()},
                rgb_duplicates=len(stamps)-len(unique),out_of_order=int(sum(np.diff(stamps)<0)),
                missing_same_stamp_depth=len(set(unique)-depth),valid_depth_fraction=stats([x['valid'] for x in groups['depth']]),
                rgb_calibration=groups['/camera/camera_info'][0],depth_calibration=groups['/camera/depth/camera_info'][0],
                availability=sum(valid)/len(valid), observed_camera_stamps=len(frame_stamps),
                missing_rgb_at_camera_stamp=len(frame_stamps)-len(unique),
                observed_camera_coverage=sum(valid)/len(frame_stamps),
                matched=len(errors),valid_without_truth=sum(valid)-len(errors),
                loss_transitions=sum(a and not b for a,b in zip(valid,valid[1:])),
                recovery_transitions=sum(not a and b for a,b in zip(valid,valid[1:])),
                translation_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                rotation_rmse_deg=float(np.sqrt(np.mean(np.square(angles)))) if angles else None,
                translation_m=stats(errors),rotation_deg=stats(angles),
                latency_s=stats([r['time_estimation'] for r in info.values()]),
                matches=stats([r['matches'] for r in info.values()]),features=stats([r['features'] for r in info.values()]))


if __name__=='__main__':
    print(json.dumps(evaluate(Path(sys.argv[1])),indent=2))
