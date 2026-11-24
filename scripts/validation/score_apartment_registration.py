"""Evaluate fixed Gazebo views and a completed patrol using truth only after capture."""
import argparse
import json
from collections import defaultdict
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from evaluate_registration import matrix
from adapters.perception.detector import ObjectDetector
from adapters.perception.depth_estimator import DepthEstimator


def room(x,y):
    if abs(y)<=1:
        return "hallway"
    if y>0:
        return "bedroom" if x<0 else "study"
    return "kitchen" if x<0 else "living"


def median(values):
    return float(np.median(values)) if len(values) else None


def score_patrol(folder):
    rows=defaultdict(list)
    for line in (folder/"registration.jsonl").open():
        r=json.loads(line);rows[r["kind"]].append(r)
    lookup=lambda k:{r["t"]:r for r in rows[k]}
    truth=sorted(rows["truth"],key=lambda r:r["t"])
    times=np.array([r["t"] for r in truth])
    rgb,info,odom=map(lookup,("rgb","info","odom"))
    samples=defaultdict(list)
    align=None
    for t in sorted(set(rgb)|set(lookup("/camera/camera_info"))):
        i=int(np.argmin(abs(times-t)))
        if abs(times[i]-t)>.05:continue
        g=matrix(truth[i]);r=info.get(t,{})
        valid=t in rgb and t in odom and r.get("lost") is False
        delta=None
        if valid:
            e=matrix(odom[t])
            if align is None:align=g@np.linalg.inv(e)
            diff=np.linalg.inv(g)@align@e
            delta=[float(np.linalg.norm(diff[:3,3])),float(np.degrees(Rotation.from_matrix(diff[:3,:3]).magnitude()))]
        samples[room(*g[:2,3])].append(dict(t=t,truth=g[:2,3].tolist(),error=delta,
                visual=(align@e)[:2,3].tolist() if valid else None,
                orb=rgb.get(t,{}).get("features"),features=r.get("features"),inliers=r.get("inliers")))
    def stats(items):
        errors=[r['error'] for r in items if r['error'] is not None]
        return dict(camera_stamps=len(items),scored=len(errors),coverage=len(errors)/len(items),
                    rmse_m_deg=np.sqrt(np.mean(np.square(errors),axis=0)).tolist() if errors else None,
                    medians={k:median([r[k] for r in items if r[k] is not None]) for k in ("features","orb","inliers")})
    return samples, {k:stats(v) for k,v in samples.items()}|{"whole_route":stats(sum(samples.values(),[]))}


def describe(folder):
    detector=ObjectDetector(confidence_threshold=.35)
    if detector.ort_session is None and detector.yolo_model is None:
        raise RuntimeError("Semantic model did not load")
    fov=json.loads((folder/"capture.json").read_text()).get("fov",1.15)
    desc=[];orb=cv2.ORB_create()
    for p in sorted(folder.glob("*.png")):
        if not p.with_suffix(".npy").exists():
            continue
        color=cv2.imread(str(p));gray=cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)
        h,w=gray.shape;f=w/(2*np.tan(fov/2))
        z=np.load(p.with_suffix(".npy"));dh,dw=z.shape
        depth=DepthEstimator();depth.update_camera_info(SimpleNamespace(k=[f,0,w/2,0,f,h/2,0,0,1],width=w,height=h))
        pts=orb.detect(gray);grid=np.zeros((3,4),int);valid_geometry=0
        for pt in pts:
            x,y=pt.pt;grid[int(3*y/h),int(4*x/w)]+=1
            v=z[int(y*dh/h),int(x*dw/w)]
            valid_geometry+=int(np.isfinite(v) and .05<v<=15.)
        objects=[]
        for label,confidence,box in detector.detect(color):
            distance,xyz=depth.estimate_3d_position(box,z,w,h)
            objects.append(dict(label=label,confidence=float(confidence),box_fraction=box.width*box.height/(w*h),
                                depth_m=distance,optical_xyz=[xyz.x,xyz.y,xyz.z] if xyz else None))
        desc.append(dict(view=p.stem,orb=len(pts),spatial_bins=grid.tolist(),occupied_bins=int(np.count_nonzero(grid)),
                         points_with_valid_depth=valid_geometry,brightness=float(gray.mean()),
                         laplacian_variance=float(cv2.Laplacian(gray,cv2.CV_32F).var()),objects=objects))
    return desc


if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("root",type=Path);ap.add_argument("patrol",type=Path)
    args=ap.parse_args();root=args.root
    samples,summary=score_patrol(args.patrol)
    (root/"registration/room-scores.json").write_text(json.dumps(summary,indent=2))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(7,5))
    ordered=sorted(sum(samples.values(),[]),key=lambda r:r["t"])
    ax.plot(*np.array([r['truth'] for r in ordered]).T,color="0.55",label="Gazebo reference")
    visual=np.array([r['visual'] for r in ordered if r['visual'] is not None])
    if visual.size:ax.scatter(*visual.T,s=8,label="Scored visual pose",c="darkorange")
    ax.set(xlabel="World x (m)",ylabel="World y (m)",aspect="equal",title="Original apartment patrol")
    ax.legend();fig.tight_layout();fig.savefig(root/"registration/patrol-trajectory.png");plt.close(fig)
    for folder in sorted(root.glob("*/fixed-*")):
        if (folder/"capture.json").is_file():
            (folder/"measurements.json").write_text(json.dumps(describe(folder),indent=2))
