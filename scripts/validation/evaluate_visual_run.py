"""Compare visual camera poses to independent, timestamp-matched Gazebo truth."""
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

folder = Path(sys.argv[1])
def load(name):
    return [json.loads(line) for line in (folder/name).read_text().splitlines()]

rows = load("visual_resources.jsonl")
samples = [r["perception"]["sample"] for r in rows if (r.get("perception") or {}).get("sample")]
def stats(values):
    return {"p50":float(np.median(values)),"p95":float(np.percentile(values,95))} if values else None

summary = dict(samples=len(samples),max_tree_pss_bytes=max(sum(r["pss_bytes"].values()) for r in rows),
    max_perception_rss_bytes=max(x["process_rss_bytes"] for x in samples),
    detector_ms=stats([x["detector_ms"] for x in samples]),tracking_ms=stats([x["tracking_ms"] for x in samples]),
    pair_wall_hz=samples[-1]["acquired_pair_hz"],processed_wall_hz=samples[-1]["processed_hz"],
    sampled_localized_classes=dict(Counter(d["class_name"] for x in samples for d in x["observations"])),
    max_sync_skew_ms=max(x["sync_skew_ms"] for x in samples),
    k_http_ms=stats([r["k_http_ms"] for r in rows if "k_http_ms" in r]),
    active_trace_samples=sum(bool(x.get("trace_id")) for x in samples))

def transform(row):
    mat = np.eye(4)
    mat[:3,:3] = Rotation.from_quat(row[2]).as_matrix()
    mat[:3,3] = row[1]
    return mat

if (folder/"camera_mount.jsonl").exists() and any("visual_odometry" in x for x in samples):
    ground,mount = load("visual_truth.jsonl"),load("camera_mount.jsonl")
    times = [np.array([i[0] for i in group]) for group in (ground,mount)]
    registration = None
    errors = []
    angles = []
    unique = {x["image_stamp_ns"]: x for x in samples if x.get("image_stamp_ns") is not None}
    valid_count = 0
    for x in unique.values():
        vo = x.get("visual_odometry",{})
        if not x.get("ready") or vo.get("state") not in ("tracking","relocalized"):
            continue
        valid_count += 1
        estimated = x.get("visual_odometry",{}).get("optical_pose")
        if estimated is None:
            continue
        t = x["image_stamp_ns"]/1e9
        if not all(v.size for v in times):
            continue
        inds = [np.argmin(abs(v-t)) for v in times]
        if max(abs(v[i]-t) for v,i in zip(times,inds))>.10:
            continue
        actual = transform(ground[inds[0]]) @ transform(mount[inds[1]])
        if registration is None:
            registration = actual @ np.linalg.inv(estimated)
        relative = actual[:3,:3].T @ (registration @ estimated)[:3,:3]
        angles.append(float(np.degrees(Rotation.from_matrix(relative).magnitude())))
        errors.append(float(np.linalg.norm((registration @ estimated)[:3,3] - actual[:3,3])))
    summary.update(unique_status_captures=len(unique),
                   sampled_tracking_availability=valid_count/len(unique) if unique else None,
                   truth_matched_fraction=len(errors)/len(unique) if unique else None,
                   visual_states=dict(Counter(x["visual_odometry"]["state"] for x in samples if "visual_odometry" in x)),
                   rotation_rmse_deg=float(np.sqrt(np.mean(np.square(angles)))) if angles else None,
                   matched_pose_count=len(errors), translation_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                   max_error_m=max(errors,default=None))
print(json.dumps(summary,indent=2))
