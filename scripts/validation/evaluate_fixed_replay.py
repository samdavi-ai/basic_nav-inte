"""Compare the native RTAB-Map estimator on pairs of real, fixed Gazebo camera views."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def evaluate(binary, capture, out, profiles):
    manifest = json.loads((capture / "capture.json").read_text())
    if not manifest.get("stereo"):
        raise ValueError("Expected the nearby-view fixture")
    out.mkdir(exist_ok=False, parents=True)
    results = []
    sequence = [0, 1, 2, 1, 0, None, None, 0, 1, 2, 1, 0]
    with tempfile.TemporaryDirectory() as scratch:
        tmp = Path(scratch)
        for room, (_, _, origin_yaw) in manifest["views"].items():
            images = {}
            for i in (0, 1, 2):
                stem = f"{room}_{i}"
                z = np.load(capture / (stem + ".npy"))
                depth = tmp / (stem + ".tiff")
                if not cv2.imwrite(str(depth), z):
                    raise OSError(depth)
                images[i] = [str(capture / (stem + ".png")), str(depth)]
            # Controlled diagnostic input loss. Returned images are original Gazebo captures.
            blank = tmp / "blank.png"
            invalid = tmp / "invalid.tiff"
            cv2.imwrite(str(blank), np.zeros_like(cv2.imread(images[0][0])))
            cv2.imwrite(str(invalid), np.full_like(z, np.nan))
            images[None] = [str(blank), str(invalid)]
            for profile in profiles:
                stem = out / f"{room}-{profile}"
                args = [str(binary), profile, str(stem.with_suffix(".csv"))]
                for i in sequence:
                    args.extend(images[i])
                with stem.with_suffix(".log").open("w") as log:
                    outcome=subprocess.run(args, stdout=log, stderr=subprocess.STDOUT,
                                           env=dict(os.environ,VIRA_REPLAY_FOV=str(manifest.get("fov",1.15))))
                if outcome.returncode:
                    results.append(dict(room=room,profile=profile,backend_exit=outcome.returncode))
                    continue
                with stem.with_suffix(".csv").open() as stream:
                    rows = list(csv.DictReader(stream))
                errors, before, after = [], [], []
                for step, (i, row) in enumerate(zip(sequence, rows, strict=True)):
                    if i is None or row["valid"] != "1":
                        continue
                    # Known fixed-fixture separation, scored after the estimator exits.
                    pos = [.06*i*math.cos(origin_yaw), -.06*i*math.sin(origin_yaw), 0]
                    rot = Rotation.from_euler("z", (0, .045, -.045)[i])
                    est_rot = Rotation.from_euler("xyz", [float(row[k]) for k in ("roll", "pitch", "yaw")])
                    errors.append([np.linalg.norm([float(row[k])-v for k,v in zip(("x","y","z"),pos)]),
                                   np.degrees((rot.inv()*est_rot).magnitude())])
                    (before if step < 5 else after).append(step)
                results.append(dict(room=room, profile=profile, valid_pre_fault=len(before), valid_post_fault=len(after),
                    clean_frames=10, false_valid_during_fault=sum(rows[i]["valid"]=="1" for i in (5,6)),
                    first_valid_post_fault=after[0]-7 if after else None,
                    rmse_m_deg=np.sqrt(np.mean(np.square(errors),axis=0)).tolist() if errors and rows[0]["valid"]=="1" else None,
                    initialized_at_first_view=rows[0]["valid"]=="1"))
    (out / "summary.json").write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    for arg in ("binary","capture","out"):
        parser.add_argument(arg,type=Path)
    parser.add_argument("--profiles", nargs="+", choices=("baseline","distributed","octree","metric","patch"),
                        default=["baseline","distributed","metric","patch"])
    a=parser.parse_args()
    evaluate(a.binary,a.capture,a.out,a.profiles)
