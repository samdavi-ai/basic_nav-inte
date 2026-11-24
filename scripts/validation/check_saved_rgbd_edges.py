"""Measure color/depth boundary agreement in the untextured saved camera frames."""
import json
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np


def measure(folder):
    distances = []
    used = 0
    with sqlite3.connect((folder / "map.db").resolve().as_uri() + "?mode=ro", uri=True) as db:
        for color, depth in db.execute("SELECT image,depth FROM Data WHERE length(depth)>0"):
            gray = cv2.imdecode(np.frombuffer(color, np.uint8), cv2.IMREAD_GRAYSCALE)
            packed = cv2.imdecode(np.frombuffer(depth, np.uint8), cv2.IMREAD_UNCHANGED)
            if packed.ndim != 3 or packed.shape[2] != 4 or packed.dtype != np.uint8:
                raise ValueError("Expected RTAB-Map lossless packed float depth")
            z = packed.view("<f4").reshape(packed.shape[:2])
            h, w = z.shape
            if gray.shape != (2*h, 2*w):
                raise ValueError("Expected collocated cameras with a 2:1 resolution ratio")
            valid = np.isfinite(z) & (z > 0)
            # Edge locations lie midway between the adjacent depth-pixel centers.
            xyz = []
            for dy, dx in ((0, 1), (1, 0)):
                a = z[:h-dy, :w-dx]
                b = z[dy:, dx:]
                mask = valid[:h-dy, :w-dx] & valid[dy:, dx:]
                jump = np.zeros_like(a)
                np.subtract(a, b, out=jump, where=mask)
                y, x = np.nonzero(mask & (np.abs(jump) > .35))
                xyz.extend(zip(2*x+dx, 2*y+dy))
            edges = cv2.Canny(gray, 30, 80)
            if xyz and edges.any():
                distance = cv2.distanceTransform(255-edges, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
                x, y = np.asarray(xyz).T
                distances.extend(distance[y, x].tolist())
                used += 1
    if not distances:
        raise ValueError("No observable finite-depth discontinuities")
    return dict(keyframes=used, depth_boundaries=len(distances),
                distance_to_nearest_rgb_edge_px=dict(zip(("median","p95","max"),
                    np.percentile(distances, (50,95,100)).tolist())),
                within_two_rgb_pixels=float(np.mean(np.asarray(distances) <= 2)))


if __name__ == "__main__":
    print(json.dumps(measure(Path(sys.argv[1])), indent=2))
