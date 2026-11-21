#!/usr/bin/env python3
"""
GRaCEmo ViRa — Scan Overlay Residual Verification Script
Evaluates LiDAR scan overlay against:
  1. Old Mismatched Map (apartment_map.pgm.bak_pre_regen)
  2. Regenerated Ground-Truth Map (apartment_map.pgm)
at 5 ground-truth poses:
  Pose 1: (0.0, 0.0, 0.0)      - Hallway Center / Starting Point
  Pose 2: (-2.0, 0.0, 0.0)     - Hallway West Transit
  Pose 3: (-5.0, -2.5, -1.57)  - Kitchen Target
  Pose 4: (-5.0, 2.2, 1.57)    - Bedroom Target
  Pose 5: (4.5, 3.0, 1.57)     - Study / Library Target

Residual metric: Mean and Max Euclidean distance (in meters) from scan hit endpoints
to nearest occupied map cell.
"""

import os
import sys
import math
import json
import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt

def load_boxes_from_world():
    """Return all static obstacle AABBs [xmin, xmax, ymin, ymax] from apartment_floor.world."""
    boxes = [
        # Outer walls (inner faces at +-9.0 and +-7.0)
        [-9.20, -9.00, -7.10, 7.10],
        [9.00, 9.20, -7.10, 7.10],
        [-9.10, 9.10, 7.00, 7.20],
        [-9.10, 9.10, -7.20, -7.00],

        # Dividing walls
        [-0.10, 0.10, 1.00, 7.00],
        [-0.10, 0.10, -7.00, -1.00],

        # Hallway North walls (doors at [-5.6, -4.4] and [4.4, 5.6])
        [-9.00, -5.60, 0.90, 1.10],
        [-4.40, 4.40, 0.90, 1.10],
        [5.60, 9.00, 0.90, 1.10],

        # Hallway South walls (doors at [-5.6, -4.4] and [4.4, 5.6])
        [-9.00, -5.60, -1.10, -0.90],
        [-4.40, 4.40, -1.10, -0.90],
        [5.60, 9.00, -1.10, -0.90],

        # Furniture
        [-8.90, -8.30, 2.90, 5.10],    # wardrobe
        [-4.45, -3.95, 3.15, 3.65],    # nightstand
        [-6.78, -5.22, 3.80, 6.18],    # master bed
        [-8.80, -7.20, -6.25, -1.75],  # kitchen counter
        [-1.87, -1.13, -6.56, -5.84],  # refrigerator
        [3.95, 5.05, -3.80, -3.20],    # coffee table
        [8.40, 8.80, -5.00, -3.00],    # tv unit
        [3.50, 5.50, -5.70, -4.70],    # sofa
        [8.05, 8.95, 5.30, 5.70],      # bookshelf
        [1.25, 1.75, 5.80, 6.20],      # cardboard box
    ]
    # Add dining table legs
    for lx in [-4.5 - 0.68, -4.5 + 0.68]:
        for ly in [-4.0 - 0.38, -4.0 + 0.38]:
            boxes.append([lx - 0.05, lx + 0.05, ly - 0.05, ly + 0.05])
    # Add study desk legs
    for lx in [4.5 - 0.80, 4.5 + 0.80]:
        for ly in [5.5 - 0.35, 5.5 + 0.35]:
            boxes.append([lx - 0.05, lx + 0.05, ly - 0.05, ly + 0.05])
    return boxes

def simulate_lidar_scan(pose, boxes, num_samples=360, min_angle=-math.pi, max_angle=math.pi, max_range=12.0, min_range=0.15):
    """Raycast 2D LiDAR from given pose (x, y, yaw)."""
    rx, ry, ryaw = pose
    angles = np.linspace(min_angle, max_angle, num_samples, endpoint=False)
    ranges = np.full(num_samples, max_range, dtype=np.float32)

    for i, a in enumerate(angles):
        phi = ryaw + a
        cos_p = math.cos(phi)
        sin_p = math.sin(phi)

        t_min_hit = max_range
        for b in boxes:
            xmin, xmax, ymin, ymax = b
            # Slab method
            tx1 = (xmin - rx) / cos_p if abs(cos_p) > 1e-6 else (-math.inf if rx > xmin else math.inf)
            tx2 = (xmax - rx) / cos_p if abs(cos_p) > 1e-6 else (math.inf if rx < xmax else -math.inf)
            if tx1 > tx2:
                tx1, tx2 = tx2, tx1

            ty1 = (ymin - ry) / sin_p if abs(sin_p) > 1e-6 else (-math.inf if ry > ymin else math.inf)
            ty2 = (ymax - ry) / sin_p if abs(sin_p) > 1e-6 else (math.inf if ry < ymax else -math.inf)
            if ty1 > ty2:
                ty1, ty2 = ty2, ty1

            t_near = max(tx1, ty1)
            t_far = min(tx2, ty2)

            if t_far >= t_near and t_near > min_range:
                if t_near < t_min_hit:
                    t_min_hit = t_near

        ranges[i] = t_min_hit

    return angles, ranges

def compute_map_distance_transform(pgm_path):
    img = Image.open(pgm_path)
    arr = np.array(img)
    occupied = (arr < 50)
    # distance to nearest occupied cell in pixel units
    # edt on non-occupied gives distance to 0s
    dt_pixels = distance_transform_edt(~occupied)
    return dt_pixels, arr.shape

def evaluate_residual(pose, angles, ranges, dt_pixels, shape, origin=(-9.0, -6.5), resolution=0.05, max_range=12.0, min_range=0.15):
    h, w = shape
    ox, oy = origin
    rx, ry, ryaw = pose

    valid_mask = (ranges > min_range) & (ranges < max_range - 0.05)
    valid_angles = angles[valid_mask]
    valid_ranges = ranges[valid_mask]

    if len(valid_ranges) == 0:
        return 0.0, 0.0, []

    residuals = []
    for a, r in zip(valid_angles, valid_ranges):
        phi = ryaw + a
        hx = rx + r * math.cos(phi)
        hy = ry + r * math.sin(phi)

        # map coordinates
        mx = int(round((hx - ox) / resolution))
        my = int(round((hy - oy) / resolution))
        row = h - 1 - my

        if 0 <= row < h and 0 <= mx < w:
            dist_m = dt_pixels[row, mx] * resolution
            residuals.append(dist_m)
        else:
            # point falls outside map bounds
            # compute distance to nearest valid boundary
            dist_x = max(0.0, ox - hx, hx - (ox + w * resolution))
            dist_y = max(0.0, oy - hy, hy - (oy + h * resolution))
            residuals.append(math.hypot(dist_x, dist_y) + 0.1)

    res_arr = np.array(residuals)
    return float(np.mean(res_arr)), float(np.max(res_arr)), residuals

def run_overlay_verification(old_pgm, new_pgm, output_json, origin_old=(-9.0, -6.5), origin_new=(-9.20, -7.20)):
    boxes = load_boxes_from_world()
    dt_old, shape_old = compute_map_distance_transform(old_pgm)
    dt_new, shape_new = compute_map_distance_transform(new_pgm)

    test_poses = [
        {"name": "Pose 1: Hallway Center (0.0, 0.0)", "coords": (0.0, 0.0, 0.0)},
        {"name": "Pose 2: Hallway West (-2.0, 0.0)", "coords": (-2.0, 0.0, 0.0)},
        {"name": "Pose 3: Kitchen Target (-5.0, -2.5)", "coords": (-5.0, -2.5, -1.57)},
        {"name": "Pose 4: Bedroom Target (-5.0, 2.2)", "coords": (-5.0, 2.2, 1.57)},
        {"name": "Pose 5: Study Target (4.5, 3.0)", "coords": (4.5, 3.0, 1.57)},
    ]

    results = []
    print("================================================================================")
    print("  SCAN OVERLAY RESIDUAL VERIFICATION AT 5 GROUND-TRUTH POSES")
    print("================================================================================")
    print(f"{'Pose':<38} | {'Old Map Mean (Max)':<20} | {'New Map Mean (Max)':<20} | {'Improvement'}")
    print("-" * 88)

    for p in test_poses:
        angles, ranges = simulate_lidar_scan(p["coords"], boxes)
        old_mean, old_max, _ = evaluate_residual(p["coords"], angles, ranges, dt_old, shape_old, origin=origin_old)
        new_mean, new_max, _ = evaluate_residual(p["coords"], angles, ranges, dt_new, shape_new, origin=origin_new)
        diff_mean = old_mean - new_mean

        p_result = {
            "pose_name": p["name"],
            "coords": p["coords"],
            "old_map": {"mean_residual_m": round(old_mean, 4), "max_residual_m": round(old_max, 4)},
            "new_map": {"mean_residual_m": round(new_mean, 4), "max_residual_m": round(new_max, 4)},
            "residual_reduction_m": round(diff_mean, 4)
        }
        results.append(p_result)
        print(f"{p['name']:<38} | {old_mean:.3f}m ({old_max:.3f}m)    | {new_mean:.3f}m ({new_max:.3f}m)    | -{diff_mean*100:.1f} cm")

    print("================================================================================")
    with open(output_json, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Results written to: {output_json}")
    return results

if __name__ == "__main__":
    old_p = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm.bak_pre_regen"
    new_p = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/apartment_map_regenerated.pgm"
    out_j = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/scan_overlay_verification_final.json"
    run_overlay_verification(old_p, new_p, out_j)
    # Also write to legacy path
    import shutil
    shutil.copyfile(out_j, "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/scan_overlay_verification.json")

