#!/usr/bin/env python3
"""
Analyze apartment_floor.world and apartment_map.pgm:
1. Search world SDF elements (boxes, walls, doors, furniture) for repeating geometry
   and features with ~0.95m spacing (0.85m - 1.05m).
2. Analyze apartment_map.pgm via autocorrelation, Hough transforms, and slice profiles
   to check for 0.95m periodicities, doorway widths, and corridor features.
"""

import os
import re
import math
import numpy as np
from PIL import Image

def analyze_sdf(sdf_path):
    print("=== SDF GEOMETRY ANALYSIS ===")
    with open(sdf_path, "r") as f:
        content = f.read()

    # Find models, links, visuals, collisions
    # Look for poses and box sizes
    box_pattern = re.compile(r'<box>\s*<size>([^<]+)</size>', re.DOTALL)
    boxes = []
    for m in box_pattern.finditer(content):
        dims = [float(v) for v in m.group(1).split()]
        boxes.append(dims)

    print(f"Total box collision/visual geometries in SDF: {len(boxes)}")
    # Check dimensions matching ~0.95m
    close_to_095 = [b for b in boxes if any(0.85 <= dim <= 1.05 for dim in b)]
    print(f"Box geometries with at least one dimension in [0.85, 1.05] m: {len(close_to_095)}")
    for b in close_to_095[:15]:
        print(f"  Box size: {b}")

    # Search for wall models and poses
    # Extract model tags with name and pose
    model_pattern = re.compile(r'<model name=[\'"]([^\'"]+)[\'"]>(.*?)</model>', re.DOTALL)
    models = []
    for match in model_pattern.finditer(content):
        m_name = match.group(1)
        m_body = match.group(2)
        pose_m = re.search(r'<pose>([^<]+)</pose>', m_body)
        pose = [float(v) for v in pose_m.group(1).split()] if pose_m else [0, 0, 0, 0, 0, 0]
        # Look for box sizes within model
        sizes = [ [float(v) for v in sm.split()] for sm in re.findall(r'<size>([^<]+)</size>', m_body) ]
        models.append({"name": m_name, "pose": pose, "sizes": sizes})

    print(f"\nTotal models in SDF: {len(models)}")
    # Analyze door openings and wall intervals
    # Check pairwise distances between model poses (especially along X and Y axes)
    x_positions = sorted([m["pose"][0] for m in models])
    y_positions = sorted([m["pose"][1] for m in models])

    print("\nPairwise X-distance diffs near 0.95m (within 0.85 - 1.05 m):")
    for i in range(len(models)):
        for j in range(i+1, len(models)):
            m1, m2 = models[i], models[j]
            dx = abs(m1["pose"][0] - m2["pose"][0])
            dy = abs(m1["pose"][1] - m2["pose"][1])
            dist = math.hypot(dx, dy)
            if 0.90 <= dist <= 1.00 or 0.90 <= dx <= 1.00:
                print(f"  Distance {dist:.3f}m (dx={dx:.3f}, dy={dy:.3f}) between '{m1['name']}' at ({m1['pose'][0]:.2f}, {m1['pose'][1]:.2f}) and '{m2['name']}' at ({m2['pose'][0]:.2f}, {m2['pose'][1]:.2f})")

def analyze_pgm(pgm_path, yaml_origin=(-9.0, -6.5), resolution=0.05):
    print("\n=== PGM OCCUPANCY GRID ANALYSIS ===")
    img = Image.open(pgm_path)
    arr = np.array(img)
    h, w = arr.shape
    print(f"Map dimensions: {w} x {h} pixels ({w*resolution:.2f} m x {h*resolution:.2f} m)")
    print(f"Resolution: {resolution} m/pixel, Origin: {yaml_origin}")

    # In standard ROS pgm:
    # 0 = occupied (black), 254 = free (white), 205 = unknown (grey)
    # Target 0.95 m corresponds to:
    pixels_095 = int(round(0.95 / resolution))  # 19 pixels
    print(f"0.95 m equals exactly {pixels_095} pixels at 0.05 m/px.")

    # Find doorways / corridor widths:
    # Where is the robot corridor?
    # Hallway is along Y = -0.5 to +0.5, X from 0.0 to -5.0
    # Let's convert coordinates to pixel indices:
    # px = (x - origin_x) / resolution
    # py = h - 1 - (y - origin_y) / resolution (or standard image coordinates)
    
    # Check doorways: door widths are often ~0.9m - 1.0m (18-20 pixels)
    # Check walls along hallway
    occupied = (arr < 50)
    print(f"Total occupied pixels: {np.sum(occupied)}")

    # Let's inspect hallway profile along X from 0 to -5 at Y ~ 0
    # Check width of corridor:
    corridor_y_indices = [int(round((y - yaml_origin[1]) / resolution)) for y in [-0.5, 0.0, 0.5]]
    # Check horizontal door openings
    # Kitchen entrance is near x = -5.0, y = -1.0
    # Bedroom entrance is near x = -5.0, y = +1.0
    doorway_x = int(round((-5.0 - yaml_origin[0]) / resolution))
    kitchen_y = int(round((-2.5 - yaml_origin[1]) / resolution))
    bedroom_y = int(round((2.2 - yaml_origin[1]) / resolution))
    origin_px = int(round((0.0 - yaml_origin[0]) / resolution))
    origin_py = int(round((0.0 - yaml_origin[1]) / resolution))

    print(f"Pixel coords for (0,0): px={origin_px}, py={origin_py}")
    print(f"Pixel coords for doorway/junction (-5.0, 0.0): px={doorway_x}")
    print(f"Pixel coords for kitchen (-5.0, -2.5): px={doorway_x}, py={kitchen_y}")
    print(f"Pixel coords for bedroom (-5.0, 2.2): px={doorway_x}, py={bedroom_y}")

    # Autocorrelation of hallway profile
    # Extract corridor strip along X axis from X=0 down to X=-5.5 at Y=0
    # Let's check vertical slices across the hallway
    hallway_widths = []
    for x in np.arange(-5.0, 0.5, 0.1):
        px = int(round((x - yaml_origin[0]) / resolution))
        col = occupied[:, px]
        # find wall positions in column
        wall_y = np.where(col)[0]
        if len(wall_y) >= 2:
            # find min gap around y=0
            py0 = int(round((0.0 - yaml_origin[1]) / resolution))
            walls_below = wall_y[wall_y < py0]
            walls_above = wall_y[wall_y > py0]
            if len(walls_below) > 0 and len(walls_above) > 0:
                gap = (walls_above[0] - walls_below[-1]) * resolution
                hallway_widths.append((round(x, 2), round(gap, 3)))

    print(f"\nHallway clearances along X axis (sample of {len(hallway_widths)} slices):")
    for x, w_gap in hallway_widths[::5]:
        print(f"  At X={x:.2f} m: Corridor clear width = {w_gap:.3f} m")

    # Check door openings
    # In apartment_floor.world or map, check door openings widths
    print("\nDoorway measurements in map:")
    # Kitchen door: wall at Y=-1.0, opening in X
    door_kitchen_y = int(round((-1.0 - yaml_origin[1]) / resolution))
    door_kitchen_row = occupied[door_kitchen_y, :]
    gaps = []
    # find free segments in this row
    free_segments = []
    in_gap = False
    gap_start = 0
    for px in range(w):
        if not door_kitchen_row[px] and not in_gap:
            in_gap = True
            gap_start = px
        elif door_kitchen_row[px] and in_gap:
            in_gap = False
            gaps.append((gap_start, px, (px - gap_start) * resolution))
    for gs, ge, width in gaps:
        x_start = gs * resolution + yaml_origin[0]
        x_end = ge * resolution + yaml_origin[0]
        if -6.0 <= x_start <= -3.0:
            print(f"  Opening at Y=-1.0 m: X=[{x_start:.2f}, {x_end:.2f}], width = {width:.3f} m")

if __name__ == "__main__":
    sdf_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world"
    pgm_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm"
    analyze_sdf(sdf_file)
    analyze_pgm(pgm_file)

