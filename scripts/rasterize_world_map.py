#!/usr/bin/env python3
"""
GRaCEmo ViRa — World vs Map Geometry Rasterization and Verification Script
Proves the mismatch between Gazebo apartment_floor.world and apartment_map.pgm:
1. Exact analytical rasterization of static collision models at 0.05 m resolution,
   with origin [-9.0, -6.5, 0] matching apartment_map.yaml.
2. Pixel-level and metric difference calculation against apartment_map.pgm.
3. Generates diff visualization and detailed metric report.
"""

import os
import sys
import json
import math
import numpy as np
from PIL import Image

def build_world_occupancy_grid(origin=(-9.20, -7.20), resolution=0.05, width=368, height=288):
    """
    Build ground-truth occupancy grid from Gazebo apartment_floor.world static models.
    Occupancy conventions (standard ROS PGM):
      0   = occupied
      254 = free space
      205 = unknown
    """
    ox, oy = origin
    # Initialize free space
    grid = np.full((height, width), 254, dtype=np.uint8)

    def mark_box(xmin, xmax, ymin, ymax):
        # Clip to grid coordinates
        col_start = max(0, int(math.floor((xmin - ox) / resolution)))
        col_end = min(width, int(math.ceil((xmax - ox) / resolution)))
        row_start_my = max(0, int(math.floor((ymin - oy) / resolution)))
        row_end_my = min(height, int(math.ceil((ymax - oy) / resolution)))

        # Convert my (0 at bottom) to image row (0 at top)
        # my = h - 1 - row => row = h - 1 - my
        # row_start corresponds to row_end_my - 1
        # row_end corresponds to row_start_my
        r_top = height - 1 - (row_end_my - 1)
        r_bottom = height - 1 - row_start_my
        grid[r_top:r_bottom + 1, col_start:col_end] = 0

    # 1. Outer West and East walls
    mark_box(-9.20, -9.00, -7.20, 7.20)  # wall_west (inner face -9.00)
    mark_box(9.00, 9.20, -7.20, 7.20)   # wall_east (inner face 9.00)
    mark_box(-9.20, 9.20, 7.00, 7.20)   # wall_north (inner face 7.00)
    mark_box(-9.20, 9.20, -7.20, -7.00) # wall_south (inner face -7.00)

    # 2. Hallway North walls (inner face Y = 0.90, outer face Y = 1.10)
    # Door openings: Bedroom at [-5.60, -4.40], Study at [4.40, 5.60]
    mark_box(-9.00, -5.60, 0.90, 1.10)  # hall_north_west
    mark_box(-4.40, 4.40, 0.90, 1.10)   # hall_north_middle
    mark_box(5.60, 9.00, 0.90, 1.10)    # hall_north_east

    # 3. Hallway South walls (inner face Y = -0.90, outer face Y = -1.10)
    # Door openings: Kitchen at [-5.60, -4.40], Living Room at [4.40, 5.60]
    mark_box(-9.00, -5.60, -1.10, -0.90)  # hall_south_west
    mark_box(-4.40, 4.40, -1.10, -0.90)   # hall_south_middle
    mark_box(5.60, 9.00, -1.10, -0.90)    # hall_south_east

    # 4. Dividing walls
    mark_box(-0.10, 0.10, 1.00, 7.00)   # wall_bedroom_study
    mark_box(-0.10, 0.10, -7.00, -1.00) # wall_kitchen_living

    # 5. Static Furniture Models at LiDAR height (Z ~ 0.255m)
    # Wardrobe in Bedroom
    mark_box(-8.90, -8.30, 2.90, 5.10)
    # Nightstand in Bedroom
    mark_box(-4.45, -3.95, 3.15, 3.65)
    # Master Bed in Bedroom (mattress + headboard)
    mark_box(-6.78, -5.22, 3.80, 6.18)

    # Kitchen Counter
    mark_box(-8.80, -7.20, -6.25, -1.75)
    # Refrigerator
    mark_box(-1.87, -1.13, -6.56, -5.84)
    # Dining Table Legs (Z=0.255m intersects legs)
    for lx in [-4.5 - 0.68, -4.5 + 0.68]:
        for ly in [-4.0 - 0.38, -4.0 + 0.38]:
            mark_box(lx - 0.05, lx + 0.05, ly - 0.05, ly + 0.05)
    # Chairs in dining area
    mark_box(-4.75, -4.25, -3.25, -2.75)  # chair_1
    mark_box(-4.75, -4.25, -5.25, -4.75)  # chair_2

    # Coffee Table in Living Room
    mark_box(3.95, 5.05, -3.80, -3.20)
    # TV Unit Console in Living Room
    mark_box(8.40, 8.80, -5.00, -3.00)
    # Living Sofa
    mark_box(3.50, 5.50, -5.70, -4.70)

    # Study Desk Legs (Z=0.255m intersects legs)
    for lx in [4.5 - 0.80, 4.5 + 0.80]:
        for ly in [5.5 - 0.35, 5.5 + 0.35]:
            mark_box(lx - 0.05, lx + 0.05, ly - 0.05, ly + 0.05)
    # Bookshelf in Study
    mark_box(8.05, 8.95, 5.30, 5.70)
    # Cardboard box in Study
    mark_box(1.25, 1.75, 5.80, 6.20)

    return grid


def analyze_and_diff(old_pgm_path, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    old_img = Image.open(old_pgm_path)
    old_arr = np.array(old_img)
    old_h, old_w = old_arr.shape
    old_ox, old_oy = -9.0, -6.5
    res = 0.05

    # Full envelope map dimensions
    new_ox, new_oy = -9.20, -7.20
    new_w, new_h = 368, 288
    new_arr = build_world_occupancy_grid(origin=(new_ox, new_oy), resolution=res, width=new_w, height=new_h)

    # Save regenerated map
    new_img = Image.fromarray(new_arr)
    new_pgm_path = os.path.join(output_dir, "apartment_map_regenerated.pgm")
    new_img.save(new_pgm_path)

    # Metrics comparison
    # 1. Hallway width
    # At X=0:
    old_mx_0 = int(round((0.0 - old_ox) / res))
    old_col = old_arr[:, old_mx_0] < 50
    old_occ_y = [old_oy + (old_h - 1 - r) * res for r in range(old_h) if old_col[r]]
    old_south_wall = max([y for y in old_occ_y if y < -0.5])
    old_north_wall = min([y for y in old_occ_y if y > 0.5])
    old_corridor_width = old_north_wall - old_south_wall

    new_mx_0 = int(round((0.0 - new_ox) / res))
    new_col = new_arr[:, new_mx_0] < 50
    new_occ_y = [new_oy + (new_h - 1 - r) * res for r in range(new_h) if new_col[r]]
    new_south_wall = max([y for y in new_occ_y if y < -0.5])
    new_north_wall = min([y for y in new_occ_y if y > 0.5])
    new_corridor_width = new_north_wall - new_south_wall

    # 2. Kitchen Doorway at Y = -1.0
    row_k_old = old_h - 1 - int(round((-1.15 - old_oy) / res))
    row_k_new = new_h - 1 - int(round((-1.00 - new_oy) / res))

    def get_gap_in_interval(arr_row, x_min_search, x_max_search, ox):
        col_start = int(round((x_min_search - ox) / res))
        col_end = int(round((x_max_search - ox) / res))
        free_cols = [c for c in range(col_start, col_end) if arr_row[c] >= 50]
        if not free_cols:
            return None, None, 0.0
        x_start = ox + min(free_cols) * res
        x_end = ox + (max(free_cols) + 1) * res
        return x_start, x_end, x_end - x_start

    old_door_x1, old_door_x2, old_door_w = get_gap_in_interval(old_arr[row_k_old, :], -7.0, -3.0, old_ox)
    new_door_x1, new_door_x2, new_door_w = get_gap_in_interval(new_arr[row_k_new, :], -7.0, -3.0, new_ox)

    # 3. Bedroom Doorway at Y = 1.0
    row_b_old = old_h - 1 - int(round((1.15 - old_oy) / res))
    row_b_new = new_h - 1 - int(round((1.00 - new_oy) / res))
    old_bdoor_x1, old_bdoor_x2, old_bdoor_w = get_gap_in_interval(old_arr[row_b_old, :], -7.0, -3.0, old_ox)
    new_bdoor_x1, new_bdoor_x2, new_bdoor_w = get_gap_in_interval(new_arr[row_b_new, :], -7.0, -3.0, new_ox)

    # 4. Compute visual diff on new_arr coordinate grid
    diff_rgb = np.full((new_h, new_w, 3), 255, dtype=np.uint8)
    # Resample old_arr onto new_arr grid
    old_on_new = np.full((new_h, new_w), 205, dtype=np.uint8)
    for r in range(new_h):
        y = new_oy + (new_h - 1 - r) * res
        old_r = old_h - 1 - int(round((y - old_oy) / res))
        if 0 <= old_r < old_h:
            for c in range(new_w):
                x = new_ox + c * res
                old_c = int(round((x - old_ox) / res))
                if 0 <= old_c < old_w:
                    old_on_new[r, c] = old_arr[old_r, old_c]

    old_occ = (old_on_new < 50)
    new_occ = (new_arr < 50)

    # Free space
    diff_rgb[~old_occ & ~new_occ] = [245, 245, 245]
    # Both occupied
    diff_rgb[old_occ & new_occ] = [40, 40, 40]
    # Phantom wall (in old map, but free in world) -> Red
    diff_rgb[old_occ & ~new_occ] = [220, 50, 50]
    # Missing obstacle (in world, but missing in old map) -> Blue
    diff_rgb[~old_occ & new_occ] = [30, 160, 240]

    diff_img_path = os.path.join(output_dir, "map_vs_world_diff.png")
    Image.fromarray(diff_rgb).save(diff_img_path)

    mismatch_stats = {
        "corridor_clear_width": {
            "world_meters": round(new_corridor_width, 3),
            "old_map_meters": round(old_corridor_width, 3),
            "error_meters": round(old_corridor_width - new_corridor_width, 3)
        },
        "hallway_north_wall_y": {
            "world_meters": round(new_north_wall, 3),
            "old_map_meters": round(old_north_wall, 3),
            "error_meters": round(old_north_wall - new_north_wall, 3)
        },
        "hallway_south_wall_y": {
            "world_meters": round(new_south_wall, 3),
            "old_map_meters": round(old_south_wall, 3),
            "error_meters": round(old_south_wall - new_south_wall, 3)
        },
        "kitchen_doorway": {
            "world_opening": [round(new_door_x1, 3), round(new_door_x2, 3)],
            "world_width_m": round(new_door_w, 3),
            "old_map_opening": [round(old_door_x1, 3), round(old_door_x2, 3)],
            "old_map_width_m": round(old_door_w, 3),
            "width_error_m": round(old_door_w - new_door_w, 3)
        },
        "bedroom_doorway": {
            "world_opening": [round(new_bdoor_x1, 3), round(new_bdoor_x2, 3)],
            "world_width_m": round(new_bdoor_w, 3),
            "old_map_opening": [round(old_bdoor_x1, 3), round(old_bdoor_x2, 3)],
            "old_map_width_m": round(old_bdoor_w, 3),
            "width_error_m": round(old_bdoor_w - new_bdoor_w, 3)
        },
        "phantom_occupied_pixels": int(np.sum(old_occ & ~new_occ)),
        "missing_occupied_pixels": int(np.sum(~old_occ & new_occ)),
        "total_discrepancy_pixels": int(np.sum(old_occ ^ new_occ))
    }

    report_path = os.path.join(output_dir, "map_mismatch_report.json")
    with open(report_path, "w") as f:
        json.dump(mismatch_stats, f, indent=2)

    print("================================================================================")
    print("  MAP vs WORLD GROUND TRUTH MISMATCH PROOF & REPORT")
    print("================================================================================")
    print(f"Hallway North Wall Y: World = {new_north_wall:+.2f} m | Old Map = {old_north_wall:+.2f} m (Error: {old_north_wall - new_north_wall:+.2f} m)")
    print(f"Hallway South Wall Y: World = {new_south_wall:+.2f} m | Old Map = {old_south_wall:+.2f} m (Error: {old_south_wall - new_south_wall:+.2f} m)")
    print(f"Hallway Clear Width:  World = {new_corridor_width:.2f} m | Old Map = {old_corridor_width:.2f} m (Error: {old_corridor_width - new_corridor_width:+.2f} m)")
    print(f"Kitchen Door Opening: World = [{new_door_x1:.2f}, {new_door_x2:.2f}] (w={new_door_w:.2f}m) | Old Map = [{old_door_x1:.2f}, {old_door_x2:.2f}] (w={old_door_w:.2f}m)")
    print(f"Bedroom Door Opening: World = [{new_bdoor_x1:.2f}, {new_bdoor_x2:.2f}] (w={new_bdoor_w:.2f}m) | Old Map = [{old_bdoor_x1:.2f}, {old_bdoor_x2:.2f}] (w={old_bdoor_w:.2f}m)")
    print(f"Discrepancy Pixels:   Phantom = {mismatch_stats['phantom_occupied_pixels']} | Missing = {mismatch_stats['missing_occupied_pixels']} | Total = {mismatch_stats['total_discrepancy_pixels']}")
    print(f"Diff Image:           {diff_img_path}")
    print(f"Regenerated Map:      {new_pgm_path}")
    print("================================================================================")
    return mismatch_stats

if __name__ == "__main__":
    old_pgm = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm"
    out_dir = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs"
    analyze_and_diff(old_pgm, out_dir)

