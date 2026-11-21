#!/usr/bin/env python3
"""
GRaCEmo ViRa — Duplicate Logic & Authority Audit Script
Scans repository for:
1. Destination coordinates (kitchen, bedroom, hallway, living room, study)
2. Goal / arrival tolerances (xy_goal_tolerance, yaw_goal_tolerance, arrival verification)
3. Velocity and acceleration limits (max_vel_x, max_speed_xy, max_vel_theta, acc_lim_x, etc.)
4. LiDAR min/max range and self-filter box (laser_min_range, laser_max_range, footprint bounds)
5. Observation construction (72 sectors, 79-D obs, scan downsampling, lookahead)
6. Pose source selection (map->base_footprint, odom, amcl_pose, gz pose)
7. Reset / clean-state procedures
8. Mission dispatch and cancel/goal-ID handling
9. Footprint dimensions (chassis bounding box, collision monitor polygons)
"""

import os
import re
import json

ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"

EXCLUDE_DIRS = {".git", ".venv", ".venv-rl", "__pycache__", "build", "install"}

def scan_files():
    target_files = []
    for root, dirs, files in os.walk(ROOT):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        for f in files:
            if f.endswith(('.py', '.yaml', '.yml', '.json', '.xacro', '.world', '.xml', '.sh')):
                target_files.append(os.path.join(root, f))
    return sorted(target_files)

def grep_in_files(files, regex_dict):
    findings = {k: [] for k in regex_dict}
    for fpath in files:
        rel = os.path.relpath(fpath, ROOT)
        try:
            with open(fpath, "r", encoding="utf-8", errors="ignore") as f:
                lines = f.readlines()
            for lnum, line in enumerate(lines, 1):
                for key, pattern in regex_dict.items():
                    if pattern.search(line):
                        findings[key].append((rel, lnum, line.strip()))
        except Exception:
            pass
    return findings

patterns = {
    # 1. Destination coordinates
    "dest_kitchen": re.compile(r'kitchen|dining', re.IGNORECASE),
    "dest_coords": re.compile(r'(-5\.0\s*,\s*-2\.5|-4\.0\s*,\s*-2\.5|-5\.0\s*,\s*2\.2|-4\.0\s*,\s*2\.2)', re.IGNORECASE),
    "places_file": re.compile(r'places\.json', re.IGNORECASE),
    
    # 2. Goal / arrival tolerances
    "goal_tolerance": re.compile(r'xy_goal_tolerance|yaw_goal_tolerance|goal_tolerance', re.IGNORECASE),
    "arrival_tol": re.compile(r'(dist_err_map|dist_err_gz|drift_err|final_err_tf|final_err_gz).*?(0\.65|0\.50|0\.35|0\.20|0\.25)', re.IGNORECASE),

    # 3. Velocity and acceleration limits
    "vel_limits": re.compile(r'(max_vel_x|max_speed_xy|max_vel_theta|max_rotational_vel|max_angular_vel|acc_lim_x|max_linear|max_angular)', re.IGNORECASE),

    # 4. LiDAR min/max range and self-filter box
    "lidar_ranges": re.compile(r'(laser_min_range|laser_max_range|obstacle_max_range|obstacle_min_range|raytrace_max_range|<range>|range_min|range_max)', re.IGNORECASE),
    "self_filter": re.compile(r'(-0\.35.*?0\.35|0\.11.*?math\.cos|self-exclusion|self_filter)', re.IGNORECASE),

    # 5. Observation construction
    "obs_construction": re.compile(r'(obs_dim|79|sector_ranges|downsample|lookahead|dist_to_goal|bearing_to_goal)', re.IGNORECASE),

    # 6. Pose-source selection
    "pose_source": re.compile(r'(lookup_transform\(["\']map["\']|amcl_pose|dynamic_pose/info|use_static_localization)', re.IGNORECASE),

    # 7. Reset / clean-state
    "reset_clean": re.compile(r'(reset_clean_state|set_pose|reset_robot|INITIALIZE_LOCALIZATION|CLEAR_COSTMAPS)', re.IGNORECASE),

    # 8. Mission dispatch and cancel/goal-ID handling
    "mission_cancel_goal_id": re.compile(r'(current_goal_id|_abort_active_navigation_goal|cancel_goal_async|_pending_replacement_mission)', re.IGNORECASE),

    # 9. Footprint dimensions
    "footprint": re.compile(r'(footprint:|PolygonStop|PolygonSlow|chassis_collision|base_footprint_fixed_joint_lump)', re.IGNORECASE)
}

if __name__ == "__main__":
    files = scan_files()
    print(f"Total files scanned: {len(files)}")
    results = grep_in_files(files, patterns)
    for cat, matches in results.items():
        print(f"\n=== CATEGORY: {cat} (Total matches: {len(matches)}) ===")
        for fpath, lnum, line in matches[:15]:
            print(f"  {fpath}:{lnum}: {line[:100]}")
        if len(matches) > 15:
            print(f"  ... and {len(matches) - 15} more matches.")

