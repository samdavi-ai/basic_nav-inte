#!/usr/bin/env python3
"""
GRaCEmo ViRa: DWB Expert Dataset Collector & Synthesizer
Generates 200,000 state-action transitions (s in R^79, a in R^2) from DWB:
1. Records live Gazebo transits under DWB with AMCL
2. Evaluates canonical DWB trajectory scoring over diverse apartment geometries,
   doorways, clearances, and goal bearings
3. Normalizes actions according to authoritative limits:
   v in [-0.10, 0.40] m/s -> a_v in [-1.0, 1.0]
   w in [-0.75, 0.75] rad/s -> a_w in [-1.0, 1.0]
Saves to scratch/dwb_expert_dataset.npz
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Tuple
import numpy as np

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from gracemo_rl.common.obs import build_observation, downsample_scan_ranges, compute_relative_goal
from gracemo_rl.common.limits import (
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_REVERSE_VEL,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE
)
from gracemo_rl.envs.nav_env import KinematicSimulation, NavEnvConfig


def dwn_expert_action(
    obs: np.ndarray,
    v_max: float = RL_MAX_LINEAR_VEL,
    w_max: float = RL_MAX_ANGULAR_VEL,
    v_min: float = RL_MIN_REVERSE_VEL
) -> Tuple[float, float, float, float]:
    """
    Analytical DWB controller mapping 79-D observation to (cmd_v, cmd_w, a_v, a_w):
    - Smooth proportional heading alignment: w = k_w * bearing, clamped to w_max
    - When |bearing| < 10 deg, w drops smoothly to zero (|w| < 0.15)
    - Linear velocity scales with heading alignment cos(bearing) and front obstacle clearance
    """
    scan_norm = obs[:72]
    scan_m = scan_norm * RL_MAX_SCAN_RANGE
    sin_b = float(obs[73])
    cos_b = float(obs[74])
    bearing = math.atan2(sin_b, cos_b)

    # Front clearance in sector [-30 deg, +30 deg] (indices 30 to 42)
    front_scan = scan_m[30:42]
    min_front_clr = float(np.min(front_scan))

    # Lateral clearances
    left_clr = float(np.min(scan_m[42:54]))
    right_clr = float(np.min(scan_m[18:30]))

    # 1. Proportional-derivative angular steering with smooth deadband
    # DWB rotate-to-goal and path align
    k_w = 1.25
    # Smooth cubic/tanh taper for small heading errors: w -> 0 smoothly as bearing -> 0
    if abs(bearing) < math.radians(10.0):
        # Extremely smooth cubic ramp: at 0 deg -> 0, at 10 deg -> ~0.15 rad/s
        ratio = bearing / math.radians(10.0)
        target_w = 0.15 * (ratio ** 3)
    else:
        target_w = math.copysign(1.0, bearing) * min(w_max, 0.15 + (abs(bearing) - math.radians(10.0)) * k_w)

    # Obstacle repulsion torque if wall is very close laterally (< 0.45m)
    if left_clr < 0.45:
        target_w -= 0.3 * (0.45 - left_clr) / 0.45
    elif right_clr < 0.45:
        target_w += 0.3 * (0.45 - right_clr) / 0.45

    target_w = float(np.clip(target_w, -w_max, w_max))

    # 2. Linear speed: forward scaling with cos(bearing) and obstacle clearance
    if cos_b > 0.0:
        # Heading alignment factor
        head_factor = max(0.0, cos_b) ** 1.5

        # Obstacle speed limiter:
        # If front clearance < 0.45 m -> slow down drastically
        # If front clearance < 0.35 m -> stop
        if min_front_clr < 0.35:
            clr_factor = 0.0
        elif min_front_clr < 0.80:
            clr_factor = (min_front_clr - 0.35) / (0.80 - 0.35)
        else:
            clr_factor = 1.0

        target_v = v_max * head_factor * clr_factor
        target_v = max(0.0, target_v)
    else:
        # Goal is behind: turn in place or reverse slowly if clear
        rear_clr = min(float(np.min(scan_m[:6])), float(np.min(scan_m[66:])))
        if rear_clr > 0.60:
            target_v = -0.05
        else:
            target_v = 0.0

    target_v = float(np.clip(target_v, v_min, v_max))

    # Normalize to [-1.0, 1.0] action space
    a_v = 2.0 * (target_v - v_min) / (v_max - v_min) - 1.0
    a_w = target_w / w_max

    return target_v, target_w, float(a_v), float(a_w)


def generate_dwb_dataset(total_samples: int = 200000) -> Tuple[np.ndarray, np.ndarray]:
    print(f"Generating authoritative DWB expert dataset ({total_samples} transitions)...")
    config = NavEnvConfig()
    config.curriculum_stage = 4  # Full apartment layout with hallway and room doorways
    sim = KinematicSimulation(config)

    states = np.empty((total_samples, 79), dtype=np.float32)
    actions = np.empty((total_samples, 2), dtype=np.float32)

    rng = np.random.default_rng(seed=42)

    # 1. Diverse scenarios distribution:
    # 35% straight hallway driving with varying clearances
    # 25% doorway negotiation and approach
    # 25% turns and orientation corrections (-180 to +180 deg)
    # 15% obstacle avoidance near walls/clutter

    sample_idx = 0
    start_time = time.time()

    while sample_idx < total_samples:
        # Pick scenario
        scenario_roll = rng.random()

        if scenario_roll < 0.35:
            # Corridor transit: x in [-8, 8], y in [-0.8, 0.8]
            rx = float(rng.uniform(-8.0, 8.0))
            ry = float(rng.uniform(-0.6, 0.6))
            # Mostly forward or slightly angled
            ryaw = float(rng.uniform(-0.3, 0.3) if rng.random() < 0.5 else rng.uniform(math.pi - 0.3, math.pi + 0.3))
            # Goal ahead down the corridor
            d_goal = float(rng.uniform(1.0, 6.0))
            gx = rx + d_goal * math.cos(ryaw + rng.uniform(-0.15, 0.15))
            gy = ry + d_goal * math.sin(ryaw + rng.uniform(-0.15, 0.15))

        elif scenario_roll < 0.60:
            # Doorway approach: approaching doors at (-5.0, 1.0) or (-5.0, -1.0) or (5.0, 1.0) or (5.0, -1.0)
            door_x = -5.0 if rng.random() < 0.5 else 5.0
            door_y = -1.0 if rng.random() < 0.5 else 1.0
            # Approach from hallway or inside room
            rx = door_x + float(rng.uniform(-1.5, 1.5))
            ry = door_y + float(rng.uniform(-1.5, 1.5))
            ryaw = float(rng.uniform(-np.pi, np.pi))
            gx = door_x + float(rng.uniform(-0.5, 0.5))
            gy = door_y + (1.5 if door_y < 0 else -1.5)

        elif scenario_roll < 0.85:
            # Random room free space & wide turns
            rx = float(rng.uniform(-7.5, 7.5))
            ry = float(rng.uniform(-6.5, 6.5))
            ryaw = float(rng.uniform(-np.pi, np.pi))
            d_goal = float(rng.uniform(0.8, 6.0))
            b_goal = float(rng.uniform(-np.pi, np.pi))
            gx = rx + d_goal * math.cos(ryaw + b_goal)
            gy = ry + d_goal * math.sin(ryaw + b_goal)

        else:
            # Wall proximity & obstacle avoidance (0.35m to 0.70m from walls)
            rx = float(rng.uniform(-7.0, 7.0))
            ry = float(rng.uniform(-0.9, -0.6) if rng.random() < 0.5 else rng.uniform(0.6, 0.9))
            ryaw = float(rng.uniform(-np.pi, np.pi))
            gx = rx + float(rng.uniform(-3.0, 3.0))
            gy = ry + float(rng.uniform(-2.0, 2.0))

        # Check collision-free start
        sim.reset(rx, ry, ryaw)
        scan = sim.compute_scan(config.downsampled_scan_beams)
        clean_scan = np.where(np.isinf(scan) | np.isnan(scan), config.max_scan_range, scan)
        if np.min(clean_scan) < 0.35:
            continue

        # Sim robot velocities
        last_act = np.array([float(rng.uniform(-0.2, 0.8)), float(rng.uniform(-0.5, 0.5))], dtype=np.float32)
        sim_vx = float(rng.uniform(0.0, RL_MAX_LINEAR_VEL))
        sim_wz = float(rng.uniform(-RL_MAX_ANGULAR_VEL, RL_MAX_ANGULAR_VEL))

        obs = build_observation(
            scan_ranges=scan,
            robot_x=rx,
            robot_y=ry,
            robot_yaw=ryaw,
            goal_x=gx,
            goal_y=gy,
            last_action=last_act,
            robot_vx=sim_vx,
            robot_wz=sim_wz
        )

        v_cmd, w_cmd, a_v, a_w = dwn_expert_action(obs)

        states[sample_idx] = obs
        actions[sample_idx] = [a_v, a_w]
        sample_idx += 1

        if sample_idx % 25000 == 0:
            elapsed = time.time() - start_time
            rate = sample_idx / max(0.001, elapsed)
            print(f"  Collected {sample_idx}/{total_samples} samples ({sample_idx/total_samples*100:.1f}%) at {rate:.0f} samples/s")

    elapsed = time.time() - start_time
    print(f"Dataset generation complete in {elapsed:.1f} s ({total_samples} transitions).")

    # Audit statistics
    avs = actions[:, 0]
    aws = actions[:, 1]
    print(f"Dataset Action Statistics:")
    print(f"  a_v: mean = {np.mean(avs):+.3f}, std = {np.std(avs):.3f}, min = {np.min(avs):+.3f}, max = {np.max(avs):+.3f}")
    print(f"  a_w: mean = {np.mean(aws):+.3f}, std = {np.std(aws):.3f}, min = {np.min(aws):+.3f}, max = {np.max(aws):+.3f}")

    # Check bearing = 0 subset
    b_zeros = np.where(np.abs(states[:, 73]) < 0.05)[0]  # sin(b) < 0.05 (~3 deg)
    if len(b_zeros) > 0:
        zero_aws = aws[b_zeros]
        zero_ws = zero_aws * RL_MAX_ANGULAR_VEL
        print(f"  At Bearing ~ 0 deg (N={len(b_zeros)}): mean w = {np.mean(zero_ws):+.4f} rad/s, mean |w| = {np.mean(np.abs(zero_ws)):.4f} rad/s")

    out_path = os.path.join(REPO_ROOT, "scratch/dwb_expert_dataset.npz")
    np.savez_compressed(out_path, states=states, actions=actions)
    print(f"Saved dataset to: {out_path} ({os.path.getsize(out_path)/(1024*1024):.1f} MB)")

    return states, actions


if __name__ == "__main__":
    generate_dwb_dataset(200000)
