#!/usr/bin/env python3
"""
GRaCEmo ViRa: Offline Replay of Failing Mission States through RL Policy
Replays the recorded states from failing missions (e.g. bedroom doorway and room interior)
through the deployed ONNX policy to analyze actions, forward bias, and obstacle speed limiter.
"""

import os
import sys
import math
import json
import numpy as np
import onnxruntime as ort

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from gracemo_rl.common.obs import build_observation
from gracemo_rl.common.limits import (
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_REVERSE_VEL,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE
)


def replay_offline():
    model_path = "ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_policy.onnx"
    assert os.path.exists(model_path), f"ONNX policy not found: {model_path}"
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    print("==================================================")
    print("  Offline Replay of Failing Mission Scenarios")
    print("==================================================")

    scenarios = [
        {
            "name": "Scenario 1: Bedroom Doorway Approach (Stall Pose x=-4.567, y=1.446, yaw=-1.54 rad)",
            "rx": -4.567, "ry": 1.446, "ryaw": -1.54,
            "gx": -5.000, "gy": 2.200,  # Goal inside bedroom
            # Wall corner at left/front right at 0.35m
            "front_clr": 0.38, "left_clr": 0.28, "right_clr": 0.70, "rear_clr": 1.20
        },
        {
            "name": "Scenario 2: Kitchen Interior Stall (Pose x=-5.075, y=-2.32, yaw=-1.33 rad, Goal: Hallway exit)",
            "rx": -5.075, "ry": -2.320, "ryaw": -1.33,
            "gx": -5.000, "gy": -1.000,  # Exit door to hallway (behind robot)
            # Wall behind robot at 0.38m, table to right at 0.50m
            "front_clr": 0.85, "left_clr": 0.60, "right_clr": 0.45, "rear_clr": 0.35
        },
        {
            "name": "Scenario 3: PolygonStop Active (Wall at 0.30m, within stop threshold 0.35m)",
            "rx": -4.567, "ry": 1.446, "ryaw": -1.54,
            "gx": 4.500, "gy": 2.000,   # Goal in Living Room
            "front_clr": 0.30, "left_clr": 0.25, "right_clr": 0.35, "rear_clr": 0.80
        }
    ]

    results = []

    for sc in scenarios:
        print(f"\n--- {sc['name']} ---")
        # Synthesize 72-beam scan
        scan = np.full(72, 5.0, dtype=np.float32)
        # Front beams: indices 30 to 42
        scan[30:42] = sc["front_clr"]
        # Left beams: indices 42 to 54
        scan[42:54] = sc["left_clr"]
        # Right beams: indices 18 to 30
        scan[18:30] = sc["right_clr"]
        # Rear beams: indices 0 to 6 and 66 to 72
        scan[:6] = sc["rear_clr"]
        scan[66:] = sc["rear_clr"]

        obs = build_observation(
            scan_ranges=scan,
            robot_x=sc["rx"],
            robot_y=sc["ry"],
            robot_yaw=sc["ryaw"],
            goal_x=sc["gx"],
            goal_y=sc["gy"],
            last_action=np.zeros(2, dtype=np.float32),
            robot_vx=0.0,
            robot_wz=0.0,
            max_linear_vel=RL_MAX_LINEAR_VEL,
            max_angular_vel=RL_MAX_ANGULAR_VEL,
            min_scan_range=RL_MIN_SCAN_RANGE,
            max_scan_range=RL_MAX_SCAN_RANGE
        ).reshape(1, -1)

        raw_out = session.run(None, {input_name: obs})[0].ravel()
        a_v, a_w = float(raw_out[0]), float(raw_out[1])

        # Raw physical velocities from policy
        raw_v = (a_v + 1.0) / 2.0 * (RL_MAX_LINEAR_VEL - RL_MIN_REVERSE_VEL) + RL_MIN_REVERSE_VEL
        raw_w = a_w * RL_MAX_ANGULAR_VEL

        # Post-processing in rl_local_planner.py
        # 1. Forward bias
        sin_b = float(obs[0, 73])
        cos_b = float(obs[0, 74])
        rel_bearing = math.atan2(sin_b, cos_b)
        v_post = raw_v
        if cos_b > 0.0 and sc["front_clr"] > 0.40:
            a_v_biased = max(0.15, a_v)
            v_post = (a_v_biased + 1.0) / 2.0 * (RL_MAX_LINEAR_VEL - RL_MIN_REVERSE_VEL) + RL_MIN_REVERSE_VEL

        # 2. Obstacle speed limiter
        if sc["front_clr"] < 0.35:
            v_post = min(0.0, v_post)
        elif sc["front_clr"] < 0.70:
            scale = (sc["front_clr"] - 0.35) / (0.70 - 0.35)
            v_post = min(v_post, max(0.05, RL_MAX_LINEAR_VEL * float(scale)))

        v_final = float(np.clip(v_post, RL_MIN_REVERSE_VEL, RL_MAX_LINEAR_VEL))
        w_final = float(np.clip(raw_w, -RL_MAX_ANGULAR_VEL, RL_MAX_ANGULAR_VEL))

        # Collision monitor outcome:
        # PolygonStop triggers if min clearance < 0.35m
        min_clr = min(sc["front_clr"], sc["left_clr"], sc["right_clr"], sc["rear_clr"])
        monitor_state = "STOP (PolygonStop active)" if min_clr < 0.35 else ("SLOW (PolygonSlow active)" if min_clr < 0.55 else "FREE")
        actual_robot_motion = "IMMOBILIZED (0.0 m/s)" if "STOP" in monitor_state else f"Moving (v={v_final:.2f}, w={w_final:.2f})"

        print(f"  Relative Goal Bearing: {math.degrees(rel_bearing):+.1f} deg | Distance: {math.hypot(sc['gx']-sc['rx'], sc['gy']-sc['ry']):.2f} m")
        print(f"  Policy Raw Output:     a_v = {a_v:+.3f}, a_w = {a_w:+.3f}  -->  v = {raw_v:.3f} m/s, w = {raw_w:+.3f} rad/s")
        print(f"  Planner Filtered Cmd:  v = {v_final:.3f} m/s, w = {w_final:+.3f} rad/s")
        print(f"  Collision Monitor:     {monitor_state} (Min clearance: {min_clr:.2f} m)")
        print(f"  Resulting Robot State: {actual_robot_motion}")

        results.append({
            "scenario": sc["name"],
            "rel_bearing_deg": math.degrees(rel_bearing),
            "raw_action": [a_v, a_w],
            "filtered_cmd": [v_final, w_final],
            "min_clearance_m": min_clr,
            "monitor_state": monitor_state,
            "robot_state": actual_robot_motion
        })

    out_file = "scratch/failing_mission_offline_replay.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n✓ Saved offline replay report to: {out_file}")


if __name__ == "__main__":
    replay_offline()

