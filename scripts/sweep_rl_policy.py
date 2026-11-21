#!/usr/bin/env python3
"""
GRaCEmo ViRa — Synthetic Sweep of RL Policy.
Tests 5 synthetic observation scenarios:
  1. Free space with goal ahead (2.0m, 0.0m)
  2. Free space with goal to left (0.0m, 2.0m)
  3. Free space with goal to right (0.0m, -2.0m)
  4. Free space with goal behind (-2.0m, 0.0m)
  5. Wall 0.4 m ahead with goal ahead (2.0m, 0.0m)
Reports actions [a_v, a_w] and tests whether policy is saturated or ignores goal.
"""

import os
import sys
import json
import numpy as np
import onnxruntime as ort

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl"))
from gracemo_rl.common.obs import build_observation, SCAN_BEAMS, DEFAULT_MAX_RANGE

def run_synthetic_sweep():
    model_path = os.path.join(
        repo_root, "ros2_ws", "src", "gracemo_rl", "gracemo_rl", "models", "gracemo_sac_policy.onnx"
    )
    if not os.path.exists(model_path):
        print(f"ERROR: Model not found at {model_path}")
        return

    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    # Scenarios
    scenarios = [
        {"name": "Free Space: Goal Ahead", "goal": (2.0, 0.0), "wall_dist": None},
        {"name": "Free Space: Goal Left",  "goal": (0.0, 2.0), "wall_dist": None},
        {"name": "Free Space: Goal Right", "goal": (0.0, -2.0), "wall_dist": None},
        {"name": "Free Space: Goal Behind", "goal": (-2.0, 0.0), "wall_dist": None},
        {"name": "Wall 0.4m Ahead: Goal Ahead", "goal": (2.0, 0.0), "wall_dist": 0.40},
    ]

    results = []
    print("================================================================================")
    print("  RL POLICY SYNTHETIC OBSERVATION SWEEP")
    print("================================================================================")
    print(f"{'Scenario':<32} | {'Goal (x,y)':<12} | {'Action [a_v, a_w]':<22} | {'Saturated?'}")
    print("-" * 80)

    for sc in scenarios:
        # Construct 360-beam scan
        scan_360 = np.full(360, DEFAULT_MAX_RANGE, dtype=np.float32)
        if sc["wall_dist"] is not None:
            # Wall ahead: angles in [-30 deg, +30 deg] -> front is around index 180 or 0
            # Note: in Gazebo planar scan index 0 is rear (-pi), index 180 is front (0 rad)
            # Beams around front: 180 +- 30 => [150, 210]
            scan_360[150:211] = sc["wall_dist"]
            # Also if index 0 is front: [0:30] and [330:360]
            scan_360[0:30] = sc["wall_dist"]
            scan_360[330:360] = sc["wall_dist"]

        obs_79 = build_observation(
            scan_ranges=scan_360,
            robot_x=0.0,
            robot_y=0.0,
            robot_yaw=0.0,
            goal_x=sc["goal"][0],
            goal_y=sc["goal"][1],
            last_action=[0.0, 0.0],
            robot_vx=0.0,
            robot_wz=0.0
        )

        obs_batch = np.expand_dims(obs_79, axis=0).astype(np.float32)
        action = session.run(None, {input_name: obs_batch})[0][0]
        av, aw = float(action[0]), float(action[1])

        # Saturated if |av| > 0.95 or |aw| > 0.95
        is_saturated = abs(av) > 0.95

        sc_res = {
            "scenario": sc["name"],
            "goal": sc["goal"],
            "wall_dist": sc["wall_dist"],
            "action": [round(av, 4), round(aw, 4)],
            "saturated": is_saturated
        }
        results.append(sc_res)
        print(f"{sc['name']:<32} | ({sc['goal'][0]:+.1f}, {sc['goal'][1]:+.1f})   | [{av:+.4f}, {aw:+.4f}]      | {is_saturated}")

    print("================================================================================")
    out_file = os.path.join(repo_root, "scratch", "rl_synthetic_sweep_results.json")
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Results saved to: {out_file}")

    # Check goal-blindness
    actions = [r["action"] for r in results]
    if all(abs(a[0] - 1.0) < 0.05 and abs(a[1]) < 0.05 for a in actions):
        print("\n⚠️ CONCLUSION: Policy is saturated at ~(1.0, 0.0) in EVERY scenario! Policy IGNORES the goal.")
    else:
        print("\nPolicy responds differentially to goal / obstacles.")

if __name__ == "__main__":
    run_synthetic_sweep()

