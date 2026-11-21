#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase 9 Failure Remediation Experiment.
Evaluates the impact of:
  1. Fix 1: Obstacle Distance Speed Limiter (forward LiDAR sector scaling)
  2. Fix 2: Reverse Bias Elimination (preventing reverse oscillation into walls)
  3. Fix 3: Residual RL Mode (blending DWB path tracker with RL policy)
Evaluates before vs after on the worst 3 cells with >= 20 runs each.
"""

import os
import sys
import math
import time
import json
import numpy as np
from typing import Dict, Any, List

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, repo_root)
sys.path.insert(0, os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl"))

from gracemo_rl.envs.nav_env import GracemoNavEnv
from gracemo_rl.common.places import load_authoritative_places
import onnxruntime as ort

from scripts.evaluate_robustness_matrix import (
    generate_scenario_for_cell,
    PurePursuitDWBController,
    RLPolicyController,
    run_episode
)


class EnhancedRLController:
    def __init__(self, onnx_path: str, mode: str = "baseline"):
        self.session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.mode = mode
        self.dwb = PurePursuitDWBController()

    def predict(self, obs: np.ndarray) -> np.ndarray:
        inputs = {self.input_name: obs.reshape(1, -1).astype(np.float32)}
        raw_action = self.session.run(None, inputs)[0].ravel().astype(np.float32)

        if self.mode == "baseline":
            return raw_action

        action = raw_action.copy()
        scan_m = obs[:72] * 10.0
        front_clr = float(np.min(scan_m[30:42]))
        cos_b = float(obs[74])
        sin_b = float(obs[73])
        bearing = math.atan2(sin_b, cos_b)

        # Fix 2: Reverse Bias Elimination (if goal is ahead & path clear, prevent reverse stagnation)
        if self.mode in ["fix2_forward_bias", "fix1_limiter_and_fix2", "all_combined", "residual_rl"]:
            if cos_b > 0.0 and front_clr > 0.40:
                action[0] = max(0.15, action[0])

        # Fix 1: Obstacle Distance Speed Limiter
        if self.mode in ["fix1_limiter", "fix1_limiter_and_fix2", "all_combined"]:
            d_stop = 0.32
            d_slow = 0.65
            if front_clr < d_stop:
                action[0] = min(0.0, action[0])
                # Steer away from closest side
                left_clr = np.min(scan_m[36:48])
                right_clr = np.min(scan_m[24:36])
                action[1] = 0.9 if left_clr > right_clr else -0.9
            elif front_clr < d_slow:
                scale = (front_clr - d_stop) / (d_slow - d_stop)
                action[0] = min(action[0], float(scale * 0.8))

        # Fix 3: Residual RL mode
        if self.mode == "residual_rl":
            base_act = self.dwb.predict(obs)
            # Policy acts as residual correction (+- 30%) on top of DWB base
            action = 0.70 * base_act + 0.30 * raw_action
            # Obstacle speed limiter on residual
            if front_clr < 0.35:
                action[0] = min(0.0, action[0])

        return np.clip(action, -1.0, 1.0).astype(np.float32)


def evaluate_config(controller, cell_name: str, num_episodes: int = 20) -> Dict[str, Any]:
    env = GracemoNavEnv(mode="kinematic")
    records = []
    for ep in range(num_episodes):
        seed = 8000 + ep * 17
        rec = run_episode(env, controller, seed, cell_name)
        records.append(rec)

    succs = [r for r in records if r["success"]]
    succ_rate = len(succs) / num_episodes * 100.0
    times = [r["time_to_goal"] for r in succs] if succs else [30.0]
    stops = [r["poly_stop_count"] for r in records]
    slows = [r["poly_slow_count"] for r in records]
    clearances = [r["min_clearance"] for r in records]

    return {
        "success_rate": succ_rate,
        "mean_time": float(np.mean(times)),
        "mean_stop": float(np.mean(stops)),
        "mean_slow": float(np.mean(slows)),
        "mean_clearance": float(np.mean(clearances)),
        "failures": [r["failure_cause"] for r in records if not r["success"]]
    }


def main():
    onnx_path = os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl", "gracemo_rl", "models", "gracemo_sac_policy.onnx")
    worst_cells = ["narrow_door_1m", "shifted_furniture_and_dynamic", "lidar_noise_2cm"]
    modes = ["baseline", "fix1_limiter", "fix1_limiter_and_fix2", "residual_rl"]

    print("==================================================================")
    print("  PHASE 9: SYSTEMATIC REMEDIATION OF TOP 3 BENCHMARK FAILURES")
    print("==================================================================")

    results = {}
    for cell in worst_cells:
        results[cell] = {}
        print(f"\n--- Testing Cell: {cell} ---")
        for mode in modes:
            ctrl = EnhancedRLController(onnx_path, mode=mode)
            res = evaluate_config(ctrl, cell, num_episodes=20)
            results[cell][mode] = res
            print(f"  Mode: {mode:22s} | Success: {res['success_rate']:5.1f}% | Stop: {res['mean_stop']:.1f} | Slow: {res['mean_slow']:.1f} | Clr: {res['mean_clearance']:.2f}m")

    with open("scratch/phase9_remediation_results.json", "w") as f:
        json.dump(results, f, indent=2)
    print("\nSaved Phase 9 results to scratch/phase9_remediation_results.json")


if __name__ == "__main__":
    main()
