#!/usr/bin/env python3
"""
GRaCEmo ViRa — Robustness Matrix Evaluation Suite (Phase 8).
Evaluates DWB baseline vs RL Local Planner across realistic disturbance cells:
  Cell 1: Baseline (1.2m door, 0cm lidar noise, nominal obstacles)
  Cell 2: Lidar Noise 1cm (sigma = 0.01m)
  Cell 3: Lidar Noise 2cm (sigma = 0.02m)
  Cell 4: Beam Dropout 5% (random 5% beam dropout)
  Cell 5: Narrow Door 1.0m (aperture = 1.0m)
  Cell 6: Clutter Shift +-0.3m & Moving Obstacle

Measures:
  - Success Rate (%)
  - Stop / Slow Interventions
  - Transit Time (s)
  - Path Length (m)
  - Minimum Clearance (m)
  - Failure Attribution & Component Diagnostics
"""

import os
import sys
import math
import time
import json
import argparse
import numpy as np
from typing import Dict, Any, List, Tuple

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl"))

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig
from gracemo_rl.common.places import load_authoritative_places

try:
    import onnxruntime as ort
    HAVE_ORT = True
except ImportError:
    HAVE_ORT = False


class PurePursuitDWBController:
    """Pure Pursuit / DWB heuristic baseline controller."""
    def __init__(self, max_v=0.50, max_w=1.50):
        self.max_v = max_v
        self.max_w = max_w

    def predict(self, obs: np.ndarray) -> np.ndarray:
        scan_m = obs[:72] * 10.0
        sin_b = float(obs[73])
        cos_b = float(obs[74])
        bearing = math.atan2(sin_b, cos_b)

        front_min = np.min(scan_m[30:42])

        if front_min < 0.38:
            left_clr = np.min(scan_m[36:48])
            right_clr = np.min(scan_m[24:36])
            turn_dir = 1.0 if left_clr > right_clr else -1.0
            return np.array([0.0, turn_dir * 0.8], dtype=np.float32)
        else:
            speed = 0.8 * max(0.1, cos_b)
            return np.array([speed, np.clip(bearing * 1.2, -1.0, 1.0)], dtype=np.float32)


class RLPolicyController:
    """ONNX Runtime Inference Controller for trained RL Policy."""
    def __init__(self, onnx_path: str):
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f"ONNX model not found at {onnx_path}")
        self.session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name

    def predict(self, obs: np.ndarray) -> np.ndarray:
        inputs = {self.input_name: obs.reshape(1, -1).astype(np.float32)}
        action = self.session.run(None, inputs)[0].ravel()
        return action.astype(np.float32)


def generate_scenario_for_cell(cell_name: str, seed: int) -> Tuple[Tuple[float, float, float], Tuple[float, float], List[Tuple[float, float, float, float]], Dict[str, Any]]:
    """Generates room-based start/goals and obstacles tailored to the cell."""
    rng = np.random.default_rng(seed)
    places = load_authoritative_places()

    # Room endpoints
    rooms = [
        ("home", (places["home"]["x"], places["home"]["y"])),
        ("kitchen", (places["kitchen"]["x"], places["kitchen"]["y"])),
        ("bedroom", (places["bedroom"]["x"], places["bedroom"]["y"])),
        ("hallway", (1.5, 0.0)),
        ("dining", (-3.5, -2.0)),
    ]

    r_idx1 = seed % len(rooms)
    r_idx2 = (seed + 1 + (seed // len(rooms))) % len(rooms)
    start_room_name, start_coords = rooms[r_idx1]
    goal_room_name, goal_coords = rooms[r_idx2]

    # Start pose with minor jitter
    start = (
        float(start_coords[0] + rng.uniform(-0.15, 0.15)),
        float(start_coords[1] + rng.uniform(-0.15, 0.15)),
        float(rng.uniform(-math.pi, math.pi))
    )
    goal = (
        float(goal_coords[0] + rng.uniform(-0.15, 0.15)),
        float(goal_coords[1] + rng.uniform(-0.15, 0.15))
    )

    # Base apartment boundary
    obstacles: List[Tuple[float, float, float, float]] = [
        (-6.0, -4.0, 3.0, -4.0),
        (3.0, -4.0, 3.0, 4.0),
        (3.0, 4.0, -6.0, 4.0),
        (-6.0, 4.0, -6.0, -4.0),
    ]

    # Interior dividing walls with doorway
    door_width = 1.0 if cell_name == "narrow_door_1m" else 1.2
    half_door = door_width / 2.0

    # Kitchen dividing wall
    obstacles.append((-6.0, -1.0, -4.0 - half_door, -1.0))
    obstacles.append((-4.0 + half_door, -1.0, 0.0, -1.0))

    # Bedroom dividing wall
    obstacles.append((-6.0, 1.0, -4.0 - half_door, 1.0))
    obstacles.append((-4.0 + half_door, 1.0, 0.0, 1.0))

    # Corridor spine
    obstacles.append((0.0, -1.0, 0.0, 1.0))

    # Apply furniture shift if cell specifies
    if cell_name == "shifted_furniture_and_dynamic":
        shift_x = float(rng.uniform(-0.3, 0.3))
        shift_y = float(rng.uniform(-0.3, 0.3))
        # Add shifted furniture boxes
        obstacles.append((-2.0 + shift_x, -2.5 + shift_y, -1.0 + shift_x, -2.5 + shift_y))
        obstacles.append((-2.0 + shift_x, 2.2 + shift_y, -1.0 + shift_x, 2.2 + shift_y))

    cell_params = {
        "door_width": door_width,
        "lidar_noise": 0.0,
        "beam_dropout": 0.0,
        "has_dynamic": (cell_name == "shifted_furniture_and_dynamic"),
    }

    if cell_name == "lidar_noise_1cm":
        cell_params["lidar_noise"] = 0.01
    elif cell_name == "lidar_noise_2cm":
        cell_params["lidar_noise"] = 0.02
    elif cell_name == "beam_dropout_5pct":
        cell_params["lidar_noise"] = 0.01
        cell_params["beam_dropout"] = 0.05
    elif cell_name == "narrow_door_1m":
        cell_params["lidar_noise"] = 0.01

    return start, goal, obstacles, cell_params


def run_episode(env: GracemoNavEnv, controller, seed: int, cell_name: str) -> Dict[str, Any]:
    start, goal, obstacles, cell_params = generate_scenario_for_cell(cell_name, seed)
    env.sim.obstacles = list(obstacles)
    env.config.scan_noise_sigma = cell_params["lidar_noise"]
    env.config.doorway_width = cell_params["door_width"]

    obs, info = env.reset(seed=seed, options={"start": start, "goal": goal})
    start_dist = info["distance_to_goal"]

    path_length = 0.0
    min_clearance = float("inf")
    poly_slow_count = 0
    poly_stop_count = 0

    rng = np.random.default_rng(seed + 100)
    done = False
    truncated = False
    step = 0
    dt = 1.0 / env.config.control_rate_hz
    failure_cause = "NONE"

    # Moving obstacle setup
    dyn_x = -1.5
    dyn_y = -0.5
    dyn_vy = 0.15

    while not (done or truncated):
        step += 1

        # Dynamic obstacle motion
        if cell_params["has_dynamic"]:
            dyn_y += dyn_vy * dt
            if dyn_y > 0.8 or dyn_y < -0.8:
                dyn_vy = -dyn_vy
            # Add dynamic obstacle line segment
            env.sim.obstacles = list(obstacles) + [
                (dyn_x - 0.2, dyn_y, dyn_x + 0.2, dyn_y)
            ]

        # Apply beam dropout if enabled
        if cell_params["beam_dropout"] > 0.0:
            dropout_mask = rng.random(72) < cell_params["beam_dropout"]
            obs[:72][dropout_mask] = 1.0  # normalize max range

        action = controller.predict(obs)
        obs, reward, done, truncated, info = env.step(action)

        scan_dist = float(np.min(obs[:72])) * env.config.max_scan_range
        min_clearance = min(min_clearance, scan_dist)

        # PolygonStop / PolygonSlow triggers
        if scan_dist < 0.28:
            poly_stop_count += 1
        elif scan_dist < 0.42:
            poly_slow_count += 1

    success = bool(info.get("success", False))
    collision = bool(info.get("collision", False))
    time_to_goal = step * dt
    actual_path = info.get("path_length", 0.0)

    if not success:
        if collision:
            failure_cause = "COLLISION"
        elif truncated:
            failure_cause = "TIMEOUT_STAGNATION"
        else:
            failure_cause = "SAFETY_HALT"

    return {
        "success": success,
        "failure_cause": failure_cause,
        "time_to_goal": time_to_goal,
        "path_length": actual_path,
        "start_dist": start_dist,
        "min_clearance": min_clearance,
        "poly_slow_count": poly_slow_count,
        "poly_stop_count": poly_stop_count,
    }


def evaluate_matrix(episodes_per_cell: int = 20) -> Dict[str, Any]:
    cells = [
        "baseline_open",
        "lidar_noise_1cm",
        "lidar_noise_2cm",
        "beam_dropout_5pct",
        "narrow_door_1m",
        "shifted_furniture_and_dynamic",
    ]

    onnx_path = os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl", "gracemo_rl", "models", "gracemo_sac_policy.onnx")
    controllers = {
        "DWB": PurePursuitDWBController(),
        "RL": RLPolicyController(onnx_path),
    }

    env = GracemoNavEnv(mode="kinematic")
    results: Dict[str, Dict[str, Any]] = {}

    for cell in cells:
        results[cell] = {}
        print(f"\nEvaluating Cell: {cell} ({episodes_per_cell} episodes/controller)...")
        for cname, ctrl in controllers.items():
            ep_records = []
            for ep in range(episodes_per_cell):
                seed = 7000 + ep * 13
                rec = run_episode(env, ctrl, seed, cell)
                ep_records.append(rec)

            succs = [r for r in ep_records if r["success"]]
            succ_rate = len(succs) / episodes_per_cell * 100.0
            times = [r["time_to_goal"] for r in succs] if succs else [env.config.max_episode_steps / env.config.control_rate_hz]
            paths = [r["path_length"] for r in succs] if succs else [0.0]
            clearances = [r["min_clearance"] for r in ep_records]
            stops = [r["poly_stop_count"] for r in ep_records]
            slows = [r["poly_slow_count"] for r in ep_records]
            failures = [r["failure_cause"] for r in ep_records if not r["success"]]

            results[cell][cname] = {
                "success_rate": succ_rate,
                "episodes": episodes_per_cell,
                "mean_time_s": float(np.mean(times)),
                "mean_path_m": float(np.mean(paths)),
                "mean_clearance_m": float(np.mean(clearances)),
                "mean_poly_stop": float(np.mean(stops)),
                "mean_poly_slow": float(np.mean(slows)),
                "failures": failures,
            }
            print(f"  [{cname}] Success: {succ_rate:5.1f}% | Stop: {np.mean(stops):.1f} | Slow: {np.mean(slows):.1f} | Clearance: {np.mean(clearances):.2f}m")

    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=20, help="Episodes per cell")
    parser.add_argument("--output", type=str, default="scratch/robustness_matrix_results.json")
    args = parser.parse_args()

    matrix_results = evaluate_matrix(args.episodes)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(matrix_results, f, indent=2)

    print(f"\nSaved Robustness Matrix Results to {args.output}")


if __name__ == "__main__":
    main()
