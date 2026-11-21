#!/usr/bin/env python3
"""
GRaCEmo ViRa: Kinematic Policy Evaluation across Curriculum Stages
Evaluates the trained policy across curriculum stages:
- Stage 1: Free space / empty room
- Stage 2: Doorways / partition
- Stage 3: Clutter & obstacles
- Stage 4: Authentic apartment hallway and doorways
Runs 100 episodes per stage and computes 95% Wilson score confidence intervals.
Saves to scratch/bc_kinematic_eval_100ep.json.
"""

import os
import sys
import math
import time
import json
from typing import Dict, List, Tuple, Any
import numpy as np

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from stable_baselines3 import SAC
import onnxruntime as ort
from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig


def wilson_score_interval(successes: int, total: int, confidence: float = 0.95) -> Tuple[float, float]:
    """Calculates Wilson score confidence interval for binomial distribution."""
    if total == 0:
        return 0.0, 0.0
    z = 1.95996  # 95% confidence
    p = successes / total
    denom = 1.0 + (z**2) / total
    centre = (p + (z**2) / (2.0 * total)) / denom
    spread = z * math.sqrt((p * (1.0 - p) + (z**2) / (4.0 * total)) / total) / denom
    lower = max(0.0, centre - spread)
    upper = min(1.0, centre + spread)
    return lower, upper


def evaluate_stage(
    session: ort.InferenceSession,
    input_name: str,
    stage: int,
    episodes: int = 100,
    max_steps: int = 500,
    seed_offset: int = 40000
) -> Dict[str, Any]:
    print(f"\n--- Evaluating Curriculum Stage {stage} ({episodes} episodes) ---")
    config = NavEnvConfig()
    config.curriculum_stage = stage
    config.max_episode_steps = max_steps
    env = GracemoNavEnv(mode="kinematic", config=config)

    successes = 0
    collisions = 0
    timeouts = 0
    rewards = []
    episode_lengths = []
    min_obstacle_distances = []

    for ep in range(episodes):
        obs, info = env.reset(seed=seed_offset + stage * 1000 + ep)
        done = False
        truncated = False
        ep_reward = 0.0
        steps = 0
        min_dist_to_obs = 10.0

        while not (done or truncated):
            obs_tensor = obs.reshape(1, -1).astype(np.float32)
            out = session.run(None, {input_name: obs_tensor})[0].ravel()
            action = out[:2]

            obs, r, done, truncated, info = env.step(action)
            ep_reward += r
            steps += 1

            # Track obstacle distance
            scan_ranges = obs[:72] * config.max_scan_range
            min_dist_to_obs = min(min_dist_to_obs, float(np.min(scan_ranges)))

        rewards.append(ep_reward)
        episode_lengths.append(steps)
        min_obstacle_distances.append(min_dist_to_obs)

        if info.get("success", False):
            successes += 1
        elif info.get("collision", False):
            collisions += 1
        elif info.get("timeout", False) or truncated:
            timeouts += 1

    succ_rate = successes / episodes
    coll_rate = collisions / episodes
    time_rate = timeouts / episodes
    ci_low, ci_high = wilson_score_interval(successes, episodes)

    print(f"Stage {stage} Results:")
    print(f"  Success Rate: {succ_rate * 100.0:.1f}% (95% CI: [{ci_low * 100.0:.1f}%, {ci_high * 100.0:.1f}%])")
    print(f"  Collisions:   {coll_rate * 100.0:.1f}% ({collisions}/{episodes})")
    print(f"  Timeouts:     {time_rate * 100.0:.1f}% ({timeouts}/{episodes})")
    print(f"  Mean Reward:  {np.mean(rewards):.2f} +/- {np.std(rewards):.2f}")
    print(f"  Mean Steps:   {np.mean(episode_lengths):.1f}")
    print(f"  Mean Min Obs: {np.mean(min_obstacle_distances):.3f} m")

    return {
        "stage": stage,
        "episodes": episodes,
        "successes": successes,
        "collisions": collisions,
        "timeouts": timeouts,
        "success_rate": succ_rate,
        "collision_rate": coll_rate,
        "timeout_rate": time_rate,
        "ci_95_low": ci_low,
        "ci_95_high": ci_high,
        "mean_reward": float(np.mean(rewards)),
        "std_reward": float(np.std(rewards)),
        "mean_steps": float(np.mean(episode_lengths)),
        "mean_min_obs_dist": float(np.mean(min_obstacle_distances))
    }


def run_curriculum_evaluation(
    model_path: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_policy.onnx",
    episodes_per_stage: int = 100,
    output_report: str = "scratch/bc_kinematic_eval_100ep.json"
) -> Dict[str, Any]:
    assert os.path.exists(model_path), f"Model path not found: {model_path}"
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    stages = [1, 2, 3, 4]
    stage_results = {}
    total_successes = 0
    total_episodes = 0

    t0 = time.time()
    for s in stages:
        res = evaluate_stage(session, input_name, stage=s, episodes=episodes_per_stage)
        stage_results[f"stage_{s}"] = res
        total_successes += res["successes"]
        total_episodes += res["episodes"]

    overall_succ_rate = total_successes / total_episodes
    ci_low, ci_high = wilson_score_interval(total_successes, total_episodes)
    elapsed = time.time() - t0

    print(f"\n==================================================")
    print(f"  Overall Kinematic Evaluation Summary ({total_episodes} episodes across {len(stages)} stages):")
    print(f"  Overall Success Rate: {overall_succ_rate * 100.0:.1f}% (95% CI: [{ci_low * 100.0:.1f}%, {ci_high * 100.0:.1f}%])")
    print(f"  Elapsed Time: {elapsed:.1f} s")
    print(f"==================================================")

    out_data = {
        "timestamp": int(time.time()),
        "model_path": model_path,
        "total_episodes": total_episodes,
        "overall_success_rate": overall_succ_rate,
        "ci_95_low": ci_low,
        "ci_95_high": ci_high,
        "elapsed_sec": elapsed,
        "stages": stage_results
    }

    os.makedirs(os.path.dirname(os.path.abspath(output_report)), exist_ok=True)
    with open(output_report, "w") as f:
        json.dump(out_data, f, indent=2)
    print(f"✓ Saved full evaluation report to: {output_report}")

    return out_data


if __name__ == "__main__":
    run_curriculum_evaluation()
