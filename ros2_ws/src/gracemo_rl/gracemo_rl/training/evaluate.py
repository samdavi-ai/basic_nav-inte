"""
Policy Evaluation Script for GRaCEmo Navigation Policy.
Evaluates success rate, episode length, path length, collision count, and min obstacle clearance.
"""

import os
import sys
import argparse
import numpy as np

try:
    import torch
    from stable_baselines3 import SAC, TD3, PPO
except ImportError:
    pass

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig


def evaluate_policy(
    model_path: str,
    algo: str = "SAC",
    mode: str = "kinematic",
    episodes: int = 50,
    seed: int = 100,
    render: bool = False
) -> dict:
    """Evaluates trained policy over N randomized episodes."""
    config = NavEnvConfig()
    env = GracemoNavEnv(mode=mode, config=config)

    device = torch.device("cpu")
    if algo.upper() == "SAC":
        model = SAC.load(model_path, env=env, device=device)
    elif algo.upper() == "TD3":
        model = TD3.load(model_path, env=env, device=device)
    elif algo.upper() == "PPO":
        model = PPO.load(model_path, env=env, device=device)
    else:
        raise ValueError(f"Unknown algorithm: {algo}")

    success_count = 0
    collision_count = 0
    timeout_count = 0
    path_lengths = []
    episode_rewards = []
    min_clearances = []

    print(f"\nEvaluating {algo} model from {model_path} over {episodes} episodes in mode='{mode}'...")

    for ep in range(episodes):
        obs, info = env.reset(seed=seed + ep)
        done = False
        truncated = False
        total_reward = 0.0
        min_clearance = float("inf")

        while not (done or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, step_info = env.step(action)
            total_reward += reward
            min_clearance = min(min_clearance, step_info.get("min_clearance", 10.0))

        if step_info.get("success", False):
            success_count += 1
        elif step_info.get("collision", False):
            collision_count += 1
        elif step_info.get("timeout", False):
            timeout_count += 1

        path_lengths.append(step_info.get("path_length", 0.0))
        episode_rewards.append(total_reward)
        min_clearances.append(min_clearance)

    success_rate = (success_count / episodes) * 100.0
    collision_rate = (collision_count / episodes) * 100.0
    avg_reward = float(np.mean(episode_rewards))
    avg_path = float(np.mean(path_lengths))
    avg_clearance = float(np.mean(min_clearances))

    print("=" * 60)
    print("             POLICY EVALUATION SUMMARY")
    print("=" * 60)
    print(f"  Episodes Evaluated:   {episodes}")
    print(f"  Success Rate:         {success_rate:.1f}% ({success_count}/{episodes})")
    print(f"  Collision Rate:       {collision_rate:.1f}% ({collision_count}/{episodes})")
    print(f"  Timeout Rate:         {(timeout_count/episodes)*100.0:.1f}% ({timeout_count}/{episodes})")
    print(f"  Average Reward:       {avg_reward:.2f}")
    print(f"  Average Path Length:  {avg_path:.2f} m")
    print(f"  Average Min Clearance:{avg_clearance:.2f} m")
    print("=" * 60 + "\n")

    return {
        "success_rate": success_rate,
        "collision_rate": collision_rate,
        "avg_reward": avg_reward,
        "avg_path_length": avg_path,
        "avg_min_clearance": avg_clearance
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate GRaCEmo Navigation Policy")
    parser.add_argument("--model", type=str, required=True, help="Path to SB3 model zip")
    parser.add_argument("--algo", type=str, default="SAC", choices=["SAC", "TD3", "PPO"])
    parser.add_argument("--mode", type=str, default="kinematic", choices=["kinematic", "gazebo"])
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=100)
    args = parser.parse_args()

    evaluate_policy(
        model_path=args.model,
        algo=args.algo,
        mode=args.mode,
        episodes=args.episodes,
        seed=args.seed
    )


if __name__ == "__main__":
    main()

