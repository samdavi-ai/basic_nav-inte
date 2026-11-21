"""
Curriculum Training Pipeline for GRaCEmo Mobile Robot Reinforcement Learning.
Vectorized multi-environment SAC training with automated diagnostics,
checkpoint sensitivity verification, curriculum promotion, and ONNX export.
"""

import os
import sys
import yaml
import json
import time
import argparse
from typing import Optional, Dict, Any, List
import numpy as np

import torch
from stable_baselines3 import SAC, TD3, PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig
from gracemo_rl.training.export_onnx import export_policy_to_onnx


class CurriculumAndAuditCallback(BaseCallback):
    """
    Evaluates policy every eval_freq steps across:
    1. Navigation outcomes (success, collision, timeout, reward)
    2. Policy action distributions (action std >= 0.1, saturation fraction)
    3. Goal sensitivity (PyTorch autograd gradient norm on goal features dims 72..74)
    4. Curriculum advancement across all vectorized workers
    """
    def __init__(
        self,
        eval_env: GracemoNavEnv,
        eval_freq: int = 25000,
        eval_episodes: int = 25,
        promotion_threshold: float = 0.85,
        output_dir: str = "models",
        verbose: int = 1
    ):
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.eval_episodes = eval_episodes
        self.promotion_threshold = promotion_threshold
        self.output_dir = output_dir
        self.current_stage = 1
        self.best_success_rate = -1.0
        self.best_model_path = os.path.join(output_dir, "gracemo_sac_best.zip")
        self.last_eval_step = 0
        self.history: List[Dict[str, Any]] = []

    def _on_step(self) -> bool:
        if (self.num_timesteps - self.last_eval_step) < self.eval_freq:
            return True
        self.last_eval_step = self.num_timesteps

        step_num = self.num_timesteps
        device = self.model.device

        # 1. Evaluate navigation episodes
        successes = 0
        collisions = 0
        timeouts = 0
        rewards = []

        # Collect sample observations during eval for sensitivity check
        obs_samples = []

        for ep in range(self.eval_episodes):
            obs, info = self.eval_env.reset(seed=ep + 20000 + self.n_calls)
            done = False
            truncated = False
            ep_reward = 0.0

            while not (done or truncated):
                if len(obs_samples) < 500:
                    obs_samples.append(obs.copy())

                action, _ = self.model.predict(obs, deterministic=True)
                obs, r, done, truncated, info = self.eval_env.step(action)
                ep_reward += r

            rewards.append(ep_reward)
            if info.get("success", False):
                successes += 1
            elif info.get("collision", False):
                collisions += 1
            elif info.get("timeout", False) or truncated:
                timeouts += 1

        success_rate = successes / self.eval_episodes
        collision_rate = collisions / self.eval_episodes
        timeout_rate = timeouts / self.eval_episodes
        mean_reward = float(np.mean(rewards))

        # 2. Compute action distribution and autograd goal sensitivity
        # Ensure at least 500 observations
        while len(obs_samples) < 500:
            o, _ = self.eval_env.reset(seed=len(obs_samples) + 50000)
            obs_samples.append(o)

        obs_batch = np.array(obs_samples, dtype=np.float32)
        obs_tensor = torch.as_tensor(obs_batch, dtype=torch.float32, device=device)
        obs_tensor.requires_grad_(True)

        actor = self.model.policy.actor
        with torch.enable_grad():
            actions = actor(obs_tensor, deterministic=True)
            v_act = actions[:, 0]
            w_act = actions[:, 1]
            grad_v = torch.autograd.grad(v_act.sum(), obs_tensor, retain_graph=True, create_graph=False)[0]
            grad_w = torch.autograd.grad(w_act.sum(), obs_tensor, retain_graph=False, create_graph=False)[0]

        a_np = actions.detach().cpu().numpy()
        std_v = float(np.std(a_np[:, 0]))
        std_w = float(np.std(a_np[:, 1]))
        sat_v_frac = float(np.mean(np.abs(a_np[:, 0]) > 0.95))

        # Goal sensitivity: average L2 norm of gradient for goal features (dims 72..74)
        goal_grad_norm = torch.norm(grad_v[:, 72:75], dim=1) + torch.norm(grad_w[:, 72:75], dim=1)
        mean_goal_sens = float(torch.mean(goal_grad_norm).item())

        # Checkpoint validation criteria
        action_std_ok = (std_v >= 0.10 and std_w >= 0.10)
        goal_sens_ok = (mean_goal_sens > 0.05)
        passed_gates = action_std_ok and goal_sens_ok

        entry = {
            "step": step_num,
            "curriculum_stage": self.current_stage,
            "success_rate": success_rate,
            "collision_rate": collision_rate,
            "timeout_rate": timeout_rate,
            "mean_reward": mean_reward,
            "std_v": std_v,
            "std_w": std_w,
            "sat_v_frac": sat_v_frac,
            "goal_sensitivity": mean_goal_sens,
            "passed_validation_gates": passed_gates
        }
        self.history.append(entry)

        # Print structured progress table
        print("\n" + "=" * 90)
        print(f" EVALUATION @ STEP {step_num:,} (Curriculum Stage {self.current_stage})")
        print("-" * 90)
        print(f"  Success Rate:    {success_rate * 100.0:5.1f}% | Collision: {collision_rate * 100.0:5.1f}% | Timeout: {timeout_rate * 100.0:5.1f}%")
        print(f"  Mean Reward:     {mean_reward:7.2f}")
        print(f"  Action Std:      a_v: {std_v:.4f} (gate >= 0.10: {'✓' if std_v >= 0.10 else '✗'}) | a_w: {std_w:.4f} (gate >= 0.10: {'✓' if std_w >= 0.10 else '✗'})")
        print(f"  Action Sat (|a_v| > 0.95): {sat_v_frac * 100.0:.1f}%")
        print(f"  Goal Sensitivity (|∇_goal|): {mean_goal_sens:.4f} (gate > 0.05: {'✓' if goal_sens_ok else '✗'})")
        print("=" * 90)

        # Save checkpoint table to scratch
        scratch_table = os.path.join(
            "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration",
            "scratch", "training_eval_table.json"
        )
        try:
            with open(scratch_table, "w") as f:
                json.dump(self.history, f, indent=2)
        except Exception:
            pass

        # Save best model
        if passed_gates and success_rate > self.best_success_rate:
            self.best_success_rate = success_rate
            self.model.save(self.best_model_path)
            if self.verbose:
                print(f"★ New best model saved to: {self.best_model_path} (Success Rate: {success_rate * 100.0:.1f}%)")

        # Curriculum promotion
        if success_rate >= self.promotion_threshold and self.current_stage < 3:
            self.current_stage += 1
            if self.verbose:
                print(f"\n🎉 Curriculum Promotion! Advancing to Stage {self.current_stage}!")
            self.eval_env.set_curriculum_stage(self.current_stage)
            try:
                self.training_env.env_method("set_curriculum_stage", self.current_stage)
            except Exception as e:
                if self.verbose:
                    print(f"Notice: env_method call: {e}")

        return True


def make_env(mode: str, seed: int, rank: int = 0, env_cfg_dict: Optional[dict] = None):
    def _init():
        config = NavEnvConfig()
        if env_cfg_dict:
            for k, v in env_cfg_dict.items():
                if hasattr(config, k):
                    setattr(config, k, v)
        env = GracemoNavEnv(mode=mode, config=config)
        env.reset(seed=seed + rank * 100)
        return Monitor(env)
    return _init


def train_policy(
    algo: str = "SAC",
    mode: str = "kinematic",
    timesteps: int = 1000000,
    config_path: Optional[str] = None,
    output_dir: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models",
    log_dir: str = "tensorboard_logs",
    seed: int = 42,
    num_envs: int = 8
):
    """Executes policy training using Stable-Baselines3 on GPU with parallel vectorized envs."""
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    # Load configuration
    cfg = {}
    if config_path and os.path.exists(config_path):
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)

    algo_name = algo.upper()
    env_cfg_dict = cfg.get("env_config", {"max_linear_vel": 0.40})

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"\n--- Initiating GRaCEmo RL Training ---")
    print(f"Algorithm:    {algo_name}")
    print(f"Mode:         {mode}")
    print(f"Device:       {device} ({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU'})")
    print(f"Parallel Envs:{num_envs}")
    print(f"Total Steps:  {timesteps:,}")
    print(f"Seed:         {seed}")
    print(f"Max Linear Vel: {env_cfg_dict.get('max_linear_vel', 0.40)} m/s")

    # Create vectorized environment
    if num_envs > 1 and mode == "kinematic":
        env = SubprocVecEnv([make_env(mode, seed, i, env_cfg_dict) for i in range(num_envs)])
    else:
        env = DummyVecEnv([make_env(mode, seed, 0, env_cfg_dict)])

    eval_cfg = NavEnvConfig()
    for k, v in env_cfg_dict.items():
        if hasattr(eval_cfg, k):
            setattr(eval_cfg, k, v)
    eval_env = GracemoNavEnv(mode=mode, config=eval_cfg)

    # Setup algorithm
    policy_kwargs = dict(net_arch=cfg.get("policy_kwargs", {}).get("net_arch", [256, 256]))

    if algo_name == "SAC":
        sac_params = cfg.get("sac", {})
        model = SAC(
            "MlpPolicy",
            env,
            learning_rate=sac_params.get("learning_rate", 3e-4),
            buffer_size=sac_params.get("buffer_size", 200000),
            learning_starts=sac_params.get("learning_starts", 5000),
            batch_size=sac_params.get("batch_size", 256),
            tau=sac_params.get("tau", 0.005),
            gamma=sac_params.get("gamma", 0.99),
            policy_kwargs=policy_kwargs,
            tensorboard_log=log_dir,
            verbose=1,
            seed=seed,
            device=device
        )
    else:
        raise ValueError(f"Unsupported algorithm: {algo_name}")

    eval_freq = cfg.get("eval_freq", 25000)
    save_freq = cfg.get("save_freq", 50000)

    audit_cb = CurriculumAndAuditCallback(
        eval_env=eval_env,
        eval_freq=eval_freq,
        eval_episodes=25,
        promotion_threshold=0.85,
        output_dir=output_dir,
        verbose=1
    )
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq,
        save_path=output_dir,
        name_prefix=f"gracemo_{algo_name.lower()}"
    )

    t0 = time.time()
    try:
        model.learn(
            total_timesteps=timesteps,
            callback=[audit_cb, checkpoint_cb],
            log_interval=cfg.get("log_interval", 100)
        )
    finally:
        env.close()

    elapsed = time.time() - t0

    # Save final model
    final_model_path = os.path.join(output_dir, f"gracemo_{algo_name.lower()}_final")
    model.save(final_model_path)
    print(f"\n✓ Training completed in {elapsed:.1f}s ({timesteps / elapsed:.0f} steps/s). Final model saved to: {final_model_path}.zip")

    # Select best model if available, else final model
    best_path = audit_cb.best_model_path
    chosen_path = best_path if os.path.exists(best_path) else final_model_path + ".zip"
    print(f"Selected model for export: {chosen_path}")

    # Export to authoritative ONNX
    onnx_output = os.path.join(output_dir, "gracemo_sac_policy.onnx")
    print(f"Exporting policy to ONNX: {onnx_output}...")
    export_policy_to_onnx(
        model_path=chosen_path,
        output_onnx_path=onnx_output,
        algo="SAC",
        obs_dim=79,
        verbose=True
    )
    print(f"✓ Authoritative ONNX exported to: {onnx_output}")

    return chosen_path


def main():
    parser = argparse.ArgumentParser(description="Train GRaCEmo RL Navigation Policy")
    parser.add_argument("--algo", type=str, default="SAC")
    parser.add_argument("--mode", type=str, default="kinematic")
    parser.add_argument("--timesteps", type=int, default=1000000)
    parser.add_argument("--config", type=str, default="/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_rl/gracemo_rl/config/training_config.yaml")
    parser.add_argument("--output-dir", type=str, default="/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_rl/gracemo_rl/models")
    parser.add_argument("--log-dir", type=str, default="tensorboard_logs")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--num-envs", type=int, default=8)
    args = parser.parse_args()

    train_policy(
        algo=args.algo,
        mode=args.mode,
        timesteps=args.timesteps,
        config_path=args.config,
        output_dir=args.output_dir,
        log_dir=args.log_dir,
        seed=args.seed,
        num_envs=args.num_envs
    )


if __name__ == "__main__":
    main()
