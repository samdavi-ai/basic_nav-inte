#!/usr/bin/env python3
"""
GRaCEmo ViRa: SAC Fine-Tuning with BC Warm-Start
Fine-tunes the BC-initialized SAC policy on kinematic navigation:
- Warm-starts actor from BC weights
- Small learning rate: 1e-4
- Trains across Stages 2 & 4 (doorways and narrow passages)
- Preserves smooth actions and low heading bias
"""

import os
import sys
import time
import math
import json
import numpy as np
import torch

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig
from gracemo_rl.training.export_onnx import export_policy_to_onnx


def make_train_env(rank: int, seed: int = 1000):
    def _init():
        cfg = NavEnvConfig()
        cfg.curriculum_stage = 2 if rank % 2 == 0 else 4
        env = GracemoNavEnv(mode="kinematic", config=cfg)
        env.reset(seed=seed + rank * 100)
        return Monitor(env)
    return _init


def run_sac_finetune(
    total_timesteps: int = 100000,
    model_input: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_best.zip",
    output_dir: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models",
    num_envs: int = 8
):
    print("=== Initiating SAC Fine-Tuning from BC Warm-Start ===")
    assert os.path.exists(model_input), f"Base model not found: {model_input}"

    vec_env = SubprocVecEnv([make_train_env(i) for i in range(num_envs)])

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Loading BC checkpoint into SAC on {device}...")
    model = SAC.load(
        model_input,
        env=vec_env,
        learning_rate=1e-4,
        learning_starts=1000,
        batch_size=256,
        tau=0.005,
        gamma=0.99,
        ent_coef="auto_0.1",
        device=device
    )

    t0 = time.time()
    try:
        model.learn(total_timesteps=total_timesteps, log_interval=20)
    finally:
        vec_env.close()

    elapsed = time.time() - t0
    print(f"Fine-tuning complete in {elapsed:.1f} s.")

    ft_model_path = os.path.join(output_dir, "gracemo_sac_finetuned.zip")
    model.save(ft_model_path)
    print(f"Saved fine-tuned SAC model to: {ft_model_path}")

    # Export to candidate ONNX
    ft_onnx_path = os.path.join(output_dir, "gracemo_sac_finetuned.onnx")
    export_policy_to_onnx(
        model_path=ft_model_path,
        output_onnx_path=ft_onnx_path,
        algo="SAC",
        obs_dim=79,
        verbose=True
    )
    print(f"Exported fine-tuned ONNX: {ft_onnx_path}")


if __name__ == "__main__":
    run_sac_finetune()
