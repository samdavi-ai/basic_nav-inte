#!/usr/bin/env python3
"""
GRaCEmo ViRa: Behavior Cloning (BC) from DWB Expert Dataset
Trains SB3 SAC actor network directly on 200,000 DWB transitions:
- Architecture: MLP [256, 256] with Tanh action head
- Target: a_v in [-1, 1], a_w in [-1, 1]
- Loss: MSE + heading regularization when |bearing| < 10 deg
- Exports to SB3 checkpoint and ONNX model
"""

import os
import sys
import time
import math
import json
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import TensorDataset, DataLoader

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from stable_baselines3 import SAC
from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig
from gracemo_rl.training.export_onnx import export_policy_to_onnx


def train_bc(
    dataset_path: str = "scratch/dwb_expert_dataset.npz",
    output_dir: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models",
    epochs: int = 30,
    batch_size: int = 256,
    lr: float = 1e-3,
    weight_decay: float = 1e-5,
    device_name: str = "cuda" if torch.cuda.is_available() else "cpu"
):
    os.makedirs(output_dir, exist_ok=True)
    device = torch.device(device_name)
    print(f"=== Initiating Behavior Cloning Training on {device} ===")

    # 1. Load dataset
    assert os.path.exists(dataset_path), f"Dataset not found at {dataset_path}"
    print(f"Loading expert transitions from {dataset_path}...")
    data = np.load(dataset_path)
    states = data["states"].astype(np.float32)
    actions = data["actions"].astype(np.float32)
    total_samples = len(states)
    print(f"Loaded {total_samples} samples. State shape: {states.shape}, Action shape: {actions.shape}")

    # Shuffle and split into train (90%) and val (10%)
    indices = np.random.RandomState(42).permutation(total_samples)
    val_size = int(0.10 * total_samples)
    train_idx, val_idx = indices[val_size:], indices[:val_size]

    train_states, train_actions = states[train_idx], actions[train_idx]
    val_states, val_actions = states[val_idx], actions[val_idx]

    train_dataset = TensorDataset(torch.from_numpy(train_states), torch.from_numpy(train_actions))
    val_dataset = TensorDataset(torch.from_numpy(val_states), torch.from_numpy(val_actions))

    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

    # 2. Instantiate SAC model to ensure 100% compatibility with SB3 and ONNX export
    env_cfg = NavEnvConfig()
    env = GracemoNavEnv(mode="kinematic", config=env_cfg)
    
    sac_model = SAC(
        "MlpPolicy",
        env,
        policy_kwargs=dict(net_arch=[256, 256]),
        learning_rate=3e-4,
        verbose=0,
        device=device
    )

    actor = sac_model.policy.actor.to(device)
    optimizer = torch.optim.AdamW(actor.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs, eta_min=1e-5)

    mse_loss_fn = nn.MSELoss()

    best_val_loss = float("inf")
    best_weights_path = os.path.join(output_dir, "actor_bc_best.pt")

    print(f"\nTraining actor across {epochs} epochs (Train: {len(train_states)}, Val: {len(val_states)})...")
    start_time = time.time()

    for epoch in range(1, epochs + 1):
        actor.train()
        total_train_loss = 0.0
        total_v_loss = 0.0
        total_w_loss = 0.0
        total_zero_heading_loss = 0.0
        num_batches = 0

        for b_states, b_actions in train_loader:
            b_states = b_states.to(device)
            b_actions = b_actions.to(device)

            optimizer.zero_grad()

            # SAC actor forward with deterministic=True returns tanh(mu) in [-1, 1]
            pred_actions = actor(b_states, deterministic=True)

            loss_v = mse_loss_fn(pred_actions[:, 0], b_actions[:, 0])
            loss_w = mse_loss_fn(pred_actions[:, 1], b_actions[:, 1])

            # Regularizer: when goal bearing is near 0 (|bearing| < 10 deg), penalize |a_w|
            # Bearing is from sin_b (dim 73) and cos_b (dim 74)
            sin_b = b_states[:, 73]
            cos_b = b_states[:, 74]
            # |sin_b| < sin(10 deg) ~ 0.1736 and cos_b > 0
            near_zero_mask = (torch.abs(sin_b) < 0.1736) & (cos_b > 0.0)
            if near_zero_mask.sum() > 0:
                loss_zero_h = torch.mean(torch.square(pred_actions[near_zero_mask, 1]))
            else:
                loss_zero_h = torch.tensor(0.0, device=device)

            total_loss = 1.0 * loss_v + 1.5 * loss_w + 0.5 * loss_zero_h

            total_loss.backward()
            torch.nn.utils.clip_grad_norm_(actor.parameters(), max_norm=1.0)
            optimizer.step()

            total_train_loss += total_loss.item()
            total_v_loss += loss_v.item()
            total_w_loss += loss_w.item()
            total_zero_heading_loss += loss_zero_h.item()
            num_batches += 1

        scheduler.step()

        # Validation
        actor.eval()
        val_loss = 0.0
        val_v_loss = 0.0
        val_w_loss = 0.0
        val_batches = 0
        val_zero_w_mag = []

        with torch.no_grad():
            for b_states, b_actions in val_loader:
                b_states = b_states.to(device)
                b_actions = b_actions.to(device)
                pred_actions = actor(b_states, deterministic=True)

                lv = mse_loss_fn(pred_actions[:, 0], b_actions[:, 0])
                lw = mse_loss_fn(pred_actions[:, 1], b_actions[:, 1])
                val_loss += (lv + 1.5 * lw).item()
                val_v_loss += lv.item()
                val_w_loss += lw.item()
                val_batches += 1

                # Record w predictions at zero bearing
                sin_b = b_states[:, 73]
                cos_b = b_states[:, 74]
                nz_mask = (torch.abs(sin_b) < 0.1736) & (cos_b > 0.0)
                if nz_mask.sum() > 0:
                    val_zero_w_mag.extend(torch.abs(pred_actions[nz_mask, 1]).cpu().numpy().tolist())

        avg_train_loss = total_train_loss / num_batches
        avg_val_loss = val_loss / val_batches
        avg_val_v = val_v_loss / val_batches
        avg_val_w = val_w_loss / val_batches
        mean_zero_w = np.mean(val_zero_w_mag) if val_zero_w_mag else 0.0

        if epoch % 5 == 0 or epoch == 1 or epoch == epochs:
            print(
                f"Epoch {epoch:02d}/{epochs:02d} | Train Loss: {avg_train_loss:.5f} | "
                f"Val Loss: {avg_val_loss:.5f} (v: {avg_val_v:.5f}, w: {avg_val_w:.5f}) | "
                f"Val |w| at 0 deg: {mean_zero_w:.4f} | LR: {scheduler.get_last_lr()[0]:.2e}"
            )

        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            torch.save(actor.state_dict(), best_weights_path)

    elapsed = time.time() - start_time
    print(f"\nTraining completed in {elapsed:.1f} s. Best Val Loss: {best_val_loss:.5f}")

    # 3. Load best weights and save models
    actor.load_state_dict(torch.load(best_weights_path, map_location=device))
    actor.eval()

    # Save SB3 model
    sb3_best_zip = os.path.join(output_dir, "gracemo_sac_best.zip")
    sac_model.save(sb3_best_zip)
    print(f"✓ Saved best SB3 model to: {sb3_best_zip}")

    # Export to ONNX
    onnx_path = os.path.join(output_dir, "gracemo_sac_policy.onnx")
    print(f"Exporting policy to ONNX: {onnx_path}...")
    success = export_policy_to_onnx(
        model_path=sb3_best_zip,
        output_onnx_path=onnx_path,
        algo="SAC",
        obs_dim=79,
        verbose=True
    )
    assert success, "ONNX export failed!"
    print(f"✓ ONNX export successfully verified with parity < 1e-5: {onnx_path}")

    # Save training metrics to json
    metrics = {
        "dataset_path": dataset_path,
        "total_samples": total_samples,
        "train_samples": len(train_states),
        "val_samples": len(val_states),
        "epochs": epochs,
        "best_val_loss": float(best_val_loss),
        "elapsed_sec": float(elapsed),
        "model_zip": sb3_best_zip,
        "onnx_path": onnx_path
    }
    metrics_path = "scratch/bc_training_metrics.json"
    with open(metrics_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"✓ Saved BC training metrics to: {metrics_path}")


if __name__ == "__main__":
    train_bc()
