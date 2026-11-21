#!/usr/bin/env python3
"""
GRaCEmo ViRa: Doorway Navigation Test
Tests policy ability to traverse narrow doorways (0.85 m width) matching apartment doors:
Robot starts on one side, goal is on the other side.
Evaluates success rate and wall collision rate.
"""

import os
import sys
import math
import numpy as np
import onnxruntime as ort

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig


def test_doorway(episodes: int = 50):
    model_path = "ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_policy.onnx"
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    config = NavEnvConfig()
    config.curriculum_stage = 2
    config.max_episode_steps = 300
    env = GracemoNavEnv(mode="kinematic", config=config)

    # In Stage 2:
    # Wall from (-10, 0) to (-0.6, 0) and (0.6, 0) to (10, 0)
    # Doorway is at x in [-0.6, 0.6], width = 1.2 m
    # Let's make doorway narrow (0.85 m): wall from -10 to -0.425 and 0.425 to 10
    env.sim.obstacles = [
        (-10.0, 0.0, -0.425, 0.0),
        (0.425, 0.0, 10.0, 0.0),
        (-10.0, -10.0, 10.0, -10.0),
        (-10.0, 10.0, 10.0, 10.0),
        (-10.0, -10.0, -10.0, 10.0),
        (10.0, -10.0, 10.0, 10.0),
    ]

    successes = 0
    collisions = 0
    timeouts = 0

    rng = np.random.default_rng(seed=123)

    for ep in range(episodes):
        # Start at y = -2.0 to -1.5, x with offset [-0.3, +0.3]
        sx = float(rng.uniform(-0.35, 0.35))
        sy = float(rng.uniform(-2.5, -1.8))
        syaw = float(rng.uniform(math.radians(70), math.radians(110)))  # Facing doorway (+y)

        # Goal on other side: y = +2.0 to +2.5, x with offset [-0.3, +0.3]
        gx = float(rng.uniform(-0.35, 0.35))
        gy = float(rng.uniform(1.8, 2.5))

        obs, _ = env.reset(options={"start": (sx, sy, syaw), "goal": (gx, gy)})
        # Reset obstacles to custom narrow doorway
        env.sim.obstacles = [
            (-10.0, 0.0, -0.425, 0.0),
            (0.425, 0.0, 10.0, 0.0),
            (-10.0, -10.0, 10.0, -10.0),
            (-10.0, 10.0, 10.0, 10.0),
            (-10.0, -10.0, -10.0, 10.0),
            (10.0, -10.0, 10.0, 10.0),
        ]

        done = False
        truncated = False
        while not (done or truncated):
            obs_tensor = obs.reshape(1, -1).astype(np.float32)
            out = session.run(None, {input_name: obs_tensor})[0].ravel()
            obs, r, done, truncated, info = env.step(out[:2])

        if info.get("success", False):
            successes += 1
        elif info.get("collision", False):
            collisions += 1
        elif info.get("timeout", False) or truncated:
            timeouts += 1

    succ_rate = successes / episodes
    print(f"Narrow Doorway Test (Width 0.85 m, {episodes} runs):")
    print(f"  Success:   {succ_rate * 100.0:.1f}% ({successes}/{episodes})")
    print(f"  Collision: {collisions / episodes * 100.0:.1f}% ({collisions}/{episodes})")
    print(f"  Timeout:   {timeouts / episodes * 100.0:.1f}% ({timeouts}/{episodes})")


if __name__ == "__main__":
    test_doorway()
