"""
Unit tests for GracemoNavEnv Gymnasium contract.
Verifies observation shape, action scaling, bounds, and reward calculation.
"""

import numpy as np
import pytest
from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig


def test_env_observation_space():
    env = GracemoNavEnv(mode="kinematic")
    obs, info = env.reset(seed=42)
    assert obs.shape == (79,), f"Expected observation shape (79,), got {obs.shape}"
    assert np.all(obs >= -1.0) and np.all(obs <= 1.0), "Observations should be bounded in [-1.0, 1.0]"
    assert "start" in info
    assert "goal" in info
    assert "distance_to_goal" in info


def test_env_action_scaling():
    config = NavEnvConfig(
        max_linear_vel=0.50,
        min_reverse_vel=-0.20,
        max_angular_vel=1.50,
        allow_reverse=True
    )
    env = GracemoNavEnv(mode="kinematic", config=config)

    # Full forward
    v, w = env.scale_action(np.array([1.0, 0.0], dtype=np.float32))
    assert np.isclose(v, 0.50), f"Expected v=0.50, got {v}"
    assert np.isclose(w, 0.00), f"Expected w=0.00, got {w}"

    # Full reverse
    v, w = env.scale_action(np.array([-1.0, 0.0], dtype=np.float32))
    assert np.isclose(v, -0.20), f"Expected v=-0.20, got {v}"

    # Max yaw left/right
    _, w_left = env.scale_action(np.array([0.0, 1.0], dtype=np.float32))
    assert np.isclose(w_left, 1.50), f"Expected w=1.50, got {w_left}"
    _, w_right = env.scale_action(np.array([0.0, -1.0], dtype=np.float32))
    assert np.isclose(w_right, -1.50), f"Expected w=-1.50, got {w_right}"


def test_env_step_contract():
    env = GracemoNavEnv(mode="kinematic")
    obs, _ = env.reset(seed=123)

    action = np.array([0.5, 0.1], dtype=np.float32)
    next_obs, reward, terminated, truncated, info = env.step(action)

    assert next_obs.shape == (79,)
    assert isinstance(reward, float)
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert "success" in info
    assert "collision" in info
    assert "path_length" in info
    assert "distance_to_goal" in info

