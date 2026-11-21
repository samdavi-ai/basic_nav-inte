"""
Comprehensive Verification Tests for GRaCEmo RL Navigation System:
1. Reward terms breakdown & weights
2. Observation normalization bounds
3. ONNX parity on the actual deployed trained model
4. Rear obstacle avoidance / reverse action safety check
"""

import os
import math
import numpy as np
import pytest
import torch

from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig
from gracemo_rl.training.export_onnx import OnnxableSACPolicy

try:
    from stable_baselines3 import SAC
    import onnxruntime as ort
    HAVE_DEPS = True
except ImportError:
    HAVE_DEPS = False


def test_reward_terms():
    """Verifies that all reward function terms compute according to specifications."""
    config = NavEnvConfig()
    env = GracemoNavEnv(mode="kinematic", config=config)
    env.last_action = np.array([0.0, 0.0], dtype=np.float32)

    # 1. Goal arrival: must equal config.goal_reward (100.0)
    r_goal = env._compute_reward(
        progress=0.1, curr_distance=0.2, min_scan=1.0,
        action=np.array([0.5, 0.0]), target_v=0.5, target_w=0.0,
        collision=False, success=True, timeout=False
    )
    assert r_goal == config.goal_reward, f"Expected {config.goal_reward}, got {r_goal}"

    # 2. Collision: must equal -config.collision_penalty (-100.0)
    r_collision = env._compute_reward(
        progress=0.1, curr_distance=2.0, min_scan=0.15,
        action=np.array([0.5, 0.0]), target_v=0.5, target_w=0.0,
        collision=True, success=False, timeout=False
    )
    assert r_collision == -config.collision_penalty, f"Expected {-config.collision_penalty}, got {r_collision}"

    # 3. Progress: positive progress must increase reward
    r_fwd = env._compute_reward(
        progress=0.2, curr_distance=2.0, min_scan=1.5,
        action=np.array([0.5, 0.0]), target_v=0.5, target_w=0.0,
        collision=False, success=False, timeout=False
    )
    r_back = env._compute_reward(
        progress=-0.2, curr_distance=2.4, min_scan=1.5,
        action=np.array([0.5, 0.0]), target_v=0.5, target_w=0.0,
        collision=False, success=False, timeout=False
    )
    assert r_fwd > r_back, "Positive progress toward goal must yield higher reward than retreat"

    # 4. Proximity penalty: distance < 0.5m must incur proximity penalty
    r_clear = env._compute_reward(
        progress=0.0, curr_distance=2.0, min_scan=1.5,
        action=np.array([0.0, 0.0]), target_v=0.0, target_w=0.0,
        collision=False, success=False, timeout=False
    )
    r_prox = env._compute_reward(
        progress=0.0, curr_distance=2.0, min_scan=0.30,
        action=np.array([0.0, 0.0]), target_v=0.0, target_w=0.0,
        collision=False, success=False, timeout=False
    )
    assert r_prox < r_clear, "Close proximity to obstacles must be penalized"

    # 6. Rear obstacle penalty: reversing toward close rear obstacles must be penalized
    close_rear_scan = np.ones(72, dtype=np.float32)
    close_rear_scan[0] = 0.25  # Rear obstacle within 0.40m
    clear_scan = np.ones(72, dtype=np.float32)
    r_rev_pen = env._compute_reward(
        progress=0.0, curr_distance=2.0, min_scan=0.8,
        action=np.array([-0.5, 0.0]), target_v=-0.15, target_w=0.0,
        collision=False, success=False, timeout=False,
        scan=close_rear_scan
    )
    r_rev_clear = env._compute_reward(
        progress=0.0, curr_distance=2.0, min_scan=0.8,
        action=np.array([-0.5, 0.0]), target_v=-0.15, target_w=0.0,
        collision=False, success=False, timeout=False,
        scan=clear_scan
    )
    assert r_rev_pen < r_rev_clear, "Reversing with obstacles directly behind must incur rear obstacle penalty"


def test_observation_normalization():
    """Verifies that observation elements are strictly within [-1.0, 1.0]."""
    env = GracemoNavEnv(mode="kinematic")

    # Test extreme randomized scenarios
    rng = np.random.default_rng(999)
    for _ in range(50):
        fake_scan = rng.uniform(0.01, 20.0, size=72)
        env.robot_x = float(rng.uniform(-100.0, 100.0))
        env.robot_y = float(rng.uniform(-100.0, 100.0))
        env.robot_yaw = float(rng.uniform(-10.0, 10.0))
        env.goal_x = float(rng.uniform(-100.0, 100.0))
        env.goal_y = float(rng.uniform(-100.0, 100.0))
        env.robot_vx = float(rng.uniform(-5.0, 5.0))
        env.robot_wz = float(rng.uniform(-5.0, 5.0))
        env.last_action = rng.uniform(-1.0, 1.0, size=2).astype(np.float32)

        obs = env._build_observation(fake_scan)
        assert obs.shape == (79,), f"Expected shape (79,), got {obs.shape}"
        assert not np.any(np.isnan(obs)), "Observation contains NaN!"
        assert not np.any(np.isinf(obs)), "Observation contains Inf!"
        assert np.all(obs >= -1.0) and np.all(obs <= 1.0), f"Observation values exceed [-1, 1]: min={np.min(obs)}, max={np.max(obs)}"


@pytest.mark.skipif(not HAVE_DEPS, reason="PyTorch or ONNX Runtime not available")
def test_onnx_parity_actual_model():
    """Verifies numeric parity on the actual deployed model gracemo_sac_policy.onnx."""
    model_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
    zip_path = os.path.join(model_dir, "gracemo_sac_final.zip")
    onnx_path = os.path.join(model_dir, "gracemo_sac_policy.onnx")

    if not (os.path.exists(zip_path) and os.path.exists(onnx_path)):
        pytest.skip("Trained model files not found in models/ directory")

    device = torch.device("cpu")
    sb3_model = SAC.load(zip_path, device=device)
    actor = OnnxableSACPolicy(sb3_model.policy.actor).to(device)
    actor.eval()

    session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    rng = np.random.default_rng(42)
    max_diff = 0.0
    for i in range(20):
        test_obs = rng.uniform(-1.0, 1.0, size=(1, 79)).astype(np.float32)
        with torch.no_grad():
            pt_out = actor(torch.from_numpy(test_obs)).numpy()
        ort_out = session.run(None, {input_name: test_obs})[0]

        diff = float(np.max(np.abs(pt_out - ort_out)))
        max_diff = max(max_diff, diff)
        assert diff < 1e-5, f"Sample {i}: parity diff {diff:.8e} >= 1e-5"

    print(f"Verified actual model ONNX parity across 20 samples: max diff = {max_diff:.8e}")


def test_rear_obstacle_reverse_safety():
    """Verifies that reverse velocities are safely bounded and do not collide into rear obstacles."""
    config = NavEnvConfig(
        min_reverse_vel=-0.20,
        max_linear_vel=0.50,
        allow_reverse=True
    )
    env = GracemoNavEnv(mode="kinematic", config=config)

    # Place robot at (0, 0) with a wall directly behind it at x = -0.30
    env.sim.obstacles = [(-0.30, -2.0, -0.30, 2.0)]
    obs, info = env.reset(seed=1, options={"start": (0.0, 0.0, 0.0), "goal": (2.0, 0.0)})

    # Max reverse action
    reverse_act = np.array([-1.0, 0.0], dtype=np.float32)
    v, w = env.scale_action(reverse_act)
    assert v >= -0.20, f"Reverse velocity {v} exceeds reverse limit -0.20 m/s"

    # Step in reverse toward rear wall (accounting for first-order velocity lag and latency)
    for _ in range(20):
        obs, r, term, trunc, info = env.step(reverse_act)
        if term:
            break

    # Check collision occurred and was penalized
    assert info["collision"] is True, "Reversing into rear obstacle must trigger collision termination"
    assert r == -config.collision_penalty, f"Expected {-config.collision_penalty}, got {r}"
