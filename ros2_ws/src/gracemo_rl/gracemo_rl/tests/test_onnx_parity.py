"""
Unit tests for ONNX Export and Numeric Parity vs PyTorch.
Ensures max difference across outputs <= 1e-5.
"""

import os
import tempfile
import numpy as np
import pytest
import torch

from gracemo_rl.envs.nav_env import GracemoNavEnv
from gracemo_rl.training.export_onnx import OnnxableSACPolicy, export_policy_to_onnx

try:
    from stable_baselines3 import SAC
    import onnxruntime as ort
    HAVE_DEPS = True
except ImportError:
    HAVE_DEPS = False


@pytest.mark.skipif(not HAVE_DEPS, reason="stable_baselines3 or onnxruntime not installed")
def test_onnx_parity_synthetic_sac():
    """Tests ONNX export and parity on a quick SAC model."""
    env = GracemoNavEnv(mode="kinematic")
    model = SAC("MlpPolicy", env, learning_starts=10, buffer_size=1000, seed=42)
    model.learn(total_timesteps=50)

    with tempfile.TemporaryDirectory() as tmpdir:
        model_zip = os.path.join(tmpdir, "sac_tmp.zip")
        onnx_file = os.path.join(tmpdir, "sac_tmp.onnx")

        model.save(model_zip)
        assert os.path.exists(model_zip)

        success = export_policy_to_onnx(
            model_path=model_zip,
            output_onnx_path=onnx_file,
            algo="SAC",
            obs_dim=79,
            verbose=False
        )

        assert os.path.exists(onnx_file)
        assert success is True, "Export to ONNX failed parity check"

        # Test multiple random vectors
        ort_session = ort.InferenceSession(onnx_file, providers=["CPUExecutionProvider"])
        input_name = ort_session.get_inputs()[0].name

        device = torch.device("cpu")
        onnxable = OnnxableSACPolicy(model.policy.actor).to(device)
        onnxable.eval()

        for seed in range(5):
            rng = np.random.default_rng(seed)
            sample_obs = rng.uniform(-1.0, 1.0, size=(1, 79)).astype(np.float32)

            with torch.no_grad():
                pt_out = onnxable(torch.from_numpy(sample_obs)).numpy()

            ort_out = ort_session.run(None, {input_name: sample_obs})[0]
            diff = np.max(np.abs(pt_out - ort_out))
            assert diff < 1e-5, f"Seed {seed}: max diff {diff} exceeded tolerance 1e-5"

