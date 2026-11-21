"""
Export Stable-Baselines3 Policy to ONNX and verify numeric parity vs PyTorch within 1e-5.
"""

import os
import sys
import argparse
import numpy as np
import torch

try:
    import onnx
    import onnxruntime as ort
    from stable_baselines3 import SAC, TD3, PPO
except ImportError:
    pass


class OnnxableSACPolicy(torch.nn.Module):
    """Wraps SB3 SAC actor for deterministic inference export."""
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        # SAC actor forward with deterministic=True returns mean action
        return self.actor(observation, deterministic=True)


class OnnxableGenericPolicy(torch.nn.Module):
    """Wraps generic SB3 actor/policy for deterministic inference."""
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, observation: torch.Tensor) -> torch.Tensor:
        if hasattr(self.policy, "actor"):
            return self.policy.actor(observation)
        return self.policy(observation)


def export_policy_to_onnx(
    model_path: str,
    output_onnx_path: str,
    algo: str = "SAC",
    obs_dim: int = 79,
    verbose: bool = True
) -> bool:
    """Exports SB3 checkpoint to ONNX and verifies numeric parity."""
    if verbose:
        print(f"Loading {algo} model from {model_path}...")

    device = torch.device("cpu")
    if algo.upper() == "SAC":
        model = SAC.load(model_path, device=device)
        onnxable = OnnxableSACPolicy(model.policy.actor)
    elif algo.upper() == "TD3":
        model = TD3.load(model_path, device=device)
        onnxable = OnnxableGenericPolicy(model.policy.actor)
    elif algo.upper() == "PPO":
        model = PPO.load(model_path, device=device)
        onnxable = OnnxableGenericPolicy(model.policy)
    else:
        raise ValueError(f"Unsupported algorithm: {algo}")

    onnxable.eval()

    # Create dummy input: batch_size=1, obs_dim=79
    dummy_input = torch.randn(1, obs_dim, dtype=torch.float32, device=device)

    # PyTorch reference output
    with torch.no_grad():
        pt_out = onnxable(dummy_input).cpu().numpy()

    # Export to ONNX using classic TorchScript exporter
    os.makedirs(os.path.dirname(os.path.abspath(output_onnx_path)), exist_ok=True)
    try:
        torch.onnx.export(
            onnxable,
            dummy_input,
            output_onnx_path,
            export_params=True,
            opset_version=14,
            do_constant_folding=True,
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}},
            dynamo=False
        )
    except TypeError:
        torch.onnx.export(
            onnxable,
            dummy_input,
            output_onnx_path,
            export_params=True,
            opset_version=14,
            do_constant_folding=True,
            input_names=["input"],
            output_names=["output"],
            dynamic_axes={"input": {0: "batch_size"}, "output": {0: "batch_size"}}
        )

    if verbose:
        print(f"Model exported to {output_onnx_path}. Verifying parity...")

    # Load ONNX with onnxruntime
    ort_session = ort.InferenceSession(output_onnx_path, providers=["CPUExecutionProvider"])
    ort_inputs = {ort_session.get_inputs()[0].name: dummy_input.cpu().numpy()}
    ort_out = ort_session.run(None, ort_inputs)[0]

    # Verify max difference across outputs
    max_diff = float(np.max(np.abs(pt_out - ort_out)))
    if verbose:
        print(f"Maximum absolute difference: {max_diff:.8e}")

    parity_ok = max_diff < 1e-5
    if parity_ok:
        if verbose:
            print("✓ ONNX export passed numeric parity verification (< 1e-5).")
    else:
        print(f"✗ ERROR: Parity failure! Difference {max_diff:.8e} >= 1e-5.")

    return parity_ok


def main():
    parser = argparse.ArgumentParser(description="Export SB3 Model to ONNX")
    parser.add_argument("--model", type=str, required=True, help="Path to SB3 .zip model")
    parser.add_argument("--output", type=str, required=True, help="Path to output .onnx file")
    parser.add_argument("--algo", type=str, default="SAC", choices=["SAC", "TD3", "PPO"])
    parser.add_argument("--obs-dim", type=int, default=79)
    args = parser.parse_args()

    success = export_policy_to_onnx(
        model_path=args.model,
        output_onnx_path=args.output,
        algo=args.algo,
        obs_dim=args.obs_dim
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

