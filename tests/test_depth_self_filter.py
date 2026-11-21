#!/usr/bin/env python3
"""
Unit tests for Depth Camera Robot-Body Self-Exclusion Filter.
Validates that:
1. Self-observation points on the robot chassis, wheels, bumper, arms, and torso are filtered (marked NaN).
2. Genuine environmental obstacles ahead, to the sides, and near the robot remain completely preserved.
"""

import math
import numpy as np
import pytest

def filter_depth_points_in_place(raw_bytes: bytearray, point_step: int = 32):
    """
    Applies the authoritative 3D robot-body self-exclusion filter using structured NumPy views.
    Transforms optical frame coordinates to base_footprint frame:
      x_bf = 0.112 + z_opt
      y_bf = -x_opt
      z_bf = 1.020 - y_opt
    Robot physical envelope: [-0.35, 0.35] x [-0.35, 0.35] x [0.00, 1.05]
    """
    if len(raw_bytes) < point_step:
        return 0

    dtype = np.dtype([
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("rest", np.uint8, point_step - 12)
    ])
    arr = np.frombuffer(raw_bytes, dtype=dtype)

    x_opt = arr["x"]
    y_opt = arr["y"]
    z_opt = arr["z"]

    # Transform to base_footprint
    x_bf = 0.112 + z_opt
    y_bf = -x_opt
    z_bf = 1.020 - y_opt

    # Self-exclusion mask
    self_mask = (
        (x_bf >= -0.35) & (x_bf <= 0.35) &
        (y_bf >= -0.35) & (y_bf <= 0.35) &
        (z_bf >= 0.00)  & (z_bf <= 1.05) &
        np.isfinite(z_opt)
    )

    filtered_count = int(np.sum(self_mask))
    if filtered_count > 0:
        x_opt[self_mask] = np.nan
        y_opt[self_mask] = np.nan
        z_opt[self_mask] = np.nan

    return filtered_count

def test_genuine_obstacles_preserved():
    # Construct synthetic pointcloud with genuine obstacles
    test_cases = [
        # (name, x_bf, y_bf, z_bf)
        ("obstacle_forward_1m", 1.112, 0.0, 1.020),
        ("obstacle_forward_0.5m", 0.612, 0.0, 0.50),
        ("obstacle_front_low_0.42m", 0.420, 0.0, 0.15),
        ("obstacle_lateral_left_0.45m", 0.20, 0.45, 0.30),
        ("obstacle_lateral_right_0.45m", 0.20, -0.45, 0.30),
        ("obstacle_high_1.5m", 0.50, 0.0, 1.50),
    ]

    p_step = 32
    raw = bytearray(len(test_cases) * p_step)
    dtype = np.dtype([
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("rest", np.uint8, p_step - 12)
    ])
    arr = np.frombuffer(raw, dtype=dtype)

    for i, (_, x_bf, y_bf, z_bf) in enumerate(test_cases):
        # Inverse transform to optical frame:
        # z_opt = x_bf - 0.112
        # x_opt = -y_bf
        # y_opt = 1.020 - z_bf
        arr["z"][i] = x_bf - 0.112
        arr["x"][i] = -y_bf
        arr["y"][i] = 1.020 - z_bf

    filtered = filter_depth_points_in_place(raw, p_step)
    assert filtered == 0, f"Expected 0 filtered points for genuine obstacles, got {filtered}"

    # Verify all coordinates remain finite and unchanged
    for i, (name, x_bf, y_bf, z_bf) in enumerate(test_cases):
        assert np.isfinite(arr["x"][i]), f"{name}: x_opt unexpectedly NaN"
        assert np.isfinite(arr["y"][i]), f"{name}: y_opt unexpectedly NaN"
        assert np.isfinite(arr["z"][i]), f"{name}: z_opt unexpectedly NaN"
        assert math.isclose(arr["z"][i], x_bf - 0.112, abs_tol=1e-5)

def test_self_body_points_filtered():
    # Construct synthetic points that fall on the robot chassis, arm, wheels, etc.
    self_cases = [
        # (name, x_bf, y_bf, z_bf)
        ("front_chassis", 0.25, 0.0, 0.20),
        ("front_bumper", 0.34, 0.10, 0.10),
        ("left_front_wheel", 0.22, 0.28, 0.12),
        ("right_front_wheel", 0.22, -0.28, 0.12),
        ("right_arm_palm", 0.05, -0.28, 0.45),
        ("torso_chest", 0.05, 0.0, 0.50),
        ("chassis_deck", 0.0, 0.0, 0.32),
    ]

    p_step = 32
    raw = bytearray(len(self_cases) * p_step)
    dtype = np.dtype([
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("rest", np.uint8, p_step - 12)
    ])
    arr = np.frombuffer(raw, dtype=dtype)

    for i, (_, x_bf, y_bf, z_bf) in enumerate(self_cases):
        arr["z"][i] = x_bf - 0.112
        arr["x"][i] = -y_bf
        arr["y"][i] = 1.020 - z_bf

    filtered = filter_depth_points_in_place(raw, p_step)
    assert filtered == len(self_cases), f"Expected all {len(self_cases)} self points filtered, got {filtered}"

    # Verify all self points are set to NaN
    for i, (name, _, _, _) in enumerate(self_cases):
        assert np.isnan(arr["x"][i]), f"{name}: x_opt should be NaN"
        assert np.isnan(arr["y"][i]), f"{name}: y_opt should be NaN"
        assert np.isnan(arr["z"][i]), f"{name}: z_opt should be NaN"

if __name__ == "__main__":
    test_genuine_obstacles_preserved()
    test_self_body_points_filtered()
    print("All depth self-filter tests PASSED successfully!")
