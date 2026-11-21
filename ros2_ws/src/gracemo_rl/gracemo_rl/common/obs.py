"""
Authoritative Observation Construction for GRaCEmo RL Policy.
Used identically by:
  - Training environment (GracemoNavEnv / nav_env.py)
  - Inference ROS 2 node (rl_local_planner.py)
  - Validation harnesses and skew unit tests
Guarantees byte-identical 79-D observation vectors (tolerance < 1e-6).
"""

import math
import numpy as np
from typing import Tuple, Optional, Sequence, Union

from gracemo_rl.common.limits import (
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE
)

OBS_DIM = 79
SCAN_BEAMS = 72
DEFAULT_MIN_RANGE = RL_MIN_SCAN_RANGE
DEFAULT_MAX_RANGE = RL_MAX_SCAN_RANGE
DEFAULT_MAX_LINEAR_VEL = RL_MAX_LINEAR_VEL
DEFAULT_MAX_ANGULAR_VEL = RL_MAX_ANGULAR_VEL
DEFAULT_DIST_SCALE = 5.0


def downsample_scan_ranges(
    ranges: Union[Sequence[float], np.ndarray],
    downsampled_beams: int = SCAN_BEAMS,
    min_range: float = DEFAULT_MIN_RANGE,
    max_range: float = DEFAULT_MAX_RANGE
) -> np.ndarray:
    """
    Downsamples raw planar LiDAR ranges (e.g. 360 beams) into normalized sector beams.
    Inf and NaN beams are replaced with max_range before min-pooling.
    Returns array of shape (downsampled_beams,) with values in [min_range/max_range, 1.0].
    """
    if ranges is None or len(ranges) == 0:
        return np.ones(downsampled_beams, dtype=np.float32)

    raw = np.array(ranges, dtype=np.float32)
    # Replace inf and nan with max_range
    raw[np.isnan(raw) | np.isinf(raw)] = max_range

    n_raw = len(raw)
    step = n_raw // downsampled_beams

    # Conservative min-pooling across sectors
    downsampled = np.empty(downsampled_beams, dtype=np.float32)
    for i in range(downsampled_beams):
        downsampled[i] = np.min(raw[i * step : (i + 1) * step])

    # Range clip and normalize to [0, 1]
    norm_scan = np.clip(downsampled, min_range, max_range) / max_range
    return norm_scan.astype(np.float32)


def compute_relative_goal(
    goal_x: float,
    goal_y: float,
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    dist_scale: float = DEFAULT_DIST_SCALE
) -> Tuple[float, float, float]:
    """
    Computes goal representation relative to robot base frame.
    Returns:
      norm_dist: tanh(euclidean_dist / dist_scale) in [0, 1]
      sin_bearing: sin(relative_angle) in [-1, 1]
      cos_bearing: cos(relative_angle) in [-1, 1]
    """
    dx = float(goal_x - robot_x)
    dy = float(goal_y - robot_y)
    dist = math.hypot(dx, dy)
    norm_dist = float(math.tanh(dist / dist_scale))

    world_bearing = math.atan2(dy, dx)
    rel_bearing = (world_bearing - float(robot_yaw) + math.pi) % (2.0 * math.pi) - math.pi
    sin_bearing = float(math.sin(rel_bearing))
    cos_bearing = float(math.cos(rel_bearing))

    return norm_dist, sin_bearing, cos_bearing


def build_observation(
    scan_ranges: Union[Sequence[float], np.ndarray],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    last_action: Union[Sequence[float], np.ndarray],
    robot_vx: float,
    robot_wz: float,
    max_linear_vel: float = DEFAULT_MAX_LINEAR_VEL,
    max_angular_vel: float = DEFAULT_MAX_ANGULAR_VEL,
    min_scan_range: float = DEFAULT_MIN_RANGE,
    max_scan_range: float = DEFAULT_MAX_RANGE,
    downsampled_beams: int = SCAN_BEAMS
) -> np.ndarray:
    """
    Constructs canonical 79-dimensional observation vector.
    Layout:
      [0:72]  : Normalized 72-sector LiDAR ranges [min/max, 1.0]
      [72]    : Normalized goal distance tanh(dist / 5.0)
      [73]    : Goal bearing sin(angle)
      [74]    : Goal bearing cos(angle)
      [75:77] : Last executed policy action [a_v, a_w] in [-1, 1]
      [77]    : Normalized linear velocity clip(vx / max_v, -1, 1)
      [78]    : Normalized angular velocity clip(wz / max_w, -1, 1)
    """
    # 1. Normalized scan (72 dims)
    norm_scan = downsample_scan_ranges(
        scan_ranges,
        downsampled_beams=downsampled_beams,
        min_range=min_scan_range,
        max_range=max_scan_range
    )

    # 2. Relative goal features (3 dims)
    norm_dist, sin_b, cos_b = compute_relative_goal(
        goal_x, goal_y, robot_x, robot_y, robot_yaw, dist_scale=DEFAULT_DIST_SCALE
    )

    # 3. Last action (2 dims)
    if last_action is None:
        action_arr = np.zeros(2, dtype=np.float32)
    else:
        action_arr = np.array(last_action, dtype=np.float32).ravel()[:2]
        if len(action_arr) < 2:
            action_arr = np.pad(action_arr, (0, 2 - len(action_arr)))

    # 4. Normalized velocities (2 dims)
    norm_v = float(np.clip(robot_vx / max_linear_vel, -1.0, 1.0))
    norm_w = float(np.clip(robot_wz / max_angular_vel, -1.0, 1.0))

    # 5. Full vector concatenation: 72 + 3 + 2 + 2 = 79
    obs = np.concatenate([
        norm_scan.astype(np.float32),
        np.array([norm_dist, sin_b, cos_b], dtype=np.float32),
        action_arr.astype(np.float32),
        np.array([norm_v, norm_w], dtype=np.float32)
    ]).astype(np.float32)

    return obs

