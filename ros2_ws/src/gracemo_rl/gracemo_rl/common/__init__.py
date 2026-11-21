"""
GRaCEmo Shared Architecture Modules:
Consolidates observation processing, navigation dynamics limits, and place topology.
"""

from .obs import downsample_scan_ranges, compute_relative_goal, build_observation, OBS_DIM
from .limits import (
    NavLimits,
    load_nav2_limits,
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_LINEAR_VEL,
    RL_MIN_REVERSE_VEL,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE,
    RL_TAU_V,
    RL_TAU_W,
    RL_LATENCY_MS,
    RL_W_GAIN,
    RL_INF_RATE
)
from .places import load_authoritative_places

__all__ = [
    "downsample_scan_ranges",
    "compute_relative_goal",
    "build_observation",
    "OBS_DIM",
    "NavLimits",
    "load_nav2_limits",
    "load_authoritative_places",
    "RL_MAX_LINEAR_VEL",
    "RL_MAX_ANGULAR_VEL",
    "RL_MIN_LINEAR_VEL",
    "RL_MIN_REVERSE_VEL",
    "RL_MIN_SCAN_RANGE",
    "RL_MAX_SCAN_RANGE",
    "RL_TAU_V",
    "RL_TAU_W",
    "RL_LATENCY_MS",
    "RL_W_GAIN",
    "RL_INF_RATE",
]

