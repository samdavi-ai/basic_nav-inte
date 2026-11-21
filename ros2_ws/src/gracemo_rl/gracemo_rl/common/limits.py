"""
Authoritative Limits & Dynamics Configuration for GRaCEmo ViRa.
Provides a SINGLE SOURCE OF TRUTH for:
- RL Local Planner Policy limits (v_max = 0.40 m/s, w_max = 0.75 rad/s)
- Training environment dynamics parameters
- Inference ROS 2 node (rl_local_planner.py)
- Policy diagnostic sweeps and parity tests
"""

import os
import yaml
from dataclasses import dataclass
from typing import Optional, List

DEFAULT_NAV2_PARAMS = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"

# ==============================================================================
# AUTHORITATIVE SINGLE SOURCE OF TRUTH FOR RL NAVIGATION LIMITS & DYNAMICS
# ==============================================================================
RL_MAX_LINEAR_VEL: float = 0.40       # m/s (authoritative max forward speed)
RL_MIN_LINEAR_VEL: float = 0.00       # m/s
RL_MIN_REVERSE_VEL: float = -0.10     # m/s (reverse allowed only below 0.1 m/s)
RL_MAX_ANGULAR_VEL: float = 0.75      # rad/s (authoritative max turn rate)

RL_TAU_V: float = 0.22                # seconds (first-order linear velocity lag)
RL_TAU_W: float = 0.20                # seconds (first-order angular velocity lag)
RL_LATENCY_MS: float = 125.0          # milliseconds (mean transport latency: 115-140 ms)
RL_W_GAIN: float = 0.75               # angular velocity transfer gain (25% slip)
RL_MIN_SCAN_RANGE: float = 0.40       # meters (chassis blindspot cutoff)
RL_MAX_SCAN_RANGE: float = 10.0       # meters
RL_INF_RATE: float = 0.24             # 24% inf rate in open space


@dataclass
class NavLimits:
    max_linear_vel: float = RL_MAX_LINEAR_VEL
    min_linear_vel: float = RL_MIN_LINEAR_VEL
    min_reverse_vel: float = RL_MIN_REVERSE_VEL
    max_angular_vel: float = RL_MAX_ANGULAR_VEL
    max_linear_accel: float = 2.50
    max_angular_accel: float = 3.20

    xy_goal_tolerance: float = 0.25
    yaw_goal_tolerance: float = 0.25

    laser_min_range: float = 0.15
    laser_max_range: float = 12.0
    rl_min_scan_range: float = RL_MIN_SCAN_RANGE
    rl_max_scan_range: float = RL_MAX_SCAN_RANGE

    footprint: str = "[ [0.35, 0.34], [0.35, -0.34], [-0.35, -0.34], [-0.35, 0.34] ]"
    footprint_inscribed_radius: float = 0.34
    footprint_circumscribed_radius: float = 0.488


def load_nav2_limits(yaml_path: Optional[str] = None) -> NavLimits:
    """Loads authoritative parameters with RL limits pinned to single source of truth."""
    path = yaml_path or DEFAULT_NAV2_PARAMS
    limits = NavLimits()

    if not os.path.exists(path):
        return limits

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        # Acceleration and deceleration bounds from velocity smoother
        vs_params = data.get("velocity_smoother", {}).get("ros__parameters", {})
        max_acc = vs_params.get("max_accel", [2.5, 0.0, 3.2])
        limits.max_linear_accel = float(max_acc[0])
        limits.max_angular_accel = float(max_acc[2])

        # Goal tolerances
        ctrl_params = data.get("controller_server", {}).get("ros__parameters", {})
        gc = ctrl_params.get("general_goal_checker", {})
        if "xy_goal_tolerance" in gc:
            limits.xy_goal_tolerance = float(gc["xy_goal_tolerance"])
        if "yaw_goal_tolerance" in gc:
            limits.yaw_goal_tolerance = float(gc["yaw_goal_tolerance"])

        # Costmap / Footprint
        lc_params = data.get("local_costmap", {}).get("local_costmap", {}).get("ros__parameters", {})
        if "footprint" in lc_params:
            limits.footprint = str(lc_params["footprint"])

    except Exception:
        pass

    # Ensure RL limits remain strictly pinned to authoritative constants
    limits.max_linear_vel = RL_MAX_LINEAR_VEL
    limits.max_angular_vel = RL_MAX_ANGULAR_VEL
    limits.min_reverse_vel = RL_MIN_REVERSE_VEL
    limits.rl_min_scan_range = RL_MIN_SCAN_RANGE
    limits.rl_max_scan_range = RL_MAX_SCAN_RANGE

    return limits
