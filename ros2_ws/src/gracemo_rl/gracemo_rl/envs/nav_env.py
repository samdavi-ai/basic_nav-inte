"""
GracemoNavEnv: Gymnasium Environment for Reinforcement Learning Local Navigation.
Supports both:
  1. Gazebo Harmonic live simulation via ROS 2 / gz.transport13
  2. Pure-Python 2D kinematic fallback for ultra-fast vectorized training, unit testing, and headless validation.
"""

import math
import time
import collections
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Tuple, Dict, Any, List

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:
    try:
        import gym
        from gym import spaces
    except ImportError:
        class _Gym:
            class Env:
                pass
        gym = _Gym()
        class _Spaces:
            @staticmethod
            def Box(*args, **kwargs):
                return None
        spaces = _Spaces()

from gracemo_rl.common.obs import build_observation, downsample_scan_ranges
from gracemo_rl.common.limits import (
    load_nav2_limits,
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_LINEAR_VEL,
    RL_MIN_REVERSE_VEL,
    RL_TAU_V,
    RL_TAU_W,
    RL_LATENCY_MS,
    RL_W_GAIN,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE,
    RL_INF_RATE
)

_limits = load_nav2_limits()


@dataclass
class NavEnvConfig:
    # Sensor configuration
    raw_scan_beams: int = 360
    downsampled_scan_beams: int = 72
    min_scan_range: float = RL_MIN_SCAN_RANGE
    max_scan_range: float = RL_MAX_SCAN_RANGE
    inf_beam_rate: float = RL_INF_RATE
    collision_threshold: float = 0.22   # meters (robot footprint inscribed radius ~0.34m)
    proximity_threshold: float = 0.50   # meters
    scan_noise_sigma: float = 0.02      # Gaussian scan noise (sigma = 0.02 m)
    rear_obstacle_threshold: float = 0.40  # meters

    # Velocity limits & Dynamics (Calibrated against Gazebo)
    max_linear_vel: float = RL_MAX_LINEAR_VEL        # 0.40 m/s
    min_linear_vel: float = RL_MIN_LINEAR_VEL        # 0.00 m/s
    allow_reverse: bool = True
    min_reverse_vel: float = RL_MIN_REVERSE_VEL      # -0.10 m/s (reverse only below 0.1 m/s)
    max_angular_vel: float = RL_MAX_ANGULAR_VEL      # 0.75 rad/s
    max_accel_v: float = 1.50           # m/s^2 (velocity smoother acceleration limit)
    max_accel_w: float = 3.20           # rad/s^2
    tau_v: float = RL_TAU_V             # 0.22 s (first-order linear velocity lag)
    tau_w: float = RL_TAU_W             # 0.20 s (first-order angular velocity lag)
    latency_ms: float = RL_LATENCY_MS   # 125.0 ms latency buffer (115-140 ms)
    w_gain: float = RL_W_GAIN           # 0.75 (angular slip 25%)
    action_rate_limit: float = 0.30     # max normalized action change per step

    # Control timing
    control_rate_hz: float = 10.0       # 10 Hz (100 ms step)
    max_episode_steps: int = 300        # 30 seconds at 10 Hz
    goal_tolerance: float = 0.35        # meters
    curriculum_stage: int = 1           # 1 to 5
    doorway_width: float = 1.20         # 1.2 m door with 26 cm margins for 0.68m robot

    # Reward weights (Item 3: Anti-bang-bang & circling penalties)
    progress_weight: float = 2.5
    goal_reward: float = 100.0
    collision_penalty: float = 100.0
    step_penalty: float = 0.02
    smoothness_weight: float = 0.05
    proximity_weight: float = 0.10
    spin_penalty: float = 0.05
    monitor_slow_penalty: float = 1.00
    monitor_stop_penalty: float = 5.00
    timeout_penalty: float = 5.00
    action_delta_penalty: float = 0.20             # penalty on |delta a|
    small_heading_w_penalty: float = 0.30          # penalty on |a_w| when |heading error| < 15 deg
    heading_align_reward: float = 0.10             # small heading-alignment reward
    circling_penalty: float = 2.00                 # penalty on circling (no progress over 3s)


def point_to_segment_dist(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Computes minimum Euclidean distance from 2D point (px, py) to line segment (x1, y1)-(x2, y2)."""
    dx = x2 - x1
    dy = y2 - y1
    l2 = dx * dx + dy * dy
    if l2 < 1e-6:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / l2))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def min_dist_to_obstacles(px: float, py: float, obstacles: List[Tuple[float, float, float, float]]) -> float:
    """Computes minimum clearance from point to list of obstacle segments."""
    if not obstacles:
        return float("inf")
    return min(point_to_segment_dist(px, py, x1, y1, x2, y2) for x1, y1, x2, y2 in obstacles)


class KinematicSimulation:
    """
    High-performance 2D unicycle kinematic simulator with vectorized ray-casting,
    acceleration limits, and 100-200 ms latency simulation.
    """
    def __init__(self, config: NavEnvConfig):
        self.config = config
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.dt = 1.0 / config.control_rate_hz

        # Latency buffer (100 - 200 ms latency simulation)
        buf_len = max(1, int(round((config.latency_ms / 1000.0) / self.dt)))
        self.latency_buffer: collections.deque = collections.deque([(0.0, 0.0)] * buf_len, maxlen=buf_len)

        # Obstacles: list of 2D line segments [(x1, y1, x2, y2), ...]
        self.obstacles: List[Tuple[float, float, float, float]] = []
        self._init_default_obstacles()

    def _init_default_obstacles(self):
        """Standard room bounding box, realistic 1.2m doorway geometry, and curriculum obstacles."""
        half_door = self.config.doorway_width / 2.0  # 0.60 m -> 1.2 m doorway
        arena_size = 7.5  # 15m x 15m training space for 1-8m goals

        # Perimeter walls
        self.obstacles = [
            (-arena_size, -arena_size, arena_size, -arena_size),
            (arena_size, -arena_size, arena_size, arena_size),
            (arena_size, arena_size, -arena_size, arena_size),
            (-arena_size, arena_size, -arena_size, -arena_size),
        ]

        # Central dividing wall with 1.2m doorway
        self.obstacles.extend([
            (0.0, -arena_size, 0.0, -half_door),
            (0.0, half_door, 0.0, arena_size),
        ])

        if self.config.curriculum_stage >= 2:
            # Additional dividing partition and clutter
            self.obstacles.extend([
                (-arena_size, 0.0, -half_door - 1.5, 0.0),
                (-1.5, 0.0, 0.0, 0.0),
                # Clutter boxes/pillars
                (-3.5, -3.5, -2.7, -3.5),
                (3.0, 3.0, 3.8, 3.0),
            ])

        if self.config.curriculum_stage >= 3:
            # Additional doorways, corridors, and clutter
            self.obstacles.extend([
                (-4.0, 2.0, -4.0, 3.0),
                (-4.0, 3.0, -3.0, 3.0),
                (2.5, -3.5, 3.5, -3.5),
                (3.5, -3.5, 3.5, -2.5),
                (-2.0, -4.5, -2.0, -3.0),
                (4.0, -1.0, 5.0, -1.0),
            ])

        if self.config.curriculum_stage >= 4:
            # Apartment layout stage: authentic 18m central hallway and room doorways
            self.obstacles.extend([
                (-arena_size, 1.0, -5.6, 1.0),
                (-4.4, 1.0, 4.4, 1.0),
                (5.6, 1.0, arena_size, 1.0),
                (-arena_size, -1.0, -5.6, -1.0),
                (-4.4, -1.0, 4.4, -1.0),
                (5.6, -1.0, arena_size, -1.0),
            ])

    def reset(self, start_x: float = 0.0, start_y: float = 0.0, start_yaw: float = 0.0):
        self.x = start_x
        self.y = start_y
        self.yaw = start_yaw
        self.vx = 0.0
        self.wz = 0.0
        buf_len = self.latency_buffer.maxlen or 1
        self.latency_buffer = collections.deque([(0.0, 0.0)] * buf_len, maxlen=buf_len)

    def step(self, target_vx: float, target_wz: float):
        # 1. Push target command to latency queue and pop delayed command (115-140 ms latency)
        self.latency_buffer.append((target_vx, target_wz))
        delayed_vx, delayed_wz = self.latency_buffer[0]

        # 2. Apply angular slip / transfer gain (25% slip -> w_gain = 0.75)
        effective_wz = delayed_wz * self.config.w_gain

        # 3. First-order velocity lag response
        # v: tau = 0.22 s, w: tau = 0.20 s
        alpha_v = 1.0 - math.exp(-self.dt / max(0.01, self.config.tau_v))
        alpha_w = 1.0 - math.exp(-self.dt / max(0.01, self.config.tau_w))

        clipped_target_vx = float(np.clip(
            delayed_vx,
            self.config.min_reverse_vel if self.config.allow_reverse else 0.0,
            self.config.max_linear_vel
        ))
        clipped_target_wz = float(np.clip(
            effective_wz,
            -self.config.max_angular_vel,
            self.config.max_angular_vel
        ))

        filt_vx = self.vx + alpha_v * (clipped_target_vx - self.vx)
        filt_wz = self.wz + alpha_w * (clipped_target_wz - self.wz)

        # 4. Bound acceleration limits
        max_dv = self.config.max_accel_v * self.dt
        max_dw = self.config.max_accel_w * self.dt
        self.vx = float(np.clip(filt_vx, self.vx - max_dv, self.vx + max_dv))
        self.wz = float(np.clip(filt_wz, self.wz - max_dw, self.wz + max_dw))

        # 5. Unicycle forward integration
        self.yaw += self.wz * self.dt
        self.yaw = (self.yaw + np.pi) % (2.0 * np.pi) - np.pi  # Wrap [-pi, pi]
        self.x += self.vx * math.cos(self.yaw) * self.dt
        self.y += self.vx * math.sin(self.yaw) * self.dt

    def compute_scan(self, num_beams: int = 72) -> np.ndarray:
        """
        High-performance vectorized 360-degree LiDAR ray-casting with 24% inf rate,
        0.40m min-range cutoff, and Gaussian noise.
        """
        angles = np.linspace(-np.pi, np.pi, num_beams, endpoint=False, dtype=np.float32) + np.float32(self.yaw)
        dx = np.cos(angles)[:, None]  # Shape: (B, 1)
        dy = np.sin(angles)[:, None]  # Shape: (B, 1)

        obs = np.array(self.obstacles, dtype=np.float32)  # Shape: (S, 4)
        x1 = obs[:, 0][None, :]
        y1 = obs[:, 1][None, :]
        x2 = obs[:, 2][None, :]
        y2 = obs[:, 3][None, :]

        v1x = np.float32(self.x) - x1
        v1y = np.float32(self.y) - y1
        v2x = x2 - x1
        v2y = y2 - y1
        v3x = -dy
        v3y = dx

        dot = v2x * v3x + v2y * v3y
        dot_mask = np.abs(dot) > 1e-6

        safe_dot = np.where(dot_mask, dot, 1.0)
        t1 = (v2x * v1y - v2y * v1x) / safe_dot
        t2 = (v1x * v3x + v1y * v3y) / safe_dot

        min_r = self.config.min_scan_range
        max_r = self.config.max_scan_range

        valid = dot_mask & (t1 >= min_r) & (t1 <= max_r) & (t2 >= 0.0) & (t2 <= 1.0)
        t1_valid = np.where(valid, t1, max_r)
        ranges = np.min(t1_valid, axis=1)

        # Realistic 24% inf rate in open space
        inf_mask = (ranges >= (max_r - 0.05)) | (np.random.random(num_beams) < (self.config.inf_beam_rate * 0.4))
        ranges = np.where(inf_mask, np.inf, ranges)

        # Min-range cutoff (0.40 m chassis blindspot)
        ranges = np.where(np.isinf(ranges), np.inf, np.maximum(min_r, ranges))

        # Add Gaussian scan noise
        if self.config.scan_noise_sigma > 0.0:
            noise = np.random.normal(0.0, self.config.scan_noise_sigma, size=num_beams).astype(np.float32)
            ranges = np.where(np.isinf(ranges), np.inf, np.clip(ranges + noise, min_r, max_r))

        return ranges.astype(np.float32)


class GracemoNavEnv(gym.Env):
    """
    Gymnasium Environment for GRaCEmo Mobile Robot Navigation.
    Continuous action space: [v_normalized, w_normalized] in [-1.0, 1.0].
    Normalized observation vector of 79 dimensions.
    """
    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, mode: str = "kinematic", config: Optional[NavEnvConfig] = None):
        super().__init__()
        self.mode = mode.lower()
        self.config = config or NavEnvConfig()

        # Observation Space:
        # [0:72]   : Downsampled LiDAR ranges normalized [0.0, 1.0]
        # [72]     : Normalized goal distance tanh(d / 5.0) in [0.0, 1.0]
        # [73, 74] : Goal bearing sin(theta), cos(theta) in [-1.0, 1.0]
        # [75, 76] : Last action [v_norm, w_norm] in [-1.0, 1.0]
        # [77, 78] : Current robot velocity [v / v_max, w / w_max] in [-1.0, 1.0]
        obs_dim = self.config.downsampled_scan_beams + 1 + 2 + 2 + 2
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32
        )

        # Action Space: continuous [normalized_v, normalized_w] in [-1.0, 1.0]
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        # Environment state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.goal_x = 3.0
        self.goal_y = 0.0
        self.prev_distance = 0.0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.step_count = 0
        self.total_path_length = 0.0
        self.monitor_trigger_count = 0
        self.pose_history: collections.deque = collections.deque([(0.0, 0.0)], maxlen=30)

        # Subsystems
        if self.mode == "kinematic":
            self.sim = KinematicSimulation(self.config)
            self.ros_node = None
        else:
            self.sim = None
            self._init_ros_bridge()

    def set_curriculum_stage(self, stage: int):
        """Updates curriculum stage and reconfigures obstacle geometry."""
        self.config.curriculum_stage = int(stage)
        if self.mode == "kinematic" and self.sim is not None:
            self.sim.config.curriculum_stage = int(stage)
            self.sim._init_default_obstacles()

    def _init_ros_bridge(self):
        """Initializes ROS 2 interfaces when running in Gazebo live mode."""
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist

        if not rclpy.ok():
            rclpy.init()

        self.ros_node = Node(f"gracemo_rl_env_{int(time.time()*1000)%100000}")
        self.latest_scan = None
        self.latest_odom = None
        self.latest_monitor_state = 0

        self.sub_scan = self.ros_node.create_subscription(
            LaserScan, "/scan", self._on_scan, 10
        )
        self.sub_odom = self.ros_node.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        self.pub_cmd_vel = self.ros_node.create_publisher(
            Twist, "/cmd_vel_rl", 10
        )

    def _on_scan(self, msg):
        self.latest_scan = msg

    def _on_odom(self, msg):
        self.latest_odom = msg

    def scale_action(self, action: np.ndarray) -> Tuple[float, float]:
        """Maps normalized action [-1, 1] to physical [v, w]."""
        a_v, a_w = float(action[0]), float(action[1])
        if self.config.allow_reverse:
            # Map [-1, 1] -> [min_reverse_vel, max_linear_vel]
            v = (a_v + 1.0) / 2.0 * (self.config.max_linear_vel - self.config.min_reverse_vel) + self.config.min_reverse_vel
        else:
            v = max(0.0, (a_v + 1.0) / 2.0 * self.config.max_linear_vel)
        w = a_w * self.config.max_angular_vel
        return float(v), float(w)

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None) -> Tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        self.step_count = 0
        self.total_path_length = 0.0
        self.last_action = np.zeros(2, dtype=np.float32)
        self.monitor_trigger_count = 0
        self.pose_history: collections.deque = collections.deque(maxlen=30)

        # Sample collision-free start and goal
        if options and "start" in options and "goal" in options:
            start_x, start_y, start_yaw = options["start"]
            self.goal_x, self.goal_y = options["goal"]
        else:
            rng = np.random.default_rng(seed)

            if self.mode == "kinematic":
                self.sim._init_default_obstacles()

            # 1. Sample collision-free start pose
            start_x, start_y = 0.0, 0.0
            for _ in range(100):
                sx = float(rng.uniform(-6.0, 6.0))
                sy = float(rng.uniform(-6.0, 6.0))
                if self.mode != "kinematic" or min_dist_to_obstacles(sx, sy, self.sim.obstacles) >= 0.60:
                    start_x, start_y = sx, sy
                    break

            start_yaw = float(rng.uniform(-np.pi, np.pi))

            # 2. Sample goal with distance 1.0 to 8.0 m and uniform relative bearing [-pi, pi]
            max_curriculum_dist = min(8.0, 2.5 + self.config.curriculum_stage * 1.5)
            goal_found = False
            for _ in range(100):
                target_dist = float(rng.uniform(1.0, max_curriculum_dist))
                target_bearing = float(rng.uniform(-np.pi, np.pi))
                world_bearing = start_yaw + target_bearing
                gx = start_x + target_dist * math.cos(world_bearing)
                gy = start_y + target_dist * math.sin(world_bearing)
                if abs(gx) < 7.0 and abs(gy) < 7.0:
                    if self.mode != "kinematic" or min_dist_to_obstacles(gx, gy, self.sim.obstacles) >= 0.50:
                        self.goal_x, self.goal_y = gx, gy
                        goal_found = True
                        break

            if not goal_found:
                self.goal_x = float(np.clip(start_x + 2.0 * math.cos(start_yaw), -6.5, 6.5))
                self.goal_y = float(np.clip(start_y + 2.0 * math.sin(start_yaw), -6.5, 6.5))

        self.robot_x = start_x
        self.robot_y = start_y
        self.robot_yaw = start_yaw
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.prev_distance = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)

        if self.mode == "kinematic":
            self.sim.reset(start_x, start_y, start_yaw)
            scan = self.sim.compute_scan(self.config.downsampled_scan_beams)
        else:
            scan = self._get_live_scan()

        obs = self._build_observation(scan)
        info = {
            "start": (start_x, start_y, start_yaw),
            "goal": (self.goal_x, self.goal_y),
            "distance_to_goal": self.prev_distance
        }
        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, dict]:
        self.step_count += 1
        raw_action = np.clip(action, -1.0, 1.0).astype(np.float32)

        # 1. Apply action-rate limit (Item 3: prevent bang-bang instantaneous switching)
        delta_a = np.clip(raw_action - self.last_action, -self.config.action_rate_limit, self.config.action_rate_limit)
        effective_action = np.clip(self.last_action + delta_a, -1.0, 1.0).astype(np.float32)

        target_v, target_w = self.scale_action(effective_action)

        prev_x, prev_y = self.robot_x, self.robot_y

        if self.mode == "kinematic":
            self.sim.step(target_v, target_w)
            self.robot_x = self.sim.x
            self.robot_y = self.sim.y
            self.robot_yaw = self.sim.yaw
            self.robot_vx = self.sim.vx
            self.robot_wz = self.sim.wz
            scan = self.sim.compute_scan(self.config.downsampled_scan_beams)
        else:
            self._send_live_velocity(target_v, target_w)
            scan = self._get_live_scan()
            self._update_live_pose()

        # Update pose history for anti-circling evaluation (Item 3)
        self.pose_history.append((self.robot_x, self.robot_y))

        # Update path length and goal metrics
        step_disp = math.hypot(self.robot_x - prev_x, self.robot_y - prev_y)
        self.total_path_length += step_disp

        curr_distance = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
        progress = self.prev_distance - curr_distance
        self.prev_distance = curr_distance

        # Clean scan for local frame polygon checks
        clean_scan = np.where(np.isinf(scan) | np.isnan(scan), self.config.max_scan_range, scan)

        # Monitor polygon checks in robot local frame
        angles = np.linspace(-np.pi, np.pi, len(clean_scan), endpoint=False, dtype=np.float32)
        px = clean_scan * np.cos(angles)
        py = clean_scan * np.sin(angles)

        # PolygonStop: [[0.36, 0.34], [0.36, -0.34], [-0.35, -0.34], [-0.35, 0.34]]
        stop_mask = (px >= -0.35) & (px <= 0.36) & (np.abs(py) <= 0.34)
        in_stop_polygon = bool(np.sum(stop_mask) >= 2)

        # PolygonSlow: [[0.65, 0.38], [0.65, -0.38], [-0.35, -0.38], [-0.35, 0.38]]
        slow_mask = (px >= -0.35) & (px <= 0.65) & (np.abs(py) <= 0.38)
        in_slow_polygon = bool(np.sum(slow_mask) >= 2)

        if in_stop_polygon:
            self.monitor_trigger_count += 1

        # Evaluate termination conditions
        min_scan = float(np.min(clean_scan)) if len(clean_scan) > 0 else self.config.max_scan_range
        if self.mode == "kinematic":
            actual_clearance = min_dist_to_obstacles(self.robot_x, self.robot_y, self.sim.obstacles)
            collision = (actual_clearance < self.config.collision_threshold)
            min_clearance = min(min_scan, actual_clearance)
        else:
            min_clearance = min_scan
            collision = (min_scan < self.config.collision_threshold)

        success = (curr_distance <= self.config.goal_tolerance)
        timeout = (self.step_count >= self.config.max_episode_steps)
        terminated = bool(collision or success)
        truncated = bool(timeout and not terminated)

        # Compute composite shaped reward
        reward = self._compute_reward(
            progress=progress,
            curr_distance=curr_distance,
            min_scan=min_scan,
            scan=scan,
            action=effective_action,
            target_v=target_v,
            target_w=target_w,
            collision=collision,
            success=success,
            timeout=timeout,
            in_stop_polygon=in_stop_polygon,
            in_slow_polygon=in_slow_polygon
        )

        self.last_action = effective_action.copy()
        obs = self._build_observation(scan)

        info = {
            "success": success,
            "collision": collision,
            "timeout": timeout,
            "min_clearance": min_scan,
            "distance_to_goal": curr_distance,
            "path_length": self.total_path_length,
            "monitor_triggers": self.monitor_trigger_count,
            "step": self.step_count
        }
        return obs, float(reward), terminated, truncated, info

    def _compute_reward(
        self,
        progress: float,
        curr_distance: float,
        min_scan: float,
        action: np.ndarray,
        target_v: float,
        target_w: float,
        collision: bool,
        success: bool,
        timeout: bool,
        scan: Optional[np.ndarray] = None,
        in_stop_polygon: bool = False,
        in_slow_polygon: bool = False
    ) -> float:
        """Comprehensive multi-objective navigation reward function with anti-bang-bang penalties."""
        # Terminal rewards
        if success:
            return self.config.goal_reward
        if collision:
            return -self.config.collision_penalty

        reward = 0.0

        # 1. Potential progress reward
        reward += self.config.progress_weight * progress

        # 2. Time-step penalty
        reward -= self.config.step_penalty

        # 3. Action smoothness and delta penalties (Item 3)
        delta_a_norm = float(np.linalg.norm(action - self.last_action))
        reward -= self.config.action_delta_penalty * delta_a_norm

        # 4. Anti-bang-bang penalty when heading is aligned (|heading error| < 15 deg) (Item 3)
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        heading_err = (math.atan2(dy, dx) - self.robot_yaw + math.pi) % (2.0 * math.pi) - math.pi
        if abs(heading_err) < math.radians(15.0):
            reward -= self.config.small_heading_w_penalty * abs(action[1])

        # 5. Small heading-alignment reward (Item 3)
        reward += self.config.heading_align_reward * math.cos(heading_err)

        # 6. Anti-circling penalty: evaluate trajectory over 3 s (30 steps at 10 Hz) (Item 3)
        if len(self.pose_history) >= 30:
            hist_x0, hist_y0 = self.pose_history[0]
            net_disp_3s = math.hypot(self.robot_x - hist_x0, self.robot_y - hist_y0)
            arc_len_3s = sum(
                math.hypot(self.pose_history[k][0] - self.pose_history[k-1][0], self.pose_history[k][1] - self.pose_history[k-1][1])
                for k in range(1, len(self.pose_history))
            )
            if arc_len_3s > 0.50 and net_disp_3s < 0.25:
                reward -= self.config.circling_penalty

        # 7. Proximity penalty when navigating close to obstacles (< 0.5m)
        if min_scan < self.config.proximity_threshold:
            prox_ratio = (self.config.proximity_threshold - min_scan) / self.config.proximity_threshold
            reward -= self.config.proximity_weight * prox_ratio

        # 8. Collision monitor stop/slow polygon penalties
        if in_stop_polygon:
            reward -= self.config.monitor_stop_penalty
            if target_v > 0.0:
                reward -= 10.0 * target_v  # Severe penalty for driving forward while violating stop polygon
        elif in_slow_polygon and target_v > 0.15:
            reward -= self.config.monitor_slow_penalty * ((target_v - 0.15) / 0.25)

        # 9. High speed near obstacle penalty
        if min_scan < 0.60 and target_v > 0.15:
            speed_near_obs_penalty = 2.0 * (target_v - 0.15) * ((0.60 - min_scan) / 0.60)
            reward -= speed_near_obs_penalty

        # 10. Reverse penalty when goal is ahead
        cos_b = math.cos(heading_err)
        if cos_b > 0.0 and target_v < -0.01:
            reward -= 3.0 * abs(target_v)

        # 11. Spin-in-place penalty (spinning fast with zero linear progress)
        if abs(target_w) > 0.5 and abs(target_v) < 0.05:
            reward -= self.config.spin_penalty

        # 12. Rear-obstacle penalty: heavily penalize reverse commands when obstacles are behind
        if scan is not None and len(scan) >= 14:
            rear_scan = np.concatenate([scan[:7], scan[-7:]])
            clean_rear = np.where(np.isinf(rear_scan) | np.isnan(rear_scan), self.config.max_scan_range, rear_scan)
            if np.min(clean_rear) < self.config.rear_obstacle_threshold and target_v < -0.01:
                reward -= 5.0 * abs(target_v)

        # 13. Timeout penalty
        if timeout:
            reward -= self.config.timeout_penalty

        return reward

    def _build_observation(self, scan: np.ndarray) -> np.ndarray:
        """Constructs authoritative normalized 79-dimensional observation vector."""
        return build_observation(
            scan_ranges=scan,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            robot_yaw=self.robot_yaw,
            goal_x=self.goal_x,
            goal_y=self.goal_y,
            last_action=self.last_action,
            robot_vx=self.robot_vx,
            robot_wz=self.robot_wz,
            max_linear_vel=self.config.max_linear_vel,
            max_angular_vel=self.config.max_angular_vel,
            min_scan_range=self.config.min_scan_range,
            max_scan_range=self.config.max_scan_range,
            downsampled_beams=self.config.downsampled_scan_beams
        )

    def _get_live_scan(self) -> np.ndarray:
        """Downsamples raw ROS 2 LaserScan to downsampled_scan_beams in meters."""
        if self.latest_scan is None or len(self.latest_scan.ranges) == 0:
            return np.full(self.config.downsampled_scan_beams, self.config.max_scan_range, dtype=np.float32)

        norm_scan = downsample_scan_ranges(
            self.latest_scan.ranges,
            downsampled_beams=self.config.downsampled_scan_beams,
            min_range=self.config.min_scan_range,
            max_range=self.config.max_scan_range
        )
        return (norm_scan * self.config.max_scan_range).astype(np.float32)

    def _update_live_pose(self):
        if self.latest_odom is not None:
            p = self.latest_odom.pose.pose.position
            self.robot_x = p.x
            self.robot_y = p.y
            q = self.latest_odom.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
            self.robot_vx = self.latest_odom.twist.twist.linear.x
            self.robot_wz = self.latest_odom.twist.twist.angular.z

    def _send_live_velocity(self, v: float, w: float):
        if self.pub_cmd_vel is not None:
            from geometry_msgs.msg import Twist
            msg = Twist()
            msg.linear.x = float(v)
            msg.angular.z = float(w)
            self.pub_cmd_vel.publish(msg)
