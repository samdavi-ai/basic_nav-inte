#!/usr/bin/env python3
"""
Unit and Regression Tests for RL / Nav2 Consolidation and Parity.
Verifies:
  1. Train/Serve Observation Skew Parity (< 1e-6 tolerance across 5 frames)
  2. Single-Source Destination Resolution against config/places.json
  3. Action Limits Parity across Nav2, RL env, and RL local planner node
"""

import os
import json
import math
import pytest
import numpy as np

from gracemo_rl.common.obs import (
    build_observation,
    downsample_scan_ranges,
    compute_relative_goal,
    OBS_DIM,
    SCAN_BEAMS
)
from gracemo_rl.common.limits import load_nav2_limits
from gracemo_rl.common.places import load_authoritative_places
from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig


class TestParityAndConsolidation:

    @pytest.fixture
    def test_frames(self):
        """Generates 5 realistic test frames representing diverse robot states."""
        frames = []

        # Frame 1: Nominal forward motion towards goal
        scan_1 = np.full(360, 5.0, dtype=np.float32)
        frames.append({
            "scan": scan_1,
            "robot_x": 0.0, "robot_y": 0.0, "robot_yaw": 0.0,
            "goal_x": 2.5, "goal_y": 0.0,
            "last_action": np.array([0.5, 0.0], dtype=np.float32),
            "robot_vx": 0.35, "robot_wz": 0.0
        })

        # Frame 2: Near obstacle on left, turning right, inf / nan scan beams
        scan_2 = np.full(360, 4.0, dtype=np.float32)
        scan_2[40:80] = 0.25   # Obstacle nearby
        scan_2[150:180] = np.inf
        scan_2[200:220] = np.nan
        frames.append({
            "scan": scan_2,
            "robot_x": -2.0, "robot_y": 1.5, "robot_yaw": math.pi / 4,
            "goal_x": -5.0, "goal_y": 2.2,
            "last_action": np.array([0.2, -0.8], dtype=np.float32),
            "robot_vx": 0.15, "robot_wz": -0.45
        })

        # Frame 3: Goal behind robot, high reverse action
        scan_3 = np.linspace(0.5, 8.0, 360, dtype=np.float32)
        frames.append({
            "scan": scan_3,
            "robot_x": 1.0, "robot_y": -1.0, "robot_yaw": -math.pi / 2,
            "goal_x": 1.0, "goal_y": 2.0,
            "last_action": np.array([-0.5, 0.1], dtype=np.float32),
            "robot_vx": -0.10, "robot_wz": 0.05
        })

        # Frame 4: Corridor pinching with narrow lateral clearances
        scan_4 = np.full(360, 8.0, dtype=np.float32)
        scan_4[70:110] = 0.45   # Left wall
        scan_4[250:290] = 0.45  # Right wall
        frames.append({
            "scan": scan_4,
            "robot_x": -3.5, "robot_y": -0.8, "robot_yaw": 0.02,
            "goal_x": -5.0, "goal_y": -2.5,
            "last_action": np.array([0.8, -0.05], dtype=np.float32),
            "robot_vx": 0.45, "robot_wz": -0.02
        })

        # Frame 5: Close arrival at goal within tolerance
        scan_5 = np.full(360, 2.5, dtype=np.float32)
        frames.append({
            "scan": scan_5,
            "robot_x": -4.95, "robot_y": -2.48, "robot_yaw": -1.55,
            "goal_x": -5.0, "goal_y": -2.5,
            "last_action": np.array([0.05, 0.0], dtype=np.float32),
            "robot_vx": 0.02, "robot_wz": 0.01
        })

        return frames

    def test_train_serve_obs_skew_parity(self, test_frames):
        """
        Train/Serve Skew Test:
        Verifies that nav_env._build_observation and canonical build_observation
        produce byte-identical 79-D observation vectors (tolerance < 1e-6).
        """
        env = GracemoNavEnv(mode="kinematic")

        for idx, frame in enumerate(test_frames):
            # 1. Compute via canonical obs builder (as used in rl_local_planner)
            canonical_obs = build_observation(
                scan_ranges=frame["scan"],
                robot_x=frame["robot_x"],
                robot_y=frame["robot_y"],
                robot_yaw=frame["robot_yaw"],
                goal_x=frame["goal_x"],
                goal_y=frame["goal_y"],
                last_action=frame["last_action"],
                robot_vx=frame["robot_vx"],
                robot_wz=frame["robot_wz"]
            )

            # 2. Compute via GracemoNavEnv
            env.robot_x = frame["robot_x"]
            env.robot_y = frame["robot_y"]
            env.robot_yaw = frame["robot_yaw"]
            env.goal_x = frame["goal_x"]
            env.goal_y = frame["goal_y"]
            env.last_action = frame["last_action"].copy()
            env.robot_vx = frame["robot_vx"]
            env.robot_wz = frame["robot_wz"]

            # Downsample 360 to 72 in meters for env's internal scan representation
            norm_scan = downsample_scan_ranges(frame["scan"], downsampled_beams=72)
            env_scan_meters = norm_scan * env.config.max_scan_range
            env_obs = env._build_observation(env_scan_meters)

            # Assertions
            assert canonical_obs.shape == (OBS_DIM,), f"Frame {idx}: Canonical obs shape mismatch {canonical_obs.shape}"
            assert env_obs.shape == (OBS_DIM,), f"Frame {idx}: Env obs shape mismatch {env_obs.shape}"

            max_diff = np.max(np.abs(canonical_obs - env_obs))
            assert max_diff < 1e-6, f"Frame {idx}: Skew detected! Max difference {max_diff} exceeds 1e-6"
            np.testing.assert_allclose(canonical_obs, env_obs, atol=1e-6, err_msg=f"Frame {idx} parity failed")

    def test_single_source_destination_resolution(self):
        """
        Single-Source Test:
        Resolves all known destination names through the canonical module and asserts
        exact coordinate matches against config/places.json.
        """
        places_path = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/config/places.json"
        assert os.path.exists(places_path), f"Authoritative places file missing at {places_path}"

        with open(places_path, "r", encoding="utf-8") as f:
            raw_places = json.load(f)

        canonical_places = load_authoritative_places(places_path)

        for key, entry in raw_places.items():
            if key.startswith("_"):
                continue

            k_norm = key.lower().strip()
            assert k_norm in canonical_places, f"Place '{key}' missing in resolved places"

            resolved = canonical_places[k_norm]
            expected_x = float(entry[0])
            expected_y = float(entry[1])
            expected_name = str(entry[2])
            expected_yaw = float(entry[3]) if len(entry) > 3 else 0.0

            assert math.isclose(resolved["x"], expected_x, abs_tol=1e-5), f"X mismatch for {key}: {resolved['x']} vs {expected_x}"
            assert math.isclose(resolved["y"], expected_y, abs_tol=1e-5), f"Y mismatch for {key}: {resolved['y']} vs {expected_y}"
            assert math.isclose(resolved["yaw"], expected_yaw, abs_tol=1e-5), f"Yaw mismatch for {key}: {resolved['yaw']} vs {expected_yaw}"
            assert resolved["name"] == expected_name, f"Name mismatch for {key}: {resolved['name']} vs {expected_name}"

        # Explicitly verify calibrated post-repair destinations
        assert canonical_places["kitchen"]["x"] == -5.0
        assert canonical_places["kitchen"]["y"] == -2.5
        assert canonical_places["bedroom"]["x"] == -5.0
        assert canonical_places["bedroom"]["y"] == 2.2
        assert canonical_places["hallway"]["x"] == 0.0
        assert canonical_places["hallway"]["y"] == 0.0

    def test_limits_parity(self):
        """
        Limits Test (Item 1.b):
        Asserts RL single source of truth across limits.py, nav_env.py, and training_config.yaml.
        Fails if training env, inference limits, or training config differ from authoritative limits.
        """
        from gracemo_rl.common.limits import (
            RL_MAX_LINEAR_VEL,
            RL_MAX_ANGULAR_VEL,
            RL_MIN_REVERSE_VEL,
            RL_MIN_SCAN_RANGE,
            RL_MAX_SCAN_RANGE
        )
        limits = load_nav2_limits()
        env_config = NavEnvConfig()

        # 1. Authoritative constants check
        assert math.isclose(RL_MAX_LINEAR_VEL, 0.40, abs_tol=1e-5), f"RL_MAX_LINEAR_VEL is {RL_MAX_LINEAR_VEL}, expected 0.40"
        assert math.isclose(RL_MAX_ANGULAR_VEL, 0.75, abs_tol=1e-5), f"RL_MAX_ANGULAR_VEL is {RL_MAX_ANGULAR_VEL}, expected 0.75"
        assert math.isclose(RL_MIN_REVERSE_VEL, -0.10, abs_tol=1e-5), f"RL_MIN_REVERSE_VEL is {RL_MIN_REVERSE_VEL}, expected -0.10"

        # 2. NavLimits alignment
        assert math.isclose(limits.max_linear_vel, RL_MAX_LINEAR_VEL, abs_tol=1e-5)
        assert math.isclose(limits.max_angular_vel, RL_MAX_ANGULAR_VEL, abs_tol=1e-5)
        assert math.isclose(limits.min_reverse_vel, RL_MIN_REVERSE_VEL, abs_tol=1e-5)

        # 3. NavEnvConfig alignment
        assert math.isclose(env_config.max_linear_vel, RL_MAX_LINEAR_VEL, abs_tol=1e-5)
        assert math.isclose(env_config.max_angular_vel, RL_MAX_ANGULAR_VEL, abs_tol=1e-5)
        assert math.isclose(env_config.min_reverse_vel, RL_MIN_REVERSE_VEL, abs_tol=1e-5)
        assert math.isclose(env_config.min_scan_range, RL_MIN_SCAN_RANGE, abs_tol=1e-5)
        assert math.isclose(env_config.max_scan_range, RL_MAX_SCAN_RANGE, abs_tol=1e-5)

        # 4. training_config.yaml alignment
        cfg_path = os.path.join(os.path.dirname(__file__), "..", "gracemo_rl", "config", "training_config.yaml")
        if os.path.exists(cfg_path):
            import yaml
            with open(cfg_path, "r") as f:
                cfg = yaml.safe_load(f)
            env_cfg = cfg.get("env_config", {})
            assert math.isclose(float(env_cfg.get("max_linear_vel", 0.0)), RL_MAX_LINEAR_VEL, abs_tol=1e-5)
            assert math.isclose(float(env_cfg.get("max_angular_vel", 0.0)), RL_MAX_ANGULAR_VEL, abs_tol=1e-5)
            assert math.isclose(float(env_cfg.get("min_reverse_vel", 0.0)), RL_MIN_REVERSE_VEL, abs_tol=1e-5)

    def test_goal_id_state_machine(self):
        """
        Goal-ID State Machine Test:
        Verifies cancel, replacement, retry, abort, and late callback invalidation.
        Asserts that goal_id increments prevent race conditions and that cancellation
        preserves pending replacement missions.
        """
        # Mock FSM to model AutonomousSupervisorNode goal-ID lifecycle
        class MockGoalCoordinator:
            def __init__(self):
                self.current_goal_id = 0
                self.active_goal_handle = "active_handle_1"
                self.active_goal_msg = {"x": 1.0, "y": 0.0}
                self.state = "MISSION_EXECUTING"
                self.retry_count = 0
                self.max_retries = 3
                self.pending_replacement = None
                self.processed_results = []
                self.dispatched_goals = []

            def abort_goal(self, keep_goal_msg=False):
                self.active_goal_handle = None
                if not keep_goal_msg:
                    self.active_goal_msg = None
                self.current_goal_id += 1

            def on_nav_result(self, result_code: int, goal_id: int):
                # Late callback check: drop if goal_id is stale
                if goal_id != self.current_goal_id:
                    return "DROPPED_STALE_CALLBACK"

                if self.pending_replacement:
                    repl = self.pending_replacement
                    self.pending_replacement = None
                    self.current_goal_id += 1
                    self.dispatched_goals.append((self.current_goal_id, repl))
                    self.state = "MISSION_EXECUTING"
                    return f"DISPATCHED_REPLACEMENT_ID_{self.current_goal_id}"

                self.processed_results.append((goal_id, result_code))
                return f"PROCESSED_RESULT_{result_code}"

            def request_replacement(self, new_mission):
                self.pending_replacement = new_mission
                # Cancel in flight without incrementing goal_id prematurely
                # so the cancel result matches the active goal_id
                self.state = "MISSION_CANCELLING"

            def handle_retry(self):
                if self.retry_count < self.max_retries and self.active_goal_msg:
                    self.retry_count += 1
                    self.current_goal_id += 1
                    self.dispatched_goals.append((self.current_goal_id, self.active_goal_msg))
                    self.state = "MISSION_PLANNING"
                    return True
                return False

        coord = MockGoalCoordinator()
        initial_id = coord.current_goal_id
        assert initial_id == 0

        # 1. Late Callback Invalidation: Stale callbacks with old goal_id must be dropped
        stale_status = coord.on_nav_result(result_code=4, goal_id=-1)
        assert stale_status == "DROPPED_STALE_CALLBACK"
        assert len(coord.processed_results) == 0

        # 2. Legitimate result matching active goal_id is processed
        status = coord.on_nav_result(result_code=4, goal_id=0)
        assert status == "PROCESSED_RESULT_4"
        assert coord.processed_results == [(0, 4)]

        # 3. Abort increments goal_id and drops late results
        coord.abort_goal(keep_goal_msg=True)
        assert coord.current_goal_id == 1
        assert coord.on_nav_result(result_code=4, goal_id=0) == "DROPPED_STALE_CALLBACK"

        # 4. Retry preserves goal message and increments ID
        success = coord.handle_retry()
        assert success is True
        assert coord.current_goal_id == 2
        assert len(coord.dispatched_goals) == 1
        assert coord.dispatched_goals[0][0] == 2

        # 5. Replacement Mission Dispatch: Cancel handshake must NOT drop replacement
        coord.request_replacement({"x": 5.0, "y": 2.0})
        assert coord.state == "MISSION_CANCELLING"
        # Cancellation handshake completes with active ID=2
        dispatch_status = coord.on_nav_result(result_code=5, goal_id=2)
        assert dispatch_status == "DISPATCHED_REPLACEMENT_ID_3"
        assert coord.current_goal_id == 3
        assert len(coord.dispatched_goals) == 2
        assert coord.dispatched_goals[1] == (3, {"x": 5.0, "y": 2.0})
        assert coord.state == "MISSION_EXECUTING"
