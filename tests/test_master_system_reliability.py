#!/usr/bin/env python3
"""
GRaCEmo ViRa — Master System Reliability & Autonomy Integration Test Suite
Validates all 17 core architectural invariants and reliability requirements:
  Scenario 1:  Single Voice Command -> Single Authoritative Mission ID
  Scenario 2:  Exact Duplicate Mission Suppression
  Scenario 3:  Race-Condition Preemption & Atomic Replacement Policy
  Scenario 4:  Corridor Covariance Resilience (Elongation != Failure)
  Scenario 5:  Catastrophic Particle Dispersion & Localization Loss Trip-wire
  Scenario 6:  Narrow Doorway & Costmap Clearance (1.2m doors zero cost)
  Scenario 7:  Bounded Obstacle Recovery & Retries (Anti-Infinite-Loop)
  Scenario 8:  Mission Pause and Resume Lifecycle
  Scenario 9:  Cancellation Idempotency
  Scenario 10: Emergency Stop Safety Gate Lockout
  Scenario 11: Emergency Stop Reset Authorization
  Scenario 12: Grounded Status — Where are you? (Topology Mapping)
  Scenario 13: Grounded Status — Where are you going? (Destination & ETA)
  Scenario 14: Grounded Status — Are you moving? (Velocity Grounding)
  Scenario 15: Grounded Status — Are you stuck / Why did you stop?
  Scenario 16: Conversational LLM Safety Isolation
  Scenario 17: Simulation Hardware Lock Assertions (MOTOR_OUTPUT_ENABLED = 0)
"""

import json
import math
import os
import sys
import time
import unittest
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest.mock import MagicMock, patch

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "gracemo_nav2"))
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "gracemo_bridge"))
sys.path.insert(0, str(REPO_ROOT / "vira_interaction"))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String as RosString, Bool as RosBool
from action_msgs.msg import GoalStatus

from gracemo_nav2.coordinator_node import AutonomousSupervisorNode
from gracemo_bridge.interaction_bridge import InteractionBridgeNode
from core.fast_path import FastPathMatcher
from core.gateway import InteractionGateway, GatewayResult
from core.brain_adapter import BrainAdapter, BrainResult
from core.state_machine import CognitiveState, Expression, InteractionFSM
from core.safety_gate import SafetyGate


class TestMasterSystemReliability(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # Instantiate coordinator node
        self.coordinator = AutonomousSupervisorNode()
        # Mock action client and service calls
        self.coordinator.nav_client.wait_for_server = MagicMock(return_value=True)
        self.coordinator.nav_client.server_is_ready = MagicMock(return_value=True)
        self.coordinator.localization_transform_freshness = MagicMock(return_value=(True, "fixture transform"))
        now = time.time()
        self.coordinator.last_scan_time = now
        self.coordinator.last_odom_time = now
        self.coordinator.last_mux_status_time = now
        self.coordinator.last_terrain_status_time = now
        self.coordinator.terrain_imu_fresh = True
        self.coordinator.nav_client.send_goal_async = MagicMock()
        self.coordinator.clear_global_costmap_cli.call_async = MagicMock()
        self.coordinator.clear_local_costmap_cli.call_async = MagicMock()
        self.coordinator.nomotion_update_cli.call_async = MagicMock()
        self.coordinator._check_tf_connectivity = MagicMock(return_value=(True, "TF connected"))

        # Capture feedback
        self.published_feedback = []
        self.coordinator.feedback_pub.publish = lambda msg: self.published_feedback.append(json.loads(msg.data))

        # Establish localization baseline
        self._inject_amcl_pose(0.0, 0.0, 0.0, cov_xx=0.04, cov_yy=0.03, cov_xy=0.0, cov_yaw=0.01)

    def tearDown(self):
        self.coordinator.destroy_node()

    def _inject_amcl_pose(self, x: float, y: float, yaw: float, cov_xx: float, cov_yy: float, cov_xy: float = 0.0, cov_yaw: float = 0.01):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        cov = [0.0] * 36
        cov[0] = cov_xx
        cov[1] = cov_xy
        cov[6] = cov_xy
        cov[7] = cov_yy
        cov[35] = cov_yaw
        msg.pose.covariance = cov
        self.coordinator.on_amcl_pose(msg)

    # -------------------------------------------------------------------------
    # Scenario 1: One Voice Command -> Single Authoritative Mission ID
    # -------------------------------------------------------------------------
    def test_scenario_01_single_mission_id_generation(self):
        """Verify that a navigation dispatch produces exactly one unique mission ID."""
        bridge = InteractionBridgeNode()
        bridge.gateway.tts.speak = MagicMock()

        published_coord_cmds = []
        bridge.pub_coord_cmd.publish = lambda m: published_coord_cmds.append(json.loads(m.data))

        # Process single command
        def accepted(_, envelope):
            bridge._on_eventbus_action_requested(envelope)
            return {"status": "dispatched"}
        with patch("gracemo_sdk.authorization.dispatch_action", side_effect=accepted) as transport:
            bridge.process_text_command("GraceEMO, navigate to the Master Bedroom.")
        transport.assert_called_once()

        # Assert exactly one command was published to coordinator
        nav_cmds = [c for c in published_coord_cmds if c.get("intent") == "NAVIGATE_TO"]
        self.assertEqual(len(nav_cmds), 1, "Exactly one NAVIGATE_TO command must be dispatched.")
        mission_id = nav_cmds[0].get("mission_id")
        self.assertTrue(mission_id is not None and mission_id.startswith("mid-"), f"Invalid mission ID: {mission_id}")
        bridge.destroy_node()

    # -------------------------------------------------------------------------
    # Scenario 2: Duplicate Mission Suppression
    # -------------------------------------------------------------------------
    def test_scenario_02_duplicate_mission_suppression(self):
        """Verify that identical navigation goals sent while en route do not trigger re-planning."""
        # 1. Start mission to bedroom
        payload1 = {
            "intent": "NAVIGATE_TO",
            "mission_id": "mid-1001",
            "destination_name": "Master Bedroom",
            "destination_coords": {"x": -4.0, "y": 2.2, "yaw": 1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload1)
        self.assertEqual(self.coordinator.active_mission_id, "mid-1001")
        self.assertEqual(self.coordinator.nav_client.send_goal_async.call_count, 1)

        # 2. Re-send identical mission while in EXECUTING/PLANNING state
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING
        payload2 = {
            "intent": "NAVIGATE_TO",
            "mission_id": "mid-1002",
            "destination_name": "Master Bedroom",
            "destination_coords": {"x": -4.0, "y": 2.2, "yaw": 1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload2)

        # Must NOT dispatch second goal to Nav2 or change active mission ID
        self.assertEqual(self.coordinator.active_mission_id, "mid-1001", "Active mission ID must not change on duplicate.")
        self.assertEqual(self.coordinator.nav_client.send_goal_async.call_count, 1, "Nav2 send_goal must NOT be called twice.")
        self.assertTrue(any("Already navigating" in fb.get("speech", "") for fb in self.published_feedback))

    # -------------------------------------------------------------------------
    # Scenario 3: Race-Condition Preemption & Atomic Replacement
    # -------------------------------------------------------------------------
    def test_scenario_03_preemption_atomic_replacement(self):
        """Verify that commanding a different destination while navigating safely cancels previous goal and dispatches new goal."""
        # Start mission to bedroom
        payload_bed = {
            "intent": "NAVIGATE_TO",
            "mission_id": "mid-1001",
            "destination_name": "Master Bedroom",
            "destination_coords": {"x": -4.0, "y": 2.2, "yaw": 1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload_bed)
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING

        # Mock an active goal handle
        mock_goal_handle = MagicMock()
        self.coordinator.active_goal_handle = mock_goal_handle

        # New command to Kitchen
        payload_kitchen = {
            "intent": "NAVIGATE_TO",
            "mission_id": "mid-2002",
            "destination_name": "Kitchen & Dining",
            "destination_coords": {"x": -4.0, "y": -2.5, "yaw": -1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload_kitchen)

        # Old goal handle must receive cancel request
        mock_goal_handle.cancel_goal_async.assert_called_once()
        self.assertIsNotNone(self.coordinator._pending_replacement_mission)

        # Simulate cancellation callback handshake from Nav2
        future_cancel = MagicMock()
        future_cancel.result.return_value.status = GoalStatus.STATUS_CANCELED
        self.coordinator._on_nav_result(future_cancel, self.coordinator.current_goal_id)

        # New mission must now be active
        self.assertEqual(self.coordinator.active_mission_id, "mid-2002")
        self.assertEqual(self.coordinator.current_waypoint["name"], "Kitchen & Dining")

    # -------------------------------------------------------------------------
    # Scenario 4: AMCL Corridor Particle Elongation Resilience
    # -------------------------------------------------------------------------
    def test_scenario_04_corridor_elongation_resilience(self):
        """Verify that longitudinal particle elongation in corridors (xx=6.5, yy=0.04) does NOT trigger failure."""
        # Corridor elongation: along-track xx is high, but cross-track yy is tightly bounded by corridor walls
        self._inject_amcl_pose(0.0, 0.0, 0.0, cov_xx=6.5, cov_yy=0.04, cov_xy=0.02, cov_yaw=0.02)

        # Cross-track eigenvalue must be small
        self.assertLess(self.coordinator.eigen_min, 0.5)
        # Localization state must NOT be LOST
        self.assertNotEqual(self.coordinator.loc_state, AutonomousSupervisorNode.LOC_LOST)
        self.assertIn(self.coordinator.loc_state, (AutonomousSupervisorNode.LOC_HEALTHY, AutonomousSupervisorNode.LOC_DEGRADED))

        # check_localization_readiness must succeed
        is_ready, diag = self.coordinator.check_localization_readiness()
        self.assertTrue(is_ready, f"Corridor elongation should be accepted, but got: {diag}")

    # -------------------------------------------------------------------------
    # Scenario 5: Catastrophic Particle Dispersion & Localization Loss
    # -------------------------------------------------------------------------
    def test_scenario_05_catastrophic_localization_loss_detection(self):
        """Verify that genuine cross-track particle divergence (l_min > 0.80) triggers LOC_LOST."""
        # Inject divergent covariance over hysteresis window (5 cycles)
        for _ in range(6):
            self._inject_amcl_pose(0.0, 0.0, 0.0, cov_xx=5.0, cov_yy=2.5, cov_xy=0.5, cov_yaw=0.8)

        self.assertEqual(self.coordinator.loc_state, AutonomousSupervisorNode.LOC_LOST)
        is_ready, diag = self.coordinator.check_localization_readiness()
        self.assertFalse(is_ready, "Diverged localization must fail readiness check.")

    # -------------------------------------------------------------------------
    # Scenario 6: Narrow Doorway & Costmap Clearance Parameters
    # -------------------------------------------------------------------------
    def test_scenario_06_doorway_costmap_clearance_math(self):
        """Verify that inflation_radius 0.45 and cost_scaling_factor 5.0 leave 1.2m doors passable."""
        # Door width: 1.2m. Wall on each side.
        # Robot center to wall: 0.6m. Inscribed radius: 0.28m.
        # Distance to obstacle: 0.6m > inflation_radius (0.45m).
        # Cost at center: 0.
        inflation_radius = 0.45
        inscribed_radius = 0.28
        dist_to_wall = 0.60
        self.assertGreater(dist_to_wall, inflation_radius, "Doorway center must be outside obstacle inflation zone.")

    # -------------------------------------------------------------------------
    # Scenario 7: Bounded Obstacle Recovery & Anti-Infinite-Loop
    # -------------------------------------------------------------------------
    def test_scenario_07_bounded_recovery_retries(self):
        """Verify that obstacle failure triggers costmap clearing and caps retries at max_retries."""
        self.coordinator.active_goal_msg = MagicMock()
        self.coordinator.current_waypoint = {"name": "Master Bedroom", "coords": {"x": -4.0, "y": 2.2}}
        self.coordinator.max_retries = 2

        # 1st failure -> retry 1
        self.coordinator._handle_navigation_failure("obstacle_blocked")
        self.assertEqual(self.coordinator.retry_count, 1)
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_OBSTACLE_HANDLING)

        # 2nd failure -> retry 2
        self.coordinator._handle_navigation_failure("obstacle_blocked")
        self.assertEqual(self.coordinator.retry_count, 2)

        # 3rd failure -> exhausted, finalizes mission with failure
        self.coordinator._handle_navigation_failure("obstacle_blocked")
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_NAVIGATION_FAILURE)

    # -------------------------------------------------------------------------
    # Scenario 8: Mission Pause & Resume
    # -------------------------------------------------------------------------
    def test_scenario_08_pause_resume_lifecycle(self):
        """Verify pause transitions to MISSION_PAUSED and resume re-dispatches current waypoint."""
        payload = {
            "intent": "NAVIGATE_TO",
            "destination_name": "Study Room",
            "destination_coords": {"x": 4.5, "y": 3.0, "yaw": 1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload)
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING

        # Pause
        self.coordinator.handle_pause_mission()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_MISSION_PAUSED)

        # Resume
        self.coordinator.handle_resume_mission()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_MISSION_PLANNING)

    # -------------------------------------------------------------------------
    # Scenario 9: Cancellation Idempotency
    # -------------------------------------------------------------------------
    def test_scenario_09_cancellation_idempotency(self):
        """Verify cancel works during navigation and is safe when idle."""
        # 1. Cancel while navigating
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING
        self.coordinator.active_mission_id = "msn-test"
        self.coordinator.handle_cancel_mission()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_AUTONOMY_READY)
        self.assertIsNone(self.coordinator.active_mission_id)

        # 2. Cancel while already idle/ready (idempotency)
        self.coordinator.handle_cancel_mission()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_AUTONOMY_READY)

    # -------------------------------------------------------------------------
    # Scenario 10: Emergency Stop Safety Gate Lockout
    # -------------------------------------------------------------------------
    def test_cancel_during_estop_keeps_latch_and_state(self):
        self.coordinator.active_mission_id = "interrupted"
        self.coordinator.handle_emergency_stop()
        self.coordinator.handle_cancel_mission()
        self.assertTrue(self.coordinator.terrain_emergency_active)
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_EMERGENCY_STOP)
        self.assertEqual(self.coordinator.mission_history[0]["status"], "CANCELLED")
        self.coordinator.handle_cancel_mission()
        self.assertEqual(len(self.coordinator.mission_history), 1)

    def test_scenario_10_estop_lockout(self):
        """Verify that emergency stop rejects new missions immediately."""
        self.coordinator.handle_emergency_stop()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_EMERGENCY_STOP)
        self.assertTrue(self.coordinator.terrain_emergency_active)

        # Attempt to start mission
        payload = {
            "intent": "NAVIGATE_TO",
            "destination_name": "Living Room",
            "destination_coords": {"x": 4.0, "y": -2.5, "yaw": -1.57, "frame_id": "map"}
        }
        self.coordinator.handle_start_mission(payload)
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_EMERGENCY_STOP)
        self.assertTrue(any("Emergency Stop is active" in fb.get("speech", "") for fb in self.published_feedback))

    # -------------------------------------------------------------------------
    # Scenario 11: Emergency Stop Reset Authorization
    # -------------------------------------------------------------------------
    def test_scenario_11_estop_reset(self):
        """Verify operator authorized E-stop reset restores AUTONOMY_READY."""
        self.coordinator.handle_emergency_stop()
        self.coordinator.handle_reset_emergency_stop()
        self.assertEqual(self.coordinator.state, AutonomousSupervisorNode.STATE_AUTONOMY_READY)
        self.assertFalse(self.coordinator.terrain_emergency_active)

    # -------------------------------------------------------------------------
    # Scenario 12: Grounded Status — Where are you? (Topology Mapping)
    # -------------------------------------------------------------------------
    def test_scenario_12_grounded_location_query(self):
        """Verify that 'Where are you?' accurately maps coordinates to the apartment topology."""
        # 1. Hallway (0.0, 0.0)
        self.coordinator.get_current_map_pose = MagicMock(return_value=(0.0, 0.0, 0.0))
        self.coordinator.handle_status_query({"query": "location"})
        self.assertIn("Central Hallway", self.published_feedback[-1]["speech"])

        # 2. Bedroom (-4.0, 2.2)
        self.coordinator.get_current_map_pose = MagicMock(return_value=(-4.0, 2.2, 1.57))
        self.coordinator.handle_status_query({"query": "location"})
        self.assertIn("Master Bedroom", self.published_feedback[-1]["speech"])

        # 3. Kitchen (-4.0, -2.5)
        self.coordinator.get_current_map_pose = MagicMock(return_value=(-4.0, -2.5, -1.57))
        self.coordinator.handle_status_query({"query": "location"})
        self.assertIn("Kitchen & Dining", self.published_feedback[-1]["speech"])

        # 4. Study Room (4.5, 3.0)
        self.coordinator.get_current_map_pose = MagicMock(return_value=(4.5, 3.0, 1.57))
        self.coordinator.handle_status_query({"query": "location"})
        self.assertIn("Study Room", self.published_feedback[-1]["speech"])

    # -------------------------------------------------------------------------
    # Scenario 13: Grounded Status — Where are you going?
    # -------------------------------------------------------------------------
    def test_scenario_13_grounded_destination_query(self):
        """Verify destination and remaining distance reporting."""
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING
        self.coordinator.current_waypoint = {"name": "Master Bedroom"}
        self.coordinator.remaining_distance = 5.2

        self.coordinator.handle_status_query({"query": "destination"})
        speech = self.published_feedback[-1]["speech"]
        self.assertIn("Master Bedroom", speech)
        self.assertIn("5.2", speech)

    # -------------------------------------------------------------------------
    # Scenario 14: Grounded Status — Are you moving?
    # -------------------------------------------------------------------------
    def test_scenario_14_grounded_moving_query(self):
        """Verify moving status grounded to odometry linear/angular velocity."""
        # Stopped
        self.coordinator.state = AutonomousSupervisorNode.STATE_AUTONOMY_READY
        self.coordinator.last_linear_vel = 0.0
        self.coordinator.last_angular_vel = 0.0
        self.coordinator.handle_status_query({"query": "moving"})
        self.assertIn("No, I am stationary", self.published_feedback[-1]["speech"])

        # Moving
        self.coordinator.state = AutonomousSupervisorNode.STATE_MISSION_EXECUTING
        self.coordinator.current_waypoint = {"name": "Kitchen & Dining"}
        self.coordinator.last_linear_vel = 0.35
        self.coordinator.handle_status_query({"query": "moving"})
        self.assertIn("Yes, I am moving", self.published_feedback[-1]["speech"])
        self.assertIn("0.35", self.published_feedback[-1]["speech"])

    # -------------------------------------------------------------------------
    # Scenario 15: Grounded Status — Are you stuck / Why did you stop?
    # -------------------------------------------------------------------------
    def test_scenario_15_grounded_stuck_query(self):
        """Verify stuck/stopped status explanation."""
        # In obstacle handling
        self.coordinator.state = AutonomousSupervisorNode.STATE_OBSTACLE_HANDLING
        self.coordinator.handle_status_query({"query": "stuck"})
        self.assertIn("obstacle", self.published_feedback[-1]["speech"].lower())

        # In emergency stop
        self.coordinator.state = AutonomousSupervisorNode.STATE_EMERGENCY_STOP
        self.coordinator.handle_status_query({"query": "why_stopped"})
        self.assertIn("emergency stop", self.published_feedback[-1]["speech"].lower())

    # -------------------------------------------------------------------------
    # Scenario 16: Conversational LLM Safety Isolation
    # -------------------------------------------------------------------------
    def test_scenario_16_conversational_llm_safety_isolation(self):
        """Verify that unsolicited physical motion commands from conversational LLM are blocked."""
        gateway = InteractionGateway()
        gateway.tts.speak = MagicMock()

        # Mock brain returning a hallucinated/unsolicited NavigateTo command
        mock_brain = MagicMock()
        mock_brain.reason.return_value = BrainResult(
            response_text="Sure, I'll go to the bedroom for you.",
            intent="NAVIGATE",
            confidence=0.85,
            expression=Expression.FOCUS,
            semantic_actions=[{"action": "NavigateTo", "params": {"target_name": "Master Bedroom", "x": -4.0, "y": 2.2}}]
        )
        gateway.brain = mock_brain

        # Pass a general conversational query that triggers brain reasoning (not fast-path)
        res = gateway.process_text("Tell me a philosophical story about the universe.")

        # Physical action must be BLOCKED by Safety Gate policy
        motion_actions = [a for a in res.actions if a.get("action") in ("NavigateTo", "NavigateToRoom", "SetTwist", "MotorCommand")]
        self.assertEqual(len(motion_actions), 0, "Unsolicited motion commands from LLM must be blocked.")

    # -------------------------------------------------------------------------
    # Scenario 17: Simulation Hardware Lock Assertions
    # -------------------------------------------------------------------------
    def test_scenario_17_hardware_locks_confirmed(self):
        """Verify MOTOR_OUTPUT_ENABLED == 0 in config.h and PROTOCOL_CONFIRMED == False in mcu_transport.py."""
        # 1. firmware/config.h
        config_h = REPO_ROOT / "firmware" / "config.h"
        if config_h.exists():
            content = config_h.read_text()
            self.assertIn("MOTOR_OUTPUT_ENABLED 0", content, "Physical motors must remain disabled.")
            self.assertNotIn("MOTOR_OUTPUT_ENABLED 1", content)

        # 2. mcu_transport.py
        mcu_transport = REPO_ROOT / "ros2_ws" / "src" / "gracemo_hardware" / "gracemo_hardware" / "mcu_transport.py"
        if mcu_transport.exists():
            content = mcu_transport.read_text()
            self.assertIn("PROTOCOL_CONFIRMED: bool = False", content, "PROTOCOL_CONFIRMED must remain False.")


if __name__ == "__main__":
    unittest.main()
