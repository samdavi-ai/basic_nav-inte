#!/usr/bin/env python3
"""
Unit & Integration Tests for GRaCEmo ROS 2 Interaction & Navigation Bridge.
Validates:
  - InteractionBridgeNode initialization and topic configuration
  - Voice/Text command ingestion via ROS 2 topic /interaction/command
  - Fast-path matching & PlaceResolver waypoint resolution
  - Navigation dispatch to Nav2 (/goal_pose) and /cmd_vel safety zeroing on emergency stop
  - Closed-loop /odom tracking and arrival announcement
"""
import json
import math
import sys
import unittest
from pathlib import Path

# Setup paths
ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "gracemo_bridge"))
sys.path.insert(0, str(ROOT / "vira_interaction"))
sys.path.insert(0, str(ROOT / "adapters" / "robot-bridge"))

import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String as RosString

from gracemo_bridge.interaction_bridge import InteractionBridgeNode
from core.state_machine import CognitiveState, Expression


class TestInteractionBridgeNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = InteractionBridgeNode()
        # Capture published messages for verification
        self.published_cmd_vel = []
        self.published_goal_pose = []
        self.published_state = []
        self.published_expression = []
        self.published_speech = []
        self.published_coord_cmd = []

        # Hook publishers to capture messages directly
        orig_pub_cmd_vel = self.node.pub_cmd_vel.publish
        orig_pub_goal = self.node.pub_goal_pose.publish
        orig_pub_state = self.node.pub_state.publish
        orig_pub_expr = self.node.pub_expression.publish
        orig_pub_speech = self.node.pub_speech.publish
        orig_pub_coord = self.node.pub_coord_cmd.publish

        self.node.pub_cmd_vel.publish = lambda msg: (
            self.published_cmd_vel.append(msg),
            orig_pub_cmd_vel(msg)
        )[0]
        self.node.pub_goal_pose.publish = lambda msg: (
            self.published_goal_pose.append(msg),
            orig_pub_goal(msg)
        )[0]
        self.node.pub_state.publish = lambda msg: (
            self.published_state.append(msg),
            orig_pub_state(msg)
        )[0]
        self.node.pub_expression.publish = lambda msg: (
            self.published_expression.append(msg),
            orig_pub_expr(msg)
        )[0]
        self.node.pub_speech.publish = lambda msg: (
            self.published_speech.append(msg),
            orig_pub_speech(msg)
        )[0]
        self.node.pub_coord_cmd.publish = lambda msg: (
            self.published_coord_cmd.append(msg),
            orig_pub_coord(msg)
        )[0]

    def tearDown(self):
        self.node.destroy_node()

    def test_initial_state_and_topics(self):
        """Node should start with IDLE state, valid PlaceResolver, and online topics."""
        self.assertEqual(self.node.fsm.state, CognitiveState.IDLE)
        self.assertEqual(self.node.fsm.expression, Expression.IDLE)
        self.assertFalse(self.node._navigating)
        self.assertIsNone(self.node._active_target)
        self.assertTrue(hasattr(self.node, "place_resolver"))

    def test_greeting_command_via_ros_topic(self):
        """Publishing a greeting command on /interaction/command triggers spoken greeting."""
        msg = RosString()
        msg.data = "hello"
        self.node._on_command_msg(msg)

        self.assertTrue(len(self.published_speech) >= 1)
        spoken_text = self.published_speech[-1].data.lower()
        self.assertTrue(
            "welcome" in spoken_text
            or "pleasure" in spoken_text
            or "graceemo" in spoken_text
            or "assistance" in spoken_text
        )

    def test_navigation_dispatch_and_goal_pose(self):
        """Navigation commands resolve waypoints and publish PoseStamped to /goal_pose."""
        msg = RosString()
        msg.data = "take me to the kitchen"
        self.node._on_command_msg(msg)

        self.assertTrue(self.node._navigating)
        self.assertIsNotNone(self.node._active_target)
        tx, ty, place_name, _ = self.node._active_target
        self.assertIn("kitchen", place_name.lower())

        # Verify /goal_pose was published
        self.assertTrue(len(self.published_goal_pose) >= 1)
        goal = self.published_goal_pose[-1]
        self.assertEqual(goal.header.frame_id, "map")
        self.assertEqual(goal.pose.position.x, tx)
        self.assertEqual(goal.pose.position.y, ty)

        # Expression should be FOCUS during active mission dispatch
        self.assertEqual(self.node.fsm.expression, Expression.FOCUS)

    def test_emergency_stop_halts_cmd_vel(self):
        """'stop' command immediately halts navigation and publishes zero Twist."""
        # 1. Start navigating
        nav_msg = RosString()
        nav_msg.data = "go to bedroom"
        self.node._on_command_msg(nav_msg)
        self.assertTrue(self.node._navigating)

        # 2. Emergency stop
        stop_msg = RosString()
        stop_msg.data = "emergency stop now"
        self.node._on_command_msg(stop_msg)

        # Verify navigation canceled
        self.assertFalse(self.node._navigating)
        self.assertIsNone(self.node._active_target)

        # Verify zero velocity Twist sent to /cmd_vel
        self.assertTrue(len(self.published_cmd_vel) >= 1)
        twist = self.published_cmd_vel[-1]
        self.assertEqual(twist.linear.x, 0.0)
        self.assertEqual(twist.angular.z, 0.0)

    def test_closed_loop_odometry_arrival(self):
        """Reaching the target destination within tolerance triggers arrival handling."""
        # Start navigation to kitchen
        nav_msg = RosString()
        nav_msg.data = "navigate to kitchen"
        self.node._on_command_msg(nav_msg)
        self.assertTrue(self.node._navigating)
        tx, ty, _, _ = self.node._active_target

        # Simulate robot arriving at the waypoint via /odom
        odom = Odometry()
        odom.pose.pose.position.x = tx + 0.05  # within tolerance_m (0.35)
        odom.pose.pose.position.y = ty - 0.05
        odom.pose.pose.orientation.w = 1.0

        self.node._on_odom_msg(odom)

        # Navigation should mark arrived & finish
        self.assertFalse(self.node._navigating)
        self.assertIsNone(self.node._active_target)

        # Expression should be HAPPY on arrival
        self.assertEqual(self.node.fsm.expression, Expression.HAPPY)

        # Arrival speech should be announced
        self.assertTrue(len(self.published_speech) >= 1)
        arrival_speech = self.published_speech[-1].data.lower()
        self.assertIn("arrived", arrival_speech)

    def test_voice_interrupted_navigation_and_resumption(self):
        """'Hi GraceEMO' pauses active navigation, and 'Bye bye GraceEMO' resumes mission."""
        # 1. Start navigation to kitchen
        nav_msg = RosString()
        nav_msg.data = "take me to the kitchen"
        self.node._on_command_msg(nav_msg)
        self.assertTrue(self.node._navigating)
        self.assertIsNotNone(self.node._active_target)
        self.assertEqual(self.node.fsm.expression, Expression.FOCUS)

        # 2. Voice interrupt: "Hi GraceEMO"
        wake_msg = RosString()
        wake_msg.data = "Hi GraceEMO"
        self.node._on_command_msg(wake_msg)

        # Verify mission pause request sent to coordinator
        self.assertTrue(self.node._interrupted_for_voice)
        self.assertTrue(len(self.published_coord_cmd) >= 1)
        last_coord = json.loads(self.published_coord_cmd[-1].data)
        self.assertEqual(last_coord.get("intent"), "PAUSE")

        # Verify state transitioned to LISTENING and expression to INTENT
        self.assertEqual(self.node.fsm.state, CognitiveState.LISTENING)
        self.assertEqual(self.node.fsm.expression, Expression.INTENT)

        # Verify spoken acknowledgment
        self.assertTrue(len(self.published_speech) >= 1)
        self.assertIn("paused", self.published_speech[-1].data.lower())

        # 3. Voice resume: "Bye bye GraceEMO"
        bye_msg = RosString()
        bye_msg.data = "Bye bye GraceEMO"
        self.node._on_command_msg(bye_msg)

        # Verify resume request sent to coordinator
        self.assertFalse(self.node._interrupted_for_voice)
        last_coord = json.loads(self.published_coord_cmd[-1].data)
        self.assertEqual(last_coord.get("intent"), "RESUME")

        # Verify state transitioned back to EXECUTING with FOCUS expression
        self.assertEqual(self.node.fsm.state, CognitiveState.EXECUTING)
        self.assertEqual(self.node.fsm.expression, Expression.FOCUS)

    def test_voice_resumption_blocked_by_safety_gate(self):
        """Voice resume is blocked when an emergency stop is active."""
        # 1. Trigger emergency stop
        stop_msg = RosString()
        stop_msg.data = "emergency stop now"
        self.node._on_command_msg(stop_msg)
        self.assertEqual(self.node.fsm.state, CognitiveState.EMERGENCY_STOP)

        # 2. Attempt voice resume: "Bye bye GraceEMO"
        bye_msg = RosString()
        bye_msg.data = "Bye bye GraceEMO"
        coord_cmd_count = len(self.published_coord_cmd)
        self.node._on_command_msg(bye_msg)

        # Coordinator RESUME should NOT be published
        self.assertEqual(len(self.published_coord_cmd), coord_cmd_count)
        # Speech warning should indicate safety fault
        self.assertIn("safety fault", self.published_speech[-1].data.lower())

    def test_tour_all_rooms_dispatches_start_mission(self):
        """Multi-room tour command dispatches START_MISSION with sequential waypoints to coordinator."""
        tour_msg = RosString()
        tour_msg.data = "GraceEMO give me a full tour of your every rooms."
        self.node._on_command_msg(tour_msg)

        self.assertTrue(len(self.published_coord_cmd) >= 1)
        last_coord = json.loads(self.published_coord_cmd[-1].data)
        self.assertEqual(last_coord.get("intent"), "START_MISSION")
        self.assertIn("Tour", last_coord.get("mission_name", ""))
        self.assertTrue(last_coord.get("mission_id"))
        self.assertTrue(last_coord.get("trace_id"))
        self.assertEqual(len(last_coord.get("waypoints", [])), 5)
        self.assertEqual(self.node.fsm.state, CognitiveState.EXECUTING)

    def test_nail_gate_phonetic_navigation(self):
        """Phonetic STT mishearing 'Nail gate to the kitchen' resolves and dispatches navigation goal."""
        nav_msg = RosString()
        nav_msg.data = "Nail gate to the kitchen."
        self.node._on_command_msg(nav_msg)

        self.assertTrue(len(self.published_goal_pose) >= 1)
        target = self.published_goal_pose[-1]
        self.assertAlmostEqual(target.pose.position.x, -5.0, places=1)
        self.assertAlmostEqual(target.pose.position.y, -2.5, places=1)
        self.assertEqual(self.node.fsm.state, CognitiveState.EXECUTING)


if __name__ == "__main__":
    unittest.main()
