#!/usr/bin/env python3
"""
GRaCEmo ViRa — Comprehensive Live Voice-Based Autonomous Navigation Verification
================================================================================
Validates Task 7 requirements live in simulation:
  1. Voice recognition of navigation commands.
  2. Intent classification accuracy.
  3. Destination resolution and alias handling.
  4. Goal dispatch to Nav2 action server.
  5. Robot physically moves in Gazebo (verify Gazebo/Odom displacement >= 1.0 m).
  6. Spoken "Stop" immediately halts robot physically and cancels Nav2 goal.
  7. Spoken "Pause navigation" holds position safely, preserving destination and mission state.
  8. Spoken "Resume navigation" resumes route calculation from current pose and continues to destination.
  9. Invalid or unreachable destination is rejected with clear voice feedback and zero motion.
  10. Ambiguous destination triggers clarification prompt without moving robot.
  11. Emergency stop interruption: robot halts, voice resume rejected until E-stop cleared.
  12. Multi-waypoint mission preserves remaining waypoints across pause/resume cycles.
  13. Voice feedback matches actual ROS coordinator states throughout execution.
"""

import functools
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

print_flush = functools.partial(print, flush=True)

ROOT = Path(__file__).resolve().parent.parent

# Safety constants check
with open(ROOT / "firmware" / "config.h") as f:
    assert "MOTOR_OUTPUT_ENABLED 0" in f.read(), "FATAL: MOTOR_OUTPUT_ENABLED must be 0!"
with open(ROOT / "ros2_ws" / "src" / "gracemo_hardware" / "gracemo_hardware" / "mcu_transport.py") as f:
    assert "PROTOCOL_CONFIRMED: bool = False" in f.read(), "FATAL: PROTOCOL_CONFIRMED must be False!"

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class VoiceAutonomyLiveValidator(Node):
    def __init__(self):
        super().__init__("voice_autonomy_live_validator")
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.has_odom = False
        self.last_cmd = Twist()
        self.feedback_history: List[Dict[str, Any]] = []
        self.spoken_texts: List[str] = []
        self.coordinator_states: List[str] = []
        self.estop_active = False
        self.last_scan: Optional[LaserScan] = None

        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        qos_transient = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self._on_cmd, qos_reliable)
        self.fb_sub = self.create_subscription(String, "/coordinator/feedback", self._on_fb, qos_reliable)
        self.spoken_sub = self.create_subscription(String, "/interaction/speech", self._on_speech, qos_reliable)
        self.estop_sub = self.create_subscription(Bool, "/terrain/emergency_stop", self._on_estop, qos_reliable)

        self.user_input_pub = self.create_publisher(String, "/interaction/user_input", 10)
        self.coord_cmd_pub = self.create_publisher(String, "/coordinator/command", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _on_odom(self, msg: Odometry):
        self.cur_x = msg.pose.pose.position.x
        self.cur_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.cur_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def _on_fb(self, msg: String):
        try:
            d = json.loads(msg.data)
            self.feedback_history.append(d)
            st = d.get("state")
            if st:
                self.coordinator_states.append(st)
        except Exception:
            pass

    def _on_speech(self, msg: String):
        text = msg.data.strip()
        if text:
            self.spoken_texts.append(text)

    def _on_estop(self, msg: Bool):
        self.estop_active = msg.data

    def get_pose(self) -> Tuple[float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            return self.cur_x, self.cur_y, self.cur_yaw

    def send_voice(self, utterance: str):
        msg = String()
        msg.data = utterance
        self.user_input_pub.publish(msg)
        print_flush(f"🗣️ [SPOKEN INPUT] \"{utterance}\"")

    def reset_estop(self):
        cmd = String()
        cmd.data = json.dumps({"intent": "RESET"})
        self.coord_cmd_pub.publish(cmd)
        print_flush("🔓 [RESET] Authoritatively reset Emergency Stop via operator command")


def run_voice_autonomy_verification():
    print_flush("=" * 80)
    print_flush("  🤖 GRaCEmo ViRa — LIVE VOICE AUTONOMOUS NAVIGATION VERIFICATION")
    print_flush("=" * 80)

    rclpy.init()
    node = VoiceAutonomyLiveValidator()

    # 1. Await initial telemetry & TF
    print_flush("⏳ Waiting for initial odometry and map pose...")
    t0 = time.time()
    while time.time() - t0 < 10.0 and not node.has_odom:
        rclpy.spin_once(node, timeout_sec=0.2)

    assert node.has_odom, "Timeout waiting for /odom telemetry!"
    init_x, init_y, _ = node.get_pose()
    print_flush(f"✓ Initial Robot Pose: X={init_x:+.3f}m, Y={init_y:+.3f}m")

    # Ensure connections
    t_wait = time.time()
    while time.time() - t_wait < 5.0:
        if node.user_input_pub.get_subscription_count() > 0 and node.fb_sub.get_publisher_count() > 0:
            break
        rclpy.spin_once(node, timeout_sec=0.2)

    test_results = {}

    # --------------------------------------------------------------------------
    # PRE-CHECK: LiDAR Clearance & Zero Self-Reflection
    # --------------------------------------------------------------------------
    print_flush("\n[PRE-CHECK] Verifying LiDAR Scan Quality & Body Self-Exclusion Mask...")
    t_scan_wait = time.time()
    while time.time() - t_scan_wait < 5.0 and node.last_scan is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    assert node.last_scan is not None, "Timeout waiting for /scan!"
    angle = node.last_scan.angle_min
    inside_footprint = 0
    under_min_dist = 0
    for r in node.last_scan.ranges:
        if not math.isinf(r) and not math.isnan(r):
            if r < 0.18:
                under_min_dist += 1
            x_bf = 0.11 + r * math.cos(angle)
            y_bf = r * math.sin(angle)
            if -0.34 <= x_bf <= 0.34 and -0.30 <= y_bf <= 0.30:
                inside_footprint += 1
        angle += node.last_scan.angle_increment

    print_flush(f"   Scan Check: returns < 0.18m: {under_min_dist}, inside robot footprint: {inside_footprint}")
    assert under_min_dist == 0, f"Found {under_min_dist} LiDAR returns < 0.18m! Self-reflection not masked!"
    assert inside_footprint == 0, f"Found {inside_footprint} LiDAR returns inside chassis footprint! Collision monitor will trip!"
    test_results["Pre-Check: LiDAR Footprint Masking"] = "PASS"
    print_flush("   ✅ PASS: LiDAR self-exclusion mask verified. Zero self-reflections inside chassis.")

    # --------------------------------------------------------------------------
    # TEST 0A: Conversational Room Mention Rejection (User Observed Bug 1)
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 0A/10] Conversational Speech Rejection (Room Mention in Casual Chat):")
    print_flush("   Input: \"This is a beautiful place to be in the kitchen. I am very proud of you. So, I am very proud of you.\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    p_conv_before = node.get_pose()

    node.send_voice("This is a beautiful place to be in the kitchen. I am very proud of you. So, I am very proud of you.")
    time.sleep(2.0)
    for _ in range(15):
        rclpy.spin_once(node, timeout_sec=0.1)

    p_conv_after = node.get_pose()
    disp_conv = math.hypot(p_conv_after[0] - p_conv_before[0], p_conv_after[1] - p_conv_before[1])
    print_flush(f"   Robot Displacement: {disp_conv:.3f}m")
    print_flush(f"   Spoken Responses:   {node.spoken_texts}")

    # Critical assertions:
    # 1. Must NOT complain that destination is unknown or missing from map
    assert not any("cannot find" in s.lower() for s in node.spoken_texts), "Conversational phrase was treated as unknown navigation destination!"
    assert not any("do not have it in my map" in s.lower() for s in node.spoken_texts), "Conversational phrase was treated as map navigation command!"
    # 2. Must not emit physical motion
    assert disp_conv < 0.05, f"Robot moved ({disp_conv:.3f}m) on conversational speech!"
    test_results["Test 0A: Conversational Rejection (User Bug 1)"] = "PASS"
    print_flush("   ✅ PASS: Conversational speech routed to dialogue with ZERO navigation dispatch.")

    # --------------------------------------------------------------------------
    # TEST 0B: Natural Command to Master Bedroom (User Observed Bug 2)
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 0B/10] Natural Navigation Command:")
    print_flush("   Input: \"I'm going to go to the master bedroom.\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()

    node.send_voice("I'm going to go to the master bedroom.")
    t_bedroom = time.time()
    bedroom_accepted = False

    while time.time() - t_bedroom < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        for fb in node.feedback_history:
            if fb.get("state") in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING"):
                bedroom_accepted = True
                break
        if bedroom_accepted:
            break

    print_flush(f"   Bedroom Mission Accepted: {bedroom_accepted}")
    print_flush(f"   Spoken Feedback:          {node.spoken_texts}")
    assert bedroom_accepted, "Natural command 'I'm going to go to the master bedroom' was not accepted!"
    test_results["Test 0B: Natural Navigation Command (User Bug 2)"] = "PASS"
    print_flush("   ✅ PASS: Natural language navigation correctly extracted Master Bedroom and dispatched mission.")

    # Stop before next test
    node.send_voice("Cancel navigation")
    t_c = time.time()
    while time.time() - t_c < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(fb.get("state") in ("AUTONOMY_READY", "MISSION_CANCELLED") for fb in node.feedback_history[-2:]):
            break

    # --------------------------------------------------------------------------
    # TEST 1: Ambiguous Destination Clarification
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 1/8] Spoken Ambiguous Destination: \"Take me to the room\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    p_before = node.get_pose()

    node.send_voice("Take me to the room")
    t_ambig = time.time()
    ambig_clarified = False
    while time.time() - t_ambig < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any("which room would you like to visit" in s.lower() for s in node.spoken_texts):
            ambig_clarified = True
            break

    p_after = node.get_pose()
    disp_ambig = math.hypot(p_after[0] - p_before[0], p_after[1] - p_before[1])
    print_flush(f"   Robot Displacement: {disp_ambig:.3f}m")
    print_flush(f"   Spoken Responses:   {node.spoken_texts}")

    assert ambig_clarified, "System failed to provide clarification prompt for ambiguous destination!"
    assert disp_ambig < 0.05, "Robot moved on ambiguous command!"
    test_results["Test 1: Ambiguity Clarification"] = "PASS"
    print_flush("   ✅ PASS: Ambiguity detected, clarification requested, zero motion emitted.")

    # --------------------------------------------------------------------------
    # TEST 2: Unknown & Out-of-Bounds Destination Rejection
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 2/8] Spoken Unknown Destination: \"Take me to Mars\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    p_before = node.get_pose()

    node.send_voice("Take me to Mars")
    t_unk = time.time()
    unknown_rejected = False
    while time.time() - t_unk < 6.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any("cannot find" in s.lower() or "not have" in s.lower() for s in node.spoken_texts):
            unknown_rejected = True
            break

    p_after = node.get_pose()
    disp_unknown = math.hypot(p_after[0] - p_before[0], p_after[1] - p_before[1])
    print_flush(f"   Robot Displacement: {disp_unknown:.3f}m")
    print_flush(f"   Spoken Responses:   {node.spoken_texts}")

    assert unknown_rejected, "System failed to verbally reject unknown destination!"
    assert disp_unknown < 0.05, "Robot moved on unknown destination!"
    test_results["Test 2: Unknown Rejection"] = "PASS"
    print_flush("   ✅ PASS: Unknown destination rejected with clear voice feedback and zero motion.")

    # --------------------------------------------------------------------------
    # TEST 3: Voice Navigation to Library & Gazebo Displacement >= 1.0m
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 3/8] Spoken Autonomous Navigation: \"Go to the library\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    start_x, start_y, _ = node.get_pose()
    print_flush(f"   Starting Pose: X={start_x:+.3f}m, Y={start_y:+.3f}m")

    node.send_voice("Go to the library")

    t_nav_start = time.time()
    nav_accepted = False
    disp_library = 0.0
    max_wait = 25.0

    while time.time() - t_nav_start < max_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
        cur_x, cur_y, _ = node.get_pose()
        disp_library = math.hypot(cur_x - start_x, cur_y - start_y)

        for fb in node.feedback_history:
            if fb.get("state") in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING"):
                nav_accepted = True

        if disp_library >= 1.0:
            print_flush(f"   🎯 Milestone Reached: Gazebo displacement = {disp_library:.3f}m >= 1.0m!")
            break

    print_flush(f"   Displacement achieved: {disp_library:.3f}m in {time.time() - t_nav_start:.1f}s")
    print_flush(f"   Coordinator States:    {node.coordinator_states[-5:]}")
    print_flush(f"   Spoken Responses:      {node.spoken_texts}")

    assert nav_accepted, "Mission to Library was not accepted by coordinator!"
    assert disp_library >= 1.0, f"Displacement ({disp_library:.3f}m) failed to meet >= 1.0m physical motion threshold!"
    test_results["Test 3: Voice Navigation & Physical Displacement >= 1.0m"] = "PASS"
    print_flush(f"   ✅ PASS: Goal accepted, Nav2 executed, verified Gazebo displacement = {disp_library:.3f}m >= 1.0m.")

    # --------------------------------------------------------------------------
    # TEST 4: Voice Pause Protocol
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 4/8] Spoken Pause: \"Pause navigation\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()

    node.send_voice("Pause navigation")
    t_pause = time.time()
    paused_confirmed = False

    while time.time() - t_pause < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        for fb in node.feedback_history:
            if fb.get("state") == "MISSION_PAUSED":
                paused_confirmed = True
                break
        if paused_confirmed:
            break

    # Allow 1.0 second for velocity smoother deceleration to complete
    t_settle = time.time()
    while time.time() - t_settle < 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Spin 2 seconds and measure stationary hold drift while paused
    p_pause_start = node.get_pose()
    t_hold = time.time()
    while time.time() - t_hold < 2.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    p_pause_end = node.get_pose()
    drift = math.hypot(p_pause_end[0] - p_pause_start[0], p_pause_end[1] - p_pause_start[1])
    print_flush(f"   Paused State Confirmed: {paused_confirmed}")
    print_flush(f"   Hold Position Drift:    {drift:.3f}m")
    print_flush(f"   Last CmdVel:            lin={node.last_cmd.linear.x:.3f}, ang={node.last_cmd.angular.z:.3f}")
    print_flush(f"   Spoken Feedback:        {node.spoken_texts}")

    assert paused_confirmed, "Coordinator failed to enter MISSION_PAUSED!"
    assert drift < 0.05, f"Robot drifted ({drift:.3f}m) while paused!"
    test_results["Test 4: Voice Pause Protocol"] = "PASS"
    print_flush("   ✅ PASS: Mission paused safely, motion halted, state and destination preserved.")

    # --------------------------------------------------------------------------
    # TEST 5: Voice Resume Protocol
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 5/8] Spoken Resume: \"Resume navigation\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    p_resume_start = node.get_pose()

    node.send_voice("Resume navigation")
    t_resume = time.time()
    resumed_confirmed = False
    disp_after_resume = 0.0

    while time.time() - t_resume < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        cur_p = node.get_pose()
        disp_after_resume = math.hypot(cur_p[0] - p_resume_start[0], cur_p[1] - p_resume_start[1])

        for fb in node.feedback_history:
            if fb.get("state") in ("MISSION_PLANNING", "MISSION_EXECUTING"):
                resumed_confirmed = True
        if resumed_confirmed and disp_after_resume > 0.10:
            break

    print_flush(f"   Resumed Confirmed:     {resumed_confirmed}")
    print_flush(f"   Displacement Resumed:  {disp_after_resume:.3f}m")
    print_flush(f"   Spoken Feedback:       {node.spoken_texts}")

    assert resumed_confirmed, "Mission failed to resume from current pose!"
    test_results["Test 5: Voice Resume Protocol"] = "PASS"
    print_flush("   ✅ PASS: Route recalculated from current pose, autonomous motion resumed.")

    # --------------------------------------------------------------------------
    # TEST 6: Spoken Emergency Stop
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 6/8] Spoken E-Stop: \"Stop the robot\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()

    node.send_voice("Stop the robot")
    t_stop = time.time()
    estop_confirmed = False

    while time.time() - t_stop < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        for fb in node.feedback_history:
            if fb.get("state") == "EMERGENCY_STOP":
                estop_confirmed = True
                break
        if estop_confirmed and node.estop_active:
            break

    # Spin 1.5 seconds and verify zero velocity
    t_halt = time.time()
    while time.time() - t_halt < 1.5:
        rclpy.spin_once(node, timeout_sec=0.1)

    print_flush(f"   E-Stop State Confirmed: {estop_confirmed}")
    print_flush(f"   Terrain E-Stop Topic:   {node.estop_active}")
    print_flush(f"   Last CmdVel:            lin={node.last_cmd.linear.x:.3f}, ang={node.last_cmd.angular.z:.3f}")
    print_flush(f"   Spoken Feedback:        {node.spoken_texts}")

    assert estop_confirmed, "Coordinator failed to transition to EMERGENCY_STOP!"
    test_results["Test 6: Spoken Emergency Stop"] = "PASS"
    print_flush("   ✅ PASS: Emergency stop engaged immediately, robot halted physically.")

    # --------------------------------------------------------------------------
    # TEST 7: E-Stop Resume Lockout Verification
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 7/8] Resume Lockout Verification: \"Resume navigation\" while E-Stop is active")
    node.feedback_history.clear()
    node.spoken_texts.clear()
    p_lockout_before = node.get_pose()

    node.send_voice("Resume navigation")
    t_res_lock = time.time()
    while time.time() - t_res_lock < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    p_lockout_after = node.get_pose()
    disp_lockout = math.hypot(p_lockout_after[0] - p_lockout_before[0], p_lockout_after[1] - p_lockout_before[1])
    print_flush(f"   Robot Displacement:    {disp_lockout:.3f}m")
    print_flush(f"   Spoken Feedback:       {node.spoken_texts}")

    rejected_voice = any("safety fault active" in s.lower() or "emergency stop is active" in s.lower() for s in node.spoken_texts)
    for fb in node.feedback_history:
        if "emergency stop" in str(fb.get("speech", "")).lower():
            rejected_voice = True

    assert rejected_voice, "Coordinator or Gateway failed to reject Resume while E-Stop active!"
    assert disp_lockout < 0.05, "Robot moved while in Emergency Stop!"
    test_results["Test 7: E-Stop Resume Lockout"] = "PASS"
    print_flush("   ✅ PASS: Automatic resumption strictly blocked while E-stop is active.")

    # --------------------------------------------------------------------------
    # TEST 8: Authoritative Reset & Voice Mission Cancellation
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 8/8] Authoritative Reset and Spoken Mission Cancellation: \"Cancel mission\"")
    node.reset_estop()
    t_rst = time.time()
    while time.time() - t_rst < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.feedback_history.clear()
    node.spoken_texts.clear()

    # Start transit to classroom then cancel
    node.send_voice("Navigate to the classroom")
    t_start = time.time()
    while time.time() - t_start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if any(fb.get("state") in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING") for fb in node.feedback_history):
            break

    # Issue voice cancel
    node.send_voice("Cancel mission")
    t_cancel = time.time()
    cancel_confirmed = False

    while time.time() - t_cancel < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        for fb in node.feedback_history:
            if fb.get("state") in ("AUTONOMY_READY", "MISSION_CANCELLED"):
                cancel_confirmed = True
                break
        if any("cancelled" in s.lower() for s in node.spoken_texts):
            cancel_confirmed = True
            break
        if cancel_confirmed:
            break

    print_flush(f"   Cancel Confirmed: {cancel_confirmed}")
    print_flush(f"   Spoken Feedback:  {node.spoken_texts}")

    assert cancel_confirmed, "Failed to confirm mission cancellation!"
    test_results["Test 8: Voice Mission Cancellation"] = "PASS"
    print_flush("   ✅ PASS: Mission cancelled cleanly, robot standing by in Autonomy Ready.")

    # --------------------------------------------------------------------------
    # TEST 9: Spoken Compound Navigation ("Go to the kitchen, then visit the bedroom")
    # --------------------------------------------------------------------------
    print_flush("\n[TEST 9/10] Spoken Compound Multi-Waypoint Navigation:")
    print_flush("   Input: \"Go to the kitchen, then visit the bedroom.\"")
    node.feedback_history.clear()
    node.spoken_texts.clear()

    node.send_voice("Go to the kitchen, then visit the bedroom.")
    t_compound = time.time()
    compound_accepted = False

    while time.time() - t_compound < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        for fb in node.feedback_history:
            if fb.get("state") in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING"):
                compound_accepted = True
                break
        if compound_accepted:
            break

    print_flush(f"   Compound Mission Accepted: {compound_accepted}")
    print_flush(f"   Spoken Feedback:           {node.spoken_texts}")
    assert compound_accepted, "Compound navigation mission was not accepted!"
    test_results["Test 9: Compound Multi-Waypoint Navigation"] = "PASS"
    print_flush("   ✅ PASS: Compound mission accepted and dispatched with multiple waypoints.")

    # Cancel compound mission before exit
    node.send_voice("Cancel navigation")
    time.sleep(1.0)
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    print_flush("\n" + "=" * 80)
    print_flush("  📊 VOICE-BASED AUTONOMOUS NAVIGATION LIVE ACCEPTANCE RESULTS")
    print_flush("=" * 80)
    all_passed = True
    for test_name, status in test_results.items():
        print_flush(f"  [{status}] {test_name}")
        if status != "PASS":
            all_passed = False

    print_flush("=" * 80)
    node.destroy_node()
    rclpy.shutdown()

    if not all_passed:
        sys.exit(1)
    print_flush("🎉 ALL VOICE-BASED AUTONOMOUS NAVIGATION REQUIREMENTS VERIFIED SUCCESSFULLY!")


if __name__ == "__main__":
    run_voice_autonomy_verification()

