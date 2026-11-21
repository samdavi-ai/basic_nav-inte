#!/usr/bin/env python3
"""
GRaCEmo ViRa — Autonomous Operation Supervisor & Mission Coordinator Node
Phase 1 & Phase 4: Central mission-state coordinator, authoritative 11-state FSM,
multi-waypoint patrol queue engine, health monitor, obstacle recovery handler,
and EventBus telemetry bridge.
"""

from __future__ import annotations
import json
import math
import os
import sys
import time
import urllib.request
import urllib.error
import threading
import uuid
from typing import Optional, Dict, Any, List, Tuple

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty


class AutonomousSupervisorNode(Node):
    """
    Central Autonomous Operation Supervisor for GRaCEmo ViRa.
    Manages the authoritative 11-state mission FSM, sequential multi-destination
    waypoint missions, subsystem health checks, obstacle replanning, and arrival verification.
    """

    # --- Phase 1 Authoritative State Machine Constants ---
    STATE_SYSTEM_OFFLINE = "SYSTEM_OFFLINE"
    STATE_SYSTEM_INITIALIZING = "SYSTEM_INITIALIZING"
    STATE_SYSTEM_HEALTH_CHECK = "SYSTEM_HEALTH_CHECK"
    STATE_WAITING_FOR_LOCALIZATION = "WAITING_FOR_LOCALIZATION"
    STATE_AUTONOMY_READY = "AUTONOMY_READY"
    STATE_MISSION_ACCEPTED = "MISSION_ACCEPTED"
    STATE_MISSION_PLANNING = "MISSION_PLANNING"
    STATE_MISSION_EXECUTING = "MISSION_EXECUTING"
    STATE_OBSTACLE_HANDLING = "OBSTACLE_HANDLING"
    STATE_DESTINATION_REACHED = "DESTINATION_REACHED"
    STATE_MISSION_COMPLETED = "MISSION_COMPLETED"
    STATE_WAITING_FOR_NEXT_MISSION = "WAITING_FOR_NEXT_MISSION"

    # Operational & Failure States
    STATE_MISSION_PAUSED = "MISSION_PAUSED"
    STATE_MISSION_CANCELLED = "MISSION_CANCELLED"
    STATE_SENSOR_FAILURE = "SENSOR_FAILURE"
    STATE_LOCALIZATION_LOST = "LOCALIZATION_LOST"
    STATE_NAVIGATION_FAILURE = "NAVIGATION_FAILURE"
    STATE_PLANNING_FAILURE = "PLANNING_FAILURE"
    STATE_COMMUNICATION_FAILURE = "COMMUNICATION_FAILURE"
    STATE_TERRAIN_SAFETY_STOP = "TERRAIN_SAFETY_STOP"
    STATE_EMERGENCY_STOP = "EMERGENCY_STOP"

    # Localization FSM States
    LOC_HEALTHY = "HEALTHY"
    LOC_DEGRADED = "DEGRADED"
    LOC_RECOVERING = "RECOVERING"
    LOC_LOST = "LOST"

    def __init__(self):
        super().__init__("gracemo_command_coordinator")
        self.declare_parameter("kernel_url", "http://127.0.0.1:7780")
        self.kernel_url = self.get_parameter("kernel_url").get_parameter_value().string_value.rstrip("/")
        self._last_kernel_error_log = 0.0

        # Initial state
        self.state = self.STATE_SYSTEM_INITIALIZING
        self._lock = threading.RLock()

        # Mission parameters and execution queue
        self.active_mission_id: Optional[str] = None
        self.active_mission_name: Optional[str] = None
        self.active_trace_id: Optional[str] = None
        self._current_command_trace_id: Optional[str] = None
        self.mission_waypoints: List[Dict[str, Any]] = []
        self.current_waypoint_index: int = 0
        self.current_waypoint: Optional[Dict[str, Any]] = None
        self.dwell_timer = None
        self.dwell_end_time: float = 0.0
        self.mission_start_time: Optional[float] = None
        self.mission_history: List[Dict[str, Any]] = []
        self._pending_replacement_mission: Optional[Dict[str, Any]] = None

        # Active goal tracking
        self.active_goal_handle = None
        self.active_goal_msg: Optional[NavigateToPose.Goal] = None
        self.current_goal_id = 0
        self.retry_count = 0
        self.max_retries = 3
        self.remaining_distance = 0.0

        # Telemetry & Robot state
        self.current_map_x = 0.0
        self.current_map_y = 0.0
        self.current_map_yaw = 0.0
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.has_localization = False
        self.amcl_covariance_xx = 0.0
        self.amcl_covariance_yy = 0.0
        self.amcl_covariance_yaw = 0.0
        self.last_amcl_time = 0.0
        self.covariance_fault_count = 0
        self.eigen_max = 0.0
        self.eigen_min = 0.0
        self.loc_state = self.LOC_HEALTHY
        self.loc_lost_counter = 0
        self.loc_hysteresis_cycles = 5

        # Phase 10: Localization Health & Recovery
        self.last_map_to_odom: Optional[Tuple[float, float]] = None
        self.in_recovery_rotation: bool = False
        self.recovery_start_time: float = 0.0
        self.last_recovery_time: float = 0.0

        # Subsystem health timestamps
        self.last_scan_time = 0.0
        self.last_odom_time = 0.0
        self.last_terrain_status_time = 0.0
        self.last_mux_status_time = 0.0
        self.terrain_imu_fresh = False
        self.terrain_safety_active = False
        self.kernel_healthy = False
        self.terrain_emergency_active = False
        self.retry_timer = None
        self.last_nomotion_update_time = 0.0
        self.last_progress_pose = (0.0, 0.0)
        self.last_progress_yaw_error: Optional[float] = None
        self.last_progress_time = self.get_clock().now()
        self.last_feedback_time = self.get_clock().now()
        self.navigation_start_time = None

        self.declare_parameter("use_static_localization", False)
        self.use_static_localization = bool(self.get_parameter("use_static_localization").value)
        self.declare_parameter("use_room_doorway_waypoints", False)
        self.use_room_doorway_waypoints = bool(self.get_parameter("use_room_doorway_waypoints").value)
        self.declare_parameter("navigation_stagnant_timeout_sec", 12.0)
        self.navigation_stagnant_timeout_sec = max(
            2.0, float(self.get_parameter("navigation_stagnant_timeout_sec").value)
        )
        # Nav2's primary XY checker is 0.25 m. The coordinator allows only a
        # small sampling margin and independently bounds Gazebo truth error
        # and map/ground-truth disagreement before confirming arrival.
        self.declare_parameter("arrival_map_tolerance_m", 0.30)
        self.arrival_map_tolerance_m = max(
            0.05, float(self.get_parameter("arrival_map_tolerance_m").value)
        )
        self.declare_parameter("arrival_ground_truth_tolerance_m", 0.35)
        self.arrival_ground_truth_tolerance_m = max(
            0.05, float(self.get_parameter("arrival_ground_truth_tolerance_m").value)
        )
        self.declare_parameter("arrival_map_ground_truth_drift_tolerance_m", 0.15)
        self.arrival_map_ground_truth_drift_tolerance_m = max(
            0.05,
            float(self.get_parameter("arrival_map_ground_truth_drift_tolerance_m").value),
        )
        self.declare_parameter("amcl_tf_future_tolerance_sec", 2.0)
        self.amcl_tf_future_tolerance_sec = max(
            0.0, float(self.get_parameter("amcl_tf_future_tolerance_sec").value)
        )
        self.declare_parameter("sim_ground_truth", False)
        self.sim_ground_truth = bool(self.get_parameter("sim_ground_truth").value)
        if self.use_static_localization:
            self.has_localization = True
            self.loc_state = self.LOC_HEALTHY
            self.get_logger().info("📌 Static identity localization enabled (AMCL bypassed, map->odom identity).")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Authoritative Apartment Reference Topology from single canonical source
        from gracemo_rl.common.places import load_authoritative_places
        self.places = load_authoritative_places()
        self.safe_home_destination = "hallway"

        # Action Client for Nav2 NavigateToPose
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Costmap clearing clients for autonomous recovery
        self.clear_global_costmap_cli = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        self.clear_local_costmap_cli = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )

        # Service client for AMCL non-motion scan/filter update
        self.nomotion_update_cli = self.create_client(Empty, "/request_nomotion_update")

        # Publishers
        self.feedback_pub = self.create_publisher(String, "/coordinator/feedback", 10)
        self.autonomy_state_pub = self.create_publisher(String, "/autonomy/state", 10)
        self.autonomy_status_pub = self.create_publisher(String, "/autonomy/mission_status", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/terrain/emergency_stop", 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.recovery_teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

        # QoS Profiles
        qos_transient = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.command_sub = self.create_subscription(
            String, "/coordinator/command", self.on_command, 10
        )
        self.autonomy_cmd_sub = self.create_subscription(
            String, "/autonomy/mission_command", self.on_command, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl_pose, qos_transient
        )
        self.terrain_sub = self.create_subscription(
            String, "/terrain/status", self.on_terrain_status, qos_reliable
        )
        self.mux_status_sub = self.create_subscription(
            String, "/cmd_vel_mux/status", self.on_mux_status, qos_reliable
        )
        self.estop_sub = self.create_subscription(
            Bool, "/terrain/emergency_stop", self.on_terrain_estop_received, qos_reliable
        )

        # Diagnostic & Supervisory Timers
        self.supervisor_timer = self.create_timer(0.2, self._supervisory_loop)  # 5 Hz
        self.watchdog_timer = self.create_timer(1.0, self._check_navigation_watchdog)
        self.status_pub_timer = self.create_timer(0.5, self._publish_telemetry_status)  # 2 Hz

        # Gazebo Ground Truth Tracking (Simulation verification)
        self.gz_pose: Optional[Tuple[float, float, float]] = None
        self.gz_node = None
        if self.sim_ground_truth:
            try:
                import gz.transport13 as gz_transport
                from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
                self.gz_node = gz_transport.Node()
                self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_dynamic_pose)
                self.get_logger().info("🛰️ Gazebo ground-truth dynamic pose listener attached.")
            except Exception as e:
                self.get_logger().warn(f"Gazebo transport not available for ground truth: {e}")

        self.get_logger().info("🛡️ GRaCEmo Autonomous Operation Supervisor online. Initializing FSM...")
        self.transition_to(self.STATE_SYSTEM_HEALTH_CHECK, "Node bringup completed")

    # -------------------------------------------------------------------------
    # Observable State Machine Transition Engine
    # -------------------------------------------------------------------------
    def transition_to(self, new_state: str, reason: str = ""):
        """
        Authoritative bounded state transition.
        Guarantees: every state change is logged, broadcast to topics, and mirrored to EventBus.
        """
        with self._lock:
            prev_state = self.state
            if prev_state == new_state:
                return

            self.state = new_state
            self.get_logger().info(f"🔄 [FSM Transition] {prev_state} ➔ {new_state} | Reason: {reason}")

            # Publish state topic
            s_msg = String()
            s_msg.data = new_state
            self.autonomy_state_pub.publish(s_msg)

            # Emit feedback envelope
            feedback_speech = self._generate_transition_speech(new_state, reason)
            self.emit_feedback(
                state=new_state,
                speech=feedback_speech,
                details={
                    "previous_state": prev_state,
                    "new_state": new_state,
                    "reason": reason,
                    "mission_id": self.active_mission_id,
                    "waypoint_index": self.current_waypoint_index,
                }
            )

            # Post asynchronously to MNSE Kernel EventBus
            self._emit_kernel_event(
                "AutonomyStateChanged",
                {
                    "previous_state": prev_state,
                    "state": new_state,
                    "reason": reason,
                    "mission_name": self.active_mission_name,
                    "x": self.current_map_x,
                    "y": self.current_map_y,
                }
            )

    def _generate_transition_speech(self, state: str, reason: str) -> str:
        if state == self.STATE_AUTONOMY_READY:
            return "Autonomous mobile robotics system is healthy and ready for missions."
        elif state == self.STATE_MISSION_ACCEPTED:
            return f"Mission accepted: {self.active_mission_name or 'Autonomous transit'}."
        elif state == self.STATE_MISSION_PLANNING:
            dest = self.current_waypoint.get("name") if self.current_waypoint else "target"
            return f"Computing optimal global path to {dest}."
        elif state == self.STATE_MISSION_EXECUTING:
            dest = self.current_waypoint.get("name") if self.current_waypoint else "destination"
            return f"Taking you to the {dest}. Please follow me!"
        elif state == self.STATE_OBSTACLE_HANDLING:
            return "An obstacle is blocking my path. Replanning..."
        elif state == self.STATE_DESTINATION_REACHED:
            dest = self.current_waypoint.get("name") if self.current_waypoint else "destination"
            return f"We have reached the destination: {dest}."
        elif state == self.STATE_MISSION_COMPLETED:
            return f"Mission completed successfully. Standing by."
        elif state == self.STATE_MISSION_PAUSED:
            return "Navigation paused. Holding safe stationary posture."
        elif state == self.STATE_MISSION_CANCELLED:
            return "Navigation cancelled. Standing by."
        elif state == self.STATE_EMERGENCY_STOP:
            return "Emergency stop engaged! All autonomous motion halted immediately."
        elif state == self.STATE_TERRAIN_SAFETY_STOP:
            return "Terrain slope safety envelope exceeded. Halting motion."
        elif state == self.STATE_NAVIGATION_FAILURE:
            return "Navigation failure: Unable to compute collision-free trajectory."
        return ""

    # -------------------------------------------------------------------------
    # Central Supervisory Loop (5 Hz)
    # -------------------------------------------------------------------------
    def _supervisory_loop(self):
        now_sec = time.time()

        # 1. Evaluate Sensor & Telemetry Health
        scan_fresh = (now_sec - self.last_scan_time) < 2.0 if self.last_scan_time > 0 else False
        odom_fresh = (now_sec - self.last_odom_time) < 1.5 if self.last_odom_time > 0 else False

        # In initializing or health check states:
        if self.state in (self.STATE_SYSTEM_INITIALIZING, self.STATE_SYSTEM_HEALTH_CHECK):
            tf_ready, _ = self._check_tf_connectivity()
            nav_server_ready = self.nav_client.server_is_ready()

            safety_status_fresh = (
                now_sec - self.last_terrain_status_time < 2.0
                and now_sec - self.last_mux_status_time < 2.0
                and self.terrain_imu_fresh
            )
            if tf_ready and nav_server_ready and scan_fresh and odom_fresh and safety_status_fresh:
                self.transition_to(self.STATE_WAITING_FOR_LOCALIZATION, "Subsystems active, verifying AMCL convergence")
            return

        # In waiting for localization:
        if self.state == self.STATE_WAITING_FOR_LOCALIZATION:
            # AMCL does not publish map->odom at the coordinator's 5 Hz loop rate.
            # Use the same bounded transform age accepted by mission dispatch.
            is_ready, diag = self.check_localization_readiness(timeout_sec=5.0)
            if is_ready:
                self.transition_to(self.STATE_AUTONOMY_READY, "Localization converged and TF verified")
            else:
                # Ping nomotion update periodically to assist initial convergence
                if (
                    self.nomotion_update_cli.service_is_ready()
                    and now_sec - self.last_nomotion_update_time >= 1.0
                ):
                    try:
                        self.nomotion_update_cli.call_async(Empty.Request())
                        self.last_nomotion_update_time = now_sec
                    except Exception:
                        pass
            return

        # Fail closed if any command-safety or navigation sensor stream disappears.
        if self.state in (self.STATE_MISSION_EXECUTING, self.STATE_MISSION_PLANNING, self.STATE_OBSTACLE_HANDLING):
            localization_tf_ok, localization_tf_reason = self.localization_transform_freshness(timeout_sec=5.0)
            terrain_stale = self.last_terrain_status_time <= 0.0 or now_sec - self.last_terrain_status_time > 2.0
            mux_stale = self.last_mux_status_time <= 0.0 or now_sec - self.last_mux_status_time > 2.0
            if not scan_fresh or not odom_fresh or not localization_tf_ok or terrain_stale or mux_stale or not self.terrain_imu_fresh:
                reason = (
                    "LiDAR, odometry, or safety pipeline telemetry became stale "
                    f"(scan={scan_fresh}, odom={odom_fresh}, localization={localization_tf_reason}, "
                    f"terrain_stale={terrain_stale}, terrain_imu_fresh={self.terrain_imu_fresh}, "
                    f"mux_stale={mux_stale})"
                )
                self.get_logger().error(f"🛑 Navigation health gate failed: {reason}")
                self._abort_active_navigation_goal()
                self.recovery_teleop_pub.publish(Twist())
                self._set_emergency_stop(True)
                failure_state = self.STATE_LOCALIZATION_LOST if not localization_tf_ok else self.STATE_SENSOR_FAILURE
                self.transition_to(failure_state, reason)
                return

        # Phase 10: Active localization recovery execution
        if self.in_recovery_rotation:
            elapsed = now_sec - self.recovery_start_time
            if elapsed < 4.5 and self.loc_state != self.LOC_HEALTHY:
                # Rotate in place to relocalize
                cmd = Twist()
                cmd.angular.z = 0.40
                self.recovery_teleop_pub.publish(cmd)
                if self.nomotion_update_cli.service_is_ready():
                    try:
                        self.nomotion_update_cli.call_async(Empty.Request())
                    except Exception:
                        pass
            else:
                # Stop rotation
                cmd = Twist()
                self.recovery_teleop_pub.publish(cmd)
                self.in_recovery_rotation = False
                self.last_recovery_time = now_sec
                self.get_logger().info(f"✅ Localization recovery finished: loc_state={self.loc_state}, elapsed={elapsed:.1f}s")
                if self.loc_state == self.LOC_HEALTHY and self.state == self.STATE_OBSTACLE_HANDLING:
                    if self.current_waypoint is not None:
                        self.transition_to(self.STATE_MISSION_EXECUTING, "Localization recovered, resuming mission")
                        self._execute_current_waypoint()
            return

        # Phase 10: Map-to-odom transform jump monitor (distinguish true jumps from doorway convergence)
        try:
            t = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            tx = float(t.transform.translation.x)
            ty = float(t.transform.translation.y)
            if self.last_map_to_odom is not None:
                jump = math.hypot(tx - self.last_map_to_odom[0], ty - self.last_map_to_odom[1])
                # True jump anomaly: detects injected offsets (>=0.25m with covariance / >=0.80m translation)
                is_anomalous_jump = (jump > 0.80) or (jump > 0.25 and self.eigen_min > 0.15)
                if is_anomalous_jump and (now_sec - self.last_recovery_time > 8.0) and self.state == self.STATE_MISSION_EXECUTING:
                    self.get_logger().warn(f"⚠️ Map-to-odom jump detected: {jump:.2f}m (eigen_min={self.eigen_min:.2f})! Initiating relocalization recovery.")
                    self._start_localization_recovery(f"map->odom jump of {jump:.2f}m")
            self.last_map_to_odom = (tx, ty)
        except Exception:
            pass

        # Check in-flight sensor loss during active navigation
        if self.state in (self.STATE_MISSION_EXECUTING, self.STATE_MISSION_PLANNING):
            if not scan_fresh and self.last_scan_time > 0:
                self.get_logger().error("❌ Sensor Timeout: LiDAR stream lost during navigation!")
                self._abort_active_navigation_goal()
                self._set_emergency_stop(True)
                self.transition_to(self.STATE_SENSOR_FAILURE, "LiDAR stream timeout > 2.0s")
                return

            if not odom_fresh and self.last_odom_time > 0:
                self.get_logger().error("❌ Odometry Timeout: Base odometry stream lost!")
                self._abort_active_navigation_goal()
                self._set_emergency_stop(True)
                self.transition_to(self.STATE_SENSOR_FAILURE, "Odometry stream timeout > 1.5s")
                return

            # 4-State Localization FSM Supervisor Guard:
            # Longitudinal corridor elongation (l_max up to 18.0) is mathematically normal in long corridors
            # True failure triggers when cross-track error (l_min > 0.80) or dispersion (l_max > 25.0) is sustained
            if self.has_localization and self.loc_state == self.LOC_LOST:
                self.covariance_fault_count += 1
                if self.covariance_fault_count >= 10 and not self.in_recovery_rotation and (now_sec - self.last_recovery_time > 8.0):
                    self._start_localization_recovery("Particle dispersion cross-track covariance exceeded")
                elif self.covariance_fault_count >= 35:
                    self.get_logger().error(
                        f"❌ Localization Degradation: Sustained particle dispersion beyond safety limits "
                        f"(l_max={self.eigen_max:.2f}, l_min={self.eigen_min:.2f}, count={self.covariance_fault_count})!"
                    )
                    self._abort_active_navigation_goal()
                    self._finalize_mission(success=False, reason="Localization covariance bounds exceeded")
                    self.transition_to(self.STATE_LOCALIZATION_LOST, "Localization covariance bounds exceeded")
                    return
            else:
                self.covariance_fault_count = 0

        # Check Dwell Countdown during multi-waypoint missions
        if self.state == self.STATE_DESTINATION_REACHED and self.dwell_end_time > 0:
            if now_sec >= self.dwell_end_time:
                self.dwell_end_time = 0.0
                self._advance_to_next_waypoint()

    def _start_localization_recovery(self, reason: str):
        if self.in_recovery_rotation:
            return
        self.get_logger().warn(f"🔄 Starting active localization recovery: {reason}")
        self.in_recovery_rotation = True
        self.recovery_start_time = time.time()
        self.last_recovery_time = time.time()
        self.loc_state = self.LOC_RECOVERING
        self._abort_active_navigation_goal(keep_goal_msg=True)
        self.transition_to(self.STATE_OBSTACLE_HANDLING, f"Localization recovery: {reason}")
        if self.nomotion_update_cli.service_is_ready():
            try:
                self.nomotion_update_cli.call_async(Empty.Request())
            except Exception:
                pass

    # -------------------------------------------------------------------------
    # Mission Management & Queue Engine (Phase 4)
    # -------------------------------------------------------------------------
    def on_command(self, msg: String):
        """Processes structured JSON mission commands from UARCC, Bridge, or Voice."""
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Malformed JSON in mission command: {e}")
            return

        intent = str(payload.get("intent") or payload.get("action") or "").upper()
        self._current_command_trace_id = payload.get("trace_id") or None
        self.get_logger().info(f"📥 Supervisor received mission intent: {intent}")

        if intent in ("NAVIGATE_TO", "START_MISSION"):
            self.handle_start_mission(payload)
        elif intent in ("PAUSE", "PAUSE_MISSION"):
            self.handle_pause_mission()
        elif intent in ("RESUME", "RESUME_MISSION", "CONTINUE"):
            self.handle_resume_mission()
        elif intent in ("CANCEL", "CANCEL_MISSION", "CANCEL_NAVIGATION"):
            self.handle_cancel_mission()
        elif intent in ("STOP", "EMERGENCY_STOP"):
            self.handle_emergency_stop()
        elif intent in ("RESET", "CLEAR_ESTOP", "RESET_ESTOP", "CLEAR_SAFETY"):
            self.handle_reset_emergency_stop()
        elif intent in ("RETURN_HOME", "RETURN_TO_RECEPTION", "RETURN_TO_SAFE"):
            self.handle_return_home()
        elif intent in ("STATUS", "CURRENT_STATUS", "MISSION_STATUS"):
            self.handle_status_query(payload)
        elif intent in ("SET_INITIAL_POSE", "INITIALIZE_LOCALIZATION"):
            self.handle_initialize_localization(payload)
        elif intent == "CLEAR_COSTMAPS":
            self._clear_costmaps_async()
        else:
            self.get_logger().warn(f"Unhandled mission intent: {intent}")

    def handle_start_mission(self, payload: dict):
        """
        Parses single destination or multi-destination sequential mission.
        Validates boundaries, coordinates, and system readiness.
        Guarantees deterministic arbitration, race-condition preemption, and duplicate suppression.
        """
        # Safety gate: reject mission if emergency stop is active
        if self.terrain_emergency_active or self.state == self.STATE_EMERGENCY_STOP:
            self.emit_feedback(
                state=self.state,
                speech="Cannot accept mission: Emergency Stop is active. Please reset E-Stop first.",
                details={"error": "estop_active"}
            )
            return

        # If waypoints list provided, multi-destination mission
        raw_waypoints = payload.get("waypoints")
        mission_name = payload.get("mission_name") or payload.get("name") or "Autonomous Mission"
        mission_id = payload.get("mission_id") or f"msn-{int(time.time() * 1000)}"
        trace_id = payload.get("trace_id") or self._current_command_trace_id

        parsed_waypoints = []
        if isinstance(raw_waypoints, list) and len(raw_waypoints) > 0:
            for wp in raw_waypoints:
                resolved = self._resolve_waypoint(wp)
                if resolved:
                    parsed_waypoints.append(resolved)
        else:
            # Single destination payload
            resolved = self._resolve_waypoint(payload)
            if resolved:
                parsed_waypoints.append(resolved)

        if not parsed_waypoints:
            self.get_logger().error("Mission rejected: No valid waypoints resolved.")
            self.emit_feedback(
                state=self.state,
                speech="Mission rejected: Invalid destination or coordinates.",
                details={"error": "no_valid_waypoints"}
            )
            return

        # Boundary validation for all waypoints (Apartment bounds: X: [-8.8, 8.8], Y: [-6.3, 6.3])
        for wp in parsed_waypoints:
            c = wp["coords"]
            if not (-8.8 <= c["x"] <= 8.8 and -6.3 <= c["y"] <= 6.3):
                self.get_logger().error(f"Mission rejected: Waypoint {wp['name']} ({c['x']}, {c['y']}) out of bounds.")
                self.emit_feedback(
                    state=self.state,
                    speech=f"Mission rejected: Destination {wp['name']} is outside the safe apartment boundaries.",
                    details={"error": "out_of_bounds", "waypoint": wp}
                )
                return

        # Do not dispatch navigation until localization, sensor liveness, Nav2,
        # and both downstream safety nodes have current telemetry.
        is_ready, diag = self.navigation_readiness()
        if not is_ready:
            self.get_logger().warn(f"Mission rejected: navigation stack is not ready: {diag}")
            self.emit_feedback(
                state=self.state,
                speech=f"Mission cannot start because navigation is not ready: {diag}.",
                details={"error": "navigation_not_ready", "reason": diag},
            )
            return

        if self.use_room_doorway_waypoints:
            start_x, start_y, _ = self.get_current_map_pose()
            routed_waypoints = self._insert_room_doorway_waypoints(
                parsed_waypoints, start_x=start_x, start_y=start_y
            )
            if len(routed_waypoints) != len(parsed_waypoints):
                self.get_logger().info(
                    f"Inserted {len(routed_waypoints) - len(parsed_waypoints)} apartment doorway staging waypoint(s) for safe room transitions."
                )
            parsed_waypoints = routed_waypoints

        # ---------------------------------------------------------------------
        # Preemption & Duplicate Suppression Policy
        # ---------------------------------------------------------------------
        if self.state in (self.STATE_MISSION_EXECUTING, self.STATE_MISSION_PLANNING):
            if self.current_waypoint:
                cur_coords = self.current_waypoint["coords"]
                new_coords = parsed_waypoints[0]["coords"]
                dist = math.hypot(cur_coords["x"] - new_coords["x"], cur_coords["y"] - new_coords["y"])
                if dist < 0.35:
                    self.get_logger().info(f"Duplicate navigation mission to '{parsed_waypoints[0]['name']}' suppressed. Continuing active trajectory.")
                    self.emit_feedback(
                        state=self.state,
                        speech=f"Already navigating to the {parsed_waypoints[0]['name']}. Continuing mission.",
                        details={"status": "duplicate_suppressed", "mission_id": self.active_mission_id, "waypoint": self.current_waypoint["name"]}
                    )
                    return

            # Atomic Replacement: Different target while navigating/planning
            self.get_logger().info(f"🔄 Preempting current mission for new mission: {parsed_waypoints[0]['name']}")
            self._pending_replacement_mission = {
                "mission_id": mission_id,
                "name": mission_name,
                "waypoints": parsed_waypoints,
                "trace_id": trace_id,
            }
            had_active_goal = self.active_goal_handle is not None
            self._abort_active_navigation_goal()
            if not had_active_goal:
                self._dispatch_pending_replacement()
            return

        # Initialize mission structure
        self.active_mission_id = mission_id
        self.active_mission_name = mission_name
        self.active_trace_id = trace_id
        self.mission_waypoints = parsed_waypoints
        self.current_waypoint_index = 0
        self.mission_start_time = time.time()
        self.retry_count = 0

        self.transition_to(self.STATE_MISSION_ACCEPTED, f"Mission '{mission_name}' with {len(parsed_waypoints)} waypoints accepted")
        self._execute_current_waypoint()

    def _resolve_waypoint(self, wp_data: dict) -> Optional[Dict[str, Any]]:
        """Resolves named place or explicit coordinates into canonical waypoint."""
        name = (
            wp_data.get("destination_name")
            or wp_data.get("destination_key")
            or wp_data.get("destination")
            or wp_data.get("name")
            or "Destination"
        )
        dwell_sec = float(wp_data.get("dwell_sec", wp_data.get("dwell_time", wp_data.get("wait_sec", 0.0))))

        coords = wp_data.get("destination_coords") or wp_data.get("coords")
        if not coords and isinstance(wp_data.get("destination"), dict) and "x" in wp_data["destination"]:
            coords = wp_data["destination"]
        if not coords and "x" in wp_data and "y" in wp_data:
            coords = {
                "x": wp_data["x"],
                "y": wp_data["y"],
                "yaw": wp_data.get("yaw", 0.0),
                "frame_id": wp_data.get("frame_id", "map")
            }
        if not coords and "target_x" in wp_data and "target_y" in wp_data:
            coords = {
                "x": wp_data["target_x"],
                "y": wp_data["target_y"],
                "yaw": wp_data.get("target_yaw", wp_data.get("yaw", 0.0)),
                "frame_id": wp_data.get("frame_id", "map")
            }

        if coords and isinstance(coords, dict) and "x" in coords and "y" in coords:
            return {
                "name": name,
                "coords": {
                    "x": float(coords["x"]),
                    "y": float(coords["y"]),
                    "yaw": float(coords.get("yaw", 0.0)),
                    "frame_id": coords.get("frame_id", "map")
                },
                "dwell_sec": dwell_sec
            }

        # Check known places
        raw_key = str(
            wp_data.get("destination_key")
            or wp_data.get("destination_name")
            or wp_data.get("destination")
            or wp_data.get("name")
            or ""
        ).lower().strip()
        key_underscore = raw_key.replace(" ", "_")
        key_spaced = raw_key.replace("_", " ")

        target_place = self.places.get(raw_key) or self.places.get(key_underscore) or self.places.get(key_spaced)
        if target_place:
            return {
                "name": target_place["name"],
                "coords": {
                    "x": target_place["x"],
                    "y": target_place["y"],
                    "yaw": target_place["yaw"],
                    "frame_id": "map"
                },
                "dwell_sec": dwell_sec
            }

        return None

    def _insert_room_doorway_waypoints(
        self, waypoints: List[Dict[str, Any]], start_x: float, start_y: float
    ) -> List[Dict[str, Any]]:
        """Add safe centerline staging poses when a mission crosses apartment rooms.

        NavFn and DWB remain responsible for obstacle-aware planning between these
        poses. The staging points only guide transitions through the known door
        openings in the bundled apartment map; they do not bypass costmaps or sensors.
        """
        doorways = {
            "Master Bedroom": (-5.0, 1.6, math.pi / 2.0),
            "Study Room": (5.0, 1.6, math.pi / 2.0),
            "Kitchen & Dining": (-5.0, -1.6, -math.pi / 2.0),
            "Living Room": (5.0, -1.6, -math.pi / 2.0),
        }
        planned: List[Dict[str, Any]] = []
        cursor_xy = (float(start_x), float(start_y))
        cursor_room = self.resolve_room_from_pose(*cursor_xy)

        for target in waypoints:
            coords = target["coords"]
            target_xy = (float(coords["x"]), float(coords["y"]))
            target_room = self.resolve_room_from_pose(*target_xy)

            if cursor_room != target_room:
                transition_stages = []
                if cursor_room in doorways:
                    x, y, inward_yaw = doorways[cursor_room]
                    transition_stages.append((cursor_room, x, y, inward_yaw + math.pi))
                if target_room in doorways:
                    x, y, inward_yaw = doorways[target_room]
                    transition_stages.append((target_room, x, y, inward_yaw))

                for room, x, y, yaw in transition_stages:
                    stage_xy = (x, y)
                    if math.hypot(cursor_xy[0] - x, cursor_xy[1] - y) < 0.65:
                        continue
                    if math.hypot(target_xy[0] - x, target_xy[1] - y) < 0.65:
                        continue
                    planned.append({
                        "name": f"{room} doorway staging",
                        "coords": {"x": x, "y": y, "yaw": yaw, "frame_id": "map"},
                        "dwell_sec": 0.0,
                        "internal_route_hint": True,
                    })
                    cursor_xy = stage_xy
                    cursor_room = room

            planned.append(target)
            cursor_xy = target_xy
            cursor_room = target_room

        return planned

    def _execute_current_waypoint(self):
        """Dispatches Nav2 action goal for the current waypoint in the queue."""
        if self.current_waypoint_index >= len(self.mission_waypoints):
            self._finalize_mission(success=True)
            return

        wp = self.mission_waypoints[self.current_waypoint_index]
        self.current_waypoint = wp
        dest_name = wp["name"]
        coords = wp["coords"]

        self.transition_to(self.STATE_MISSION_PLANNING, f"Planning path to waypoint {self.current_waypoint_index + 1}/{len(self.mission_waypoints)}: {dest_name}")

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server (/navigate_to_pose) unavailable.")
            self.transition_to(self.STATE_PLANNING_FAILURE, "Nav2 action server not responding")
            return

        # Construct NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = coords.get("frame_id", "map")
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(coords["x"])
        goal_msg.pose.pose.position.y = float(coords["y"])
        goal_msg.pose.pose.position.z = 0.0

        yaw = float(coords.get("yaw", 0.0))
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.active_goal_msg = goal_msg
        self.current_goal_id += 1
        goal_id = self.current_goal_id

        # Update progress trackers
        now = self.get_clock().now()
        self.navigation_start_time = now
        self.last_progress_time = now
        cur_x, cur_y, cur_yaw = self.get_current_map_pose()
        self.last_progress_pose = (cur_x, cur_y)
        self.last_progress_yaw_error = self._goal_yaw_error(cur_yaw)

        send_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=lambda fb: self._on_nav_feedback(fb, goal_id)
        )
        send_future.add_done_callback(lambda fut: self._on_goal_response(fut, goal_id))

    def _on_goal_response(self, future, goal_id: int):
        if goal_id != self.current_goal_id:
            return
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal rejected by Nav2 for {self.current_waypoint['name']}")
            self.transition_to(self.STATE_PLANNING_FAILURE, f"Planner rejected trajectory to {self.current_waypoint['name']}")
            self._handle_navigation_failure("goal_rejected")
            return

        self.active_goal_handle = goal_handle
        cur_x, cur_y, cur_yaw = self.get_current_map_pose()
        self.last_progress_pose = (cur_x, cur_y)
        self.last_progress_yaw_error = self._goal_yaw_error(cur_yaw)
        self.last_progress_time = self.get_clock().now()
        self.transition_to(self.STATE_MISSION_EXECUTING, f"Path confirmed. Tracking trajectory to {self.current_waypoint['name']}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_nav_result(fut, goal_id))

    def _on_nav_feedback(self, feedback_msg, goal_id: int):
        if goal_id != self.current_goal_id:
            return
        fb = feedback_msg.feedback
        if hasattr(fb, "distance_remaining"):
            self.remaining_distance = fb.distance_remaining
        self.last_feedback_time = self.get_clock().now()

    def _on_nav_result(self, future, goal_id: int):
        if self._pending_replacement_mission:
            self.get_logger().info("Executing queued preemption mission following cancellation handshake.")
            self._dispatch_pending_replacement()
            return
        if goal_id != self.current_goal_id:
            return
        status = future.result().status
        self.active_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            # Physical Arrival Verification
            target = self.current_waypoint["coords"]
            cur_x, cur_y, _ = self.get_current_map_pose()
            dist_err_map = math.hypot(cur_x - target["x"], cur_y - target["y"])

            # Requirement 5: Require arrival = TF map-frame pose within tolerance of the goal AND matching Gazebo ground truth
            cur_gz = self.get_gazebo_ground_truth()
            gz_ok = True
            dist_err_gz = dist_err_map
            drift_err = 0.0
            if cur_gz is not None:
                dist_err_gz = math.hypot(cur_gz[0] - target["x"], cur_gz[1] - target["y"])
                drift_err = math.hypot(cur_x - cur_gz[0], cur_y - cur_gz[1])
                if self.sim_ground_truth:
                    gz_ok = (
                        dist_err_gz <= self.arrival_ground_truth_tolerance_m
                        and drift_err <= self.arrival_map_ground_truth_drift_tolerance_m
                    )
                else:
                    gz_ok = True  # Real robot uses TF map pose only (sim_ground_truth default false)

            self.get_logger().info(
                f"Nav2 reports SUCCESS. Verifying arrival: Map error={dist_err_map:.3f}m, "
                f"Gazebo ground-truth error={dist_err_gz:.3f}m, Drift={drift_err:.3f}m "
                f"(map tol={self.arrival_map_tolerance_m:.2f}m, "
                f"Gazebo tol={self.arrival_ground_truth_tolerance_m:.2f}m, "
                f"drift tol={self.arrival_map_ground_truth_drift_tolerance_m:.2f}m)"
            )

            if dist_err_map <= self.arrival_map_tolerance_m and gz_ok:
                self.transition_to(
                    self.STATE_DESTINATION_REACHED,
                    f"Arrival confirmed at {self.current_waypoint['name']} (map_err={dist_err_map:.2f}m, gz_err={dist_err_gz:.2f}m)"
                )
                dwell = self.current_waypoint.get("dwell_sec", 0.0)
                if dwell > 0.0:
                    self.get_logger().info(f"⏳ Dwelling at {self.current_waypoint['name']} for {dwell:.1f}s...")
                    self.dwell_end_time = time.time() + dwell
                else:
                    self._advance_to_next_waypoint()
            else:
                self.get_logger().error(
                    f"❌ Arrival rejected: map error {dist_err_map:.2f}m, Gazebo error "
                    f"{dist_err_gz:.2f}m, or map/Gazebo drift {drift_err:.2f}m exceeds "
                    "the configured arrival guard."
                )
                self._handle_navigation_failure("arrival_ground_truth_mismatch")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Nav2 goal canceled.")
            if self._pending_replacement_mission:
                self.get_logger().info("Executing queued preemption mission following cancellation handshake.")
                self._dispatch_pending_replacement()
                return
            if self.state not in (self.STATE_MISSION_PAUSED, self.STATE_EMERGENCY_STOP):
                self.transition_to(self.STATE_MISSION_CANCELLED, "Goal cancellation confirmed")

        else:
            self.get_logger().warn(f"Nav2 goal failed or aborted with status code {status}.")
            self._handle_navigation_failure(f"nav2_code_{status}")

    def _dispatch_pending_replacement(self):
        """Dispatches an atomically queued preempting mission once prior goal is cleared."""
        if not self._pending_replacement_mission:
            return
        ready, reason = self.navigation_readiness()
        if not ready:
            pending = self._pending_replacement_mission
            self._pending_replacement_mission = None
            self.get_logger().error(f"Queued replacement mission blocked by navigation health gate: {reason}")
            self.emit_feedback(
                state=self.state,
                speech=f"Replacement mission was not started because navigation is not ready: {reason}.",
                details={"error": "navigation_not_ready", "reason": reason, "mission_id": pending["mission_id"]},
            )
            self.active_mission_id = None
            self.active_mission_name = None
            self.mission_waypoints = []
            self.current_waypoint = None
            return
        m = self._pending_replacement_mission
        self._pending_replacement_mission = None
        self.active_mission_id = m["mission_id"]
        self.active_mission_name = m["name"]
        self.active_trace_id = m.get("trace_id")
        self.mission_waypoints = m["waypoints"]
        self.current_waypoint_index = 0
        self.mission_start_time = time.time()
        self.retry_count = 0
        self.transition_to(
            self.STATE_MISSION_ACCEPTED,
            f"Preempted with mission '{m['name']}' ({len(m['waypoints'])} waypoints)"
        )
        self._execute_current_waypoint()

    def _advance_to_next_waypoint(self):
        """Advances pointer to next destination in sequential mission queue."""
        self.current_waypoint_index += 1
        self.retry_count = 0
        if self.current_waypoint_index < len(self.mission_waypoints):
            self._execute_current_waypoint()
        else:
            self._finalize_mission(success=True)

    def _finalize_mission(self, success: bool, reason: str = "", cancelled: bool = False):
        """Records mission outcome in history and returns to idle ready state."""
        duration = time.time() - self.mission_start_time if self.mission_start_time else 0.0
        cur_x, cur_y, _ = self.get_current_map_pose()

        entry = {
            "mission_id": self.active_mission_id or f"msn-{int(time.time())}",
            "name": self.active_mission_name or "Mission",
            "success": success,
            "status": "COMPLETED" if success else ("CANCELLED" if cancelled else "FAILED"),
            "reason": reason,
            "waypoints_completed": self.current_waypoint_index if success else max(0, self.current_waypoint_index - 1),
            "total_waypoints": len(self.mission_waypoints),
            "duration_sec": round(duration, 1),
            "end_pose": {"x": round(cur_x, 2), "y": round(cur_y, 2)},
            "timestamp": time.time(),
        }
        self.mission_history.insert(0, entry)
        if len(self.mission_history) > 30:
            self.mission_history.pop()

        if success:
            self.transition_to(self.STATE_MISSION_COMPLETED, f"All {len(self.mission_waypoints)} destinations reached")
            self.transition_to(self.STATE_WAITING_FOR_NEXT_MISSION, "Mission completed. Ready for next assignment")
            self.transition_to(self.STATE_AUTONOMY_READY, "System standing by in Autonomous Ready mode")
        elif cancelled:
            self.transition_to(self.STATE_MISSION_CANCELLED, reason)
            self.transition_to(self.STATE_AUTONOMY_READY, "System standing by in Autonomous Ready mode")
        else:
            self.transition_to(self.STATE_NAVIGATION_FAILURE, f"Mission incomplete: {reason}")

        self.active_mission_id = None
        self.active_mission_name = None
        self.active_trace_id = None
        self.mission_waypoints = []
        self.current_waypoint = None

    # -------------------------------------------------------------------------
    # Autonomous Recovery & Obstacle Replanning
    # -------------------------------------------------------------------------
    def _handle_navigation_failure(self, failure_reason: str):
        """Bounded autonomous recovery: clears costmaps and retries up to max_retries."""
        if self.retry_count < self.max_retries and self.active_goal_msg is not None:
            self.retry_count += 1
            self.transition_to(
                self.STATE_OBSTACLE_HANDLING,
                f"Obstacle intervention: retry {self.retry_count}/{self.max_retries} for {self.current_waypoint['name']}"
            )
            self._clear_costmaps_async()

            # Retry on the ROS clock so a paused simulation cannot race a
            # wall-clock thread and issue motion while the simulator is stopped.
            if self.retry_timer is not None:
                self.destroy_timer(self.retry_timer)
            self.retry_timer = self.create_timer(1.5, self._retry_current_goal)
        else:
            self.get_logger().error(f"❌ Retries exhausted for {self.current_waypoint['name']}. Mission halted.")
            self._finalize_mission(success=False, reason=f"Retries exhausted ({failure_reason})")

    def _retry_current_goal(self):
        timer = self.retry_timer
        self.retry_timer = None
        if timer is not None:
            self.destroy_timer(timer)

        if self.state == self.STATE_OBSTACLE_HANDLING and self.active_goal_msg:
            ready, reason = self.navigation_readiness()
            if not ready:
                self.get_logger().error(f"Retry blocked by navigation health gate: {reason}")
                self._set_emergency_stop(True)
                self._finalize_mission(success=False, reason=f"Retry blocked: {reason}")
                return
            self.transition_to(self.STATE_MISSION_PLANNING, f"Re-attempting navigation trajectory ({self.retry_count}/{self.max_retries})")
            self.current_goal_id += 1
            goal_id = self.current_goal_id
            now = self.get_clock().now()
            self.navigation_start_time = now
            self.last_progress_time = now
            cur_x, cur_y, cur_yaw = self.get_current_map_pose()
            self.last_progress_pose = (cur_x, cur_y)
            self.last_progress_yaw_error = self._goal_yaw_error(cur_yaw)
            self.active_goal_msg.pose.header.stamp = now.to_msg()
            send_future = self.nav_client.send_goal_async(
                self.active_goal_msg, feedback_callback=lambda fb: self._on_nav_feedback(fb, goal_id)
            )
            send_future.add_done_callback(lambda fut: self._on_goal_response(fut, goal_id))

    def _check_navigation_watchdog(self):
        """Detects stalled or trapped robot during active trajectory execution."""
        if self.state != self.STATE_MISSION_EXECUTING or not self.active_goal_handle:
            return

        now = self.get_clock().now()
        cur_x, cur_y, cur_yaw = self.get_current_map_pose()
        disp = math.hypot(cur_x - self.last_progress_pose[0], cur_y - self.last_progress_pose[1])

        yaw_error = self._goal_yaw_error(cur_yaw)
        yaw_progress = (
            yaw_error is not None
            and self.last_progress_yaw_error is not None
            and self.last_progress_yaw_error - yaw_error >= 0.08
        )
        # Progress must be visible in the robot pose or goal-heading error.
        # Commanded/odometry velocity alone can stay nonzero against a wall and
        # previously kept a trapped robot's watchdog alive indefinitely.
        is_making_progress = disp >= 0.08 or yaw_progress
        if is_making_progress:
            self.last_progress_pose = (cur_x, cur_y)
            self.last_progress_yaw_error = yaw_error
            self.last_progress_time = now
            return

        stagnant_sec = (now - self.last_progress_time).nanoseconds / 1e9
        total_sec = (now - self.navigation_start_time).nanoseconds / 1e9 if self.navigation_start_time else 0.0

        # Assist AMCL particle convergence during slow creep or goal deceleration
        if stagnant_sec > 2.0 and self.nomotion_update_cli.service_is_ready():
            try:
                self.nomotion_update_cli.call_async(Empty.Request())
            except Exception:
                pass

        if stagnant_sec > self.navigation_stagnant_timeout_sec or total_sec > 180.0:
            self.get_logger().warn(
                f"⏱️ Trajectory Watchdog: No measured progress for {stagnant_sec:.1f}s "
                f"(limit {self.navigation_stagnant_timeout_sec:.1f}s). Triggering obstacle recovery."
            )
            self._abort_active_navigation_goal(keep_goal_msg=True)
            self._handle_navigation_failure(f"watchdog_stagnant_{stagnant_sec:.0f}s")

    def _goal_yaw_error(self, current_yaw: float) -> Optional[float]:
        """Return absolute heading error to the active waypoint when available."""
        if not self.current_waypoint:
            return None
        target_yaw = float(self.current_waypoint.get("coords", {}).get("yaw", 0.0))
        delta = target_yaw - current_yaw
        return abs(math.atan2(math.sin(delta), math.cos(delta)))

    def _clear_costmaps_async(self):
        req = ClearEntireCostmap.Request()
        if self.clear_global_costmap_cli.service_is_ready():
            self.clear_global_costmap_cli.call_async(req)
        if self.clear_local_costmap_cli.service_is_ready():
            self.clear_local_costmap_cli.call_async(req)
        self.get_logger().info("🧹 Sent async requests to clear global and local costmaps.")

    # -------------------------------------------------------------------------
    # Mission Control Handlers (Pause, Resume, Cancel, Return Home, E-Stop)
    # -------------------------------------------------------------------------
    def handle_pause_mission(self):
        """Pauses in-flight mission and holds stationary position."""
        if self.state not in (self.STATE_MISSION_EXECUTING, self.STATE_MISSION_PLANNING, self.STATE_OBSTACLE_HANDLING):
            self.emit_feedback(state=self.state, speech="No active mission to pause.", details={"status": "not_active"})
            return

        self.get_logger().info("⏸️ Operator requested PAUSE mission.")
        self._abort_active_navigation_goal()
        self.transition_to(self.STATE_MISSION_PAUSED, "Mission paused by operator instruction")

    def handle_resume_mission(self):
        """Resumes paused mission at the current waypoint."""
        if self.terrain_emergency_active or self.terrain_safety_active or self.state in (
            self.STATE_EMERGENCY_STOP,
            self.STATE_TERRAIN_SAFETY_STOP,
        ):
            self.emit_feedback(
                state=self.state,
                speech="Cannot resume mission while emergency stop or safety fault is active. Clear fault and reset first.",
                details={"status": "estop_active"}
            )
            return

        if self.state != self.STATE_MISSION_PAUSED or not self.current_waypoint:
            self.emit_feedback(state=self.state, speech="No paused mission available to resume.", details={"status": "no_paused_mission"})
            return

        ready, reason = self.navigation_readiness()
        if not ready:
            self.emit_feedback(
                state=self.state,
                speech=f"Mission cannot resume because navigation is not ready: {reason}.",
                details={"status": "navigation_not_ready", "reason": reason},
            )
            return

        self.get_logger().info(f"▶️ Resuming mission for waypoint {self.current_waypoint['name']}")
        self.retry_count = 0
        self._execute_current_waypoint()

    def handle_reset_emergency_stop(self):
        """Authoritatively resets emergency stop following safety clearance."""
        if not self.terrain_emergency_active and self.state not in (
            self.STATE_EMERGENCY_STOP,
            self.STATE_TERRAIN_SAFETY_STOP,
        ):
            self.emit_feedback(state=self.state, speech="No emergency stop is latched.", details={"status": "no_estop"})
            return
        if self.terrain_safety_active:
            self.emit_feedback(
                state=self.state,
                speech="Terrain safety fault is still active. Clear the slope fault before resetting.",
                details={"status": "terrain_fault_active"},
            )
            return
        ready, reason = self.navigation_readiness(ignore_emergency_stop=True)
        if not ready:
            self.emit_feedback(
                state=self.state,
                speech=f"Emergency stop remains latched because navigation is not ready: {reason}.",
                details={"status": "navigation_not_ready", "reason": reason},
            )
            return
        self.get_logger().info("🔓 Operator authorized Emergency Stop Reset.")
        self._set_emergency_stop(False)
        self.transition_to(self.STATE_AUTONOMY_READY, "Emergency stop reset authorized by operator")

    def handle_cancel_mission(self):
        """Cancels active mission, clears queue, and returns to Autonomy Ready."""
        self.get_logger().info("⏹️ Operator requested CANCEL mission.")
        self._abort_active_navigation_goal()
        self._finalize_mission(success=False, reason="Cancelled by operator", cancelled=True)

    def handle_emergency_stop(self):
        """Asserts emergency stop gate, cancels motion, and holds safety halt."""
        self.get_logger().warn("🚨 Authoritative EMERGENCY STOP engaged!")
        self._abort_active_navigation_goal()
        self._set_emergency_stop(True)
        self.transition_to(self.STATE_EMERGENCY_STOP, "Operator asserted Emergency Stop")

    def handle_return_home(self):
        """Aborts current mission and navigates to the designated safe home location."""
        self.get_logger().info(f"🏠 Returning to designated safe destination: {self.safe_home_destination}")
        home_wp = self.places.get(self.safe_home_destination, self.places["hallway"])
        payload = {
            "mission_name": "Return to Base",
            "waypoints": [
                {
                    "name": home_wp["name"],
                    "coords": {"x": home_wp["x"], "y": home_wp["y"], "yaw": home_wp["yaw"], "frame_id": "map"},
                    "dwell_sec": 0.0
                }
            ]
        }
        self.handle_start_mission(payload)

    def resolve_room_from_pose(self, x: float, y: float) -> str:
        """Grounds map coordinates to authoritative apartment room topology."""
        if -1.3 <= y <= 1.3:
            return "Central Hallway"
        elif y > 1.3:
            return "Master Bedroom" if x < 0.0 else "Study Room"
        else:  # y < -1.3
            return "Kitchen & Dining" if x < 0.0 else "Living Room"

    def handle_status_query(self, payload: Optional[dict] = None):
        """Emits instantaneous verbal and JSON status grounded to real pose, velocity, and topology."""
        cur_x, cur_y, _ = self.get_current_map_pose()
        room = self.resolve_room_from_pose(cur_x, cur_y)
        query_type = str(payload.get("query", "general")).lower() if payload else "general"

        if query_type in ("location", "where_are_you", "where"):
            if self.state == self.STATE_MISSION_EXECUTING and self.current_waypoint:
                speech = f"I am currently in the {room}, navigating towards the {self.current_waypoint['name']}."
            else:
                speech = f"I am currently positioned inside the {room} at coordinates X={cur_x:.1f}, Y={cur_y:.1f}."
        elif query_type in ("destination", "where_are_you_going", "where_going"):
            if self.state == self.STATE_MISSION_EXECUTING and self.current_waypoint:
                speech = f"I am navigating to the {self.current_waypoint['name']}. Remaining distance is {self.remaining_distance:.1f} meters."
            else:
                speech = "I do not have an active navigation destination. I am standing by."
        elif query_type in ("moving", "are_you_moving", "motion"):
            is_moving = abs(self.last_linear_vel) > 0.02 or abs(self.last_angular_vel) > 0.02
            if is_moving and self.current_waypoint:
                speech = f"Yes, I am moving towards the {self.current_waypoint['name']} at {abs(self.last_linear_vel):.2f} meters per second."
            elif self.state == self.STATE_MISSION_EXECUTING:
                speech = "I am currently navigating, but momentarily paused for trajectory computation."
            elif self.state == self.STATE_OBSTACLE_HANDLING:
                speech = "I am stopped because an obstacle is blocking my path. Replanning..."
            elif self.state == self.STATE_EMERGENCY_STOP:
                speech = "No, I am halted due to an emergency stop."
            else:
                speech = "No, I am stationary and standing by."
        elif query_type in ("stuck", "are_you_stuck", "why_stopped", "stopped"):
            if self.state == self.STATE_OBSTACLE_HANDLING:
                speech = "I encountered an obstacle in my path. I am clearing costmaps and finding an alternate route."
            elif self.state == self.STATE_EMERGENCY_STOP:
                speech = "I stopped because the emergency stop is active."
            elif self.state == self.STATE_MISSION_PAUSED:
                speech = "I stopped because navigation was paused by the operator."
            elif self.state == self.STATE_AUTONOMY_READY:
                speech = "I am not stuck; I am standing by safely at my current location."
            else:
                speech = f"My current operational state is {self.state}."
        else:
            # General status report
            if self.state == self.STATE_MISSION_EXECUTING and self.current_waypoint:
                speech = f"Navigating to {self.current_waypoint['name']} from {room}. Remaining distance is {self.remaining_distance:.1f} meters."
            elif self.state == self.STATE_AUTONOMY_READY:
                speech = f"System is ready in Autonomous Operation Mode inside the {room}."
            elif self.state == self.STATE_EMERGENCY_STOP:
                speech = "System is locked in Emergency Stop state."
            else:
                speech = f"Autonomous state is {self.state} in the {room}."

        self.emit_feedback(
            state=self.state,
            speech=speech,
            details={
                "state": self.state,
                "current_pose": {"x": round(cur_x, 2), "y": round(cur_y, 2)},
                "room": room,
                "active_waypoint": self.current_waypoint.get("name") if self.current_waypoint else None,
                "distance_remaining": round(self.remaining_distance, 2),
                "linear_vel": round(self.last_linear_vel, 3),
                "angular_vel": round(self.last_angular_vel, 3),
                "localization_state": self.loc_state,
                "eigen_max": round(self.eigen_max, 3),
                "eigen_min": round(self.eigen_min, 3),
                "query_type": query_type,
            }
        )

    def _abort_active_navigation_goal(self, keep_goal_msg: bool = False):
        """Safely issues cancel to Nav2 if an action goal handle is active."""
        if self.active_goal_handle:
            try:
                self.active_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Failed to cancel Nav2 goal handle: {e}")
            self.active_goal_handle = None
        if not keep_goal_msg:
            self.active_goal_msg = None
        self.current_goal_id += 1

    def _set_emergency_stop(self, active: bool):
        """Controls authoritative terrain monitor emergency stop gate."""
        self.terrain_emergency_active = active
        e_msg = Bool()
        e_msg.data = active
        self.emergency_stop_pub.publish(e_msg)

    # -------------------------------------------------------------------------
    # Telemetry, Subscriptions, and Diagnostics
    # -------------------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        self.last_odom_time = time.time()
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y
        self.last_linear_vel = msg.twist.twist.linear.x
        self.last_angular_vel = msg.twist.twist.angular.z

    def on_scan(self, msg: LaserScan):
        self.last_scan_time = time.time()

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        self.last_amcl_time = time.time()
        self.current_map_x = msg.pose.pose.position.x
        self.current_map_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_map_yaw = math.atan2(siny_cosp, cosy_cosp)
        cov = msg.pose.covariance
        c_xx = cov[0]
        c_xy = cov[1]
        c_yy = cov[7]
        c_yaw = cov[35]
        self.amcl_covariance_xx = c_xx
        self.amcl_covariance_yy = c_yy
        self.amcl_covariance_yaw = c_yaw
        self.has_localization = True

        # Positional covariance eigenvalue decomposition
        trace = c_xx + c_yy
        diff = c_xx - c_yy
        disc = math.sqrt(max(0.0, diff * diff + 4.0 * c_xy * c_xy))
        l_max = (trace + disc) / 2.0
        l_min = (trace - disc) / 2.0
        self.eigen_max = l_max
        self.eigen_min = l_min

        # 4-State Localization FSM with temporal hysteresis
        # Distinguish corridor elongation (l_max up to 18.0) from cross-track failure (l_min > 0.80)
        is_lost = (l_min > 0.80) or (l_max > 25.0) or (c_yaw > 0.85)
        if is_lost:
            self.loc_lost_counter += 1
            if self.loc_lost_counter >= self.loc_hysteresis_cycles:
                self.loc_state = self.LOC_LOST
        else:
            self.loc_lost_counter = 0
            if l_max > 4.0 or l_min > 0.35 or c_yaw > 0.20:
                self.loc_state = self.LOC_DEGRADED
            else:
                self.loc_state = self.LOC_HEALTHY

    def on_terrain_status(self, msg: String):
        self.last_terrain_status_time = time.time()
        try:
            data = json.loads(msg.data)
            self.terrain_imu_fresh = data.get("imu_fresh") is True
            breached = (
                data.get("slope_state") == "LIMIT_EXCEEDED"
                or data.get("safety_envelope_breached") is True
            )
            was_breached = self.terrain_safety_active
            self.terrain_safety_active = breached
            if breached and not was_breached:
                self.get_logger().error("🛑 Terrain safety envelope breach detected!")
                self._abort_active_navigation_goal()
                self.recovery_teleop_pub.publish(Twist())
                self._set_emergency_stop(True)
                self.transition_to(self.STATE_TERRAIN_SAFETY_STOP, "Pitch/roll angle limit exceeded")
        except Exception:
            pass

    def on_mux_status(self, _msg: String):
        self.last_mux_status_time = time.time()

    def on_terrain_estop_received(self, msg: Bool):
        if msg.data == self.terrain_emergency_active:
            return  # Ignore redundant echo

        # A false message from another publisher must not release a latched stop.
        # The explicit reset handler updates this node's state before publishing its
        # own false echo, so this callback can safely accept only assertions.
        if not msg.data:
            self.get_logger().warn("Ignoring external E-Stop clear; use the coordinator reset command after health checks.")
            return

        self.terrain_emergency_active = msg.data
        if msg.data:
            self._abort_active_navigation_goal()
            self.in_recovery_rotation = False
            self.recovery_teleop_pub.publish(Twist())
            if self.state != self.STATE_EMERGENCY_STOP:
                self.transition_to(self.STATE_EMERGENCY_STOP, "E-Stop received on /terrain/emergency_stop")
        elif self.state in (self.STATE_EMERGENCY_STOP, self.STATE_TERRAIN_SAFETY_STOP):
            if self.terrain_safety_active:
                self.transition_to(self.STATE_TERRAIN_SAFETY_STOP, "E-Stop cleared but terrain fault remains active")
                return
            ready, reason = self.navigation_readiness(ignore_emergency_stop=True)
            if ready:
                self.transition_to(self.STATE_AUTONOMY_READY, "E-Stop released and navigation health verified")
            else:
                self.transition_to(self.STATE_WAITING_FOR_LOCALIZATION, f"E-Stop cleared; waiting for navigation health: {reason}")

    def _on_gz_dynamic_pose(self, msg):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                q = p.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.gz_pose = (float(p.position.x), float(p.position.y), float(yaw))
                break

    def get_gazebo_ground_truth(self) -> Optional[Tuple[float, float, float]]:
        return self.gz_pose

    def get_current_map_pose(self) -> Tuple[float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            px = t.transform.translation.x
            py = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return px, py, yaw
        except Exception:
            return self.current_map_x, self.current_map_y, self.current_map_yaw

    def _check_tf_connectivity(self) -> Tuple[bool, str]:
        try:
            if self.tf_buffer.can_transform("map", "base_footprint", rclpy.time.Time()):
                return True, "TF connected"
        except Exception as e:
            return False, str(e)
        return False, "Transform 'map' -> 'base_footprint' unavailable"

    def check_localization_readiness(self, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        tf_ok, tf_diag = self._check_tf_connectivity()
        if not tf_ok:
            return False, tf_diag

        if self.use_static_localization:
            cur_x, cur_y, _ = self.get_current_map_pose()
            return True, f"Static localization active at map=({cur_x:.2f}, {cur_y:.2f})"

        if not self.has_localization:
            return False, "Awaiting AMCL initial pose publication"

        if not all(math.isfinite(value) for value in (
            self.current_map_x,
            self.current_map_y,
            self.amcl_covariance_xx,
            self.amcl_covariance_yy,
            self.amcl_covariance_yaw,
            self.eigen_min,
            self.eigen_max,
        )):
            return False, "AMCL pose or covariance contains non-finite values"
        if self.amcl_covariance_xx <= 0.0 and self.amcl_covariance_yy <= 0.0:
            return False, "AMCL covariance is uninitialized"

        # Check cross-track covariance bounds and localization state
        if self.eigen_min > 0.80 or self.loc_state == self.LOC_LOST:
            return False, f"AMCL cross-track covariance high (l_min={self.eigen_min:.2f} > 0.80, state={self.loc_state})"

        tf_fresh, tf_diag = self.localization_transform_freshness(timeout_sec=timeout_sec)
        if not tf_fresh:
            return False, tf_diag

        cur_x, cur_y, _ = self.get_current_map_pose()
        return True, f"Localization healthy ({self.loc_state}) at map=({cur_x:.2f}, {cur_y:.2f})"

    def localization_transform_freshness(self, timeout_sec: float = 5.0) -> Tuple[bool, str]:
        if self.use_static_localization:
            return True, "static localization"
        try:
            transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
            stamp = rclpy.time.Time.from_msg(transform.header.stamp)
            age_sec = (self.get_clock().now() - stamp).nanoseconds / 1e9
            # Nav2 AMCL intentionally future-dates map->odom by its configured
            # transform_tolerance (1.5 s in nav2_params.yaml). Allow that bounded
            # lead while still rejecting larger clock/TF inconsistencies.
            if age_sec < -self.amcl_tf_future_tolerance_sec:
                return False, (
                    f"map->odom transform is too far in the future ({-age_sec:.2f}s > "
                    f"{self.amcl_tf_future_tolerance_sec:.2f}s)"
                )
            if age_sec > timeout_sec:
                return False, f"map->odom transform is stale ({age_sec:.2f}s > {timeout_sec:.2f}s)"
            return True, f"map->odom age {age_sec:.2f}s"
        except Exception as exc:
            return False, f"map->odom transform unavailable: {exc}"

    def navigation_readiness(self, ignore_emergency_stop: bool = False) -> Tuple[bool, str]:
        now_sec = time.time()
        if self.terrain_safety_active:
            return False, "terrain safety envelope is breached"
        if self.terrain_emergency_active and not ignore_emergency_stop:
            return False, "emergency stop is latched"
        if not self.nav_client.server_is_ready():
            return False, "Nav2 navigate_to_pose action server is unavailable"
        if self.last_scan_time <= 0.0 or now_sec - self.last_scan_time > 2.0:
            return False, "LiDAR scan is missing or stale"
        if self.last_odom_time <= 0.0 or now_sec - self.last_odom_time > 1.5:
            return False, "odometry is missing or stale"
        if self.last_terrain_status_time <= 0.0 or now_sec - self.last_terrain_status_time > 2.0:
            return False, "terrain safety monitor status is missing or stale"
        if not self.terrain_imu_fresh:
            return False, "terrain safety monitor reports stale IMU telemetry"
        if self.last_mux_status_time <= 0.0 or now_sec - self.last_mux_status_time > 2.0:
            return False, "velocity multiplexer status is missing or stale"
        return self.check_localization_readiness(timeout_sec=5.0)

    def handle_initialize_localization(self, payload: dict):
        coords = payload.get("coords") or payload.get("pose") or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        px = float(coords.get("x", 0.0))
        py = float(coords.get("y", 0.0))
        yaw = float(coords.get("yaw", 0.0))

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
        pose_msg.pose.pose.position.x = px
        pose_msg.pose.pose.position.y = py
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        cov = [0.0] * 36
        # Tight initial pose covariance (0.1m stddev in x/y, ~0.07 rad in yaw)
        cov[0] = 0.01
        cov[7] = 0.01
        cov[35] = 0.005
        pose_msg.pose.covariance = cov

        self.initial_pose_pub.publish(pose_msg)
        # Force non-motion updates to converge particle cloud immediately
        if self.nomotion_update_cli.service_is_ready():
            for _ in range(5):
                try:
                    self.nomotion_update_cli.call_async(Empty.Request())
                except Exception:
                    pass

        self.emit_feedback(
            state=self.state,
            speech=f"Initial pose broadcast at X={px:.2f}, Y={py:.2f}.",
            details={"action": "set_initial_pose", "x": px, "y": py}
        )

    # -------------------------------------------------------------------------
    # Telemetry Status Publication (2 Hz) & EventBus Bridge
    # -------------------------------------------------------------------------
    def _publish_telemetry_status(self):
        """Publishes comprehensive operational mission status on /autonomy/mission_status."""
        cur_x, cur_y, cur_yaw = self.get_current_map_pose()
        now_sec = time.time()

        dwell_rem = max(0.0, self.dwell_end_time - now_sec) if self.dwell_end_time > 0 else 0.0
        elapsed_m = (now_sec - self.mission_start_time) if self.mission_start_time else 0.0

        status_payload = {
            "state": self.state,
            "mission": {
                "id": self.active_mission_id,
                "name": self.active_mission_name,
                "waypoint_index": self.current_waypoint_index,
                "total_waypoints": len(self.mission_waypoints),
                "active_waypoint": self.current_waypoint.get("name") if self.current_waypoint else None,
                "remaining_distance": round(self.remaining_distance, 2),
                "dwell_remaining_sec": round(dwell_rem, 1),
                "elapsed_sec": round(elapsed_m, 1),
            },
            "robot": {
                "pose": {"x": round(cur_x, 3), "y": round(cur_y, 3), "yaw": round(cur_yaw, 3)},
                "room": self.resolve_room_from_pose(cur_x, cur_y),
                "twist": {"linear": round(self.last_linear_vel, 3), "angular": round(self.last_angular_vel, 3)},
                "has_localization": self.has_localization,
                "localization_state": self.loc_state,
                "covariance": {"xx": round(self.amcl_covariance_xx, 4), "yy": round(self.amcl_covariance_yy, 4)},
                "eigenvalues": {"max": round(self.eigen_max, 4), "min": round(self.eigen_min, 4)},
                "battery_pct": 98.5,
            },
            "subsystems": {
                "scan_fresh": (now_sec - self.last_scan_time) < 2.0 if self.last_scan_time > 0 else False,
                "odom_fresh": (now_sec - self.last_odom_time) < 1.5 if self.last_odom_time > 0 else False,
                "nav2_ready": self.nav_client.server_is_ready(),
                "estop_active": self.terrain_emergency_active,
            },
            "history": self.mission_history[:5],
            "timestamp": now_sec,
        }

        msg = String()
        msg.data = json.dumps(status_payload)
        self.autonomy_status_pub.publish(msg)

        st_msg = String()
        st_msg.data = self.state
        self.autonomy_state_pub.publish(st_msg)

    def emit_feedback(self, state: str, speech: str, details: dict):
        msg = String()
        data = {
            "state": state,
            "speech": speech,
            "details": details,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        msg.data = json.dumps(data)
        self.feedback_pub.publish(msg)

    def _emit_kernel_event(self, event_type: str, data: dict):
        """Asynchronously dispatches canonical correlated telemetry to the MNSE EventBus."""
        trace_id = self.active_trace_id or self._current_command_trace_id
        try:
            trace_id = str(uuid.UUID(str(trace_id))) if trace_id else str(uuid.uuid4())
        except (ValueError, TypeError, AttributeError):
            trace_id = str(uuid.uuid4())

        def _post():
            try:
                payload = {
                    "id": str(uuid.uuid4()),
                    "trace_id": trace_id,
                    "session_id": "ros2_autonomy",
                    "source": "System",
                    "observed_by": "AutonomousSupervisor",
                    "event_type": {"type": event_type, "data": data},
                    "timestamp": int(time.time()),
                    "parent_event_id": None,
                }
                req = urllib.request.Request(
                    f"{self.kernel_url}/emit",
                    data=json.dumps(payload).encode("utf-8"),
                    headers={"Content-Type": "application/json"}
                )
                with urllib.request.urlopen(req, timeout=0.5):
                    pass
            except Exception as exc:
                now = time.time()
                if now - self._last_kernel_error_log >= 10.0:
                    self._last_kernel_error_log = now
                    self.get_logger().warning(f"Kernel telemetry post failed ({self.kernel_url}/emit): {exc}")

        t = threading.Thread(target=_post, daemon=True)
        t.start()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
