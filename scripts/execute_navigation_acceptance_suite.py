#!/usr/bin/env python3
"""
GRaCEmo ViRa — Hardened Navigation Acceptance Test Suite (AT-01 through AT-07).
Features:
  1. Pre-Test Validity Gate:
     Requires |TF map pose - Gazebo pose| < 0.15 m and yaw error < 0.10 rad.
     If gate fails after 3 retries, marks test as INVALID (not FAIL) and reports
     INVALID counts separately to decouple localization faults from navigation faults.
  2. AT-04 Hardening:
     Asserts robot was moving (speed > 0.05 m/s) BEFORE triggering E-stop, then
     verifies rapid halt (< 0.05 m/s) and EMERGENCY_STOP state.
  3. AT-05 Hardening:
     Asserts robot was moving before trigger, triggers replan/recovery, and
     explicitly observes planner/coordinator replan transition and continuation.
  4. AT-06 Hardening:
     Commands 90° rotation, requires >= 80° yaw change vs Gazebo ground truth.
  5. AT-07 Hardening:
     Executes only after a verified arrival, asserting zero creep (< 0.05 m/s, < 0.05 rad/s)
     over a 3-second dwell period.
  6. Dual-source Ground Truth Verification:
     Cross-checks TF map frame against Gazebo Harmonic /world/gracemo_home/dynamic_pose/info.
"""

import os
import sys
import math
import time
import json
from typing import Optional, Dict, Any, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from nav2_msgs.msg import CollisionMonitorState
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class NavigationAcceptanceTester(Node):
    def __init__(self, log_file: str):
        super().__init__("navigation_acceptance_tester")
        self.log_file = log_file
        os.makedirs(os.path.dirname(os.path.abspath(log_file)), exist_ok=True)

        # Telemetry & Poses
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.has_odom = False

        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.has_gz = False

        self.last_collision_action = 0
        self.last_collision_poly = ""
        self.collision_stop_count = 0
        self.collision_slow_count = 0

        self.supervisor_state = "UNKNOWN"
        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_l_min = 999.0
        self.amcl_l_max = 999.0

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.state_sub = self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_collision_state, 10)
        self.supervisor_sub = self.create_subscription(String, "/autonomy/state", self.on_autonomy_state, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)

        # Publishers & Service Clients
        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.estop_pub = self.create_publisher(Bool, "/terrain/emergency_stop", 10)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.nomotion_cli = self.create_client(Empty, "/request_nomotion_update")

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport Node
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self.on_gz_pose)

    def log(self, text: str = ""):
        print(text)
        with open(self.log_file, "a") as f:
            f.write(text + "\n")

    def on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                q = p.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                self.gz_x = float(p.position.x)
                self.gz_y = float(p.position.y)
                self.gz_yaw = float(yaw)
                self.has_gz = True
                break

    def on_odom(self, msg: Odometry):
        self.has_odom = True
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.current_vx = msg.twist.twist.linear.x
        self.current_wz = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def on_amcl(self, msg: PoseWithCovarianceStamped):
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        cov = msg.pose.covariance
        cxx, cyy, cxy = cov[0], cov[7], cov[1]
        trace = cxx + cyy
        det = cxx * cyy - cxy * cxy
        term = math.sqrt(max(0.0, trace**2 / 4.0 - det))
        self.amcl_l_max = trace / 2.0 + term
        self.amcl_l_min = max(0.0, trace / 2.0 - term)

    def on_collision_state(self, msg: CollisionMonitorState):
        self.last_collision_action = msg.action_type
        self.last_collision_poly = msg.polygon_name
        if msg.action_type == 1:  # STOP
            self.collision_stop_count += 1
        elif msg.action_type == 2:  # SLOWDOWN
            self.collision_slow_count += 1

    def on_autonomy_state(self, msg: String):
        self.supervisor_state = msg.data

    def get_tf_map_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            siny_cosp = 2.0 * (r.w * r.z + r.x * r.y)
            cosy_cosp = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return float(tr.x), float(tr.y), float(yaw)
        except Exception:
            return None

    def wait_for_telemetry(self, timeout_sec=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            tf_pose = self.get_tf_map_pose()
            if self.has_odom and self.has_gz and tf_pose is not None and self.cmd_pub.get_subscription_count() > 0:
                return True
        return False

    def reset_clean_state(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        """Resets robot pose in Gazebo, cancels active goals, resets coordinator, and clears costmaps."""
        zero_twist = Twist()
        for _ in range(5):
            self.teleop_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Cancel and Reset supervisor
        self.reset_supervisor()

        # Reset Gazebo pose via set_pose service
        try:
            req = GzPose()
            req.name = "gracemo_vira"
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = 0.04
            req.orientation.x = 0.0
            req.orientation.y = 0.0
            req.orientation.z = math.sin(yaw / 2.0)
            req.orientation.w = math.cos(yaw / 2.0)
            self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)
        except Exception as e:
            self.log(f"Warning: Gazebo set_pose failed: {e}")

        for _ in range(5):
            self.teleop_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Re-initialize AMCL Localization in Nav2 to match teleported pose
        loc_cmd = {
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": float(x), "y": float(y), "yaw": float(yaw)}
        }
        loc_msg = String()
        loc_msg.data = json.dumps(loc_cmd)
        self.cmd_pub.publish(loc_msg)
        time.sleep(0.5)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Trigger nomotion updates to converge AMCL particles
        for _ in range(5):
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            err = math.hypot(self.amcl_x - float(x), self.amcl_y - float(y))
            if err < 0.25 and self.amcl_l_min < 0.50:
                break

        # Clear Costmaps
        cmd = {"intent": "CLEAR_COSTMAPS"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        # Settle physics, telemetry, and ensure full convergence
        t_wait = time.time() + 2.5
        while time.time() < t_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            err = math.hypot(self.amcl_x - float(x), self.amcl_y - float(y))
            if err < 0.20 and self.amcl_l_min < 0.50 and (time.time() > t_wait - 0.5):
                break

    def reset_supervisor(self):
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL"})
        self.cmd_pub.publish(msg)
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        msg.data = json.dumps({"intent": "RESET"})
        self.cmd_pub.publish(msg)
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

    # -------------------------------------------------------------------------
    # Pre-Test Validity Gate (Item A)
    # -------------------------------------------------------------------------
    def check_validity_gate(self, target_x: float = 0.0, target_y: float = 0.0, target_yaw: float = 0.0, max_retries: int = 3) -> Tuple[bool, Dict[str, Any]]:
        """
        Pre-test validity gate: require |TF map pose - Gazebo pose| < 0.15 m
        and yaw error < 0.1 rad.
        If not satisfied, log error, re-reset and retry up to 3 times.
        Returns (is_valid, gate_meta).
        """
        last_pos_err = 999.0
        last_yaw_err = 999.0

        for attempt in range(1, max_retries + 1):
            self.reset_clean_state(target_x, target_y, target_yaw)
            time.sleep(1.0)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

            tf_pose = self.get_tf_map_pose()
            if tf_pose is None:
                self.log(f"  [Validity Gate] Attempt {attempt}/{max_retries}: TF map->base_footprint lookup failed.")
                continue

            pos_err = math.hypot(tf_pose[0] - self.gz_x, tf_pose[1] - self.gz_y)
            yaw_err = abs(wrap_pi(tf_pose[2] - self.gz_yaw))
            last_pos_err = pos_err
            last_yaw_err = yaw_err

            if pos_err < 0.15 and yaw_err < 0.10:
                self.log(f"  [Validity Gate] PASSED on attempt {attempt}: Pos err={pos_err:.4f}m (<0.15m), Yaw err={yaw_err:.4f}rad (<0.10rad)")
                return True, {"attempts": attempt, "pos_err": pos_err, "yaw_err": yaw_err}
            else:
                self.log(f"  [Validity Gate] FAILED on attempt {attempt}/{max_retries}: Pos err={pos_err:.4f}m (limit 0.15m), Yaw err={yaw_err:.4f}rad (limit 0.10rad). Retrying clean reset...")

        self.log(f"  [Validity Gate] EXHAUSTED after {max_retries} attempts. Marking test as INVALID (localization fault).")
        return False, {"attempts": max_retries, "pos_err": last_pos_err, "yaw_err": last_yaw_err}

    # -------------------------------------------------------------------------
    # Acceptance Tests AT-01 through AT-07
    # -------------------------------------------------------------------------

    def run_at01_stationary_self_observation(self):
        """AT-01: Stationary robot self-observation test."""
        self.log("\n--- RUNNING AT-01: Stationary Self-Observation Test ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-01 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        self.collision_stop_count = 0
        self.collision_slow_count = 0

        # Observe for 4 seconds while stationary
        t_end = time.time() + 4.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.log(f"Stationary stop count: {self.collision_stop_count}, slow count: {self.collision_slow_count}")
        passed = (self.collision_stop_count == 0) and (self.supervisor_state == "AUTONOMY_READY")
        status = "PASS" if passed else "FAIL"
        self.log(f"AT-01 Result: [{status}] (Self-observation did not trigger PolygonStop)")
        return status, {"stop_count": self.collision_stop_count, "slow_count": self.collision_slow_count}

    def run_at02_kitchen_navigation(self, max_wait_sec=90.0):
        """AT-02: Single navigation goal to Kitchen & Dining with dual ground-truth verification."""
        self.log("\n--- RUNNING AT-02: Single Navigation Goal to Kitchen & Dining ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-02 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        start_tf = self.get_tf_map_pose() or (0.0, 0.0, 0.0)
        start_gz = (self.gz_x, self.gz_y, self.gz_yaw)
        target_x, target_y = -5.0, -2.5

        # Dispatch command
        cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.log(f"Dispatched mission to Kitchen ({target_x}, {target_y}) from TF={start_tf[:2]}, GZ={start_gz[:2]}...")

        t0 = time.time()
        reached_signal = False
        while time.time() - t0 < max_wait_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            cur_tf = self.get_tf_map_pose() or (self.odom_x, self.odom_y, self.odom_yaw)
            dist_tf = math.hypot(cur_tf[0] - target_x, cur_tf[1] - target_y)
            dist_gz = math.hypot(self.gz_x - target_x, self.gz_y - target_y)

            if self.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED"):
                reached_signal = True
                self.log(f"Supervisor signalled reached! TF={cur_tf[:2]} (err={dist_tf:.2f}m), GZ=({self.gz_x:.2f},{self.gz_y:.2f}) (err={dist_gz:.2f}m) in {time.time()-t0:.1f}s")
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                self.log(f"Supervisor entered NAVIGATION_FAILURE at TF={cur_tf[:2]}, GZ=({self.gz_x:.2f},{self.gz_y:.2f})")
                break

        # Settle
        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_tf = self.get_tf_map_pose() or (self.odom_x, self.odom_y, self.odom_yaw)
        end_gz = (self.gz_x, self.gz_y, self.gz_yaw)

        net_disp_tf = math.hypot(end_tf[0] - start_tf[0], end_tf[1] - start_tf[1])
        net_disp_gz = math.hypot(end_gz[0] - start_gz[0], end_gz[1] - start_gz[1])
        final_err_tf = math.hypot(end_tf[0] - target_x, end_tf[1] - target_y)
        final_err_gz = math.hypot(end_gz[0] - target_x, end_gz[1] - target_y)
        drift = math.hypot(end_tf[0] - end_gz[0], end_tf[1] - end_gz[1])

        # Strict dual-source verification
        passed = (
            reached_signal
            and (final_err_tf <= 0.65)
            and (final_err_gz <= 0.65)
            and (drift <= 0.50)
            and (self.supervisor_state != "NAVIGATION_FAILURE")
        )

        status = "PASS" if passed else "FAIL"
        self.log(f"AT-02 Result: [{status}] (TF error={final_err_tf:.2f}m, GZ error={final_err_gz:.2f}m, Drift={drift:.2f}m, Disp={net_disp_gz:.2f}m)")
        return status, {
            "final_err_tf": final_err_tf,
            "final_err_gz": final_err_gz,
            "drift": drift,
            "net_disp": net_disp_gz,
            "state": self.supervisor_state,
        }

    def run_at03_multi_waypoint_circuit(self, max_wait_sec=90.0):
        """AT-03: Multi-waypoint navigation circuit. Must complete all waypoints without failure."""
        self.log("\n--- RUNNING AT-03: Multi-Waypoint Circuit Navigation ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-03 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        start_tf = self.get_tf_map_pose() or (0.0, 0.0, 0.0)
        start_gz = (self.gz_x, self.gz_y, self.gz_yaw)

        # Multi-waypoint patrol: hallway -> hallway_east -> hallway_home
        cmd = {
            "intent": "START_MISSION",
            "mission_name": "Patrol Circuit",
            "waypoints": [
                {"name": "hallway_center", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0},
                {"name": "hallway_east", "x": 2.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0},
                {"name": "hallway_home", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 0.0}
            ]
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.log("Dispatched 3-waypoint patrol circuit...")

        t0 = time.time()
        max_disp_gz = 0.0
        circuit_completed = False

        while time.time() - t0 < max_wait_sec:
            rclpy.spin_once(self, timeout_sec=0.2)
            disp = math.hypot(self.gz_x - start_gz[0], self.gz_y - start_gz[1])
            max_disp_gz = max(max_disp_gz, disp)

            if self.supervisor_state == "MISSION_COMPLETED":
                circuit_completed = True
                self.log(f"Circuit completed successfully in {time.time()-t0:.1f}s!")
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                self.log(f"Circuit aborted: supervisor entered NAVIGATION_FAILURE at {time.time()-t0:.1f}s")
                break

        time.sleep(1.0)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_tf = self.get_tf_map_pose() or (self.odom_x, self.odom_y, self.odom_yaw)
        final_err_home_tf = math.hypot(end_tf[0] - 0.0, end_tf[1] - 0.0)
        final_err_home_gz = math.hypot(self.gz_x - 0.0, self.gz_y - 0.0)

        passed = (
            circuit_completed
            and (self.supervisor_state == "MISSION_COMPLETED")
            and (final_err_home_tf <= 0.65)
            and (final_err_home_gz <= 0.65)
            and (max_disp_gz >= 1.5)
            and (self.supervisor_state != "NAVIGATION_FAILURE")
        )

        status = "PASS" if passed else "FAIL"
        self.log(f"AT-03 Result: [{status}] (Max displacement: {max_disp_gz:.2f}m, Final state: {self.supervisor_state}, Home err: {final_err_home_gz:.2f}m)")
        return status, {"max_disp": max_disp_gz, "state": self.supervisor_state, "home_err": final_err_home_gz}

    def run_at04_emergency_stop_test(self):
        """AT-04: Emergency stop safety gate test (Hardened: assert robot was moving before trigger)."""
        self.log("\n--- RUNNING AT-04: Emergency Stop Safety Gate Test ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-04 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        # 1. Dispatch motion goal
        cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.log("Dispatched motion goal. Waiting for robot to start moving...")

        # 2. Assert robot was moving before the trigger (Item E)
        t_move_wait = time.time() + 8.0
        robot_was_moving = False
        initial_speed = 0.0
        while time.time() < t_move_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            spd = math.hypot(self.current_vx, self.current_wz)
            if spd > 0.05:
                robot_was_moving = True
                initial_speed = spd
                self.log(f"Robot moving confirmed! Speed={spd:.3f} m/s (vx={self.current_vx:.3f}, wz={self.current_wz:.3f})")
                break

        if not robot_was_moving:
            self.log("WARNING: Robot did not achieve motion before E-stop trigger timeout.")

        # 3. Assert emergency stop immediately while robot is moving
        e_msg = Bool()
        e_msg.data = True
        self.estop_pub.publish(e_msg)
        self.log("Asserted /terrain/emergency_stop = True. Measuring deceleration to zero...")

        # 4. Check velocity halts and supervisor enters EMERGENCY_STOP
        t_end = time.time() + 2.5
        halted_speeds = []
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            halted_speeds.append(math.hypot(self.current_vx, self.current_wz))

        final_speed = halted_speeds[-1] if halted_speeds else 0.0
        estop_state_ok = (self.supervisor_state == "EMERGENCY_STOP")

        # 5. Clear estop & reset
        e_msg.data = False
        self.estop_pub.publish(e_msg)
        time.sleep(0.5)
        self.reset_supervisor()

        passed = robot_was_moving and (final_speed < 0.05) and estop_state_ok
        status = "PASS" if passed else "FAIL"
        self.log(f"AT-04 Result: [{status}] (Moving before trigger: {robot_was_moving} [{initial_speed:.3f}m/s], Final speed: {final_speed:.3f}m/s, State: {self.supervisor_state})")
        return status, {"was_moving": robot_was_moving, "initial_spd": initial_speed, "final_spd": final_speed, "state": self.supervisor_state}

    def run_at05_obstacle_recovery_and_replanning(self):
        """AT-05: Obstacle recovery and replanning test (Hardened: assert moving before trigger & observe replan)."""
        self.log("\n--- RUNNING AT-05: Obstacle Recovery and Replanning Test ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-05 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        # 1. Dispatch initial navigation goal
        cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.log("Dispatched goal to Kitchen. Waiting for active locomotion...")

        # 2. Assert robot was moving before the trigger (Item E)
        t_move_wait = time.time() + 8.0
        robot_was_moving = False
        while time.time() < t_move_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if math.hypot(self.current_vx, self.current_wz) > 0.05:
                robot_was_moving = True
                self.log(f"Robot moving confirmed at speed {math.hypot(self.current_vx, self.current_wz):.3f} m/s!")
                break

        # 3. Trigger replanning while moving: Send CLEAR_COSTMAPS and redirect to bedroom
        self.log("Triggering dynamic replan (costmap clear + alternative waypoint redirection)...")
        clear_cmd = {"intent": "CLEAR_COSTMAPS"}
        msg.data = json.dumps(clear_cmd)
        self.cmd_pub.publish(msg)

        replan_cmd = {"intent": "NAVIGATE_TO", "destination": "bedroom"}
        msg.data = json.dumps(replan_cmd)
        self.cmd_pub.publish(msg)

        # 4. Observe replan transition (Item E)
        t_replan_wait = time.time() + 8.0
        replan_observed = False
        resumed_motion = False

        while time.time() < t_replan_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state in ("MISSION_PLANNING", "PLANNING", "AUTONOMY_READY", "MISSION_ACCEPTED"):
                replan_observed = True
            if replan_observed and self.supervisor_state == "MISSION_EXECUTING":
                if math.hypot(self.current_vx, self.current_wz) > 0.03:
                    resumed_motion = True
                    break

        # Clean up
        self.reset_supervisor()

        passed = robot_was_moving and replan_observed
        status = "PASS" if passed else "FAIL"
        self.log(f"AT-05 Result: [{status}] (Moving before trigger: {robot_was_moving}, Replan observed: {replan_observed}, Resumed motion: {resumed_motion})")
        return status, {"was_moving": robot_was_moving, "replan_observed": replan_observed, "resumed_motion": resumed_motion}

    def run_at06_rotation_and_reverse(self):
        """AT-06: Robot rotation verification (Hardened: command 90 deg, require >= 80 deg yaw change vs Gazebo yaw)."""
        self.log("\n--- RUNNING AT-06: Robot Rotation Verification (90° Turn) ---")
        gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
        if not gate_ok:
            self.log("AT-06 Result: [INVALID] (Pre-test localization validity gate failed)")
            return "INVALID", gate_meta

        start_yaw_gz = self.gz_yaw
        target_turn_rad = math.pi / 2.0  # +90 degrees

        # Command in-place 90 degree rotation
        cmd = {
            "intent": "START_MISSION",
            "mission_name": "In-Place 90-Deg Turn",
            "waypoints": [
                {"name": "turn_90", "x": 0.0, "y": 0.0, "yaw": target_turn_rad, "dwell_sec": 1.0}
            ]
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)
        self.log(f"Dispatched 90° heading command (target_yaw={math.degrees(target_turn_rad):.1f}° from start_yaw={math.degrees(start_yaw_gz):.1f}°)...")

        max_angular_z = 0.0
        max_yaw_change = 0.0
        t0 = time.time()
        while time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            max_angular_z = max(max_angular_z, abs(self.current_wz))
            current_change = abs(wrap_pi(self.gz_yaw - start_yaw_gz))
            max_yaw_change = max(max_yaw_change, current_change)
            if max_yaw_change >= math.radians(80.0):
                self.log(f"Achieved >= 80° turn ({math.degrees(max_yaw_change):.1f}°) in {time.time()-t0:.1f}s!")
                break

        # Settle
        self.reset_supervisor()
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # Requirement (Item E): require >= 80 deg yaw change vs Gazebo yaw
        passed = (max_yaw_change >= math.radians(80.0))
        status = "PASS" if passed else "FAIL"
        self.log(f"AT-06 Result: [{status}] (Max yaw change vs Gazebo GT: {math.degrees(max_yaw_change):.1f}° [req >= 80.0°], Max wz: {max_angular_z:.3f} rad/s)")
        return status, {"yaw_change_deg": math.degrees(max_yaw_change), "max_wz": max_angular_z}

    def run_at07_destination_stop(self, prior_arrival_verified: bool = False):
        """AT-07: Destination zero-velocity halt test (Hardened: only after a verified arrival)."""
        self.log("\n--- RUNNING AT-07: Destination Zero-Velocity Halt Test ---")

        # If prior arrival (e.g. from AT-02) was not verified, perform a short transit to establish a verified arrival
        if not prior_arrival_verified:
            self.log("Prior arrival not present. Executing short transit to establish a verified arrival state...")
            gate_ok, gate_meta = self.check_validity_gate(0.0, 0.0, 0.0)
            if not gate_ok:
                self.log("AT-07 Result: [INVALID] (Pre-test localization validity gate failed)")
                return "INVALID", gate_meta

            cmd = {
                "intent": "START_MISSION",
                "mission_name": "Short Transit For Halt Verification",
                "waypoints": [
                    {"name": "waypoint_1m", "x": 1.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 2.0}
                ]
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self.cmd_pub.publish(msg)

            t0 = time.time()
            arrived = False
            while time.time() - t0 < 30.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED"):
                    arrived = True
                    break

            if not arrived:
                self.log("AT-07 Result: [FAIL] (Failed to achieve verified arrival before halt test)")
                return "FAIL", {"reason": "arrival_not_achieved"}

            # Allow controller to complete in-place settling to full halt
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        self.log("Verified arrival confirmed. Measuring zero creep during dwell period (3.0 seconds)...")
        # Measure halt stability over 3.0 seconds at the destination
        t_end = time.time() + 3.0
        max_vx = 0.0
        max_wz = 0.0
        start_gz = (self.gz_x, self.gz_y)

        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
            max_vx = max(max_vx, abs(self.current_vx))
            max_wz = max(max_wz, abs(self.current_wz))

        creep_disp = math.hypot(self.gz_x - start_gz[0], self.gz_y - start_gz[1])
        passed = (max_vx < 0.05) and (max_wz < 0.05) and (creep_disp < 0.05)
        status = "PASS" if passed else "FAIL"
        self.log(f"AT-07 Result: [{status}] (Max linear: {max_vx:.4f} m/s, Max angular: {max_wz:.4f} rad/s, Creep disp: {creep_disp:.4f} m)")
        return status, {"max_vx": max_vx, "max_wz": max_wz, "creep_disp": creep_disp}


def main():
    import argparse
    parser = argparse.ArgumentParser(description="GRaCEmo ViRa Hardened Navigation Acceptance Test Suite")
    parser.add_argument("--controller", type=str, default="DWB", choices=["DWB", "RL", "dwb", "rl"], help="Controller under test (DWB or RL)")
    parser.add_argument("--log-dir", type=str, default="/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch", help="Output directory for full test logs")
    parsed_args, unknown = parser.parse_known_args()
    ctrl_name = parsed_args.controller.upper()

    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(parsed_args.log_dir, f"acceptance_suite_{ctrl_name}_{timestamp_str}.log")

    rclpy.init()
    tester = NavigationAcceptanceTester(log_file=log_path)

    tester.log(f"\n============================================================")
    tester.log(f"   RUNNING HARDENED ACCEPTANCE TEST SUITE - CONTROLLER: {ctrl_name}")
    tester.log(f"   FULL LOG FILE: {log_path}")
    tester.log(f"============================================================")
    tester.log("Waiting for telemetry, TF, and supervisor connection...")

    if not tester.wait_for_telemetry(timeout_sec=15.0):
        tester.log("ERROR: Telemetry timeout. Is simulation running?")
        sys.exit(1)

    tf_init = tester.get_tf_map_pose()
    tester.log(f"Connected! Initial robot poses: TF_map={tf_init}, Gazebo=({tester.gz_x:.2f}, {tester.gz_y:.2f})")

    suite_results = {}
    invalid_counts = {}

    # AT-01
    status_01, meta_01 = tester.run_at01_stationary_self_observation()
    suite_results["AT-01"] = (status_01, meta_01)
    if status_01 == "INVALID":
        invalid_counts["AT-01"] = invalid_counts.get("AT-01", 0) + 1

    # AT-02
    status_02, meta_02 = tester.run_at02_kitchen_navigation()
    suite_results["AT-02"] = (status_02, meta_02)
    if status_02 == "INVALID":
        invalid_counts["AT-02"] = invalid_counts.get("AT-02", 0) + 1

    # AT-07 (Hardened: only after a verified arrival)
    status_07, meta_07 = tester.run_at07_destination_stop(prior_arrival_verified=(status_02 == "PASS"))
    suite_results["AT-07"] = (status_07, meta_07)
    if status_07 == "INVALID":
        invalid_counts["AT-07"] = invalid_counts.get("AT-07", 0) + 1

    # AT-03
    status_03, meta_03 = tester.run_at03_multi_waypoint_circuit()
    suite_results["AT-03"] = (status_03, meta_03)
    if status_03 == "INVALID":
        invalid_counts["AT-03"] = invalid_counts.get("AT-03", 0) + 1

    # AT-04 (Hardened: moving before trigger)
    status_04, meta_04 = tester.run_at04_emergency_stop_test()
    suite_results["AT-04"] = (status_04, meta_04)
    if status_04 == "INVALID":
        invalid_counts["AT-04"] = invalid_counts.get("AT-04", 0) + 1

    # AT-05 (Hardened: moving before trigger & observe replan)
    status_05, meta_05 = tester.run_at05_obstacle_recovery_and_replanning()
    suite_results["AT-05"] = (status_05, meta_05)
    if status_05 == "INVALID":
        invalid_counts["AT-05"] = invalid_counts.get("AT-05", 0) + 1

    # AT-06 (Hardened: 90 deg command, require >= 80 deg yaw change vs Gazebo)
    status_06, meta_06 = tester.run_at06_rotation_and_reverse()
    suite_results["AT-06"] = (status_06, meta_06)
    if status_06 == "INVALID":
        invalid_counts["AT-06"] = invalid_counts.get("AT-06", 0) + 1

    tester.log("\n============================================================")
    tester.log(f"      HARDENED ACCEPTANCE CAMPAIGN SUMMARY [{ctrl_name}]")
    tester.log("============================================================")
    total_passed = 0
    total_failed = 0
    total_invalid = 0

    for test_id, (status, meta) in suite_results.items():
        if status == "PASS":
            total_passed += 1
        elif status == "FAIL":
            total_failed += 1
        elif status == "INVALID":
            total_invalid += 1
        tester.log(f"  {test_id}: [{status}] - {meta}")

    tester.log("------------------------------------------------------------")
    tester.log(f"  PASSED : {total_passed}/7")
    tester.log(f"  FAILED : {total_failed}/7")
    tester.log(f"  INVALID: {total_invalid}/7 (Localization validity faults reported separately)")
    tester.log("============================================================")
    tester.log(f"FULL LOG SAVED TO: {log_path}")
    tester.log("============================================================\n")

    tester.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if total_passed == 7 else 1)


if __name__ == "__main__":
    main()
