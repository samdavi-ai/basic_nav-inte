#!/usr/bin/env python3
"""
GRaCEmo ViRa — Gate E 50 Consecutive Random Multi-Room Missions.
Executes 50 consecutive random multi-room missions (AMCL ON, static OFF, zero manual reset).
For each mission:
  - Randomly selects target room from authoritative places (kitchen, bedroom, home, study, living room).
  - Evaluates completion status.
  - Verifies reached vs Gazebo ground truth using the configured 0.65 m arrival tolerance.
  - Records collision-monitor observations and a heuristic false-blocked indicator.
  - Detects false reached and false blocked events.
Requires fresh ROS sensors, an advancing simulation clock, a fresh Gazebo ground-truth pose,
Nav2/coordinator readiness, and an inactive E-stop before sending any goal. It does not
randomize obstacle placement; results characterize repeated navigation in the fixed apartment.
Outputs full summary to scratch/gate_e_50_missions_results.json.
"""

import os
import sys
import time
import math
import json
import random
import signal
import uuid

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl"))
from gracemo_rl.common.places import load_authoritative_places

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class GateEMissionCampaign(Node):
    def __init__(self, target_missions: int = 50):
        super().__init__("gate_e_mission_campaign")
        self.target_missions = target_missions
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0

        self.supervisor_state = "UNKNOWN"
        self.loc_state = "UNKNOWN"
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_speed = 0.0
        self.clock_advances = 0
        self.last_clock_sec = None
        self.scan_count = 0
        self.imu_count = 0
        self.odom_count = 0
        self.measure_mission_path = False
        self.mission_path_length_m = 0.0
        self.previous_odom_xy = None
        self.current_campaign_mission_id = None
        self.latest_status = None
        self.terrain_status = None
        self.mux_status = None
        self.last_ground_truth_wall = 0.0
        self.collision_stop_count = 0
        self.collision_slow_count = 0

        self.sub_state = self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.sub_status = self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.sub_clock = self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.sub_imu = self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.sub_terrain = self.create_subscription(String, "/terrain/status", self._on_terrain_status, 10)
        self.sub_mux = self.create_subscription(String, "/cmd_vel_mux/status", self._on_mux_status, 10)
        self.sub_slow = self.create_subscription(Bool, "/collision_monitor/slowdown_active", self._on_slow, 10)
        self.sub_stop = self.create_subscription(Bool, "/collision_monitor/stop_active", self._on_stop, 10)

        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)
        self.places = load_authoritative_places()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gz_node = gz_transport.Node()
        # Gazebo Harmonic publishes moving model poses on dynamic_pose/info.
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_pose)

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_status = data
            self.loc_state = data.get("robot", {}).get("localization_state", "UNKNOWN")
        except Exception:
            pass

    def _on_clock(self, msg: Clock):
        now = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self.last_clock_sec is not None and now > self.last_clock_sec:
            self.clock_advances += 1
        self.last_clock_sec = now

    def _on_scan(self, _msg: LaserScan):
        self.scan_count += 1

    def _on_imu(self, _msg: Imu):
        self.imu_count += 1

    def _on_terrain_status(self, msg: String):
        try:
            self.terrain_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.terrain_status = None

    def _on_mux_status(self, msg: String):
        try:
            self.mux_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.mux_status = None

    def _on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.odom_count += 1
        current_xy = (self.robot_x, self.robot_y)
        if self.measure_mission_path and self.previous_odom_xy is not None:
            self.mission_path_length_m += math.hypot(
                current_xy[0] - self.previous_odom_xy[0],
                current_xy[1] - self.previous_odom_xy[1],
            )
        self.previous_odom_xy = current_xy

    def _on_slow(self, msg: Bool):
        if msg.data:
            self.collision_slow_count += 1

    def _on_stop(self, msg: Bool):
        if msg.data:
            self.collision_stop_count += 1

    def _on_gz_pose(self, msg):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                self.gz_x = p.position.x
                self.gz_y = p.position.y
                q = p.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.gz_yaw = math.atan2(siny_cosp, cosy_cosp)
                self.last_ground_truth_wall = time.monotonic()
                break

    def ready_for_campaign(self):
        status = self.latest_status or {}
        subsystems = status.get("subsystems", {})
        terrain = self.terrain_status or {}
        mux = self.mux_status or {}
        return (
            status.get("state") in ("AUTONOMY_READY", "WAITING_FOR_NEXT_MISSION")
            and subsystems.get("nav2_ready") is True
            and subsystems.get("scan_fresh") is True
            and subsystems.get("odom_fresh") is True
            and subsystems.get("estop_active") is False
            and terrain.get("imu_fresh") is True
            and terrain.get("simulation_mode") is True
            and self.clock_advances >= 10
            and self.scan_count >= 5
            and self.imu_count >= 5
            and self.odom_count >= 5
            and time.monotonic() - self.last_ground_truth_wall <= 2.0
            and self.pub_cmd.get_subscription_count() > 0
            and mux.get("active_source") == "NONE"
            and mux.get("zero_command_held") is True
            and abs(float(mux.get("current_linear_x", 1.0))) < 0.03
            and abs(float(mux.get("current_angular_z", 1.0))) < 0.03
        )

    def wait_for_campaign_ready(self, timeout_sec: float = 60.0) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ready_for_campaign():
                return True
        return False

    def wait_for_safe_idle(self, timeout_sec: float = 8.0) -> bool:
        """Require coordinator, mux, sensors, and terrain monitor to be healthy before continuing."""
        return self.wait_for_campaign_ready(timeout_sec=timeout_sec)

    def cancel_campaign_goal_and_wait(self, timeout_sec: float = 8.0) -> bool:
        """Cancel this runner's active goal before shutdown or campaign abort."""
        active_mission = (self.latest_status or {}).get("mission", {}).get("id")
        if not self.current_campaign_mission_id or active_mission != self.current_campaign_mission_id:
            return True
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL_MISSION"})
        self.pub_cmd.publish(msg)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            status = self.latest_status or {}
            active_mission = status.get("mission", {}).get("id")
            state = status.get("state")
            twist = status.get("robot", {}).get("twist", {})
            stopped = abs(float(twist.get("linear", 0.0))) < 0.03 and abs(float(twist.get("angular", 0.0))) < 0.03
            if active_mission is None and state == "AUTONOMY_READY" and stopped:
                self.current_campaign_mission_id = None
                return True
        return False

    def get_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = float(math.atan2(siny_cosp, cosy_cosp))
            return x, y, yaw
        except Exception:
            return None

    def execute_campaign(self, seed: int = 42) -> dict:
        self.get_logger().info(f"=== Starting Gate E Campaign: {self.target_missions} Random Multi-Room Missions ===")
        # Seed random for reproducibility
        random.seed(seed)
        run_id = uuid.uuid4().hex[:8]

        available_rooms = ["kitchen", "bedroom", "home", "study", "living_room"]
        
        results = []
        consecutive_passes = 0
        false_reached_count = 0
        false_blocked_count = 0
        campaign_abort_reason = None

        # Wait for ready and settle
        t0 = time.time()
        while time.time() - t0 < 60.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state in ("AUTONOMY_READY", "WAITING_FOR_NEXT_MISSION"):
                break
        time.sleep(3.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        current_location = min(
            available_rooms,
            key=lambda room: math.hypot(
                self.gz_x - self.places[room]["x"],
                self.gz_y - self.places[room]["y"],
            ),
        )

        for m_idx in range(1, self.target_missions + 1):
            # Select random target distinct from current
            choices = [r for r in available_rooms if r != current_location]
            target_room = random.choice(choices)
            target_data = self.places[target_room]
            tx = target_data["x"]
            ty = target_data["y"]
            tyaw = target_data.get("yaw", 0.0)

            self.get_logger().info(f"\n--- Mission {m_idx}/{self.target_missions}: {current_location} -> {target_room} ({tx:.2f}, {ty:.2f}) ---")

            mission_name = f"RandomMission_{m_idx}_{target_room}"
            mission_id = f"gate-e-{run_id}-{m_idx:03d}"
            self.current_campaign_mission_id = mission_id
            start_ground_truth = {"x": self.gz_x, "y": self.gz_y, "yaw": self.gz_yaw}
            start_tf_pose = self.get_tf_pose()
            collision_slow_before = self.collision_slow_count
            collision_stop_before = self.collision_stop_count
            self.mission_path_length_m = 0.0
            self.previous_odom_xy = (self.robot_x, self.robot_y)
            self.measure_mission_path = True
            mission_payload = {
                "intent": "START_MISSION",
                "mission_id": mission_id,
                "mission_name": mission_name,
                "waypoints": [
                    {"name": target_room, "x": tx, "y": ty, "yaw": tyaw, "dwell_sec": 1.0}
                ]
            }

            # Reset local state before dispatching to prevent reading stale previous-mission states
            self.supervisor_state = "DISPATCHED"
            msg = String(data=json.dumps(mission_payload))

            # Dispatch once. Repeated publishes can preempt/restart a mission if status is delayed.
            t_accept = time.time()
            accepted = False
            cancel_confirmed = None
            safe_to_continue_after_failure = None
            self.pub_cmd.publish(msg)
            while time.time() - t_accept < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
                status = self.latest_status or {}
                active_mission = status.get("mission", {}).get("id")
                if (
                    active_mission == mission_id
                    and status.get("state") in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING")
                ):
                    accepted = True
                    break

            if not accepted:
                self.get_logger().error(f"❌ Mission {m_idx} rejected or ignored by coordinator (state: {self.supervisor_state})")
                final_status = "DISPATCH_FAILED"
                fail_reason = f"Coordinator did not accept mission (state: {self.supervisor_state})"
                duration = time.time() - t_accept
                mission_done = False
            else:
                t_start = time.time()
                mission_done = False
                final_status = "UNKNOWN"
                fail_reason = None
                has_executed = False

                timeout_sec = 90.0

                while time.time() - t_start < timeout_sec:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    status = self.latest_status or {}
                    status_state = status.get("state", self.supervisor_state)
                    active_mission = status.get("mission", {}).get("id")
                    history_entry = next(
                        (
                            item for item in status.get("history", [])
                            if item.get("mission_id") == mission_id
                        ),
                        None,
                    )
                    if active_mission == mission_id and status_state in ("MISSION_EXECUTING", "DESTINATION_REACHED"):
                        has_executed = True

                    if history_entry is not None:
                        mission_done = bool(history_entry.get("success"))
                        final_status = history_entry.get("status", "COMPLETED" if mission_done else "FAILED")
                        fail_reason = history_entry.get("reason") or (None if mission_done else final_status)
                        break
                    elif has_executed and active_mission is None and status_state in ("WAITING_FOR_NEXT_MISSION", "AUTONOMY_READY"):
                        mission_done = True
                        final_status = "COMPLETED"
                        break
                    elif status_state in ("NAVIGATION_FAILURE", "LOCALIZATION_LOST", "SENSOR_FAILURE"):
                        final_status = status_state
                        fail_reason = status_state
                        break

                if not mission_done and final_status == "UNKNOWN":
                    final_status = "TIMEOUT"
                    fail_reason = f"Exceeded {timeout_sec}s timeout"
                    cancel_confirmed = self.cancel_campaign_goal_and_wait()

                duration = time.time() - t_start
            
            # Finish a final telemetry sample before freezing per-mission counters.
            settle_start = time.monotonic()
            while time.monotonic() - settle_start < 0.3:
                rclpy.spin_once(self, timeout_sec=0.05)
            self.measure_mission_path = False

            # Ground truth verification vs Gazebo
            gz_dist_to_goal = math.hypot(self.gz_x - tx, self.gz_y - ty)
            tf_pose = self.get_tf_pose()
            tf_dist_to_goal = math.hypot(tf_pose[0] - tx, tf_pose[1] - ty) if tf_pose else 999.0
            drift = math.hypot(self.gz_x - tf_pose[0], self.gz_y - tf_pose[1]) if tf_pose else 999.0

            # Arrival tolerance is 0.65m (matching coordinator and AT-02/03/07 standards)
            is_gz_at_goal = (gz_dist_to_goal <= 0.65)
            
            is_false_reached = False
            is_false_blocked = False

            if final_status == "COMPLETED":
                if not is_gz_at_goal:
                    # Coordinator claimed completed, but Gazebo is not within goal tolerance!
                    is_false_reached = True
                    false_reached_count += 1
                    self.get_logger().error(f"⚠️ FALSE REACHED! TF claimed reached, but GZ dist is {gz_dist_to_goal:.3f}m (>0.65m)")
                else:
                    consecutive_passes += 1
                    current_location = target_room
                    self.get_logger().info(f"✅ Mission {m_idx} SUCCESS! GZ err={gz_dist_to_goal:.3f}m, Drift={drift:.3f}m ({duration:.1f}s)")
            else:
                # Mission failed or timed out: check if falsely blocked
                # A false block occurs if robot aborted when path and surroundings were completely clear
                mission_stop_count = self.collision_stop_count - collision_stop_before
                if mission_stop_count == 0 and drift < 0.20 and gz_dist_to_goal > 0.50:
                    is_false_blocked = True
                    false_blocked_count += 1
                self.get_logger().warn(f"❌ Mission {m_idx} FAILED ({final_status}, {fail_reason}). GZ dist={gz_dist_to_goal:.3f}m, Drift={drift:.3f}m")

            results.append({
                "mission_id": m_idx,
                "coordinator_mission_id": mission_id,
                "target_room": target_room,
                "target_coords": [tx, ty],
                "start_ground_truth": start_ground_truth,
                "start_map_pose": {
                    "x": round(start_tf_pose[0], 3),
                    "y": round(start_tf_pose[1], 3),
                    "yaw": round(start_tf_pose[2], 3),
                } if start_tf_pose else None,
                "end_ground_truth": {"x": round(self.gz_x, 3), "y": round(self.gz_y, 3), "yaw": round(self.gz_yaw, 3)},
                "status": final_status,
                "success": (final_status == "COMPLETED" and not is_false_reached),
                "duration_sec": round(duration, 2),
                "odom_path_length_m": round(self.mission_path_length_m, 3),
                "gz_dist_to_goal_m": round(gz_dist_to_goal, 3),
                "tf_dist_to_goal_m": round(tf_dist_to_goal, 3),
                "map_vs_gz_drift_m": round(drift, 3),
                "collision_slowdown_events": self.collision_slow_count - collision_slow_before,
                "collision_stop_events": self.collision_stop_count - collision_stop_before,
                "cancel_confirmed": cancel_confirmed,
                "safe_to_continue_after_failure": safe_to_continue_after_failure,
                "false_reached": is_false_reached,
                "false_blocked": is_false_blocked,
                "loc_state": self.loc_state,
                "fail_reason": fail_reason
            })

            active_after_mission = (self.latest_status or {}).get("mission", {}).get("id")
            if active_after_mission != mission_id:
                self.current_campaign_mission_id = None
            if not results[-1]["success"]:
                # A false success or failed dispatch is never safe to retry automatically.
                if is_false_reached or final_status == "DISPATCH_FAILED":
                    campaign_abort_reason = f"mission {m_idx} ended as {final_status}; stopping on an untrusted mission outcome"
                    self.get_logger().error(f"Campaign stopped: {campaign_abort_reason}")
                    break

                # A failed/timeout mission may be followed by another only after confirmed
                # cancellation and a fresh ready + zero-output state from the command mux.
                if final_status == "TIMEOUT" and cancel_confirmed is True:
                    safe_to_continue_after_failure = self.wait_for_safe_idle(timeout_sec=8.0)
                else:
                    safe_to_continue_after_failure = self.wait_for_safe_idle(timeout_sec=1.0)

                results[-1]["safe_to_continue_after_failure"] = safe_to_continue_after_failure
                if not safe_to_continue_after_failure:
                    campaign_abort_reason = f"mission {m_idx} ended as {final_status}; safe idle/readiness could not be re-established"
                    self.get_logger().error(f"Campaign stopped: {campaign_abort_reason}")
                    break

                current_location = min(
                    available_rooms,
                    key=lambda room: math.hypot(
                        self.gz_x - self.places[room]["x"],
                        self.gz_y - self.places[room]["y"],
                    ),
                )
                self.get_logger().warn(
                    f"Recorded mission {m_idx} as failed, confirmed zero-output readiness, and continuing from {current_location}."
                )
            else:
                current_location = target_room

            # Brief settle between missions (continuous AMCL, no manual reset)
            t_settle = time.time()
            while time.time() - t_settle < 2.5:
                rclpy.spin_once(self, timeout_sec=0.1)

        completed_count = sum(1 for r in results if r["success"])
        completion_rate = (completed_count / self.target_missions) * 100.0

        # Breakdown failures by reason
        failure_breakdown = {}
        for r in results:
            if not r["success"]:
                reason = r.get("fail_reason") or r["status"]
                failure_breakdown[reason] = failure_breakdown.get(reason, 0) + 1

        summary = {
            "campaign": "Gate E: 50 Consecutive Random Multi-Room Missions",
            "run_id": run_id,
            "seed": seed,
            "scenario": "fixed apartment map with static obstacles; obstacle placement was not randomized",
            "amcl_enabled": True,
            "static_localization": False,
            "manual_resets_performed": 0,
            "target_missions": self.target_missions,
            "attempted_missions": len(results),
            "completed": completed_count,
            "all_missions_successful": completed_count == len(results) == self.target_missions,
            "campaign_status": "COMPLETED" if len(results) == self.target_missions else "ABORTED",
            "completion_rate_pct": round(completion_rate, 2),
            "false_reached_count": false_reached_count,
            "false_blocked_count": false_blocked_count,
            "failures_by_reason": failure_breakdown,
            "campaign_abort_reason": campaign_abort_reason,
            "mission_records": results
        }
        return summary

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Gate E: 50 Consecutive Random Multi-Room Missions")
    parser.add_argument("--missions", type=int, default=50, help="Number of missions (default: 50)")
    parser.add_argument("--seed", type=int, default=42, help="Deterministic target ordering seed (default: 42)")
    parser.add_argument("--out", type=str, default="scratch/gate_e_50_missions_results.json", help="Output file path")
    args = parser.parse_args()

    rclpy.init()
    # Keep the ROS context alive long enough for the finally block to cancel an active goal.
    def interrupt_campaign(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt_campaign)
    node = GateEMissionCampaign(target_missions=args.missions)
    try:
        if not node.wait_for_campaign_ready():
            status = node.latest_status or {}
            print(json.dumps({
                "campaign": "Gate E: Consecutive Random Multi-Room Missions",
                "result": "NOT_RUN",
                "reason": "Gazebo clock, fresh sensors, ground truth, Nav2/coordinator readiness, or E-stop gate did not pass",
                "clock_advances": node.clock_advances,
                "scan_messages": node.scan_count,
                "imu_messages": node.imu_count,
                "odom_messages": node.odom_count,
                "coordinator_state": status.get("state"),
                "terrain_status": node.terrain_status,
                "ground_truth_age_sec": round(time.monotonic() - node.last_ground_truth_wall, 3)
                if node.last_ground_truth_wall else None,
            }, sort_keys=True), flush=True)
            return 2
        summary = node.execute_campaign(seed=args.seed)
        out_file = os.path.join(repo_root, args.out)
        os.makedirs(os.path.dirname(out_file), exist_ok=True)
        with open(out_file, "w") as f:
            json.dump(summary, f, indent=2)
        print("\n=======================================================")
        print("  GATE E 50-MISSION CAMPAIGN SUMMARY")
        print("=======================================================")
        print(f"Total Missions:       {summary['target_missions']}")
        print(f"Completed:            {summary['completed']}/{summary['target_missions']} ({summary['completion_rate_pct']}%)")
        print(f"False Reached vs GZ:  {summary['false_reached_count']}")
        print(f"False Blocked vs GZ:  {summary['false_blocked_count']}")
        print(f"Failures by Reason:   {summary['failures_by_reason']}")
        print(f"Results File:         {out_file}")
        print("=======================================================")
        return 0 if summary["attempted_missions"] == summary["target_missions"] else 1
    except KeyboardInterrupt:
        print("Campaign interrupted; requesting a safe cancellation before exit.", flush=True)
        return 130
    finally:
        if rclpy.ok():
            node.cancel_campaign_goal_and_wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    raise SystemExit(main())

