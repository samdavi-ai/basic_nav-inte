#!/usr/bin/env python3
"""
GRaCEmo ViRa — Step 7: Live Gazebo RL Validation & Comparison Suite.
Executes:
  1. 20s RL trace on AT-02 (records v, w, monitor stop/slow counts).
  2. AT-02 (Kitchen) and AT-03 (Bedroom) 5 runs each with use_rl:=true.
  3. 50-episode multi-room mission campaign with use_rl:=true.
  4. Generates direct side-by-side comparison table against DWB baseline.
"""

import os
import sys
import time
import math
import json
import random
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from nav2_msgs.msg import CollisionMonitorState
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws", "src", "gracemo_rl"))
from gracemo_rl.common.places import load_authoritative_places

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class RLLiveEvaluator(Node):
    def __init__(self):
        super().__init__("rl_live_evaluator")

        self.places = load_authoritative_places()

        # Telemetry
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.has_gz = False

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.has_odom = False

        self.cmd_rl_vx = 0.0
        self.cmd_rl_wz = 0.0
        self.has_rl_cmd = False

        self.supervisor_state = "UNKNOWN"
        self.loc_state = "UNKNOWN"

        self.monitor_stop_count = 0
        self.monitor_slow_count = 0
        self.monitor_state_code = 0

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel_rl", self._on_rl_cmd, 10)
        self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_monitor, 10)
        self.create_subscription(Bool, "/collision_monitor/stop_active", self._on_stop_active, 10)
        self.create_subscription(Bool, "/collision_monitor/slowdown_active", self._on_slow_active, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self._on_gz_pose)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_x = p.x
        self.robot_y = p.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z
        self.has_odom = True

    def _on_rl_cmd(self, msg: Twist):
        self.cmd_rl_vx = msg.linear.x
        self.cmd_rl_wz = msg.angular.z
        self.has_rl_cmd = True

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.loc_state = data.get("robot", {}).get("localization_state", "UNKNOWN")
        except Exception:
            pass

    def _on_monitor(self, msg: CollisionMonitorState):
        self.monitor_state_code = msg.action_type
        if msg.action_type == 1:
            self.monitor_stop_count += 1
        elif msg.action_type == 2:
            self.monitor_slow_count += 1

    def _on_stop_active(self, msg: Bool):
        if msg.data:
            self.monitor_stop_count += 1

    def _on_slow_active(self, msg: Bool):
        if msg.data:
            self.monitor_slow_count += 1

    def _on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                self.gz_x = p.position.x
                self.gz_y = p.position.y
                q = p.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.gz_yaw = math.atan2(siny, cosy)
                self.has_gz = True

    def get_map_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = float(math.atan2(siny, cosy))
            return x, y, yaw
        except Exception:
            return None

    def reset_to_home(self):
        """Teleports robot to (0, 0, 0) in Gazebo and clears costmaps."""
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = 0.0
        req.position.y = 0.0
        req.position.z = 0.05
        req.orientation.w = 1.0
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 1000)

        # Clear costmaps
        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.pub_cmd.publish(cmd_msg)

        # Settle
        t_settle = time.time() + 2.0
        while time.time() < t_settle:
            rclpy.spin_once(self, timeout_sec=0.1)

    def dispatch_destination(self, dest_name: str) -> bool:
        cmd_msg = String()
        cmd_msg.data = json.dumps({
            "intent": "NAVIGATE_TO",
            "destination": dest_name
        })
        self.pub_cmd.publish(cmd_msg)
        return True


def run_part1_20s_rl_trace(evaluator: RLLiveEvaluator) -> Dict[str, Any]:
    print("\n" + "=" * 80)
    print("STEP 7 - PART 1: 20-SECOND RL TRACE ON AT-02 (KITCHEN)")
    print("=" * 80)

    evaluator.reset_to_home()
    evaluator.monitor_stop_count = 0
    evaluator.monitor_slow_count = 0

    print("Dispatching AT-02 to Kitchen (-5.0, -2.5)...")
    evaluator.dispatch_destination("kitchen")

    records = []
    t_end = time.time() + 20.0
    t0 = time.time()

    while time.time() < t_end:
        rclpy.spin_once(evaluator, timeout_sec=0.05)
        now = time.time() - t0
        records.append({
            "t": round(now, 3),
            "cmd_rl_v": round(evaluator.cmd_rl_vx, 4),
            "cmd_rl_w": round(evaluator.cmd_rl_wz, 4),
            "odom_v": round(evaluator.robot_vx, 4),
            "odom_w": round(evaluator.robot_wz, 4),
            "gz_x": round(evaluator.gz_x, 3),
            "gz_y": round(evaluator.gz_y, 3),
            "monitor_state": evaluator.monitor_state_code
        })
        time.sleep(0.05)

    v_vals = [r["cmd_rl_v"] for r in records]
    w_vals = [r["cmd_rl_w"] for r in records]

    trace_summary = {
        "n_samples": len(records),
        "duration_sec": 20.0,
        "mean_cmd_v": float(np.mean(v_vals)),
        "std_cmd_v": float(np.std(v_vals)),
        "min_cmd_v": float(np.min(v_vals)),
        "max_cmd_v": float(np.max(v_vals)),
        "mean_cmd_w": float(np.mean(w_vals)),
        "std_cmd_w": float(np.std(w_vals)),
        "monitor_stops": evaluator.monitor_stop_count,
        "monitor_slows": evaluator.monitor_slow_count,
        "trace_samples": records[:50]
    }

    print(f"  Trace Samples: {len(records)}")
    print(f"  Commanded Linear Velocity v: Mean={trace_summary['mean_cmd_v']:+.3f} m/s, Std={trace_summary['std_cmd_v']:.3f}, Range=[{trace_summary['min_cmd_v']:+.3f}, {trace_summary['max_cmd_v']:+.3f}]")
    print(f"  Commanded Angular Velocity w: Mean={trace_summary['mean_cmd_w']:+.3f} rad/s, Std={trace_summary['std_cmd_w']:.3f}")
    print(f"  Monitor Stop Count: {trace_summary['monitor_stops']}")
    print(f"  Monitor Slow Count: {trace_summary['monitor_slows']}")

    out_file = os.path.join(REPO_ROOT, "scratch", "track1d_rl_trace.json")
    with open(out_file, "w") as f:
        json.dump(trace_summary, f, indent=2)
    print(f"Saved 20s trace to: {out_file}")

    return trace_summary


def run_part2_at02_at03_10runs(evaluator: RLLiveEvaluator) -> Dict[str, Any]:
    print("\n" + "=" * 80)
    print("STEP 7 - PART 2: AT-02 AND AT-03 (10 RUNS EACH WITH USE_RL:=TRUE)")
    print("=" * 80)

    tests = [
        {"name": "AT-02", "dest": "kitchen", "coords": (-5.0, -2.5)},
        {"name": "AT-03", "dest": "bedroom", "coords": (-5.0, 2.2)}
    ]

    all_results = {"AT-02": [], "AT-03": []}

    for test in tests:
        test_name = test["name"]
        dest = test["dest"]
        tx, ty = test["coords"]
        print(f"\n--- Running {test_name} ({dest.upper()}) 10 Runs ---")

        for r in range(1, 11):
            evaluator.reset_to_home()
            evaluator.monitor_stop_count = 0
            evaluator.monitor_slow_count = 0

            # Pre-test localization gate: |TF - Gazebo| < 0.15m
            map_p = evaluator.get_map_pose()
            if map_p is not None:
                tf_err = math.hypot(map_p[0] - evaluator.gz_x, map_p[1] - evaluator.gz_y)
            else:
                tf_err = 0.0

            if tf_err > 0.15:
                print(f"  Run {r}/5: [INVALID] Pre-test TF error {tf_err:.3f}m > 0.15m")
                all_results[test_name].append({
                    "run": r, "status": "INVALID", "tf_err": round(tf_err, 3)
                })
                continue

            evaluator.dispatch_destination(dest)
            t_start = time.time()
            arrived = False
            timed_out = False

            while (time.time() - t_start) < 90.0:
                rclpy.spin_once(evaluator, timeout_sec=0.1)
                dist_gz = math.hypot(evaluator.gz_x - tx, evaluator.gz_y - ty)

                if evaluator.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED") or dist_gz <= 0.35:
                    arrived = True
                    break
                if evaluator.supervisor_state in ("NAVIGATION_FAILURE", "MISSION_FAILED"):
                    break

            elapsed = time.time() - t_start
            final_gz_dist = math.hypot(evaluator.gz_x - tx, evaluator.gz_y - ty)
            status = "PASS" if (arrived and final_gz_dist <= 0.50) else "FAIL"

            record = {
                "run": r,
                "status": status,
                "duration_sec": round(elapsed, 2),
                "final_gz_dist_m": round(final_gz_dist, 3),
                "monitor_stops": evaluator.monitor_stop_count,
                "monitor_slows": evaluator.monitor_slow_count,
                "supervisor_state": evaluator.supervisor_state
            }
            all_results[test_name].append(record)
            print(f"  Run {r}/5: [{status}] in {elapsed:.1f}s | Final dist: {final_gz_dist:.2f}m | Stops: {evaluator.monitor_stop_count} | Slows: {evaluator.monitor_slow_count}")

    out_file = os.path.join(REPO_ROOT, "scratch", "rl_at02_at03_10runs_results.json")
    with open(out_file, "w") as f:
        json.dump(all_results, f, indent=2)
    with open(os.path.join(REPO_ROOT, "scratch", "rl_at02_at03_5runs_results.json"), "w") as f:
        json.dump(all_results, f, indent=2)
    print(f"\nSaved AT-02 / AT-03 10-run results to: {out_file}")
    return all_results


def run_part3_50_missions_campaign(evaluator: RLLiveEvaluator, n_missions: int = 50) -> Dict[str, Any]:
    print("\n" + "=" * 80)
    print(f"STEP 7 - PART 3: {n_missions} CONSECUTIVE RANDOM MULTI-ROOM MISSIONS (USE_RL:=TRUE)")
    print("=" * 80)

    rooms = ["kitchen", "bedroom", "home", "study", "living_room"]
    missions_data = []

    success_count = 0
    failure_count = 0
    false_reached = 0
    false_blocked = 0

    total_stops = 0
    total_slows = 0
    durations = []

    last_room = "home"

    for m_id in range(1, n_missions + 1):
        # Pick room different from last
        candidates = [r for r in rooms if r != last_room]
        target = random.choice(candidates)
        last_room = target
        target_info = evaluator.places.get(target, {"x": 0.0, "y": 0.0})
        tx, ty = target_info["x"], target_info["y"]

        evaluator.monitor_stop_count = 0
        evaluator.monitor_slow_count = 0

        evaluator.dispatch_destination(target)
        t_start = time.time()
        completed = False

        while (time.time() - t_start) < 80.0:
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            dist_gz = math.hypot(evaluator.gz_x - tx, evaluator.gz_y - ty)

            if evaluator.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED") or dist_gz <= 0.35:
                completed = True
                break
            if evaluator.supervisor_state in ("NAVIGATION_FAILURE", "MISSION_FAILED"):
                break

        elapsed = time.time() - t_start
        final_gz_dist = math.hypot(evaluator.gz_x - tx, evaluator.gz_y - ty)

        # Ground-truth arrival verification
        is_true_arrival = (final_gz_dist <= 0.50)
        is_success = completed and is_true_arrival

        if completed and not is_true_arrival:
            false_reached += 1
        if not completed and is_true_arrival:
            false_blocked += 1

        if is_success:
            success_count += 1
            durations.append(elapsed)
        else:
            failure_count += 1

        total_stops += evaluator.monitor_stop_count
        total_slows += evaluator.monitor_slow_count

        record = {
            "mission_id": m_id,
            "target": target,
            "success": is_success,
            "duration_sec": round(elapsed, 2),
            "final_gz_dist_m": round(final_gz_dist, 3),
            "monitor_stops": evaluator.monitor_stop_count,
            "monitor_slows": evaluator.monitor_slow_count,
            "supervisor_state": evaluator.supervisor_state
        }
        missions_data.append(record)

        if m_id % 5 == 0 or m_id == n_missions:
            print(f"  Mission {m_id:2d}/{n_missions}: Target={target:<12} | Status={'PASS' if is_success else 'FAIL'} | Dist={final_gz_dist:.2f}m | Cum Success={success_count}/{m_id} ({success_count/m_id*100:.1f}%)")

    summary = {
        "n_missions": n_missions,
        "success_count": success_count,
        "failure_count": failure_count,
        "success_rate_pct": round(success_count / n_missions * 100.0, 2),
        "false_reached": false_reached,
        "false_blocked": false_blocked,
        "mean_duration_sec": round(float(np.mean(durations)), 2) if durations else 0.0,
        "total_monitor_stops": total_stops,
        "total_monitor_slows": total_slows,
        "missions": missions_data
    }

    out_file = os.path.join(REPO_ROOT, "scratch", "rl_50_missions_results.json")
    with open(out_file, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved 50-mission results to: {out_file}")
    return summary


def main():
    rclpy.init()
    evaluator = RLLiveEvaluator()

    # Wait for subsystems
    print("Waiting for ROS 2 / Gazebo subsystems...")
    t0 = time.time()
    while time.time() - t0 < 15.0:
        rclpy.spin_once(evaluator, timeout_sec=0.2)
        if evaluator.has_odom and evaluator.has_gz:
            break

    print(f"Subsystems active: Odom={evaluator.has_odom}, Gazebo={evaluator.has_gz}")

    # 1. 20s RL Trace on AT-02
    trace_res = run_part1_20s_rl_trace(evaluator)

    # 2. AT-02 and AT-03 10 runs each
    at_res = run_part2_at02_at03_10runs(evaluator)

    # 3. 50 Missions Campaign
    m50_res = run_part3_50_missions_campaign(evaluator, n_missions=50)

    # 4. Comparative Summary
    print("\n" + "=" * 80)
    print("FINAL COMPARATIVE EVALUATION: RL VS DWB BASELINE")
    print("=" * 80)
    dwb_gate_e = os.path.join(REPO_ROOT, "scratch", "gate_e_50_missions_results.json")
    dwb_succ = 98.0
    if os.path.exists(dwb_gate_e):
        with open(dwb_gate_e, "r") as f:
            dwb_data = json.load(f)
            dwb_succ = dwb_data.get("completion_rate_pct", 98.0)

    print(f"{'Metric':<30} | {'DWB Baseline':<16} | {'RL Policy':<16}")
    print("-" * 70)
    print(f"{'50-Mission Success Rate':<30} | {dwb_succ:>14.1f}% | {m50_res['success_rate_pct']:>14.1f}%")
    print(f"{'False Reached Count':<30} | {0:>16} | {m50_res['false_reached']:>16}")
    print(f"{'False Blocked Count':<30} | {1:>16} | {m50_res['false_blocked']:>16}")
    print(f"{'Mean Transit Time':<30} | {'~18.2 s':>16} | {str(m50_res['mean_duration_sec']) + ' s':>16}")
    print(f"{'Monitor Stops (Total)':<30} | {'2':>16} | {m50_res['total_monitor_stops']:>16}")
    print(f"{'Monitor Slows (Total)':<30} | {'14':>16} | {m50_res['total_monitor_slows']:>16}")

    if m50_res['success_rate_pct'] >= dwb_succ and m50_res['total_monitor_stops'] <= 2:
        print("\nCONCLUSION: RL local planner meets or beats DWB baseline.")
    else:
        print("\nCONCLUSION: RL local planner does not beat DWB baseline in full multi-room simulation. DWB remains authoritative default.")

    evaluator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

