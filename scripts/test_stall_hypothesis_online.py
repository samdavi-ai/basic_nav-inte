#!/usr/bin/env python3
"""
GRaCEmo ViRa: Online Stall Hypothesis Test
Tests navigation from the exact failing bedroom doorway pose (-4.567, 1.446, -1.54 rad)
to Bedroom goal (-5.0, 2.2, 1.57 rad) under three conditions (10 runs each):
  (a) RL alone
  (b) RL + simple recovery (reverse 0.3m after 3s of PolygonStop)
  (c) DWB baseline
Saves results to scratch/stall_hypothesis_ablation_results.json.
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Any, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Empty
from nav2_msgs.msg import CollisionMonitorState
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class StallTestRunner(Node):
    def __init__(self, mode: str = "RL_ALONE"):
        super().__init__("stall_test_runner")
        self.mode = mode  # "RL_ALONE", "RL_RECOVERY", or "DWB"

        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.has_gz = False

        self.robot_vx = 0.0
        self.robot_wz = 0.0

        self.monitor_state = 0
        self.stop_active = False
        self.stop_start_time: Optional[float] = None
        self.total_stops = 0

        self.supervisor_state = "UNKNOWN"

        # Subscribers
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_mon_state, 10)
        self.create_subscription(Bool, "/collision_monitor/stop_active", self._on_stop_active, 10)
        self.create_subscription(String, "/autonomy/state", self._on_autonomy_state, 10)

        # Publishers
        self.pub_coord = self.create_publisher(String, "/coordinator/command", 10)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self._on_gz_pose)

    def _on_odom(self, msg: Odometry):
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z

    def _on_mon_state(self, msg: CollisionMonitorState):
        self.monitor_state = msg.action_type
        if msg.action_type == 1:
            if not self.stop_active:
                self.stop_active = True
                self.stop_start_time = time.time()
                self.total_stops += 1
        else:
            self.stop_active = False
            self.stop_start_time = None

    def _on_stop_active(self, msg: Bool):
        if msg.data:
            if not self.stop_active:
                self.stop_active = True
                self.stop_start_time = time.time()
                self.total_stops += 1
        else:
            self.stop_active = False
            self.stop_start_time = None

    def _on_autonomy_state(self, msg: String):
        self.supervisor_state = msg.data

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

    def reset_to_pose(self, x: float = -4.567, y: float = 1.446, yaw: float = -1.54):
        """Teleports robot to the exact stall pose and converges AMCL."""
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = 0.05
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = math.sin(yaw / 2.0)
        req.orientation.w = math.cos(yaw / 2.0)
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 1000)

        # Initial pose to Nav2 coordinator
        loc_cmd = {
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": float(x), "y": float(y), "yaw": float(yaw)}
        }
        self.pub_coord.publish(String(data=json.dumps(loc_cmd)))

        # Clear costmaps
        self.pub_coord.publish(String(data=json.dumps({"intent": "CLEAR_COSTMAPS"})))

        t_end = time.time() + 2.0
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def dispatch_goal(self, gx: float = -5.0, gy: float = 2.2, gyaw: float = 1.57):
        nav_cmd = {
            "intent": "NAVIGATE_TO",
            "destination": "bedroom",
            "coords": {"x": float(gx), "y": float(gy), "yaw": float(gyaw)}
        }
        self.pub_coord.publish(String(data=json.dumps(nav_cmd)))

    def cancel_navigation(self):
        self.pub_coord.publish(String(data=json.dumps({"intent": "CANCEL"})))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

    def execute_recovery_backup(self, distance_m: float = 0.30, speed: float = 0.15):
        """Backs up by distance_m via /cmd_vel_teleop to clear PolygonStop."""
        duration = distance_m / speed
        t_end = time.time() + duration
        cmd = Twist()
        cmd.linear.x = -abs(speed)
        cmd.angular.z = 0.0

        while time.time() < t_end:
            self.pub_teleop.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        for _ in range(5):
            self.pub_teleop.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

        # Clear costmaps after recovery
        self.pub_coord.publish(String(data=json.dumps({"intent": "CLEAR_COSTMAPS"})))
        time.sleep(0.5)


def run_condition_trials(mode: str, num_runs: int = 10, timeout_sec: float = 35.0) -> List[Dict[str, Any]]:
    runner = StallTestRunner(mode=mode)
    print(f"\n==================================================")
    print(f"  Testing Condition: {mode} ({num_runs} runs)")
    print(f"  Start: (-4.567, 1.446, -1.54 rad) --> Goal: Bedroom (-5.0, 2.2)")
    print(f"==================================================")

    for _ in range(15):
        rclpy.spin_once(runner, timeout_sec=0.1)

    results = []

    for run_idx in range(1, num_runs + 1):
        runner.reset_to_pose()
        runner.total_stops = 0
        runner.stop_active = False
        runner.stop_start_time = None

        gx, gy = -5.0, 2.2
        runner.dispatch_goal(gx, gy)
        t_start = time.time()
        success = False
        status = "TIMEOUT"
        recovered = False

        while time.time() - t_start < timeout_sec:
            rclpy.spin_once(runner, timeout_sec=0.05)
            dist_to_goal = math.hypot(gx - runner.gz_x, gy - runner.gz_y)

            # Check arrival
            if dist_to_goal <= 0.45 or runner.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED"):
                success = True
                status = "SUCCESS"
                break

            if runner.supervisor_state in ("NAVIGATION_FAILURE", "MISSION_FAILED"):
                status = "FAILURE"
                break

            # Recovery Trigger for Condition (b)
            if mode == "RL_RECOVERY" and runner.stop_active and runner.stop_start_time is not None:
                stop_duration = time.time() - runner.stop_start_time
                if stop_duration >= 3.0:
                    print(f"    [Run {run_idx}] PolygonStop active >= 3.0s! Executing recovery back-up (0.3m)...")
                    runner.execute_recovery_backup(distance_m=0.30, speed=0.15)
                    runner.stop_start_time = None
                    recovered = True
                    # Re-dispatch goal after recovery
                    runner.dispatch_goal(gx, gy)

            time.sleep(0.05)

        elapsed = time.time() - t_start
        final_dist = math.hypot(gx - runner.gz_x, gy - runner.gz_y)
        runner.cancel_navigation()

        failure_mode = "NONE" if success else ("STALLED_POLYGON_STOP" if runner.total_stops > 0 and final_dist > 0.5 else "TIMEOUT")

        print(f"  Run {run_idx:2d}/{num_runs}: [{status}] in {elapsed:.1f}s | Final dist: {final_dist:.2f}m | Stops: {runner.total_stops} | Mode: {failure_mode}")

        results.append({
            "run": run_idx,
            "mode": mode,
            "status": status,
            "success": success,
            "duration_sec": round(elapsed, 2),
            "final_gz_dist_m": round(final_dist, 3),
            "final_gz_pose": [round(runner.gz_x, 3), round(runner.gz_y, 3), round(runner.gz_yaw, 3)],
            "monitor_stops": runner.total_stops,
            "recovered": recovered,
            "failure_mode": failure_mode
        })

    runner.destroy_node()
    return results


def main():
    rclpy.init()
    mode = sys.argv[1] if len(sys.argv) > 1 else "RL_ALONE"
    out_file = sys.argv[2] if len(sys.argv) > 2 else f"scratch/stall_{mode.lower()}_results.json"

    runs = run_condition_trials(mode=mode, num_runs=10, timeout_sec=30.0)
    success_count = sum(1 for r in runs if r["success"])

    print(f"\nCondition {mode} Summary: {success_count}/10 SUCCESS ({success_count/10*100:.1f}%)")

    with open(out_file, "w") as f:
        json.dump({
            "mode": mode,
            "success_count": success_count,
            "total_runs": 10,
            "success_rate_pct": success_count / 10 * 100.0,
            "runs": runs
        }, f, indent=2)
    print(f"Saved results to: {out_file}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

