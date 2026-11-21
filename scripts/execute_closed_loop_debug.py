#!/usr/bin/env python3
"""
GRaCEmo ViRa: Closed-Loop Debug with Static Localization
Runs RL controller to:
1. 2m straight ahead in hallway: (2.0, 0.0) — 5 runs
2. Kitchen door: (-5.0, -1.0) — 5 runs
Logs trajectory track, duration, net displacement, monitor state, and failure mode.
Saves raw JSON to scratch/closed_loop_debug_results.json
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Any
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.msg import CollisionMonitorState

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class ClosedLoopDebugger(Node):
    def __init__(self):
        super().__init__("closed_loop_debugger")
        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.cmd_rl_sub = self.create_subscription(Twist, "/cmd_vel_rl", self._on_rl_cmd, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.mon_sub = self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_mon, 10)

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self._on_gz_pose)

        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_wz = 0.0

        self.rl_vx = 0.0
        self.rl_wz = 0.0

        self.monitor_state = 0
        self.monitor_stops = 0
        self.monitor_slows = 0

    def _on_rl_cmd(self, msg: Twist):
        self.rl_vx = msg.linear.x
        self.rl_wz = msg.angular.z

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_x = p.x
        self.odom_y = p.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z

    def _on_mon(self, msg: CollisionMonitorState):
        self.monitor_state = msg.action_type
        if msg.action_type == 1:
            self.monitor_stops += 1
        elif msg.action_type == 2:
            self.monitor_slows += 1

    def _on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                self.gz_x = p.position.x
                self.gz_y = p.position.y
                q = p.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.gz_yaw = math.atan2(siny, cosy)

    def reset_robot(self):
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = 0.0
        req.position.y = 0.0
        req.position.z = 0.05
        req.orientation.w = 1.0
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = 0.0
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 1000)

        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.cmd_pub.publish(cmd_msg)
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

    def dispatch_goal(self, gx: float, gy: float, gyaw: float = 0.0):
        nav_cmd = {
            "intent": "NAVIGATE_TO",
            "destination": f"goal_{gx}_{gy}",
            "coords": {"x": gx, "y": gy, "yaw": gyaw}
        }
        msg = String()
        msg.data = json.dumps(nav_cmd)
        self.cmd_pub.publish(msg)

    def cancel_goal(self):
        cancel_msg = String()
        cancel_msg.data = json.dumps({"intent": "CANCEL"})
        self.cmd_pub.publish(cancel_msg)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)


def run_trials(target_name: str, gx: float, gy: float, gyaw: float, num_runs: int = 5, timeout_sec: float = 25.0):
    debugger = ClosedLoopDebugger()
    for _ in range(10):
        rclpy.spin_once(debugger, timeout_sec=0.1)

    trials = []
    print(f"\nRunning {num_runs} closed-loop trials to {target_name} ({gx}, {gy})...")

    for run_idx in range(1, num_runs + 1):
        debugger.reset_robot()
        debugger.monitor_stops = 0
        debugger.monitor_slows = 0

        debugger.dispatch_goal(gx, gy, gyaw)
        t_start = time.time()
        track = []
        status = "TIMEOUT"

        while time.time() - t_start < timeout_sec:
            rclpy.spin_once(debugger, timeout_sec=0.05)
            d_goal = math.hypot(gx - debugger.gz_x, gy - debugger.gz_y)
            track.append({
                "t": round(time.time() - t_start, 3),
                "x": round(debugger.gz_x, 3),
                "y": round(debugger.gz_y, 3),
                "yaw": round(debugger.gz_yaw, 3),
                "rl_v": round(debugger.rl_vx, 3),
                "rl_w": round(debugger.rl_wz, 3),
                "mon": debugger.monitor_state
            })

            if d_goal < 0.35:
                status = "SUCCESS"
                break

            time.sleep(0.05)

        debugger.cancel_goal()

        final_d = math.hypot(gx - debugger.gz_x, gy - debugger.gz_y)
        net_disp = math.hypot(debugger.gz_x, debugger.gz_y)

        # Determine failure mode if not success
        failure_mode = "NONE"
        if status != "SUCCESS":
            # Check if circling: high angular velocity with near-zero progress
            mean_w = np.mean([abs(p["rl_w"]) for p in track])
            if mean_w > 0.8 and net_disp < 1.0:
                failure_mode = "CIRCLING_IN_PLACE"
            elif debugger.monitor_stops > 0:
                failure_mode = "COLLISION_MONITOR_STOP"
            elif net_disp < 0.2:
                failure_mode = "STALLED_AT_START"
            else:
                failure_mode = "DEVIATED_FROM_GOAL"

        print(f" Run {run_idx}/{num_runs}: status={status}, final_dist={final_d:.2f}m, net_disp={net_disp:.2f}m, stops={debugger.monitor_stops}, mode={failure_mode}")

        trials.append({
            "run": run_idx,
            "target": target_name,
            "goal": [gx, gy],
            "status": status,
            "failure_mode": failure_mode,
            "duration_sec": round(time.time() - t_start, 2),
            "final_pose": [round(debugger.gz_x, 3), round(debugger.gz_y, 3), round(debugger.gz_yaw, 3)],
            "final_dist_to_goal": round(final_d, 3),
            "net_displacement": round(net_disp, 3),
            "monitor_stops": debugger.monitor_stops,
            "monitor_slows": debugger.monitor_slows,
            "track_sample_count": len(track),
            "track_preview": track[::10]  # Every 0.5s
        })

    debugger.destroy_node()
    return trials


def main():
    rclpy.init()
    results = {}

    # Target 1: 2m straight ahead in hallway (10 runs)
    results["hallway_2m"] = run_trials("hallway_2m", gx=2.0, gy=0.0, gyaw=0.0, num_runs=10, timeout_sec=20.0)

    # Target 2: Kitchen door (10 runs)
    results["kitchen_door"] = run_trials("kitchen_door", gx=-5.0, gy=-1.0, gyaw=-1.57, num_runs=10, timeout_sec=25.0)

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/closed_loop_debug_results.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    print(f"\nClosed-loop debug results saved to: {out_file}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

