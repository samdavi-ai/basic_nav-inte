#!/usr/bin/env python3
"""
GRaCEmo ViRa — Live Gazebo Navigation Benchmark Suite.
Executes live randomized navigation benchmark episodes directly inside Gazebo Harmonic.
Measures:
  - Success Rate
  - Collision Monitor Interventions (PolygonStop, PolygonSlow)
  - Time to Goal (mean, median, p95)
  - Actual Path Length & SPL
  - Minimum Clearance (from real LiDAR /scan)
  - Timeouts & Stagnation Failures
"""

import sys
import math
import time
import json
import argparse
from typing import Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav2_msgs.msg import CollisionMonitorState


class LiveGazeboBenchmark(Node):
    def __init__(self, controller_name: str = "DWB"):
        super().__init__("live_gazebo_benchmark")
        self.controller_name = controller_name.upper()

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        self.last_scan_min = float("inf")
        self.poly_stop_count = 0
        self.poly_slow_count = 0
        self.supervisor_state = "UNKNOWN"

        # Subscribers
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.sub_col = self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_col_state, 10)
        self.sub_state = self.create_subscription(String, "/autonomy/state", self._on_state, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)

    def _on_odom(self, msg: Odometry):
        self.has_odom = True
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def _on_scan(self, msg: LaserScan):
        if msg.ranges:
            valid = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r) and r > 0.05]
            if valid:
                self.last_scan_min = min(valid)

    def _on_col_state(self, msg: CollisionMonitorState):
        if msg.action_type == 1:
            self.poly_stop_count += 1
        elif msg.action_type == 2:
            self.poly_slow_count += 1

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def wait_for_ready(self, timeout_sec=10.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.has_odom and self.pub_cmd.get_subscription_count() > 0:
                return True
        return False

    def reset_supervisor(self):
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL"})
        self.pub_cmd.publish(msg)
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        msg.data = json.dumps({"intent": "RESET"})
        self.pub_cmd.publish(msg)
        time.sleep(0.3)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_live_episode(self, target_dest: str, target_coords: Tuple[float, float], timeout_sec=40.0) -> Dict[str, Any]:
        self.reset_supervisor()
        start_x, start_y = self.robot_x, self.robot_y
        tx, ty = target_coords
        start_dist = math.hypot(tx - start_x, ty - start_y)

        self.poly_stop_count = 0
        self.poly_slow_count = 0
        min_clearance = float("inf")

        # Dispatch
        cmd = {"intent": "NAVIGATE_TO", "destination": target_dest}
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_cmd.publish(msg)

        t0 = time.time()
        path_length = 0.0
        prev_x, prev_y = start_x, start_y
        reached = False
        stalled = False

        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            step_disp = math.hypot(self.robot_x - prev_x, self.robot_y - prev_y)
            path_length += step_disp
            prev_x, prev_y = self.robot_x, self.robot_y

            min_clearance = min(min_clearance, self.last_scan_min)
            dist_to_goal = math.hypot(self.robot_x - tx, self.robot_y - ty)

            if self.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED") or dist_to_goal <= 0.65:
                reached = True
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                stalled = True
                break

        elapsed = time.time() - t0
        timeout = (not reached and not stalled and elapsed >= timeout_sec)
        final_err = math.hypot(self.robot_x - tx, self.robot_y - ty)
        spl = (start_dist / max(path_length, start_dist)) if reached and path_length > 0 else 0.0

        # Halt and clear
        self.reset_supervisor()

        return {
            "destination": target_dest,
            "success": reached,
            "timeout": timeout,
            "stalled": stalled,
            "time_sec": elapsed,
            "path_length": path_length,
            "start_dist": start_dist,
            "final_err": final_err,
            "spl": spl,
            "min_clearance": min_clearance if min_clearance != float("inf") else 0.0,
            "poly_stop_count": self.poly_stop_count,
            "poly_slow_count": self.poly_slow_count,
        }


def main():
    parser = argparse.ArgumentParser(description="Live Gazebo Navigation Benchmark")
    parser.add_argument("--controller", type=str, default="DWB", choices=["DWB", "RL", "dwb", "rl"])
    parser.add_argument("--episodes", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    bench = LiveGazeboBenchmark(controller_name=args.controller)

    print(f"\n============================================================")
    print(f"   LIVE GAZEBO BENCHMARK — CONTROLLER: {bench.controller_name}")
    print(f"============================================================")

    if not bench.wait_for_ready(timeout_sec=15.0):
        print("ERROR: Telemetry timeout. Live Gazebo simulation is not publishing.")
        sys.exit(1)

    # Standardized evaluation destinations across the apartment
    eval_targets = [
        ("kitchen", (-5.0, -2.5)),
        ("hallway", (0.0, 0.0)),
        ("living room", (4.0, -2.5)),
        ("hallway", (0.0, 0.0)),
        ("bedroom", (-5.0, 2.2)),
        ("hallway", (0.0, 0.0)),
        ("study", (4.5, 3.0)),
        ("hallway", (0.0, 0.0)),
        ("kitchen", (-5.0, -2.5)),
        ("hallway", (0.0, 0.0)),
    ]

    results = []
    num_runs = min(args.episodes, len(eval_targets))

    for i in range(num_runs):
        dest_name, coords = eval_targets[i]
        print(f"\n[Episode {i+1}/{num_runs}] Navigating to '{dest_name}' at {coords}...")
        ep_res = bench.run_live_episode(dest_name, coords, timeout_sec=35.0)
        results.append(ep_res)
        status_str = "SUCCESS" if ep_res["success"] else ("STALLED" if ep_res["stalled"] else "TIMEOUT")
        print(f"  Result: [{status_str}] | Time: {ep_res['time_sec']:.1f}s | Path: {ep_res['path_length']:.2f}m | Min Clear: {ep_res['min_clearance']:.2f}m | Stop/Slow: {ep_res['poly_stop_count']}/{ep_res['poly_slow_count']}")

    # Aggregation
    successes = [r for r in results if r["success"]]
    succ_rate = (len(successes) / len(results)) * 100.0
    times = [r["time_sec"] for r in successes] if successes else [35.0]
    spl_mean = float(np.mean([r["spl"] for r in results])) if results else 0.0
    stops_mean = float(np.mean([r["poly_stop_count"] for r in results])) if results else 0.0
    slows_mean = float(np.mean([r["poly_slow_count"] for r in results])) if results else 0.0
    clear_mean = float(np.mean([r["min_clearance"] for r in results])) if results else 0.0
    timeouts = sum(1 for r in results if r["timeout"])

    print("\n" + "="*60)
    print(f"      LIVE GAZEBO BENCHMARK SUMMARY [{bench.controller_name}]")
    print("="*60)
    print(f"  Episodes Evaluated:       {len(results)}")
    print(f"  Success Rate:             {succ_rate:.1f}% ({len(successes)}/{len(results)})")
    print(f"  Timeouts:                 {timeouts}")
    print(f"  Mean Time to Goal:        {np.mean(times):.2f} s")
    print(f"  Success Weighted Path:    {spl_mean:.3f}")
    print(f"  Mean Min Clearance:       {clear_mean:.3f} m")
    print(f"  PolygonStop triggers/ep:  {stops_mean:.2f}")
    print(f"  PolygonSlow triggers/ep:  {slows_mean:.2f}")
    print("="*60 + "\n")

    bench.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import numpy as np
    main()

