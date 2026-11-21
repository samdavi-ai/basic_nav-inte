#!/usr/bin/env python3
"""
Track 1d: Run 20 s RL Trace on AT-02 and report v, w and monitor counts
"""

import os
import sys
import math
import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav2_msgs.msg import CollisionMonitorState

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class RLTraceAuditor(Node):
    def __init__(self):
        super().__init__("rl_trace_auditor")
        self.cmd_rl = Twist()
        self.cmd_mux = Twist()
        self.odom = Odometry()
        self.monitor_state = CollisionMonitorState()

        self.stop_count = 0
        self.slow_count = 0

        self.create_subscription(Twist, "/cmd_vel_rl", self.on_rl_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_mux", self.on_mux_cmd, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_monitor, 10)

        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.gz_node = gz_transport.Node()

    def on_rl_cmd(self, msg):
        self.cmd_rl = msg

    def on_mux_cmd(self, msg):
        self.cmd_mux = msg

    def on_odom(self, msg):
        self.odom = msg

    def on_monitor(self, msg):
        self.monitor_state = msg
        if msg.action_type == 1:
            self.stop_count += 1
        elif msg.action_type == 2:
            self.slow_count += 1

    def reset_robot(self):
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = 0.0
        req.position.y = 0.0
        req.position.z = 0.05
        req.orientation.w = 1.0
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 1000)

        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.cmd_pub.publish(cmd_msg)
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)


def run_rl_trace():
    rclpy.init()
    auditor = RLTraceAuditor()
    print("Resetting robot to (0,0)...")
    auditor.reset_robot()

    print("Dispatching mission to kitchen (-5.0, -2.5)...")
    nav_cmd = {
        "intent": "NAVIGATE_TO",
        "destination": "kitchen",
        "coords": {"x": -5.0, "y": -2.5, "yaw": -1.57}
    }
    msg = String()
    msg.data = json.dumps(nav_cmd)
    auditor.cmd_pub.publish(msg)

    # Record 20 seconds of trace data at 10 Hz
    t_end = time.time() + 20.0
    records = []

    print("Recording 20 s trace on AT-02...")
    while time.time() < t_end:
        rclpy.spin_once(auditor, timeout_sec=0.1)
        rec = {
            "time": time.time(),
            "rl_v": auditor.cmd_rl.linear.x,
            "rl_w": auditor.cmd_rl.angular.z,
            "mux_v": auditor.cmd_mux.linear.x,
            "mux_w": auditor.cmd_mux.angular.z,
            "odom_vx": auditor.odom.twist.twist.linear.x,
            "odom_wz": auditor.odom.twist.twist.angular.z,
            "robot_x": auditor.odom.pose.pose.position.x,
            "robot_y": auditor.odom.pose.pose.position.y,
            "monitor_action": auditor.monitor_state.action_type,
            "monitor_poly": auditor.monitor_state.polygon_name
        }
        records.append(rec)
        time.sleep(0.05)

    # Cancel goal
    cancel_msg = String()
    cancel_msg.data = json.dumps({"intent": "CANCEL"})
    auditor.cmd_pub.publish(cancel_msg)
    for _ in range(10):
        rclpy.spin_once(auditor, timeout_sec=0.05)

    # Compute summary
    rl_vs = [r["rl_v"] for r in records]
    rl_ws = [r["rl_w"] for r in records]
    odom_vxs = [r["odom_vx"] for r in records]
    odom_wzs = [r["odom_wz"] for r in records]

    print("\n================== 20 s RL TRACE ON AT-02 ==================")
    print(f"Sample count: {len(records)}")
    print(f"RL Command v: mean = {np.mean(rl_vs):.3f} m/s, min = {np.min(rl_vs):.3f}, max = {np.max(rl_vs):.3f}")
    print(f"RL Command w: mean = {np.mean(rl_ws):.3f} rad/s, min = {np.min(rl_ws):.3f}, max = {np.max(rl_ws):.3f}")
    print(f"Odom vx:      mean = {np.mean(odom_vxs):.3f} m/s, max = {np.max(odom_vxs):.3f}")
    print(f"Odom wz:      mean = {np.mean(odom_wzs):.3f} rad/s, max = {np.max(odom_wzs):.3f}")
    print(f"Collision Monitor Stops (PolygonStop): {auditor.stop_count}")
    print(f"Collision Monitor Slows (PolygonSlow): {auditor.slow_count}")
    final_x = records[-1]["robot_x"]
    final_y = records[-1]["robot_y"]
    net_disp = math.hypot(final_x, final_y)
    print(f"Final robot position: ({final_x:.3f}, {final_y:.3f}), net displacement: {net_disp:.3f} m")
    print("============================================================")

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/track1d_rl_trace.json"
    with open(out_file, "w") as f:
        json.dump({
            "samples": len(records),
            "rl_v_mean": float(np.mean(rl_vs)),
            "rl_w_mean": float(np.mean(rl_ws)),
            "odom_vx_mean": float(np.mean(odom_vxs)),
            "odom_wz_mean": float(np.mean(odom_wzs)),
            "stop_count": auditor.stop_count,
            "slow_count": auditor.slow_count,
            "final_pos": [final_x, final_y],
            "net_displacement": net_disp,
            "records": records[::5] # subsample for log
        }, f, indent=2)

    rclpy.shutdown()


if __name__ == "__main__":
    run_rl_trace()

