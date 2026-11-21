#!/usr/bin/env python3
"""
GRaCEmo ViRa: Dynamics Identification Tool
Measures Gazebo's actual physical response to step commands on cmd_vel:
1. Linear step: v = 0.3 m/s, w = 0.0 rad/s
2. Angular step: v = 0.0 m/s, w = 1.0 rad/s
Computes:
- Rise time (t_90)
- First-order time constant (tau)
- Effective transport latency (delay)
- Steady-state velocity and slip ratio
Saves results to scratch/gazebo_dynamics_step_response.json
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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class DynamicsProbeNode(Node):
    def __init__(self):
        super().__init__("gazebo_dynamics_probe")
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)

        self.latest_odom: Odometry = Odometry()
        self.odom_vx = 0.0
        self.odom_wz = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.last_gz_time = 0.0
        self.last_gz_x = 0.0
        self.last_gz_y = 0.0
        self.last_gz_yaw = 0.0
        self.gz_vx = 0.0
        self.gz_wz = 0.0

        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self._on_gz_pose)

    def _on_odom(self, msg: Odometry):
        self.latest_odom = msg
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_wz = msg.twist.twist.angular.z
        p = msg.pose.pose.position
        self.odom_x = p.x
        self.odom_y = p.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def _on_gz_pose(self, msg: GzPoseV):
        now = time.time()
        for p in msg.pose:
            if p.name == "gracemo_vira":
                x = p.position.x
                y = p.position.y
                q = p.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)

                if self.last_gz_time > 0.0:
                    dt = now - self.last_gz_time
                    if dt > 0.005:
                        dx = x - self.last_gz_x
                        dy = y - self.last_gz_y
                        dist = math.hypot(dx, dy)
                        dyaw = (yaw - self.last_gz_yaw + math.pi) % (2.0 * math.pi) - math.pi
                        self.gz_vx = dist / dt
                        self.gz_wz = dyaw / dt

                self.last_gz_x = x
                self.last_gz_y = y
                self.last_gz_yaw = yaw
                self.last_gz_time = now
                self.gz_x = x
                self.gz_y = y
                self.gz_yaw = yaw

    def reset_robot(self):
        # Stop motion
        for _ in range(5):
            self.pub_teleop.publish(Twist())
            time.sleep(0.05)

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

        # Clear costmaps
        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.pub_cmd.publish(cmd_msg)
        time.sleep(1.0)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)


def run_dynamics_experiment():
    rclpy.init()
    node = DynamicsProbeNode()

    # Allow discovery
    print("Waiting for /odom and Gazebo transport...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n--- TEST 1: LINEAR STEP RESPONSE (v_cmd = 0.30 m/s, w_cmd = 0.0 rad/s) ---")
    node.reset_robot()

    step_cmd_v = 0.30
    step_duration = 3.0
    v_records: List[Dict[str, float]] = []

    t_start = time.time()
    t_end = t_start + step_duration

    rate_hz = 50
    dt = 1.0 / rate_hz

    while time.time() < t_end:
        now = time.time()
        elapsed = now - t_start

        # Publish step command
        cmd = Twist()
        cmd.linear.x = step_cmd_v
        cmd.angular.z = 0.0
        node.pub_teleop.publish(cmd)

        rclpy.spin_once(node, timeout_sec=0.01)

        v_records.append({
            "t": elapsed,
            "cmd_v": step_cmd_v,
            "cmd_w": 0.0,
            "odom_vx": node.odom_vx,
            "odom_wz": node.odom_wz,
            "gz_vx": node.gz_vx,
            "gz_wz": node.gz_wz,
            "gz_x": node.gz_x,
            "gz_y": node.gz_y
        })
        time.sleep(dt)

    # Stop motion
    for _ in range(10):
        node.pub_teleop.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Analyze linear step response
    times = np.array([r["t"] for r in v_records])
    odom_vxs = np.array([r["odom_vx"] for r in v_records])
    gz_vxs = np.array([r["gz_vx"] for r in v_records])

    # Steady state from last 1.0s
    tail_idx = np.where(times >= (step_duration - 1.0))[0]
    linear_ss_odom = float(np.mean(odom_vxs[tail_idx]))
    linear_ss_gz = float(np.mean(gz_vxs[tail_idx]))

    # Rise time (t_90: time to reach 90% of ss)
    t90_thresh = 0.90 * linear_ss_odom
    t63_thresh = 0.632 * linear_ss_odom

    idx_90 = np.where(odom_vxs >= t90_thresh)[0]
    t_90_v = float(times[idx_90[0]]) if len(idx_90) > 0 else float("nan")

    idx_63 = np.where(odom_vxs >= t63_thresh)[0]
    tau_v = float(times[idx_63[0]]) if len(idx_63) > 0 else float("nan")

    # Latency: time to reach 5% of ss
    idx_5 = np.where(odom_vxs >= 0.05 * linear_ss_odom)[0]
    lat_v = float(times[idx_5[0]]) if len(idx_5) > 0 else 0.0

    slip_v = (step_cmd_v - linear_ss_odom) / step_cmd_v

    print(f"  Linear Steady State (Odom): {linear_ss_odom:.4f} m/s (Target: {step_cmd_v} m/s)")
    print(f"  Linear Steady State (Gazebo): {linear_ss_gz:.4f} m/s")
    print(f"  Linear Rise Time (t_90): {t_90_v:.3f} s")
    print(f"  Linear Time Constant (tau): {tau_v:.3f} s")
    print(f"  Linear Latency: {lat_v * 1000.0:.1f} ms")
    print(f"  Linear Slip Ratio: {slip_v * 100.0:.2f}%")

    print("\n--- TEST 2: ANGULAR STEP RESPONSE (v_cmd = 0.0 m/s, w_cmd = 1.00 rad/s) ---")
    node.reset_robot()

    step_cmd_w = 1.00
    w_records: List[Dict[str, float]] = []

    t_start = time.time()
    t_end = t_start + step_duration

    while time.time() < t_end:
        now = time.time()
        elapsed = now - t_start

        # Publish step command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = step_cmd_w
        node.pub_teleop.publish(cmd)

        rclpy.spin_once(node, timeout_sec=0.01)

        w_records.append({
            "t": elapsed,
            "cmd_v": 0.0,
            "cmd_w": step_cmd_w,
            "odom_vx": node.odom_vx,
            "odom_wz": node.odom_wz,
            "gz_vx": node.gz_vx,
            "gz_wz": node.gz_wz,
            "gz_yaw": node.gz_yaw
        })
        time.sleep(dt)

    # Stop motion
    for _ in range(10):
        node.pub_teleop.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.02)

    # Analyze angular step response
    times_w = np.array([r["t"] for r in w_records])
    odom_wzs = np.array([r["odom_wz"] for r in w_records])
    gz_wzs = np.array([r["gz_wz"] for r in w_records])

    tail_idx_w = np.where(times_w >= (step_duration - 1.0))[0]
    angular_ss_odom = float(np.mean(odom_wzs[tail_idx_w]))
    angular_ss_gz = float(np.mean(gz_wzs[tail_idx_w]))

    t90_thresh_w = 0.90 * angular_ss_odom
    t63_thresh_w = 0.632 * angular_ss_odom

    idx_90_w = np.where(odom_wzs >= t90_thresh_w)[0]
    t_90_w = float(times_w[idx_90_w[0]]) if len(idx_90_w) > 0 else float("nan")

    idx_63_w = np.where(odom_wzs >= t63_thresh_w)[0]
    tau_w = float(times_w[idx_63_w[0]]) if len(idx_63_w) > 0 else float("nan")

    idx_5_w = np.where(odom_wzs >= 0.05 * angular_ss_odom)[0]
    lat_w = float(times_w[idx_5_w[0]]) if len(idx_5_w) > 0 else 0.0

    slip_w = (step_cmd_w - angular_ss_odom) / step_cmd_w

    print(f"  Angular Steady State (Odom): {angular_ss_odom:.4f} rad/s (Target: {step_cmd_w} rad/s)")
    print(f"  Angular Steady State (Gazebo): {angular_ss_gz:.4f} rad/s")
    print(f"  Angular Rise Time (t_90): {t_90_w:.3f} s")
    print(f"  Angular Time Constant (tau): {tau_w:.3f} s")
    print(f"  Angular Latency: {lat_w * 1000.0:.1f} ms")
    print(f"  Angular Slip Ratio: {slip_w * 100.0:.2f}%")

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gazebo_dynamics_step_response.json"
    result = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "linear": {
            "cmd_v": step_cmd_v,
            "ss_odom": linear_ss_odom,
            "ss_gz": linear_ss_gz,
            "t_90": t_90_v,
            "tau": tau_v,
            "latency_ms": lat_v * 1000.0,
            "slip_ratio": slip_v,
            "samples": v_records[::2]
        },
        "angular": {
            "cmd_w": step_cmd_w,
            "ss_odom": angular_ss_odom,
            "ss_gz": angular_ss_gz,
            "t_90": t_90_w,
            "tau": tau_w,
            "latency_ms": lat_w * 1000.0,
            "slip_ratio": slip_w,
            "samples": w_records[::2]
        }
    }

    with open(out_file, "w") as f:
        json.dump(result, f, indent=2)

    print(f"\nSaved step response dynamics to: {out_file}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_dynamics_experiment()

