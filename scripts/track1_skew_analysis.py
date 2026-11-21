#!/usr/bin/env python3
"""
Track 1: RL Observation Skew, Policy Sweep, and Comparison Analysis
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
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String

import onnxruntime as ort

# Add repo to sys.path
REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

from gracemo_rl.common.obs import (
    build_observation,
    downsample_scan_ranges,
    compute_relative_goal,
    OBS_DIM
)
from gracemo_rl.common.limits import load_nav2_limits
from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class Track1Auditor(Node):
    def __init__(self):
        super().__init__("track1_auditor")
        self.latest_scan = None
        self.latest_odom = None
        self.latest_plan = None

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)

        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.gz_node = gz_transport.Node()

    def on_scan(self, msg):
        self.latest_scan = msg

    def on_odom(self, msg):
        self.latest_odom = msg

    def on_plan(self, msg):
        self.latest_plan = msg

    def reset_robot(self, x=0.0, y=0.0, yaw=0.0):
        # Gazebo teleport
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = x
        req.position.y = y
        req.position.z = 0.05
        req.orientation.w = math.cos(yaw / 2.0)
        req.orientation.z = math.sin(yaw / 2.0)
        self.gz_node.request("/world/apartment_floor/set_pose", req, GzPose, GzBoolean, 1000)

        # Clear costmaps
        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.cmd_pub.publish(cmd_msg)

        # Let settle
        time.sleep(1.0)
        for _ in range(20):
            rclpy.spin_once(self, timeout_sec=0.05)


def run_track1():
    rclpy.init()
    auditor = Track1Auditor()

    print("Resetting robot to (0,0)...")
    auditor.reset_robot(0.0, 0.0, 0.0)

    # Dispatch navigation goal to kitchen to generate /plan
    print("Dispatching mission to kitchen (-5.0, -2.5) to capture /plan...")
    nav_cmd = {
        "intent": "NAVIGATE_TO",
        "destination": "kitchen",
        "coords": {"x": -5.0, "y": -2.5, "yaw": -1.57}
    }
    msg = String()
    msg.data = json.dumps(nav_cmd)
    auditor.cmd_pub.publish(msg)

    # Wait for /scan, /odom, /plan
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(auditor, timeout_sec=0.1)
        if auditor.latest_scan is not None and auditor.latest_odom is not None and auditor.latest_plan is not None:
            break

    # Immediately cancel mission
    cancel_msg = String()
    cancel_msg.data = json.dumps({"intent": "CANCEL"})
    auditor.cmd_pub.publish(cancel_msg)
    for _ in range(10):
        rclpy.spin_once(auditor, timeout_sec=0.05)

    scan = auditor.latest_scan
    odom = auditor.latest_odom
    plan = auditor.latest_plan

    if scan is None or odom is None:
        print("ERROR: Could not capture live telemetry frame!")
        return

    print(f"Captured live frame: scan={len(scan.ranges)} beams, "
          f"odom=({odom.pose.pose.position.x:.3f}, {odom.pose.pose.position.y:.3f}), "
          f"plan={len(plan.poses) if plan else 0} poses")

    # =========================================================================
    # Part A: Feed through nav_env.py and rl_local_planner.py
    # =========================================================================
    p = odom.pose.pose.position
    q = odom.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    vx = odom.twist.twist.linear.x
    wz = odom.twist.twist.angular.z

    # Extract lookahead target from plan or fallback
    if plan and len(plan.poses) > 5:
        target_pt = plan.poses[min(10, len(plan.poses)-1)].pose.position
        gx, gy = target_pt.x, target_pt.y
    else:
        gx, gy = -5.0, -2.5

    last_action = np.array([0.0, 0.0], dtype=np.float32)

    # 1. Canonical / rl_local_planner obs
    obs_planner = build_observation(
        scan_ranges=scan.ranges,
        robot_x=p.x,
        robot_y=p.y,
        robot_yaw=yaw,
        goal_x=gx,
        goal_y=gy,
        last_action=last_action,
        robot_vx=vx,
        robot_wz=wz
    )

    # 2. nav_env.py obs
    env = GracemoNavEnv(mode="kinematic")
    env.robot_x = p.x
    env.robot_y = p.y
    env.robot_yaw = yaw
    env.goal_x = gx
    env.goal_y = gy
    env.last_action = last_action.copy()
    env.robot_vx = vx
    env.robot_wz = wz

    # Env live scan conversion
    raw_ranges = np.array(scan.ranges, dtype=np.float32)
    norm_scan_for_env = downsample_scan_ranges(raw_ranges, downsampled_beams=72)
    env_scan_meters = norm_scan_for_env * env.config.max_scan_range
    obs_env = env._build_observation(env_scan_meters)

    diffs = np.abs(obs_planner - obs_env)
    max_diff = float(np.max(diffs))

    print("\n--- TRACK 1a: Observation Comparison ---")
    print(f"Max absolute difference: {max_diff:.8e}")
    print(f"obs_planner shape: {obs_planner.shape}, obs_env shape: {obs_env.shape}")

    # Sector scan summary
    scan_diff = diffs[:72]
    goal_diff = diffs[72:75]
    action_diff = diffs[75:77]
    vel_diff = diffs[77:79]
    print(f"Max Scan diff (dims 0-71):   {np.max(scan_diff):.8e}")
    print(f"Max Goal diff (dims 72-74):  {np.max(goal_diff):.8e}")
    print(f"Max Action diff (dims 75-76):{np.max(action_diff):.8e}")
    print(f"Max Vel diff (dims 77-78):   {np.max(vel_diff):.8e}")

    # =========================================================================
    # Part B: Compare sensor / env specifications
    # =========================================================================
    raw = np.array(scan.ranges)
    inf_count = int(np.isinf(raw).sum())
    nan_count = int(np.isnan(raw).sum())
    finite_mask = np.isfinite(raw)
    min_observed_range = float(np.min(raw[finite_mask])) if finite_mask.any() else 0.0
    max_observed_range = float(np.max(raw[finite_mask])) if finite_mask.any() else 0.0

    print("\n--- TRACK 1b: Kinematic vs Live LiDAR Specs ---")
    print(f"Live /scan angle_min: {scan.angle_min:.4f} rad, angle_max: {scan.angle_max:.4f} rad, inc: {scan.angle_increment:.4f}")
    print(f"Live /scan range_min: {scan.range_min:.4f} m, range_max: {scan.range_max:.4f} m")
    print(f"Live raw beams: {len(raw)}, inf count: {inf_count} ({inf_count/len(raw)*100:.1f}%), nan count: {nan_count}")
    print(f"Live observed min range: {min_observed_range:.3f} m, max: {max_observed_range:.3f} m")

    # Action mapping: what physical velocity does policy action (0, 0) produce?
    nav_limits = load_nav2_limits()
    # With allow_reverse=True:
    # v = (a_v + 1.0) / 2.0 * (v_max - v_min_rev) + v_min_rev
    # For a_v = 0.0: v = 0.5 * (0.50 - (-0.20)) + (-0.20) = 0.5 * 0.70 - 0.20 = 0.15 m/s
    a_zero_v = (0.0 + 1.0) / 2.0 * (nav_limits.max_linear_vel - nav_limits.min_reverse_vel) + nav_limits.min_reverse_vel
    a_zero_w = 0.0 * nav_limits.max_angular_vel
    print(f"Action mapping for a=(0, 0) [allow_reverse=True]: v = {a_zero_v:.3f} m/s, w = {a_zero_w:.3f} rad/s")
    # For a_v = -1.0: v = -0.20 m/s; for a_v = +1.0: v = +0.50 m/s

    # =========================================================================
    # Part C: Sweep Policy with Synthetic Observations
    # =========================================================================
    model_path = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_rl/share/gracemo_rl/models/gracemo_sac_policy.onnx"
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    sweep_cases = [
        ("Free space, goal ahead (dx=3.0, dy=0.0)", 3.0, 0.0, np.full(72, 1.0, dtype=np.float32)),
        ("Free space, goal left (dx=0.0, dy=3.0)", 0.0, 3.0, np.full(72, 1.0, dtype=np.float32)),
        ("Free space, goal right (dx=0.0, dy=-3.0)", 0.0, -3.0, np.full(72, 1.0, dtype=np.float32)),
        ("Free space, goal behind (dx=-3.0, dy=0.0)", -3.0, 0.0, np.full(72, 1.0, dtype=np.float32)),
        ("Wall 0.4m ahead, goal ahead (dx=3.0, dy=0.0)", 3.0, 0.0, None),
    ]

    print("\n--- TRACK 1c: Policy Synthetic Sweep ---")
    sweep_results = []
    for desc, g_dx, g_dy, scan_norm in sweep_cases:
        if scan_norm is None:
            # Wall 0.4m ahead: front beams (indices 30-42) set to 0.40m / 10.0 = 0.04
            scan_norm = np.full(72, 1.0, dtype=np.float32)
            scan_norm[30:43] = 0.04  # 0.4m obstacle ahead

        norm_dist = float(np.tanh(math.hypot(g_dx, g_dy) / 5.0))
        angle = math.atan2(g_dy, g_dx)
        sin_b = float(math.sin(angle))
        cos_b = float(math.cos(angle))

        syn_obs = np.concatenate([
            scan_norm,
            np.array([norm_dist, sin_b, cos_b], dtype=np.float32),
            np.array([0.0, 0.0], dtype=np.float32),  # last action
            np.array([0.0, 0.0], dtype=np.float32)   # velocity
        ]).astype(np.float32).reshape(1, -1)

        out = session.run(None, {input_name: syn_obs})[0].ravel()
        a_v, a_w = float(out[0]), float(out[1])

        # Map to physical command
        cmd_v = (a_v + 1.0) / 2.0 * (nav_limits.max_linear_vel - nav_limits.min_reverse_vel) + nav_limits.min_reverse_vel
        cmd_w = a_w * nav_limits.max_angular_vel

        sweep_results.append({
            "scenario": desc,
            "raw_action": [a_v, a_w],
            "cmd_v": cmd_v,
            "cmd_w": cmd_w
        })
        print(f"Scenario: {desc}")
        print(f"  Policy Action: [a_v={a_v:.4f}, a_w={a_w:.4f}] => cmd_v={cmd_v:.3f} m/s, cmd_w={cmd_w:.3f} rad/s")

    # Save summary
    out_data = {
        "max_obs_diff": max_diff,
        "obs_planner_sample": obs_planner.tolist(),
        "obs_env_sample": obs_env.tolist(),
        "scan_specs": {
            "angle_min": scan.angle_min,
            "angle_max": scan.angle_max,
            "angle_inc": scan.angle_increment,
            "range_min": scan.range_min,
            "range_max": scan.range_max,
            "inf_count": inf_count,
            "nan_count": nan_count,
            "min_obs_range": min_observed_range,
            "max_obs_range": max_observed_range,
        },
        "action_mapping": {
            "a_zero": [a_zero_v, a_zero_w],
            "formula": "v = (a_v + 1)/2 * (v_max - v_min_rev) + v_min_rev"
        },
        "sweep_results": sweep_results
    }

    with open("/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/track1_skew_report.json", "w") as f:
        json.dump(out_data, f, indent=2)
    print("\nSaved Track 1 report to scratch/track1_skew_report.json")

    rclpy.shutdown()


if __name__ == "__main__":
    run_track1()

