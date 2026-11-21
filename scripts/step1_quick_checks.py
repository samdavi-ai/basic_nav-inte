#!/usr/bin/env python3
"""
GRaCEmo ViRa: Step 1 Quick Checks
1. Bearing sign / convention:
   - Verifies observation builder produces identical sin/cos at +30 deg left
   - Commands w = +0.5 rad/s for 2.0 s in Gazebo and verifies positive yaw change (+w turns left toward goal)
2. Limits & Step Response (3 repeated runs per axis):
   - Linear step response (v_cmd = 0.30 m/s): 3 runs -> mean, std
   - Angular step response (w_cmd = 0.75 rad/s): 3 runs -> mean, std
Saves to scratch/quick_checks_step_response_3runs.json
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

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))
from gracemo_rl.common.obs import build_observation, compute_relative_goal
from gracemo_rl.envs.nav_env import GracemoNavEnv


class QuickCheckProbe(Node):
    def __init__(self):
        super().__init__("quick_check_probe")
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)

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

        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.pub_cmd.publish(cmd_msg)
        time.sleep(1.0)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)


def check_bearing_parity() -> Dict[str, Any]:
    d = 2.0
    b_deg = 30.0
    b_rad = math.radians(b_deg)
    gx = d * math.cos(b_rad)
    gy = d * math.sin(b_rad)

    norm_d, sin_b, cos_b = compute_relative_goal(gx, gy, 0.0, 0.0, 0.0)
    obs_node = build_observation(
        scan_ranges=np.full(360, 5.0, dtype=np.float32),
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0,
        goal_x=gx, goal_y=gy,
        last_action=np.zeros(2, dtype=np.float32),
        robot_vx=0.0, robot_wz=0.0
    )
    env = GracemoNavEnv(mode="kinematic")
    env.reset()
    env.robot_x = 0.0
    env.robot_y = 0.0
    env.robot_yaw = 0.0
    env.goal_x = gx
    env.goal_y = gy
    obs_env = env._build_observation(np.full(72, 5.0, dtype=np.float32))

    diff = float(np.max(np.abs(obs_node[72:75] - obs_env[72:75])))

    return {
        "goal_deg": b_deg,
        "sin_bearing": float(sin_b),
        "cos_bearing": float(cos_b),
        "node_sin": float(obs_node[73]),
        "node_cos": float(obs_node[74]),
        "env_sin": float(obs_env[73]),
        "env_cos": float(obs_env[74]),
        "max_diff": diff,
        "parity": diff < 1e-6
    }


def run_checks():
    rclpy.init()
    probe = QuickCheckProbe()

    print("Checking bearing convention parity...")
    bearing_result = check_bearing_parity()
    print(f"Parity: {bearing_result['parity']}, max_diff: {bearing_result['max_diff']:.2e}, sin={bearing_result['sin_bearing']:.4f}, cos={bearing_result['cos_bearing']:.4f}")

    print("Waiting for /odom and Gazebo transport...")
    for _ in range(20):
        rclpy.spin_once(probe, timeout_sec=0.1)

    # 1. Yaw turn sign test: command w=+0.5 for 2.0s
    print("\n--- 1. YAW TURN SIGN CHECK: Command w = +0.5 rad/s for 2.0 s ---")
    probe.reset_robot()
    time.sleep(0.5)
    for _ in range(10):
        rclpy.spin_once(probe, timeout_sec=0.05)

    yaw_init = probe.gz_yaw
    t_start = time.time()
    t_end = t_start + 2.0
    yaw_trace = []

    while time.time() < t_end:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        probe.pub_teleop.publish(cmd)
        rclpy.spin_once(probe, timeout_sec=0.02)
        yaw_trace.append({"t": round(time.time() - t_start, 3), "yaw": round(probe.gz_yaw, 4), "wz": round(probe.odom_wz, 4)})
        time.sleep(0.02)

    for _ in range(5):
        probe.pub_teleop.publish(Twist())
        rclpy.spin_once(probe, timeout_sec=0.05)

    yaw_final = probe.gz_yaw
    delta_yaw = (yaw_final - yaw_init + math.pi) % (2.0 * math.pi) - math.pi
    yaw_sign_positive = delta_yaw > 0.0
    print(f"  Initial yaw: {yaw_init:.4f} rad, Final yaw: {yaw_final:.4f} rad, Delta yaw: {delta_yaw:.4f} rad")
    print(f"  +w commands turn toward +Y (left / goal): {yaw_sign_positive}")

    # 2. Linear Step Response: 3 Runs (v_cmd = 0.30 m/s, w_cmd = 0.0)
    print("\n--- 2. LINEAR STEP RESPONSE (3 REPEATED RUNS: v_cmd = 0.30 m/s) ---")
    linear_runs = []
    step_duration = 3.0

    for r_idx in range(1, 4):
        probe.reset_robot()
        v_records = []
        t_start = time.time()
        t_end = t_start + step_duration

        while time.time() < t_end:
            now = time.time()
            elapsed = now - t_start
            cmd = Twist()
            cmd.linear.x = 0.30
            cmd.angular.z = 0.0
            probe.pub_teleop.publish(cmd)
            rclpy.spin_once(probe, timeout_sec=0.01)
            v_records.append({
                "t": elapsed, "odom_vx": probe.odom_vx, "gz_vx": probe.gz_vx
            })
            time.sleep(0.02)

        for _ in range(5):
            probe.pub_teleop.publish(Twist())
            rclpy.spin_once(probe, timeout_sec=0.02)

        times = np.array([r["t"] for r in v_records])
        odom_vxs = np.array([r["odom_vx"] for r in v_records])
        tail_idx = np.where(times >= (step_duration - 1.0))[0]
        ss_odom = float(np.mean(odom_vxs[tail_idx]))

        idx_90 = np.where(odom_vxs >= 0.90 * ss_odom)[0]
        t_90 = float(times[idx_90[0]]) if len(idx_90) > 0 else float("nan")

        idx_63 = np.where(odom_vxs >= 0.632 * ss_odom)[0]
        tau = float(times[idx_63[0]]) if len(idx_63) > 0 else float("nan")

        idx_5 = np.where(odom_vxs >= 0.05 * ss_odom)[0]
        lat = float(times[idx_5[0]]) if len(idx_5) > 0 else 0.0

        slip = (0.30 - ss_odom) / 0.30

        print(f"  Run {r_idx}: ss={ss_odom:.4f} m/s, t_90={t_90:.3f} s, tau={tau:.3f} s, lat={lat*1000.0:.1f} ms, slip={slip*100.0:.2f}%")
        linear_runs.append({
            "run": r_idx, "ss": ss_odom, "t_90": t_90, "tau": tau, "latency_ms": lat * 1000.0, "slip": slip
        })

    # 3. Angular Step Response: 3 Runs (w_cmd = 0.75 rad/s, v_cmd = 0.0)
    print("\n--- 3. ANGULAR STEP RESPONSE (3 REPEATED RUNS: w_cmd = 0.75 rad/s) ---")
    angular_runs = []

    for r_idx in range(1, 4):
        probe.reset_robot()
        w_records = []
        t_start = time.time()
        t_end = t_start + step_duration

        while time.time() < t_end:
            now = time.time()
            elapsed = now - t_start
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.75
            probe.pub_teleop.publish(cmd)
            rclpy.spin_once(probe, timeout_sec=0.01)
            w_records.append({
                "t": elapsed, "odom_wz": probe.odom_wz, "gz_wz": probe.gz_wz
            })
            time.sleep(0.02)

        for _ in range(5):
            probe.pub_teleop.publish(Twist())
            rclpy.spin_once(probe, timeout_sec=0.02)

        times_w = np.array([r["t"] for r in w_records])
        odom_wzs = np.array([r["odom_wz"] for r in w_records])
        tail_idx_w = np.where(times_w >= (step_duration - 1.0))[0]
        ss_odom_w = float(np.mean(odom_wzs[tail_idx_w]))

        idx_90_w = np.where(odom_wzs >= 0.90 * ss_odom_w)[0]
        t_90_w = float(times_w[idx_90_w[0]]) if len(idx_90_w) > 0 else float("nan")

        idx_63_w = np.where(odom_wzs >= 0.632 * ss_odom_w)[0]
        tau_w = float(times_w[idx_63_w[0]]) if len(idx_63_w) > 0 else float("nan")

        idx_5_w = np.where(odom_wzs >= 0.05 * ss_odom_w)[0]
        lat_w = float(times_w[idx_5_w[0]]) if len(idx_5_w) > 0 else 0.0

        slip_w = (0.75 - ss_odom_w) / 0.75

        print(f"  Run {r_idx}: ss={ss_odom_w:.4f} rad/s, t_90={t_90_w:.3f} s, tau={tau_w:.3f} s, lat={lat_w*1000.0:.1f} ms, slip={slip_w*100.0:.2f}%")
        angular_runs.append({
            "run": r_idx, "ss": ss_odom_w, "t_90": t_90_w, "tau": tau_w, "latency_ms": lat_w * 1000.0, "slip": slip_w
        })

    # Summary statistics
    lin_ss = [r["ss"] for r in linear_runs]
    lin_t90 = [r["t_90"] for r in linear_runs]
    lin_tau = [r["tau"] for r in linear_runs]
    lin_lat = [r["latency_ms"] for r in linear_runs]
    lin_slip = [r["slip"] for r in linear_runs]

    ang_ss = [r["ss"] for r in angular_runs]
    ang_t90 = [r["t_90"] for r in angular_runs]
    ang_tau = [r["tau"] for r in angular_runs]
    ang_lat = [r["latency_ms"] for r in angular_runs]
    ang_slip = [r["slip"] for r in angular_runs]

    summary = {
        "bearing_parity": bearing_result,
        "yaw_turn_check": {
            "cmd_w": 0.5,
            "duration_sec": 2.0,
            "yaw_init": yaw_init,
            "yaw_final": yaw_final,
            "delta_yaw": delta_yaw,
            "yaw_sign_positive": yaw_sign_positive
        },
        "linear_mean": {
            "cmd_v": 0.30,
            "ss_mean": float(np.mean(lin_ss)),
            "ss_std": float(np.std(lin_ss)),
            "t90_mean": float(np.mean(lin_t90)),
            "tau_mean": float(np.mean(lin_tau)),
            "lat_mean_ms": float(np.mean(lin_lat)),
            "slip_mean": float(np.mean(lin_slip))
        },
        "angular_mean": {
            "cmd_w": 0.75,
            "ss_mean": float(np.mean(ang_ss)),
            "ss_std": float(np.std(ang_ss)),
            "t90_mean": float(np.mean(ang_t90)),
            "tau_mean": float(np.mean(ang_tau)),
            "lat_mean_ms": float(np.mean(ang_lat)),
            "slip_mean": float(np.mean(ang_slip))
        },
        "linear_runs": linear_runs,
        "angular_runs": angular_runs
    }

    out_file = os.path.join(REPO_ROOT, "scratch/quick_checks_step_response_3runs.json")
    with open(out_file, "w") as f:
        json.dump(summary, f, indent=2)

    print(f"\nQuick checks and 3-run step response saved to: {out_file}")
    print(f"Linear Mean:  ss = {summary['linear_mean']['ss_mean']:.4f} m/s, tau = {summary['linear_mean']['tau_mean']:.3f} s, latency = {summary['linear_mean']['lat_mean_ms']:.1f} ms, slip = {summary['linear_mean']['slip_mean']*100:.2f}%")
    print(f"Angular Mean: ss = {summary['angular_mean']['ss_mean']:.4f} rad/s, tau = {summary['angular_mean']['tau_mean']:.3f} s, latency = {summary['angular_mean']['lat_mean_ms']:.1f} ms, slip = {summary['angular_mean']['slip_mean']*100:.2f}%")

    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_checks()
