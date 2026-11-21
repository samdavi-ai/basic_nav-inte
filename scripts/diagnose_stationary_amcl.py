#!/usr/bin/env python3
"""
GRaCEmo ViRa — Stationary AMCL-vs-Gazebo 10 Hz / 60 s Diagnostic (5 repeats)
Logs AMCL, TF map->base_footprint, and Gazebo true pose at 10 Hz for 60 s while
stationary at (0,0) across 5 trials, starting immediately after initial pose publish.
Analyzes error magnitude and checks for ~0.95 m discrete jumps.
"""

import os
import sys
import time
import math
import json
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
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


class StationaryAMCLDiagnoser(Node):
    def __init__(self):
        super().__init__("stationary_amcl_diagnoser")

        # Telemetry storage
        self.gz_pose: Optional[Dict[str, float]] = None
        self.amcl_pose: Optional[Dict[str, float]] = None
        self.amcl_cov: Optional[List[float]] = None
        self.odom_pose: Optional[Dict[str, float]] = None

        # ROS subscriptions & publishers
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.nomotion_cli = self.create_client(Empty, "/request_nomotion_update")

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_pose)

    def _on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                q = p.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
                self.gz_pose = {
                    "x": float(p.position.x),
                    "y": float(p.position.y),
                    "z": float(p.position.z),
                    "yaw": float(yaw),
                }
                break

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.odom_pose = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "yaw": float(yaw),
        }

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.amcl_pose = {
            "x": float(p.x),
            "y": float(p.y),
            "z": float(p.z),
            "yaw": float(yaw),
        }
        self.amcl_cov = list(msg.pose.covariance)

    def get_tf_map_pose(self) -> Optional[Dict[str, float]]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            siny = 2.0 * (r.w * r.z + r.x * r.y)
            cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            yaw = math.atan2(siny, cosy)
            return {"x": float(tr.x), "y": float(tr.y), "yaw": float(yaw)}
        except Exception:
            return None

    def reset_to_origin(self):
        # 1. Zero velocity
        zero_twist = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        # 2. Reset Gazebo pose
        try:
            req = GzPose()
            req.name = "gracemo_vira"
            req.position.x = 0.0
            req.position.y = 0.0
            req.position.z = 0.04
            req.orientation.x = 0.0
            req.orientation.y = 0.0
            req.orientation.z = 0.0
            req.orientation.w = 1.0
            self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)
        except Exception as e:
            self.get_logger().warn(f"Gazebo set_pose failed: {e}")

        for _ in range(5):
            self.cmd_vel_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        # 3. Publish initialpose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.068
        pose_msg.pose.covariance = cov

        self.initial_pose_pub.publish(pose_msg)
        rclpy.spin_once(self, timeout_sec=0.1)

        # Call nomotion update
        if self.nomotion_cli.service_is_ready():
            try:
                self.nomotion_cli.call_async(Empty.Request())
            except Exception:
                pass


def run_trials():
    rclpy.init()
    node = StationaryAMCLDiagnoser()

    # Wait for initial telemetry
    print("Waiting for Gazebo telemetry...")
    t_start = time.time()
    while time.time() - t_start < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.gz_pose is not None:
            break

    if node.gz_pose is None:
        print("ERROR: Could not receive Gazebo telemetry!")
        sys.exit(1)

    all_trials = []
    num_trials = 5
    duration_sec = 60.0
    dt = 0.1  # 10 Hz

    print(f"\n=======================================================")
    print(f"Starting Stationary AMCL Drift Diagnostic ({num_trials} Trials x {duration_sec}s @ 10Hz)")
    print(f"=======================================================\n")

    for trial_idx in range(1, num_trials + 1):
        print(f"--- Starting Trial {trial_idx}/{num_trials} ---")
        node.reset_to_origin()

        # Wait 0.5s to let initial pose settle
        time.sleep(0.5)

        trial_samples = []
        t0 = time.time()
        next_tick = t0

        jump_events = []
        prev_err = None

        sample_idx = 0
        total_samples = int(duration_sec / dt)

        while sample_idx < total_samples:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.time()
            if now >= next_tick:
                t_rel = round(now - t0, 3)
                next_tick += dt
                sample_idx += 1

                tf_pose = node.get_tf_map_pose()
                gz = dict(node.gz_pose) if node.gz_pose else {"x": 0.0, "y": 0.0, "yaw": 0.0}
                amcl = dict(node.amcl_pose) if node.amcl_pose else dict(gz)

                # Errors
                err_amcl_xy = math.hypot(amcl["x"] - gz["x"], amcl["y"] - gz["y"])
                err_amcl_yaw = abs(wrap_pi(amcl["yaw"] - gz["yaw"]))

                if tf_pose:
                    err_tf_xy = math.hypot(tf_pose["x"] - gz["x"], tf_pose["y"] - gz["y"])
                    err_tf_yaw = abs(wrap_pi(tf_yaw := tf_pose["yaw"] - gz["yaw"]))
                else:
                    err_tf_xy = err_amcl_xy
                    err_tf_yaw = err_amcl_yaw

                sample = {
                    "t_rel": t_rel,
                    "gz_pose": gz,
                    "amcl_pose": amcl,
                    "tf_pose": tf_pose,
                    "err_amcl_xy": round(err_amcl_xy, 4),
                    "err_amcl_yaw": round(err_amcl_yaw, 4),
                    "err_tf_xy": round(err_tf_xy, 4),
                    "err_tf_yaw": round(err_tf_yaw, 4),
                }
                trial_samples.append(sample)

                # Check for step jump between consecutive samples
                if prev_err is not None:
                    delta_err = err_amcl_xy - prev_err
                    if abs(delta_err) >= 0.30:  # Significant step jump
                        jump_events.append({
                            "t_rel": t_rel,
                            "delta_m": round(delta_err, 4),
                            "from_m": round(prev_err, 4),
                            "to_m": round(err_amcl_xy, 4),
                        })
                prev_err = err_amcl_xy

        # Trial statistics
        amcl_errors = [s["err_amcl_xy"] for s in trial_samples]
        tf_errors = [s["err_tf_xy"] for s in trial_samples]
        max_amcl_err = max(amcl_errors)
        mean_amcl_err = sum(amcl_errors) / len(amcl_errors)
        max_tf_err = max(tf_errors)
        mean_tf_err = sum(tf_errors) / len(tf_errors)

        trial_summary = {
            "trial_id": trial_idx,
            "samples_count": len(trial_samples),
            "max_amcl_err_m": round(max_amcl_err, 4),
            "mean_amcl_err_m": round(mean_amcl_err, 4),
            "max_tf_err_m": round(max_tf_err, 4),
            "mean_tf_err_m": round(mean_tf_err, 4),
            "jump_events": jump_events,
            "samples": trial_samples,
        }
        all_trials.append(trial_summary)

        print(f"Trial {trial_idx} complete: Max AMCL Err: {max_amcl_err:.4f}m, Mean: {mean_amcl_err:.4f}m, "
              f"Max TF Err: {max_tf_err:.4f}m, Jumps: {len(jump_events)}")
        if jump_events:
            for j in jump_events:
                print(f"  -> Jump at t={j['t_rel']}s: delta={j['delta_m']:+.4f}m ({j['from_m']:.4f} -> {j['to_m']:.4f})")

    # Output file
    out_dir = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch"
    os.makedirs(out_dir, exist_ok=True)
    out_file = os.path.join(out_dir, "stationary_amcl_trials.json")
    with open(out_file, "w") as f:
        json.dump(all_trials, f, indent=2)

    print("\n=======================================================")
    print("STATIONARY AMCL DIAGNOSTIC SUMMARY REPORT")
    print("=======================================================")
    print(f"Total Trials: {num_trials}")
    print(f"Duration per trial: {duration_sec} s @ 10 Hz ({len(all_trials[0]['samples'])} samples/trial)")
    print(f"Saved full dataset to: {out_file}\n")

    print("| Trial | Max AMCL Err (m) | Mean AMCL Err (m) | Max TF Err (m) | Mean TF Err (m) | Discrete Jumps (>0.30m) |")
    print("|---|---|---|---|---|---|")
    total_jumps = 0
    all_max_errs = []
    for t in all_trials:
        j_count = len(t["jump_events"])
        total_jumps += j_count
        all_max_errs.append(t["max_amcl_err_m"])
        j_desc = f"{j_count} jumps" if j_count > 0 else "None (stable)"
        print(f"| Trial {t['trial_id']} | {t['max_amcl_err_m']:.4f} | {t['mean_amcl_err_m']:.4f} | {t['max_tf_err_m']:.4f} | {t['mean_tf_err_m']:.4f} | {j_desc} |")

    overall_max = max(all_max_errs)
    print("\nKey Diagnostic Questions:")
    print(f"1. What is the maximum error when stationary at (0,0)? -> {overall_max:.4f} m")
    print(f"2. Does it jump in ~0.95 m steps when stationary at (0,0)? -> ", end="")
    if any(any(0.85 <= abs(j["delta_m"]) <= 1.05 for j in t["jump_events"]) for t in all_trials):
        print("YES, discrete ~0.95m jumps were detected while stationary.")
    else:
        print(f"NO, no ~0.95m jumps observed while stationary (total discrete jumps across all trials: {total_jumps}).")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_trials()

