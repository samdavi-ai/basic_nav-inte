#!/usr/bin/env python3
"""
GRaCEmo ViRa — AMCL Particle Cloud Capture and Spatial Distribution Plotter.
Captures /particle_cloud when error > 0.5 m, overlays on apartment_map,
and generates both a high-resolution PNG plot and numerical cluster analysis.
"""

import os
import sys
import time
import math
import json
from typing import Optional, Dict, Any, List

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import ParticleCloud
from std_srvs.srv import Empty
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV


class ParticleCloudAuditor(Node):
    def __init__(self):
        super().__init__("particle_cloud_auditor")

        self.gz_pose: Optional[Dict[str, float]] = None
        self.amcl_pose: Optional[Dict[str, float]] = None
        self.latest_cloud: Optional[ParticleCloud] = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(ParticleCloud, "/particle_cloud", self._on_particle_cloud, qos)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

    def _on_particle_cloud(self, msg: ParticleCloud):
        self.latest_cloud = msg

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


def plot_and_analyze(particles_data, gz_pose, amcl_pose, tf_pose, output_png, output_json):
    px = [p["x"] for p in particles_data]
    py = [p["y"] for p in particles_data]
    pw = [p["weight"] for p in particles_data]
    pyaw = [p["yaw"] for p in particles_data]

    # Load occupancy grid
    map_pgm = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm"
    map_origin = (-9.0, -6.5)
    map_res = 0.05

    img = Image.open(map_pgm)
    map_arr = np.array(img)
    h, w = map_arr.shape
    extent = [map_origin[0], map_origin[0] + w * map_res, map_origin[1], map_origin[1] + h * map_res]

    fig, ax = plt.subplots(figsize=(12, 9), dpi=150)
    # Origin of PGM in ROS image is lower-left
    ax.imshow(map_arr, cmap="gray", origin="lower", extent=extent, alpha=0.6)

    # Plot particles
    sc = ax.scatter(px, py, c=pw, cmap="viridis", s=15, alpha=0.7, label=f"Particles (N={len(px)})")
    plt.colorbar(sc, ax=ax, label="Particle Weight")

    # Plot True Pose (Gazebo)
    ax.plot(gz_pose["x"], gz_pose["y"], "ro", markersize=10, label="True Pose (Gazebo GT)")
    dx_gt = 0.5 * math.cos(gz_pose["yaw"])
    dy_gt = 0.5 * math.sin(gz_pose["yaw"])
    ax.arrow(gz_pose["x"], gz_pose["y"], dx_gt, dy_gt, color="red", width=0.04, head_width=0.15)

    # Plot AMCL Pose
    ax.plot(amcl_pose["x"], amcl_pose["y"], "bs", markersize=9, label="AMCL Filter Mean")
    dx_amcl = 0.5 * math.cos(amcl_pose["yaw"])
    dy_amcl = 0.5 * math.sin(amcl_pose["yaw"])
    ax.arrow(amcl_pose["x"], amcl_pose["y"], dx_amcl, dy_amcl, color="blue", width=0.04, head_width=0.15)

    if tf_pose:
        ax.plot(tf_pose["x"], tf_pose["y"], "g^", markersize=8, label="TF map->base_footprint")

    # Zoom in around robot region for clarity
    cx = (gz_pose["x"] + amcl_pose["x"]) / 2.0
    cy = (gz_pose["y"] + amcl_pose["y"]) / 2.0
    span_x = max(abs(gz_pose["x"] - amcl_pose["x"]) + 2.5, 4.0)
    span_y = max(abs(gz_pose["y"] - amcl_pose["y"]) + 2.0, 3.5)
    ax.set_xlim(cx - span_x, cx + span_x)
    ax.set_ylim(cy - span_y, cy + span_y)

    err_xy = math.hypot(amcl_pose["x"] - gz_pose["x"], amcl_pose["y"] - gz_pose["y"])
    err_yaw_deg = math.degrees(abs(amcl_pose["yaw"] - gz_pose["yaw"]))

    ax.set_title(f"AMCL Particle Distribution vs Ground Truth\n"
                 f"Position Error: {err_xy:.3f} m, Yaw Error: {err_yaw_deg:.1f}°", fontsize=14)
    ax.set_xlabel("Map X (m)")
    ax.set_ylabel("Map Y (m)")
    ax.legend(loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)

    plt.tight_layout()
    plt.savefig(output_png)
    plt.close()
    print(f"Saved plot to: {output_png}")

    # Statistical cluster analysis
    px_np = np.array(px)
    py_np = np.array(py)
    analysis = {
        "particle_count": len(px),
        "mean_x": float(np.mean(px_np)),
        "mean_y": float(np.mean(py_np)),
        "std_x": float(np.std(px_np)),
        "std_y": float(np.std(py_np)),
        "min_x": float(np.min(px_np)),
        "max_x": float(np.max(px_np)),
        "span_x": float(np.max(px_np) - np.min(px_np)),
        "span_y": float(np.max(py_np) - np.min(py_np)),
        "error_xy_m": float(err_xy),
        "gz_pose": gz_pose,
        "amcl_pose": amcl_pose,
        "tf_pose": tf_pose,
    }
    with open(output_json, "w") as f:
        json.dump(analysis, f, indent=2)
    print(f"Saved numerical analysis to: {output_json}")
    return analysis


def main():
    rclpy.init()
    node = ParticleCloudAuditor()

    print("ParticleCloudAuditor waiting for telemetry...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.gz_pose and node.latest_cloud:
            break

    # Wait or monitor for error > 0.5m
    print("Monitoring /particle_cloud and checking AMCL error...")
    captured = False
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.gz_pose and node.amcl_pose and node.latest_cloud:
            err = math.hypot(node.amcl_pose["x"] - node.gz_pose["x"], node.amcl_pose["y"] - node.gz_pose["y"])
            if err >= 0.50:
                print(f"Trigger condition MET: AMCL error = {err:.4f} m >= 0.50 m!")
                captured = True
                break

    if not captured and node.latest_cloud:
        print(f"Current AMCL error is {err:.4f}m. Capturing current cloud...")
        captured = True

    if captured and node.latest_cloud:
        cloud = node.latest_cloud
        particles = []
        for p in cloud.particles:
            q = p.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            particles.append({
                "x": float(p.pose.position.x),
                "y": float(p.pose.position.y),
                "z": float(p.pose.position.z),
                "yaw": float(yaw),
                "weight": float(p.weight),
            })

        out_png = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/particle_cloud_error_0.5m.png"
        out_json = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/particle_cloud_analysis.json"
        analysis = plot_and_analyze(particles, node.gz_pose, node.amcl_pose, node.get_tf_map_pose(), out_png, out_json)
        print("Analysis completed successfully.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

