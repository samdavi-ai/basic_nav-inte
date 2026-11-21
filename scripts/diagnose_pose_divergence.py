#!/usr/bin/env python3
"""
Diagnostic script for Phase 2: Pose Divergence Analysis
Captures simultaneous timestamps and positions from:
  1. Gazebo model ground-truth
  2. /odom topic
  3. TF odom -> base_footprint
  4. TF map -> base_footprint
  5. /amcl_pose
"""

import os
import sys
import time
import math
import json
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV

class PoseDivergenceDiagnoser(Node):
    def __init__(self):
        super().__init__("pose_divergence_diagnoser")

        self.gz_pose = None
        self.gz_time = 0.0
        self.odom_pose = None
        self.odom_time = 0.0
        self.amcl_pose = None
        self.amcl_time = 0.0

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

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
                self.gz_pose = (p.position.x, p.position.y, p.position.z, yaw)
                self.gz_time = time.time()
                break

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.odom_pose = (p.x, p.y, p.z, yaw)
        self.odom_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.amcl_pose = (p.x, p.y, p.z, yaw)
        self.amcl_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def sample_all(self):
        rclpy.spin_once(self, timeout_sec=0.1)

        tf_odom_base = None
        try:
            t = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            yaw = math.atan2(2.0*(r.w*r.z + r.x*r.y), 1.0 - 2.0*(r.y*r.y + r.z*r.z))
            tf_odom_base = (tr.x, tr.y, tr.z, yaw)
        except Exception:
            pass

        tf_map_base = None
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            yaw = math.atan2(2.0*(r.w*r.z + r.x*r.y), 1.0 - 2.0*(r.y*r.y + r.z*r.z))
            tf_map_base = (tr.x, tr.y, tr.z, yaw)
        except Exception:
            pass

        return {
            "wall_time": time.time(),
            "gazebo_ground_truth": self.gz_pose,
            "ros_odom_topic": self.odom_pose,
            "tf_odom_to_base_footprint": tf_odom_base,
            "tf_map_to_base_footprint": tf_map_base,
            "amcl_pose": self.amcl_pose
        }

def main():
    rclpy.init()
    node = PoseDivergenceDiagnoser()

    # Collect for 2 seconds
    t0 = time.time()
    while time.time() - t0 < 2.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    data = node.sample_all()
    print("="*70)
    print("PHASE 2 POSE COMPARISON AND DIVERGENCE DIAGNOSTIC")
    print("="*70)
    for k, v in data.items():
        if isinstance(v, tuple):
            print(f"  {k:<30}: X={v[0]:+7.3f}, Y={v[1]:+7.3f}, Z={v[2]:+7.3f}, Yaw={v[3]:+6.2f} rad ({math.degrees(v[3]):+6.1f}°)")
        else:
            print(f"  {k:<30}: {v}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

