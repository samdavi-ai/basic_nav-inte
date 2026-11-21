#!/usr/bin/env python3
"""
GRaCEmo ViRa — Ground-Truth Navigation & Coordinate Alignment Logger.
Logs at 2 Hz:
  1. Gazebo True Pose (world frame via Gazebo Transport)
  2. /odom Pose (odometry frame)
  3. /amcl_pose (map frame)
  4. TF map -> base_footprint
  5. TF map -> odom
  6. TF odom -> base_footprint
  7. Supervisor Belief (from /autonomy/mission_status)
  8. Map Origin vs Gazebo World Origin & Spawn Pose
"""

import os
import sys
import time
import math
import json
from datetime import datetime
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV


class GroundTruthLogger(Node):
    def __init__(self, output_file: str = "ground_truth_trace.jsonl"):
        super().__init__("ground_truth_logger")
        self.output_file = output_file
        os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

        # State storage
        self.gz_pose: Optional[Dict[str, float]] = None
        self.gz_time: float = 0.0
        self.odom_pose: Optional[Dict[str, float]] = None
        self.amcl_pose: Optional[Dict[str, float]] = None
        self.supervisor_pose: Optional[Dict[str, float]] = None
        self.supervisor_state: str = "UNKNOWN"

        # Map metadata
        self.map_origin = (-9.0, -6.5, 0.0)
        self.gazebo_spawn_pose = (0.0, 0.0, 0.025, 0.0)

        # Subscriptions
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(String, "/autonomy/mission_status", self._on_mission_status, 10)
        self.create_subscription(String, "/autonomy/state", self._on_autonomy_state, 10)

        # TF Buffer & Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport Node
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_pose)

        self.get_logger().info(f"GroundTruthLogger initialized. Logging to {self.output_file}")

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
                self.gz_time = time.time()
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

    def _on_mission_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            if "robot" in data and "pose" in data["robot"]:
                self.supervisor_pose = data["robot"]["pose"]
            if "state" in data:
                self.supervisor_state = data["state"]
        except Exception:
            pass

    def _on_autonomy_state(self, msg: String):
        self.supervisor_state = msg.data

    def _lookup_tf(self, target_frame: str, source_frame: str) -> Optional[Dict[str, float]]:
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z))
            return {
                "x": float(tr.x),
                "y": float(tr.y),
                "z": float(tr.z),
                "yaw": float(yaw),
            }
        except Exception:
            return None

    def sample_tick(self) -> Dict[str, Any]:
        iso_ts = datetime.utcnow().isoformat() + "Z"
        tf_map_base = self._lookup_tf("map", "base_footprint")
        tf_map_odom = self._lookup_tf("map", "odom")
        tf_odom_base = self._lookup_tf("odom", "base_footprint")

        sample = {
            "timestamp": iso_ts,
            "wall_sec": time.time(),
            "supervisor_state": self.supervisor_state,
            "gazebo_world_pose": self.gz_pose,
            "ros_odom_pose": self.odom_pose,
            "amcl_pose": self.amcl_pose,
            "tf_map_to_base_footprint": tf_map_base,
            "tf_map_to_odom": tf_map_odom,
            "tf_odom_to_base_footprint": tf_odom_base,
            "supervisor_belief_pose": self.supervisor_pose,
            "map_origin": self.map_origin,
            "gazebo_spawn_pose": self.gazebo_spawn_pose,
        }

        # Compute offsets and mismatches
        diffs = {}
        if self.gz_pose and tf_map_base:
            diffs["gz_world_vs_tf_map_xy_dist"] = math.hypot(
                self.gz_pose["x"] - tf_map_base["x"],
                self.gz_pose["y"] - tf_map_base["y"]
            )
            diffs["gz_world_vs_tf_map_yaw_diff_deg"] = math.degrees(
                abs((self.gz_pose["yaw"] - tf_map_base["yaw"] + math.pi) % (2.0 * math.pi) - math.pi)
            )

        if self.amcl_pose and tf_map_base:
            diffs["amcl_vs_tf_map_xy_dist"] = math.hypot(
                self.amcl_pose["x"] - tf_map_base["x"],
                self.amcl_pose["y"] - tf_map_base["y"]
            )

        if self.supervisor_pose and tf_map_base:
            diffs["supervisor_vs_tf_map_xy_dist"] = math.hypot(
                self.supervisor_pose["x"] - tf_map_base["x"],
                self.supervisor_pose["y"] - tf_map_base["y"]
            )

        sample["frame_mismatches"] = diffs

        # Append to log file
        with open(self.output_file, "a") as f:
            f.write(json.dumps(sample) + "\n")

        return sample


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Ground Truth Navigation Logger")
    parser.add_argument("--output", type=str, default="/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/ground_truth_trace.jsonl")
    parser.add_argument("--duration", type=float, default=10.0, help="Run duration in seconds")
    parser.add_argument("--hz", type=float, default=2.0, help="Logging frequency in Hz")
    args = parser.parse_args()

    rclpy.init()
    logger_node = GroundTruthLogger(output_file=args.output)

    dt = 1.0 / args.hz
    t_end = time.time() + args.duration

    print(f"Logging ground-truth at {args.hz} Hz for {args.duration}s...")
    last_tick = time.time()

    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(logger_node, timeout_sec=0.02)
        if time.time() - last_tick >= dt:
            sample = logger_node.sample_tick()
            last_tick = time.time()

            # Print concise 1-line log
            gz_p = sample["gazebo_world_pose"] or {"x": 0.0, "y": 0.0, "yaw": 0.0}
            tf_m = sample["tf_map_to_base_footprint"] or {"x": 0.0, "y": 0.0, "yaw": 0.0}
            od_p = sample["ros_odom_pose"] or {"x": 0.0, "y": 0.0, "yaw": 0.0}
            am_p = sample["amcl_pose"] or {"x": 0.0, "y": 0.0, "yaw": 0.0}
            sb_p = sample["supervisor_belief_pose"] or {"x": 0.0, "y": 0.0, "yaw": 0.0}
            mismatches = sample["frame_mismatches"]

            gz_map_diff = mismatches.get("gz_world_vs_tf_map_xy_dist", -1.0)
            print(
                f"[{sample['timestamp'][-13:-1]}] State:{sample['supervisor_state']:<16} | "
                f"GZ:({gz_p['x']:+.2f},{gz_p['y']:+.2f}) | "
                f"TF_map:({tf_m['x']:+.2f},{tf_m['y']:+.2f}) | "
                f"Odom:({od_p['x']:+.2f},{od_p['y']:+.2f}) | "
                f"AMCL:({am_p['x']:+.2f},{am_p['y']:+.2f}) | "
                f"Sup:({sb_p['x']:+.2f},{sb_p['y']:+.2f}) | "
                f"GZ-TF_dist:{gz_map_diff:.3f}m"
            )

    logger_node.destroy_node()
    rclpy.shutdown()
    print(f"Logging complete. Saved to {args.output}")


if __name__ == "__main__":
    main()

