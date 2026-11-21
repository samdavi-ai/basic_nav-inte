#!/usr/bin/env python3
"""
GRaCEmo ViRa — Control Experiment: AT-02 and AT-03 (5 runs each, DWB only)
with static identity localization (AMCL disabled).
Logs:
  - arrival (TF and Gazebo)
  - failure reason
  - collision_monitor stop/slow counts
  - position relative to door edges (x=-5.6 and x=-4.4, y=-1.0)
"""

import os
import sys
import time
import math
import json
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from nav2_msgs.msg import CollisionMonitorState
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


class ControlExperimentRunner(Node):
    def __init__(self):
        super().__init__("control_experiment_runner")
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.has_gz = False

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.supervisor_state = "UNKNOWN"
        self.stop_count = 0
        self.slow_count = 0

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_col, 10)

        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

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
                self.gz_x = float(p.position.x)
                self.gz_y = float(p.position.y)
                self.gz_yaw = float(math.atan2(siny, cosy))
                self.has_gz = True
                break

    def _on_odom(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def _on_col(self, msg: CollisionMonitorState):
        if msg.action_type == 1:
            self.stop_count += 1
        elif msg.action_type == 2:
            self.slow_count += 1

    def get_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            siny = 2.0 * (r.w * r.z + r.x * r.y)
            cosy = 1.0 - 2.0 * (r.y * r.y + r.z * r.z)
            yaw = math.atan2(siny, cosy)
            return float(tr.x), float(tr.y), float(yaw)
        except Exception:
            return None

    def reset_robot(self, x=0.0, y=0.0, yaw=0.0):
        # Stop
        zero = Twist()
        for _ in range(5):
            self.teleop_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)

        # Cancel supervisor
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL"})
        self.cmd_pub.publish(msg)
        time.sleep(0.2)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
        msg.data = json.dumps({"intent": "RESET"})
        self.cmd_pub.publish(msg)
        time.sleep(0.2)
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        # Teleport Gazebo
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = float(x)
        req.position.y = float(y)
        req.position.z = 0.04
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = math.sin(yaw / 2.0)
        req.orientation.w = math.cos(yaw / 2.0)
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)

        for _ in range(5):
            self.teleop_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)

        # Clear costmaps
        cmd = {"intent": "CLEAR_COSTMAPS"}
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        # Settle
        t_end = time.time() + 1.5
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def door_rel_pos(self, x: float, y: float):
        # Door opening: x in [-5.6, -4.4], y = -1.0
        # West edge: (-5.6, -1.0), East edge: (-4.4, -1.0)
        d_west = math.hypot(x - (-5.6), y - (-1.0))
        d_east = math.hypot(x - (-4.4), y - (-1.0))
        d_center = math.hypot(x - (-5.0), y - (-1.0))
        lat_off_wall = y - (-1.0)  # positive = in corridor (y>-1), negative = in kitchen (y<-1)
        return {
            "dist_west_jamb_m": round(d_west, 3),
            "dist_east_jamb_m": round(d_east, 3),
            "dist_door_center_m": round(d_center, 3),
            "lat_offset_wall_m": round(lat_off_wall, 3),
            "robot_xy": (round(x, 3), round(y, 3))
        }

    def run_at02(self, run_idx: int):
        print(f"\n--- Running AT-02 (Kitchen Navigation) Run {run_idx}/5 ---")
        self.reset_robot(0.0, 0.0, 0.0)
        self.stop_count = 0
        self.slow_count = 0

        target_x, target_y = -5.0, -2.5
        cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        t0 = time.time()
        max_wait = 65.0
        arrived = False
        fail_reason = "NONE"

        while time.time() - t0 < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED"):
                arrived = True
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                fail_reason = "SUPERVISOR_NAVIGATION_FAILURE"
                break

        if not arrived and fail_reason == "NONE":
            fail_reason = "TIMEOUT"

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        tf_p = self.get_tf_pose()
        gz_p = (self.gz_x, self.gz_y, self.gz_yaw)

        err_tf = math.hypot(tf_p[0] - target_x, tf_p[1] - target_y) if tf_p else 999.0
        err_gz = math.hypot(gz_p[0] - target_x, gz_p[1] - target_y)
        rel_door = self.door_rel_pos(gz_p[0], gz_p[1])

        # Dual arrival check: < 0.65m
        actual_arrived = (err_tf <= 0.65) and (err_gz <= 0.65) and (self.supervisor_state != "NAVIGATION_FAILURE")

        res = {
            "test": "AT-02",
            "run": run_idx,
            "arrived": actual_arrived,
            "supervisor_state": self.supervisor_state,
            "failure_reason": "NONE" if actual_arrived else fail_reason,
            "tf_pose": tf_p,
            "gz_pose": gz_p,
            "err_tf": round(err_tf, 3),
            "err_gz": round(err_gz, 3),
            "collision_stop_count": self.stop_count,
            "collision_slow_count": self.slow_count,
            "door_relative": rel_door,
            "duration_sec": round(time.time() - t0, 1)
        }
        print(f"AT-02 Run {run_idx}: Arrived={actual_arrived}, TF_err={err_tf:.2f}m, GZ_err={err_gz:.2f}m, Reason={res['failure_reason']}, StopCount={self.stop_count}")
        print(f"  Pos relative to door edges: {rel_door}")
        return res

    def run_at03(self, run_idx: int):
        print(f"\n--- Running AT-03 (Multi-Waypoint Circuit) Run {run_idx}/5 ---")
        self.reset_robot(0.0, 0.0, 0.0)
        self.stop_count = 0
        self.slow_count = 0

        cmd = {
            "intent": "START_MISSION",
            "mission_name": "Patrol Circuit",
            "waypoints": [
                {"name": "hallway_center", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0},
                {"name": "hallway_east", "x": 2.5, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0},
                {"name": "hallway_home", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 0.0}
            ]
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        t0 = time.time()
        max_wait = 90.0
        completed = False
        fail_reason = "NONE"

        while time.time() - t0 < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state == "MISSION_COMPLETED":
                completed = True
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                fail_reason = "SUPERVISOR_NAVIGATION_FAILURE"
                break

        if not completed and fail_reason == "NONE":
            fail_reason = "TIMEOUT"

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        tf_p = self.get_tf_pose()
        gz_p = (self.gz_x, self.gz_y, self.gz_yaw)

        err_home_tf = math.hypot(tf_p[0] - 0.0, tf_p[1] - 0.0) if tf_p else 999.0
        err_home_gz = math.hypot(gz_p[0] - 0.0, gz_p[1] - 0.0)
        rel_door = self.door_rel_pos(gz_p[0], gz_p[1])

        actual_arrived = completed and (err_home_tf <= 0.65) and (err_home_gz <= 0.65) and (self.supervisor_state != "NAVIGATION_FAILURE")

        res = {
            "test": "AT-03",
            "run": run_idx,
            "arrived": actual_arrived,
            "supervisor_state": self.supervisor_state,
            "failure_reason": "NONE" if actual_arrived else fail_reason,
            "tf_pose": tf_p,
            "gz_pose": gz_p,
            "err_tf": round(err_home_tf, 3),
            "err_gz": round(err_home_gz, 3),
            "collision_stop_count": self.stop_count,
            "collision_slow_count": self.slow_count,
            "door_relative": rel_door,
            "duration_sec": round(time.time() - t0, 1)
        }
        print(f"AT-03 Run {run_idx}: Arrived={actual_arrived}, TF_home_err={err_home_tf:.2f}m, GZ_home_err={err_home_gz:.2f}m, Reason={res['failure_reason']}, StopCount={self.stop_count}")
        return res


def main():
    rclpy.init()
    runner = ControlExperimentRunner()

    print("Waiting for Gazebo telemetry and TF...")
    t0 = time.time()
    while time.time() - t0 < 10.0:
        rclpy.spin_once(runner, timeout_sec=0.1)
        if runner.has_gz and runner.get_tf_pose() is not None:
            break

    if not runner.has_gz:
        print("ERROR: Telemetry timeout.")
        sys.exit(1)

    print("Starting Control Experiment (Static Localization Active)...")

    at02_results = []
    for i in range(1, 6):
        r = runner.run_at02(i)
        at02_results.append(r)

    at03_results = []
    for i in range(1, 6):
        r = runner.run_at03(i)
        at03_results.append(r)

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/control_experiment_results.json"
    with open(out_file, "w") as f:
        json.dump({"at02": at02_results, "at03": at03_results}, f, indent=2)

    print(f"\nAll 10 runs complete. Saved to {out_file}")
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

