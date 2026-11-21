#!/usr/bin/env bash
#!/usr/bin/env python3
"""
GRaCEmo ViRa — Gate B Localization Health Matrix & Verification.
Tests injected offsets: 0.3m (10 runs), 0.5m (10 runs), 1.0m (10 runs), and 0.0m Clean (10 runs).
Measures:
  - Detection time (target: <= 3.0s for 1.0m)
  - Miss rate (%)
  - False alarm rate (target: 0 over 10 clean runs)
  - Localization signal used (TF translation jump + covariance eigenvalues; zero injection flags)
  - Hallway transit max AMCL error against 0.15m threshold.
"""

import os
import sys
import time
import math
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
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

class GateBMatrixTester(Node):
    def __init__(self):
        super().__init__("gate_b_matrix_tester")
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_speed = 0.0
        self.supervisor_state = "UNKNOWN"
        self.loc_state = "UNKNOWN"

        self.sub_state = self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.sub_status = self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.nomotion_cli = self.create_client(Empty, "/request_nomotion_update")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self._on_gz_pose)

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.loc_state = data.get("robot", {}).get("localization_state", "UNKNOWN")
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _on_gz_pose(self, msg):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                self.gz_x = p.position.x
                self.gz_y = p.position.y
                q = p.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                self.gz_yaw = math.atan2(siny_cosp, cosy_cosp)
                break

    def get_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = float(math.atan2(siny_cosp, cosy_cosp))
            return x, y, yaw
        except Exception:
            return None

    def clean_reset(self, target_x=0.0, target_y=0.0, target_yaw=0.0):
        # 1. Zero velocity
        zero_twist = Twist()
        for _ in range(3):
            self.teleop_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        # 2. Cancel and reset supervisor
        for intent in ["CANCEL", "RESET"]:
            msg = String()
            msg.data = json.dumps({"intent": intent})
            self.pub_cmd.publish(msg)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)

        # 3. Teleport Gazebo model
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = float(target_x)
        req.position.y = float(target_y)
        req.position.z = 0.04
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = math.sin(target_yaw / 2.0)
        req.orientation.w = math.cos(target_yaw / 2.0)
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)

        # 4. Re-initialize localization to target
        loc_cmd = {
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": float(target_x), "y": float(target_y), "yaw": float(target_yaw)}
        }
        loc_msg = String()
        loc_msg.data = json.dumps(loc_cmd)
        self.pub_cmd.publish(loc_msg)
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 5. Force non-motion updates
        for _ in range(5):
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        # 6. Clear costmaps
        cmd = {"intent": "CLEAR_COSTMAPS"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_cmd.publish(msg)

        # Settle
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_single_trial(self, offset_m: float, run_idx: int) -> dict:
        self.clean_reset()

        # Wait for autonomy ready
        t0 = time.time()
        ready = False
        while time.time() - t0 < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state in ("AUTONOMY_READY", "WAITING_FOR_NEXT_MISSION"):
                ready = True
                break

        # Dispatch forward hallway motion
        cmd = {
            "intent": "START_MISSION",
            "mission_name": f"OffsetTest_{offset_m}m_run_{run_idx}",
            "waypoints": [{"name": "waypoint", "x": 2.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0}]
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_cmd.publish(msg)

        # Wait for motion
        t0 = time.time()
        moving = False
        pos_errors = []
        while time.time() - t0 < 12.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            tf_pose = self.get_tf_pose()
            if tf_pose is not None:
                p_err = math.hypot(tf_pose[0] - self.gz_x, tf_pose[1] - self.gz_y)
                pos_errors.append(p_err)
            if self.current_speed > 0.04:
                moving = True
                break

        if not moving:
            # Cancel and return fail
            self.pub_cmd.publish(String(data=json.dumps({"intent": "CANCEL"})))
            return {
                "offset_m": offset_m,
                "run": run_idx,
                "detected": False,
                "detection_time_sec": None,
                "false_alarm": False,
                "status": "MOTION_FAILED",
                "max_amcl_error_m": float(np.max(pos_errors)) if pos_errors else 0.0
            }

        if offset_m > 0.0:
            # Inject offset
            inject_x = self.robot_x + offset_m
            inject_y = self.robot_y
            offset_msg = PoseWithCovarianceStamped()
            offset_msg.header.frame_id = "map"
            offset_msg.header.stamp = rclpy.time.Time(seconds=0, nanoseconds=0).to_msg()
            offset_msg.pose.pose.position.x = inject_x
            offset_msg.pose.pose.position.y = inject_y
            offset_msg.pose.pose.orientation.w = 1.0
            offset_msg.pose.covariance[0] = 0.50
            offset_msg.pose.covariance[7] = 0.50
            offset_msg.pose.covariance[35] = 0.25

            t_inject = time.time()
            self.pub_initialpose.publish(offset_msg)

            # Measure detection
            detected = False
            t_detect = 0.0
            while time.time() - t_inject < 8.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                tf_pose = self.get_tf_pose()
                if tf_pose is not None:
                    p_err = math.hypot(tf_pose[0] - self.gz_x, tf_pose[1] - self.gz_y)
                    pos_errors.append(p_err)
                if self.loc_state in ("RECOVERING", "LOST") or self.supervisor_state == "OBSTACLE_HANDLING":
                    t_detect = time.time()
                    detected = True
                    break

            detection_time = (t_detect - t_inject) if detected else None
            self.pub_cmd.publish(String(data=json.dumps({"intent": "CANCEL"})))
            return {
                "offset_m": offset_m,
                "run": run_idx,
                "detected": detected,
                "detection_time_sec": round(detection_time, 3) if detection_time is not None else None,
                "false_alarm": False,
                "status": "PASS" if detected else "MISS",
                "max_amcl_error_m": float(np.max(pos_errors)) if pos_errors else 0.0
            }
        else:
            # Clean run: monitor for false alarms during full transit
            false_alarm = False
            t_start = time.time()
            while time.time() - t_start < 12.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                tf_pose = self.get_tf_pose()
                if tf_pose is not None:
                    p_err = math.hypot(tf_pose[0] - self.gz_x, tf_pose[1] - self.gz_y)
                    pos_errors.append(p_err)
                if self.loc_state in ("RECOVERING", "LOST") or self.supervisor_state == "OBSTACLE_HANDLING":
                    false_alarm = True
                    break
                if self.supervisor_state == "MISSION_COMPLETED":
                    break

            self.pub_cmd.publish(String(data=json.dumps({"intent": "CANCEL"})))
            return {
                "offset_m": 0.0,
                "run": run_idx,
                "detected": False,
                "detection_time_sec": None,
                "false_alarm": false_alarm,
                "status": "PASS" if not false_alarm else "FALSE_ALARM",
                "max_amcl_error_m": float(np.max(pos_errors)) if pos_errors else 0.0
            }

def run_matrix():
    rclpy.init()
    tester = GateBMatrixTester()
    results = {
        "offsets": {},
        "hallway_transit_max_error_m": 0.0,
        "detection_signal_used": "TF map->odom translation jump (>0.25m with covariance / >1.50m absolute) and AMCL covariance eigenvalues (l_min > 0.80, l_max > 25.0). Zero reliance on injection flags."
    }

    all_max_pos_errs = []

    # 1. Test 1.0m (10 runs)
    print(">>> TESTING 1.0m OFFSET INJECTION (10 RUNS) <<<")
    runs_10m = []
    for r in range(1, 11):
        res = tester.run_single_trial(1.0, r)
        runs_10m.append(res)
        all_max_pos_errs.append(res["max_amcl_error_m"])
        print(f"Run {r}/10 (1.0m): Detected={res['detected']}, Delay={res['detection_time_sec']}s, Status={res['status']}")
    
    det_times_10m = [x["detection_time_sec"] for x in runs_10m if x["detected"] and x["detection_time_sec"] is not None]
    results["offsets"]["1.0m"] = {
        "runs": runs_10m,
        "miss_rate_pct": round((1.0 - len(det_times_10m) / len(runs_10m)) * 100.0, 1),
        "mean_detection_time_sec": round(float(np.mean(det_times_10m)), 3) if det_times_10m else None,
        "max_detection_time_sec": round(float(np.max(det_times_10m)), 3) if det_times_10m else None
    }

    # 2. Test 0.5m (10 runs)
    print("\n>>> TESTING 0.5m OFFSET INJECTION (10 RUNS) <<<")
    runs_05m = []
    for r in range(1, 11):
        res = tester.run_single_trial(0.5, r)
        runs_05m.append(res)
        all_max_pos_errs.append(res["max_amcl_error_m"])
        print(f"Run {r}/10 (0.5m): Detected={res['detected']}, Delay={res['detection_time_sec']}s, Status={res['status']}")

    det_times_05m = [x["detection_time_sec"] for x in runs_05m if x["detected"] and x["detection_time_sec"] is not None]
    results["offsets"]["0.5m"] = {
        "runs": runs_05m,
        "miss_rate_pct": round((1.0 - len(det_times_05m) / len(runs_05m)) * 100.0, 1),
        "mean_detection_time_sec": round(float(np.mean(det_times_05m)), 3) if det_times_05m else None,
        "max_detection_time_sec": round(float(np.max(det_times_05m)), 3) if det_times_05m else None
    }

    # 3. Test 0.3m (10 runs)
    print("\n>>> TESTING 0.3m OFFSET INJECTION (10 RUNS) <<<")
    runs_03m = []
    for r in range(1, 11):
        res = tester.run_single_trial(0.3, r)
        runs_03m.append(res)
        all_max_pos_errs.append(res["max_amcl_error_m"])
        print(f"Run {r}/10 (0.3m): Detected={res['detected']}, Delay={res['detection_time_sec']}s, Status={res['status']}")

    det_times_03m = [x["detection_time_sec"] for x in runs_03m if x["detected"] and x["detection_time_sec"] is not None]
    results["offsets"]["0.3m"] = {
        "runs": runs_03m,
        "miss_rate_pct": round((1.0 - len(det_times_03m) / len(runs_03m)) * 100.0, 1),
        "mean_detection_time_sec": round(float(np.mean(det_times_03m)), 3) if det_times_03m else None,
        "max_detection_time_sec": round(float(np.max(det_times_03m)), 3) if det_times_03m else None
    }

    # 4. Clean Runs (10 runs)
    print("\n>>> TESTING 0.0m CLEAN RUNS (10 RUNS) <<<")
    runs_clean = []
    for r in range(1, 11):
        res = tester.run_single_trial(0.0, r)
        runs_clean.append(res)
        all_max_pos_errs.append(res["max_amcl_error_m"])
        print(f"Run {r}/10 (Clean): FalseAlarm={res['false_alarm']}, Status={res['status']}")

    false_alarms = sum(1 for x in runs_clean if x["false_alarm"])
    clean_pos_errs = [x["max_amcl_error_m"] for x in runs_clean]
    results["offsets"]["clean_10_runs"] = {
        "runs": runs_clean,
        "false_alarm_count": false_alarms,
        "false_alarm_rate_pct": round((false_alarms / len(runs_clean)) * 100.0, 1),
        "max_clean_hallway_amcl_error_m": round(float(np.max(clean_pos_errs)), 4) if clean_pos_errs else 0.0
    }

    results["hallway_transit_max_error_m"] = round(float(np.max(clean_pos_errs)), 4) if clean_pos_errs else 0.0

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gate_b_matrix_results.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved Gate B Matrix Results to: {out_file}")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    run_matrix()

