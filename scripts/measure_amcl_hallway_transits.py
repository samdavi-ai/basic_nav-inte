#!/usr/bin/env python3
"""
Track 3: Measure AMCL-vs-Gazebo error over 5 hallway transits with fixed reset procedure.
"""

import os
import sys
import math
import time
import json
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class AMCLTransitTester(Node):
    def __init__(self):
        super().__init__("amcl_transit_tester")
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0

        self.amcl_x = 0.0
        self.amcl_y = 0.0
        self.amcl_yaw = 0.0
        self.has_amcl = False

        self.supervisor_state = "UNKNOWN"

        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(String, "/autonomy/state", self.on_state, 10)

        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.teleop_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.nomotion_cli = self.create_client(Empty, "/request_nomotion_update")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self.on_gz_pose)

    def on_amcl(self, msg):
        self.amcl_x = msg.pose.pose.position.x
        self.amcl_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.amcl_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_amcl = True

    def on_state(self, msg):
        self.supervisor_state = msg.data

    def on_gz_pose(self, msg):
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

    def fixed_reset_procedure(self, target_x=0.0, target_y=0.0, target_yaw=0.0):
        """Authoritative Fixed Reset Procedure with tight covariance and forced non-motion convergence."""
        # 1. Zero velocity
        zero_twist = Twist()
        for _ in range(5):
            self.teleop_pub.publish(zero_twist)
            rclpy.spin_once(self, timeout_sec=0.02)

        # 2. Cancel and reset supervisor
        for intent in ["CANCEL", "RESET"]:
            msg = String()
            msg.data = json.dumps({"intent": intent})
            self.cmd_pub.publish(msg)
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

        # 4. Publish initial pose with tight covariance via coordinator
        loc_cmd = {
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": float(target_x), "y": float(target_y), "yaw": float(target_yaw)}
        }
        loc_msg = String()
        loc_msg.data = json.dumps(loc_cmd)
        self.cmd_pub.publish(loc_msg)
        time.sleep(0.3)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        # 5. Force 5 non-motion updates to converge particle filter
        for _ in range(5):
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # 6. Clear costmaps
        cmd = {"intent": "CLEAR_COSTMAPS"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        # Settle
        time.sleep(1.0)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)


def run_amcl_transits(num_transits: int = 5, out_filename: str = "track3_amcl_transits.json"):
    rclpy.init()
    tester = AMCLTransitTester()

    print(f"\n================== AMCL {num_transits}-TRANSIT EVALUATION ==================")
    print("Testing with fixed reset procedure: tight covariance (0.01) + forced non-motion updates.")

    transits = []

    for t_idx in range(num_transits):
        print(f"\n--- Hallway Transit {t_idx + 1} of {num_transits} ---")
        print("Executing fixed reset procedure...")
        tester.fixed_reset_procedure(0.0, 0.0, 0.0)

        # Verify initial error after reset
        init_tf = tester.get_tf_pose() or (tester.amcl_x, tester.amcl_y, tester.amcl_yaw)
        init_pos_err = math.hypot(init_tf[0] - tester.gz_x, init_tf[1] - tester.gz_y)
        init_yaw_err = abs(wrap_pi(init_tf[2] - tester.gz_yaw))
        print(f"Post-Reset Error: Pos = {init_pos_err*1000:.1f} mm, Yaw = {init_yaw_err*180/math.pi:.2f} deg")

        # Command hallway transit: drive forward 1.5 m along x-axis in central hallway
        # (Waypoints: (1.5, 0.0) -> (0.0, 0.0))
        nav_cmd = {
            "intent": "START_MISSION",
            "mission_name": f"Hallway Transit {t_idx+1}",
            "waypoints": [
                {"name": "hallway_forward", "x": 1.5, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0},
                {"name": "hallway_origin", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 0.5}
            ]
        }
        msg = String()
        msg.data = json.dumps(nav_cmd)
        tester.cmd_pub.publish(msg)

        pos_errors = []
        yaw_errors = []
        timestamps = []
        t0 = time.time()
        timeout = 40.0

        while time.time() - t0 < timeout:
            rclpy.spin_once(tester, timeout_sec=0.1)
            tf_pose = tester.get_tf_pose()
            if tf_pose is None and tester.has_amcl:
                tf_pose = (tester.amcl_x, tester.amcl_y, tester.amcl_yaw)
            if tf_pose is not None:
                p_err = math.hypot(tf_pose[0] - tester.gz_x, tf_pose[1] - tester.gz_y)
                y_err = abs(wrap_pi(tf_pose[2] - tester.gz_yaw))
                pos_errors.append(p_err)
                yaw_errors.append(y_err)
                timestamps.append(time.time() - t0)

            if tester.supervisor_state == "MISSION_COMPLETED":
                print(f"Transit {t_idx+1} completed in {time.time()-t0:.1f}s!")
                break
            if tester.supervisor_state == "NAVIGATION_FAILURE":
                print(f"Transit {t_idx+1} failed in {time.time()-t0:.1f}s")
                break

        # Analyze errors
        max_p_err = float(np.max(pos_errors)) if pos_errors else 0.0
        mean_p_err = float(np.mean(pos_errors)) if pos_errors else 0.0
        final_p_err = float(pos_errors[-1]) if pos_errors else 0.0
        max_y_err = float(np.max(yaw_errors)) if yaw_errors else 0.0
        mean_y_err = float(np.mean(yaw_errors)) if yaw_errors else 0.0

        # Check for ~0.95m jumps
        has_095m_jump = any(p > 0.80 for p in pos_errors)

        transit_summary = {
            "transit": t_idx + 1,
            "post_reset_pos_err_mm": init_pos_err * 1000.0,
            "mean_pos_err_m": mean_p_err,
            "max_pos_err_m": max_p_err,
            "final_pos_err_m": final_p_err,
            "mean_yaw_err_deg": mean_y_err * 180.0 / math.pi,
            "max_yaw_err_deg": max_y_err * 180.0 / math.pi,
            "jump_095m_detected": has_095m_jump,
            "state": tester.supervisor_state
        }
        transits.append(transit_summary)

        print(f"  Summary: Mean pos error = {mean_p_err:.3f} m, Max pos error = {max_p_err:.3f} m, Final = {final_p_err:.3f} m")
        print(f"  Mean yaw error = {mean_y_err*180/math.pi:.2f} deg, 0.95m jump detected: {has_095m_jump}")

    # Output table
    print("\n================== 5-TRANSIT RESULTS TABLE ==================")
    print(f"{'Transit':<8} | {'Reset Err (mm)':<14} | {'Mean Err (m)':<12} | {'Max Err (m)':<11} | {'Final Err (m)':<13} | {'Max Yaw (deg)':<13} | {'0.95m Jump?'}")
    print("-" * 90)
    for r in transits:
        print(f"{r['transit']:<8} | {r['post_reset_pos_err_mm']:<14.1f} | {r['mean_pos_err_m']:<12.3f} | {r['max_pos_err_m']:<11.3f} | {r['final_pos_err_m']:<13.3f} | {r['max_yaw_err_deg']:<13.2f} | {str(r['jump_095m_detected'])}")
    print("=============================================================")

    out_file = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scratch", out_filename)
    with open(out_file, "w") as f:
        json.dump(transits, f, indent=2)
    print(f"Saved results to {out_file}")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    out = sys.argv[2] if len(sys.argv) > 2 else "track3_amcl_transits.json"
    run_amcl_transits(n, out)

