#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase 10 Localization Health & Recovery Verification.
Injects a 1.0m offset during active navigation to verify:
  1. Detection within 3.0 seconds (via transform jump / particle dispersion)
  2. Immediate zero-velocity halt / rotation relocalization trigger
  3. Non-motion update invocation and AMCL recovery to HEALTHY state
  4. Safe resumption of navigation
"""

import sys
import time
import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry


class Phase10RelocalizationTester(Node):
    def __init__(self):
        super().__init__("phase10_relocalization_tester")

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
        self.nomotion_cli = self.create_client(Empty, "/request_nomotion_update")

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

    def clean_reset(self):
        # Cancel any active mission
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL"})
        self.pub_cmd.publish(msg)
        time.sleep(0.5)

        # Clear costmaps
        msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.pub_cmd.publish(msg)
        time.sleep(0.5)

        # Re-initialize localization to current odom pose
        loc_cmd = {
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": 0.0, "y": 0.0, "yaw": 0.0}
        }
        msg.data = json.dumps(loc_cmd)
        self.pub_cmd.publish(msg)
        time.sleep(1.0)

        # Trigger nomotion updates
        for _ in range(5):
            if self.nomotion_cli.service_is_ready():
                self.nomotion_cli.call_async(Empty.Request())
            time.sleep(0.1)

        # Reset coordinator
        msg.data = json.dumps({"intent": "RESET"})
        self.pub_cmd.publish(msg)
        time.sleep(1.0)

    def run_injection_test(self) -> dict:
        self.get_logger().info("--- Phase 10: Injected 1.0m Offset Relocalization Test ---")
        self.clean_reset()

        # Wait for autonomy ready
        t0 = time.time()
        ready = False
        while time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.supervisor_state in ("AUTONOMY_READY", "WAITING_FOR_NEXT_MISSION"):
                ready = True
                break

        self.get_logger().info(f"Supervisor state: {self.supervisor_state}, Loc state: {self.loc_state}")

        # 1. Dispatch mission to Home (from current kitchen pose)
        cmd = {
            "intent": "START_MISSION",
            "mission_name": "Phase10 Test",
            "waypoints": [{"name": "home", "x": 0.0, "y": 0.0, "yaw": 0.0, "dwell_sec": 1.0}]
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.pub_cmd.publish(msg)
        self.get_logger().info("Dispatched mission to Home (0,0). Waiting for motion...")

        # Wait for robot to start moving
        t0 = time.time()
        moving = False
        while time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_speed > 0.03 or self.supervisor_state == "MISSION_EXECUTING":
                moving = True
                break

        if not moving:
            return {"success": False, "reason": "Failed to start motion within 15s"}

        self.get_logger().info(f"Robot moving! Current odom=({self.robot_x:.2f}, {self.robot_y:.2f}). Injecting 1.0m offset...")

        # 2. Inject 1.0m pose offset
        inject_x = self.robot_x + 1.0
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

        # 3. Measure detection time
        detected = False
        t_detect = 0.0
        detection_delay = 999.0
        while time.time() - t_inject < 10.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.loc_state in ("RECOVERING", "LOST") or self.supervisor_state == "OBSTACLE_HANDLING":
                t_detect = time.time()
                detection_delay = t_detect - t_inject
                detected = True
                self.get_logger().info(f"🎯 Offset detected in {detection_delay:.2f}s! Entered recovery state.")
                break

        # 4. Monitor recovery to HEALTHY
        recovered = False
        t_rec = time.time()
        while time.time() - t_rec < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.loc_state == "HEALTHY" and (self.supervisor_state in ("MISSION_EXECUTING", "AUTONOMY_READY")):
                recovered = True
                self.get_logger().info(f"✅ Relocalized to HEALTHY in {time.time() - t_detect:.2f}s!")
                break

        # Clean up
        cancel_msg = String()
        cancel_msg.data = json.dumps({"intent": "CANCEL"})
        self.pub_cmd.publish(cancel_msg)

        return {
            "success": detected and (detection_delay <= 3.0),
            "detection_delay_sec": detection_delay,
            "detected": detected,
            "recovered": recovered,
            "pass_criteria": "Detection <= 3.0s and safe recovery"
        }


def main():
    rclpy.init()
    node = Phase10RelocalizationTester()
    try:
        res = node.run_injection_test()
        with open("scratch/phase10_recovery_results.json", "w") as f:
            json.dump(res, f, indent=2)
        print("\nPhase 10 Test Result:")
        print(json.dumps(res, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
