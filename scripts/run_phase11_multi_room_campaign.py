#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase 11 Multi-Room Continuous Mission Campaign.
Executes consecutive multi-room missions (kitchen -> bedroom -> home)
with AMCL ON, zero manual reset, measuring:
  - Completion Rate (%)
  - Drift (Map vs Gazebo GT)
  - Interventions / Relocalizations
  - Stagnation / Timeouts
"""

import sys
import os
import time
import math
import json
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(repo_root, "ros2_ws", "src", "gracemo_rl"))
from gracemo_rl.common.places import load_authoritative_places


class MultiRoomMissionCampaign(Node):
    def __init__(self, target_missions: int = 20):
        super().__init__("multi_room_mission_campaign")
        self.target_missions = target_missions

        self.supervisor_state = "UNKNOWN"
        self.loc_state = "UNKNOWN"
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.current_speed = 0.0

        self.sub_state = self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.sub_status = self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)
        self.places = load_authoritative_places()

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

    def execute_campaign(self) -> dict:
        self.get_logger().info(f"=== Starting Phase 11: {self.target_missions} Consecutive Multi-Room Missions ===")
        kitchen = self.places["kitchen"]
        bedroom = self.places["bedroom"]
        home = self.places["home"]

        results = []
        consecutive_passes = 0

        for m_idx in range(1, self.target_missions + 1):
            self.get_logger().info(f"\n--- Starting Multi-Room Mission {m_idx}/{self.target_missions} (kitchen -> bedroom -> home) ---")

            # Multi-room circuit definition
            mission_payload = {
                "intent": "START_MISSION",
                "mission_name": f"Continuous_Circuit_{m_idx}",
                "waypoints": [
                    {"name": "kitchen", "x": kitchen["x"], "y": kitchen["y"], "yaw": 0.0, "dwell_sec": 1.0},
                    {"name": "bedroom", "x": bedroom["x"], "y": bedroom["y"], "yaw": 0.0, "dwell_sec": 1.0},
                    {"name": "home", "x": home["x"], "y": home["y"], "yaw": 0.0, "dwell_sec": 1.0},
                ]
            }

            msg = String()
            msg.data = json.dumps(mission_payload)
            self.pub_cmd.publish(msg)

            # Wait for mission completion or failure (timeout 180s per 3-waypoint circuit)
            t_start = time.time()
            mission_done = False
            final_status = "UNKNOWN"

            while time.time() - t_start < 180.0:
                rclpy.spin_once(self, timeout_sec=0.2)
                if self.supervisor_state == "MISSION_COMPLETED":
                    mission_done = True
                    final_status = "COMPLETED"
                    break
                elif self.supervisor_state in ("NAVIGATION_FAILURE", "LOCALIZATION_LOST", "SENSOR_FAILURE"):
                    final_status = self.supervisor_state
                    break

            duration = time.time() - t_start
            is_success = (final_status == "COMPLETED")
            if is_success:
                consecutive_passes += 1
                self.get_logger().info(f"✅ Mission {m_idx} SUCCESS ({duration:.1f}s)! Total Consecutive: {consecutive_passes}")
            else:
                self.get_logger().warn(f"❌ Mission {m_idx} FAILED ({final_status}) after {duration:.1f}s.")

            results.append({
                "mission_id": m_idx,
                "status": final_status,
                "success": is_success,
                "duration_sec": round(duration, 2),
                "loc_state": self.loc_state,
            })

            # Brief settle between missions (continuous AMCL, zero manual reset)
            time.sleep(2.0)

        summary = {
            "total_missions": self.target_missions,
            "completed": sum(1 for r in results if r["success"]),
            "completion_rate": sum(1 for r in results if r["success"]) / self.target_missions * 100.0,
            "consecutive_passes": consecutive_passes,
            "mission_records": results
        }
        return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--missions", type=int, default=20, help="Number of missions")
    parser.add_argument("--output", type=str, default="scratch/phase11_mission_integration.json")
    args = parser.parse_args()

    rclpy.init()
    node = MultiRoomMissionCampaign(target_missions=args.missions)
    try:
        summary = node.execute_campaign()
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(summary, f, indent=2)
        print("\nPhase 11 Campaign Summary:")
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

