#!/usr/bin/env python3
"""
Track 2c: Controlled Inflation Ablation (5 runs AT-02, 5 runs AT-03 @ 0.55 vs 0.40)
"""

import os
import sys
import time
import json
import yaml
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.msg import CollisionMonitorState

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
PARAMS_YAML = os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_nav2/config/nav2_params.yaml")


class AblationRunner(Node):
    def __init__(self):
        super().__init__("ablation_runner")
        self.gz_x = 0.0
        self.gz_y = 0.0
        self.gz_yaw = 0.0
        self.supervisor_state = "UNKNOWN"
        self.stop_count = 0
        self.slow_count = 0

        self.create_subscription(String, "/autonomy/state", self.on_state, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_monitor, 10)
        self.cmd_pub = self.create_publisher(String, "/coordinator/command", 10)

        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/pose/info", self.on_gz_pose)

    def on_state(self, msg):
        self.supervisor_state = msg.data

    def on_monitor(self, msg):
        if msg.action_type == 1:
            self.stop_count += 1
        elif msg.action_type == 2:
            self.slow_count += 1

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

    def reset_robot(self):
        # Cancel & Reset supervisor
        for intent in ["CANCEL", "RESET"]:
            msg = String()
            msg.data = json.dumps({"intent": intent})
            self.cmd_pub.publish(msg)
            time.sleep(0.2)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Teleport Gz to (0,0)
        req = GzPose()
        req.name = "gracemo_vira"
        req.position.x = 0.0
        req.position.y = 0.0
        req.position.z = 0.05
        req.orientation.w = 1.0
        self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)

        # Re-initialize localization in coordinator
        loc_msg = String()
        loc_msg.data = json.dumps({
            "intent": "INITIALIZE_LOCALIZATION",
            "coords": {"x": 0.0, "y": 0.0, "yaw": 0.0}
        })
        self.cmd_pub.publish(loc_msg)

        # Clear costmaps
        cmd_msg = String()
        cmd_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.cmd_pub.publish(cmd_msg)

        self.stop_count = 0
        self.slow_count = 0
        time.sleep(1.0)
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_at02_single(self, timeout=50.0):
        self.reset_robot()
        cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        t0 = time.time()
        passed = False
        reason = "TIMEOUT"
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.supervisor_state == "DESTINATION_REACHED":
                passed = True
                reason = "REACHED"
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                passed = False
                reason = "NAVIGATION_FAILURE"
                break

        dur = time.time() - t0
        err = math.hypot(self.gz_x - (-5.0), self.gz_y - (-2.5))
        return {
            "test": "AT-02",
            "passed": passed,
            "reason": reason,
            "duration": dur,
            "final_gz": [self.gz_x, self.gz_y],
            "goal_err": err,
            "stops": self.stop_count,
            "slows": self.slow_count
        }

    def run_at03_single(self, timeout=30.0):
        self.reset_robot()
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
        passed = False
        reason = "TIMEOUT"
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.supervisor_state == "MISSION_COMPLETED":
                passed = True
                reason = "COMPLETED"
                break
            if self.supervisor_state == "NAVIGATION_FAILURE":
                passed = False
                reason = "NAVIGATION_FAILURE"
                break

        dur = time.time() - t0
        disp = math.hypot(self.gz_x, self.gz_y)
        return {
            "test": "AT-03",
            "passed": passed,
            "reason": reason,
            "duration": dur,
            "final_gz": [self.gz_x, self.gz_y],
            "disp": disp,
            "stops": self.stop_count,
            "slows": self.slow_count
        }


def set_dynamic_inflation(node: Node, radius: float):
    """Sets inflation parameter via ROS 2 parameter service on active costmaps."""
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

    for srv_name in ["/local_costmap/local_costmap/set_parameters", "/global_costmap/global_costmap/set_parameters"]:
        cli = node.create_client(SetParameters, srv_name)
        if cli.wait_for_service(timeout_sec=2.0):
            req = SetParameters.Request()
            p = Parameter()
            p.name = "inflation_layer.inflation_radius"
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(radius))
            req.parameters = [p]
            cli.call_async(req)
    print(f"Dynamically requested inflation_layer.inflation_radius = {radius}")
    time.sleep(1.0)


def main():
    rclpy.init()
    runner = AblationRunner()

    results = {
        "inflation_0.55": {"at02": [], "at03": []},
        "inflation_0.40": {"at02": [], "at03": []}
    }

    print("\n========== [ABLATION SET 1] INFLATION RADIUS = 0.55 ==========")
    set_dynamic_inflation(runner, 0.55)
    for i in range(5):
        print(f"AT-02 [0.55] Run {i+1}/5...")
        res = runner.run_at02_single()
        print(f"  Result: {res['reason']} in {res['duration']:.1f}s, stops={res['stops']}, slows={res['slows']}")
        results["inflation_0.55"]["at02"].append(res)

    for i in range(5):
        print(f"AT-03 [0.55] Run {i+1}/5...")
        res = runner.run_at03_single()
        print(f"  Result: {res['reason']} in {res['duration']:.1f}s, stops={res['stops']}, slows={res['slows']}, final=({res['final_gz'][0]:.2f}, {res['final_gz'][1]:.2f})")
        results["inflation_0.55"]["at03"].append(res)

    print("\n========== [ABLATION SET 2] INFLATION RADIUS = 0.40 ==========")
    set_dynamic_inflation(runner, 0.40)
    for i in range(5):
        print(f"AT-02 [0.40] Run {i+1}/5...")
        res = runner.run_at02_single()
        print(f"  Result: {res['reason']} in {res['duration']:.1f}s, stops={res['stops']}, slows={res['slows']}")
        results["inflation_0.40"]["at02"].append(res)

    for i in range(5):
        print(f"AT-03 [0.40] Run {i+1}/5...")
        res = runner.run_at03_single()
        print(f"  Result: {res['reason']} in {res['duration']:.1f}s, stops={res['stops']}, slows={res['slows']}, final=({res['final_gz'][0]:.2f}, {res['final_gz'][1]:.2f})")
        results["inflation_0.40"]["at03"].append(res)

    # Revert back to 0.55
    set_dynamic_inflation(runner, 0.55)
    print("\nReverted dynamic inflation_radius back to 0.55.")

    out_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/inflation_ablation_side_by_side.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"Saved side-by-side results to {out_file}")

    runner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
