#!/usr/bin/env python3
"""Exercise the autonomous E-stop, mission rejection, and reset in Gazebo.

Requires an already running headless Gazebo/Nav2 stack. This script sends one
bounded simulated navigation goal, asserts E-stop while the robot is moving,
checks the base stops, verifies a new goal is rejected while latched, then
requests an explicit reset only after the stop checks pass.
"""

import argparse
import json
import math
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class GazeboEstopSmoke(Node):
    def __init__(self):
        super().__init__("gazebo_estop_smoke")
        self.command_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.status = None
        self.terrain = None
        self.state = None
        self.feedback = []
        self.odom = None
        self.odom_events = []
        self.scan_count = 0
        self.odom_count = 0
        self.clock_advances = 0
        self.last_clock = None
        self.cmd = None
        self.last_cmd_time = 0.0
        self.cmd_events = []
        self.estop_value = None
        self.estop_events = []
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 10)
        self.create_subscription(String, "/terrain/status", self._on_terrain, 10)
        self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)
        self.create_subscription(String, "/coordinator/feedback", self._on_feedback, 10)
        self.create_subscription(Bool, "/terrain/emergency_stop", self._on_estop, 10)

    def _on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_clock is not None and value > self.last_clock:
            self.clock_advances += 1
        self.last_clock = value

    def _on_scan(self, _msg):
        self.scan_count += 1

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        v = msg.twist.twist
        self.odom = {
            "x": float(p.x),
            "y": float(p.y),
            "linear_x": float(v.linear.x),
            "angular_z": float(v.angular.z),
        }
        self.odom_events.append((time.monotonic(), dict(self.odom)))
        self.odom_count += 1

    def _on_cmd(self, msg):
        self.cmd = {
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
        }
        self.last_cmd_time = time.monotonic()
        self.cmd_events.append((self.last_cmd_time, dict(self.cmd)))

    def _on_terrain(self, msg):
        try:
            self.terrain = json.loads(msg.data)
        except (TypeError, ValueError):
            self.terrain = None

    def _on_state(self, msg):
        self.state = msg.data

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.status = None

    def _on_feedback(self, msg):
        try:
            self.feedback.append(json.loads(msg.data))
        except (TypeError, ValueError):
            pass

    def _on_estop(self, msg):
        self.estop_value = bool(msg.data)
        self.estop_events.append(self.estop_value)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def publish_command(self, payload):
        msg = String()
        msg.data = json.dumps(payload)
        self.command_pub.publish(msg)

    def ready_for_motion(self):
        if not self.status or not self.terrain or not self.odom:
            return False
        subsystems = self.status.get("subsystems", {})
        return (
            self.status.get("state") == "AUTONOMY_READY"
            and subsystems.get("nav2_ready") is True
            and subsystems.get("scan_fresh") is True
            and subsystems.get("odom_fresh") is True
            and subsystems.get("estop_active") is False
            and self.terrain.get("imu_fresh") is True
            and self.terrain.get("simulation_mode") is True
            and self.clock_advances >= 5
            and self.scan_count >= 5
            and self.odom_count >= 5
            and self.command_pub.get_subscription_count() > 0
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=4.5)
    parser.add_argument("--goal-y", type=float, default=3.0)
    parser.add_argument("--ready-timeout", type=float, default=45.0)
    parser.add_argument("--motion-timeout", type=float, default=25.0)
    parser.add_argument("--out", help="Optional JSON evidence file")
    args = parser.parse_args()
    if not (-8.8 <= args.goal_x <= 8.8 and -6.3 <= args.goal_y <= 6.3):
        parser.error("Goal is outside the apartment's configured safe bounds.")
    if args.ready_timeout <= 0 or args.motion_timeout <= 0:
        parser.error("Timeouts must be positive.")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = GazeboEstopSmoke()
    result = {
        "result": "FAIL",
        "scope": "Gazebo only",
        "goal": {"x": args.goal_x, "y": args.goal_y},
        "checks": {},
        "state": None,
        "error": None,
    }
    estop_asserted = False
    mission_sent = False
    mission_id = f"gazebo-estop-{uuid.uuid4().hex[:12]}"
    try:
        if not node.wait_for(node.ready_for_motion, args.ready_timeout):
            raise RuntimeError("Gazebo sensors, Nav2, terrain monitor, or coordinator were not ready")

        start = dict(node.odom)
        node.publish_command({
            "intent": "NAVIGATE_TO",
            "mission_id": mission_id,
            "mission_name": "Gazebo E-stop smoke",
            "destination_coords": {
                "x": args.goal_x,
                "y": args.goal_y,
                "yaw": 0.0,
                "frame_id": "map",
            },
        })
        mission_sent = True
        result["checks"]["mission_started_and_moving"] = node.wait_for(
            lambda: node.state == "MISSION_EXECUTING"
            and node.cmd is not None
            and (abs(node.cmd["linear_x"]) > 0.02 or abs(node.cmd["angular_z"]) > 0.02)
            and node.odom is not None
            and math.hypot(node.odom["x"] - start["x"], node.odom["y"] - start["y"]) > 0.01,
            args.motion_timeout,
        )
        if not result["checks"]["mission_started_and_moving"]:
            raise RuntimeError("Robot did not begin the guarded navigation goal")

        stop_pose = dict(node.odom)
        stop_request_time = time.monotonic()
        node.publish_command({"intent": "STOP"})
        estop_asserted = True
        result["checks"]["estop_latched"] = node.wait_for(
            lambda: node.state == "EMERGENCY_STOP" and node.estop_value is True,
            5.0,
        )
        if not result["checks"]["estop_latched"]:
            raise RuntimeError("Coordinator/terrain monitor did not latch E-stop")

        zero_seen = False
        max_cmd_after_stop = 0.0
        max_speed_after_stop = 0.0
        max_drift_after_stop = 0.0
        settle_deadline = time.monotonic() + 3.0
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.odom is not None:
                speed_mag = max(abs(node.odom["linear_x"]), abs(node.odom["angular_z"]))
                max_speed_after_stop = max(max_speed_after_stop, speed_mag)
                max_drift_after_stop = max(
                    max_drift_after_stop,
                    math.hypot(node.odom["x"] - stop_pose["x"], node.odom["y"] - stop_pose["y"]),
                )
        stop_commands = [cmd for stamp, cmd in node.cmd_events if stamp >= stop_request_time]
        for cmd in stop_commands:
            command_mag = max(abs(cmd["linear_x"]), abs(cmd["angular_z"]))
            zero_seen |= command_mag <= 0.001
            max_cmd_after_stop = max(max_cmd_after_stop, command_mag)
        stable_cutoff = time.monotonic() - 1.0
        stable_odom = [sample for stamp, sample in node.odom_events if stamp >= stable_cutoff]
        stable_speed = max(
            (max(abs(sample["linear_x"]), abs(sample["angular_z"])) for sample in stable_odom),
            default=float("inf"),
        )
        result["checks"]["zero_command_observed"] = zero_seen
        result["checks"]["base_stopped"] = stable_speed <= 0.05 and max_drift_after_stop <= 0.15
        result["max_command_magnitude_after_stop"] = round(max_cmd_after_stop, 4)
        result["peak_odom_speed_after_stop"] = round(max_speed_after_stop, 4)
        result["max_odom_speed_last_second"] = round(stable_speed, 4) if math.isfinite(stable_speed) else None
        result["max_drift_after_stop_m"] = round(max_drift_after_stop, 4)
        if not result["checks"]["zero_command_observed"] or not result["checks"]["base_stopped"]:
            raise RuntimeError("E-stop did not produce a zero command and bounded stop")

        blocked_id = f"gazebo-estop-blocked-{uuid.uuid4().hex[:8]}"
        feedback_start = len(node.feedback)
        node.publish_command({
            "intent": "NAVIGATE_TO",
            "mission_id": blocked_id,
            "mission_name": "Must be rejected while E-stop is latched",
            "destination_coords": {"x": -4.5, "y": 3.0, "yaw": 0.0, "frame_id": "map"},
        })
        result["checks"]["mission_rejected_while_latched"] = node.wait_for(
            lambda: any(
                item.get("details", {}).get("error") == "estop_active"
                for item in node.feedback[feedback_start:]
            ),
            3.0,
        )
        if not result["checks"]["mission_rejected_while_latched"] or node.state != "EMERGENCY_STOP":
            raise RuntimeError("A new mission was not rejected while E-stop remained latched")
        node.spin_for(1.0)
        result["checks"]["no_motion_while_latched"] = (
            node.state == "EMERGENCY_STOP"
            and node.odom is not None
            and max(abs(node.odom["linear_x"]), abs(node.odom["angular_z"])) <= 0.05
        )
        if not result["checks"]["no_motion_while_latched"]:
            raise RuntimeError("Robot moved while E-stop was latched")

        node.publish_command({"intent": "RESET_ESTOP"})
        result["checks"]["explicit_reset_returned_ready"] = node.wait_for(
            lambda: node.state == "AUTONOMY_READY" and node.estop_value is False,
            8.0,
        )
        if not result["checks"]["explicit_reset_returned_ready"]:
            raise RuntimeError("Explicit reset did not clear E-stop and return to ready")
        node.spin_for(1.0)
        result["checks"]["zero_held_after_reset"] = (
            node.cmd is None
            or (abs(node.cmd["linear_x"]) <= 0.001 and abs(node.cmd["angular_z"]) <= 0.001)
        )
        if not result["checks"]["zero_held_after_reset"]:
            raise RuntimeError("Unexpected motion command after E-stop reset")

        result["result"] = "PASS"
    except Exception as exc:
        result["error"] = str(exc)
        # Keep the simulated robot safe on all failure exits. Do not auto-reset.
        if mission_sent or estop_asserted:
            node.publish_command({"intent": "STOP"})
            node.spin_for(0.5)
    finally:
        result["state"] = node.state
        result["estop_active"] = node.estop_value
        result["state_history"] = [item.get("state") for item in node.feedback if item.get("state")]
        if args.out:
            with open(args.out, "w", encoding="utf-8") as output:
                json.dump(result, output, indent=2)
                output.write("\n")
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
