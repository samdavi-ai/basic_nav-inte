#!/usr/bin/env python3
"""Run one guarded point-to-point acceptance mission on the live Gazebo stack.

This script never launches a simulator. Before publishing a goal, it requires an
advancing ROS simulation clock, fresh LiDAR/odometry/IMU telemetry, an active
Nav2 action server, and the coordinator's AUTONOMY_READY state.
"""

import argparse
import json
import math
import time
import uuid

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


FAILURE_STATES = {
    "NAVIGATION_FAILURE",
    "MISSION_CANCELLED",
    "EMERGENCY_STOP",
    "TERRAIN_SAFETY_STOP",
    "LOCALIZATION_LOST",
}


class GazeboNavigationSmoke(Node):
    def __init__(self):
        super().__init__("gazebo_navigation_smoke")
        self.command_pub = self.create_publisher(String, "/coordinator/command", 10)
        self.status = None
        self.terrain = None
        self.odom = None
        self.scan_count = 0
        self.odom_count = 0
        self.clock_ticks = 0
        self._last_clock_sec = None
        self.state = None
        self.states = []

        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/terrain/status", self._on_terrain, 10)
        self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)

    def _on_clock(self, msg):
        value = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        if self._last_clock_sec is not None and value > self._last_clock_sec:
            self.clock_ticks += 1
        self._last_clock_sec = value

    def _on_scan(self, _msg):
        self.scan_count += 1

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (float(p.x), float(p.y))
        self.odom_count += 1

    def _on_terrain(self, msg):
        try:
            self.terrain = json.loads(msg.data)
        except (TypeError, ValueError):
            self.terrain = None

    def _on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.states.append(msg.data)
            print(f"STATE {msg.data}", flush=True)

    def _on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.status = None

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

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
            and self.clock_ticks >= 10
            and self.scan_count >= 5
            and self.odom_count >= 5
            and self.command_pub.get_subscription_count() > 0
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-x", type=float, default=0.9)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--ready-timeout", type=float, default=30.0)
    parser.add_argument("--goal-timeout", type=float, default=90.0)
    parser.add_argument("--min-displacement", type=float, default=0.25)
    parser.add_argument("--out", help="Optional JSON evidence file")
    return parser.parse_args()


def publish_command(node, payload):
    msg = String()
    msg.data = json.dumps(payload)
    node.command_pub.publish(msg)


def main():
    args = parse_args()
    if not (-8.8 <= args.goal_x <= 8.8 and -6.3 <= args.goal_y <= 6.3):
        raise SystemExit("Goal is outside the apartment's configured safe bounds.")
    if args.ready_timeout <= 0 or args.goal_timeout <= 0 or args.min_displacement < 0:
        raise SystemExit("Timeouts must be positive and minimum displacement cannot be negative.")

    rclpy.init()
    node = GazeboNavigationSmoke()
    try:
        deadline = time.monotonic() + args.ready_timeout
        while time.monotonic() < deadline and not node.ready_for_motion():
            rclpy.spin_once(node, timeout_sec=0.1)

        if not node.ready_for_motion():
            print(json.dumps({
                "result": "NOT_RUN",
                "reason": "Gazebo clock, fresh sensors, terrain monitor, Nav2, or coordinator readiness gate failed",
                "clock_advances": node.clock_ticks,
                "scan_messages": node.scan_count,
                "odom_messages": node.odom_count,
                "coordinator_state": node.status.get("state") if node.status else None,
                "terrain_status": node.terrain,
            }, sort_keys=True), flush=True)
            return 2

        mission_id = f"gazebo-smoke-{uuid.uuid4().hex[:12]}"
        odom_start = node.odom
        map_start = node.status.get("robot", {}).get("pose", {})
        payload = {
            "intent": "NAVIGATE_TO",
            "mission_id": mission_id,
            "mission_name": "Gazebo point-to-point smoke",
            "destination_coords": {
                "x": args.goal_x,
                "y": args.goal_y,
                "yaw": args.goal_yaw,
                "frame_id": "map",
            },
        }
        publish_command(node, payload)
        print(f"MISSION_SENT {json.dumps(payload, sort_keys=True)}", flush=True)

        max_displacement = 0.0
        mission_completed = False
        failure_state = None
        start_time = time.monotonic()
        deadline = start_time + args.goal_timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom:
                max_displacement = max(
                    max_displacement,
                    math.hypot(node.odom[0] - odom_start[0], node.odom[1] - odom_start[1]),
                )
            if "MISSION_COMPLETED" in node.states:
                mission_completed = True
                break
            failure_state = next((state for state in reversed(node.states) if state in FAILURE_STATES), None)
            if failure_state:
                break
            history = node.status.get("history", []) if node.status else []
            if any(item.get("mission_id") == mission_id and item.get("success") is True for item in history):
                mission_completed = True
                break

        if not mission_completed and not failure_state:
            publish_command(node, {"intent": "CANCEL_MISSION"})
            node.spin_for(2.0)
            failure_state = "GOAL_TIMEOUT"

        # Let the coordinator publish its final mission-history entry and pose.
        node.spin_for(1.0)
        final_status = node.status or {}
        final_odom = node.odom
        final_pose = final_status.get("robot", {}).get("pose", {})
        goal_error = None
        if "x" in final_pose and "y" in final_pose:
            goal_error = math.hypot(final_pose["x"] - args.goal_x, final_pose["y"] - args.goal_y)
        final_history = next(
            (item for item in final_status.get("history", []) if item.get("mission_id") == mission_id),
            None,
        )
        result = {
            "result": "PASS" if mission_completed and goal_error is not None and goal_error <= 0.65 and max_displacement >= args.min_displacement else "FAIL",
            "mission_id": mission_id,
            "mission_completed": mission_completed,
            "failure_state": failure_state,
            "state_transitions": node.states,
            "goal_map": {"x": args.goal_x, "y": args.goal_y},
            "start_map_pose": map_start,
            "final_map_pose": final_pose,
            "goal_error_m": round(goal_error, 3) if goal_error is not None else None,
            "odom_start": odom_start,
            "odom_final": final_odom,
            "max_odom_displacement_m": round(max_displacement, 3),
            "mission_history": final_history,
            "wall_time_sec": round(time.monotonic() - start_time, 1),
            "simulation_clock_advances": node.clock_ticks,
            "scan_messages": node.scan_count,
            "odom_messages": node.odom_count,
        }
        if args.out:
            with open(args.out, "w", encoding="utf-8") as output:
                json.dump(result, output, indent=2)
                output.write("\n")
        print(json.dumps(result, sort_keys=True), flush=True)
        return 0 if result["result"] == "PASS" else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
