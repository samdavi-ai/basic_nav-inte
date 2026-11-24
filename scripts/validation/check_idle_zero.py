#!/usr/bin/env python3
"""Measure the idle velocity stream for three seconds after ROS discovery."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node("idle_zero_observer")
    speeds = {topic: [] for topic in ("cmd_vel", "cmd_vel_mux", "cmd_vel_smoothed")}
    poses = []
    states = []

    def record(topic, msg):
        speeds[topic].append(max(abs(value) for axis in (msg.linear, msg.angular)
                                 for value in (axis.x, axis.y, axis.z)))

    for topic in speeds:
        node.create_subscription(Twist, "/" + topic, lambda msg, t=topic: record(t, msg), 10)
    node.create_subscription(String, "/cmd_vel_mux/status", lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(Odometry, "/odom", lambda m: poses.append(
        (m.pose.pose.position.x, m.pose.pose.position.y)), 10)

    # DDS participants discover at different times. Distinguish missing telemetry
    # from a zero command, and measure all topics over the same observation window.
    discovery_deadline = time.monotonic() + 15.0
    while not (all(speeds.values()) and states and poses) and time.monotonic() < discovery_deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    discovered = bool(all(speeds.values()) and states and poses)
    for stream in (*speeds.values(), poses, states):
        stream.clear()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    passed = (discovered and bool(states) and bool(poses)
              and all(len(v) >= 20 and max(v) == 0.0 for v in speeds.values())
              and all(s.get("active_source") == "NONE" and s.get("zero_command_held") is True
                      for s in states))
    chain = {}
    for topic in ('cmd_vel_nav','cmd_vel_guarded_nav','cmd_vel_mux','cmd_vel_smoothed','cmd_vel_safe','cmd_vel'):
        chain[topic] = dict(publishers=[e.node_name for e in node.get_publishers_info_by_topic('/'+topic)],
                            subscribers=[e.node_name for e in node.get_subscriptions_info_by_topic('/'+topic)])
    print(json.dumps(dict(command_graph=chain,passed=passed, discovery_complete=discovered, duration_s=3,
                          samples={k: len(v) for k, v in speeds.items()},
                          peak={k: max(v, default=None) for k, v in speeds.items()},
                          mux_state=states[-1] if states else None,
                          odom_displacement_m=math.dist(poses[0], poses[-1]) if poses else None),
                     indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
