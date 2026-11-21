#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase 1 Read-Only Velocity Pipeline Diagnostics
Traces velocity propagation through all 5 layers + Gazebo DiffDrive:
  /cmd_vel_nav -> /cmd_vel_mux -> /cmd_vel_smoothed -> /cmd_vel_safe -> /cmd_vel -> Gazebo /cmd_vel -> Wheel Joints -> Model Pose
"""

import os
import sys
import time
import math
import json
import threading
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.twist_pb2 import Twist as GzTwist


class VelocityPipelineDiagnostic(Node):
    def __init__(self):
        super().__init__("velocity_pipeline_diagnostic")

        # Telemetry storage
        self.cmd_nav: Optional[Twist] = None
        self.cmd_mux: Optional[Twist] = None
        self.cmd_smoothed: Optional[Twist] = None
        self.cmd_safe: Optional[Twist] = None
        self.cmd_final: Optional[Twist] = None

        self.gz_cmd: Optional[Tuple[float, float]] = None
        self.gz_pose: Optional[Tuple[float, float, float, float]] = None
        self.odom_pose: Optional[Tuple[float, float, float, float]] = None
        self.wheel_velocities: Dict[str, float] = {}

        self.nav_feedback_dist: Optional[float] = None
        self.nav_status: Optional[int] = None
        self.terrain_status: Optional[dict] = None
        self.mux_status: Optional[dict] = None

        # Subscriptions
        qos = 10
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_cmd_nav, qos)
        self.create_subscription(Twist, "/cmd_vel_mux", self._on_cmd_mux, qos)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_cmd_smoothed, qos)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_cmd_safe, qos)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_final, qos)

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "/cmd_vel_mux/status", self._on_mux_status, qos)
        self.create_subscription(String, "/terrain/status", self._on_terrain_status, qos)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # NavigateToPose action client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Gazebo Transport Node
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_pose)
        self.gz_node.subscribe(GzTwist, "/cmd_vel", self._on_gz_cmd)

    def _on_cmd_nav(self, m: Twist): self.cmd_nav = m
    def _on_cmd_mux(self, m: Twist): self.cmd_mux = m
    def _on_cmd_smoothed(self, m: Twist): self.cmd_smoothed = m
    def _on_cmd_safe(self, m: Twist): self.cmd_safe = m
    def _on_cmd_final(self, m: Twist): self.cmd_final = m

    def _on_gz_cmd(self, msg: GzTwist):
        self.gz_cmd = (msg.linear.x, msg.angular.z)

    def _on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                q = p.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny, cosy)
                self.gz_pose = (p.position.x, p.position.y, p.position.z, yaw)
                break

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        self.odom_pose = (p.x, p.y, p.z, yaw)

    def _on_joint_states(self, msg: JointState):
        for name, vel in zip(msg.name, msg.velocity):
            if "wheel" in name:
                self.wheel_velocities[name] = vel

    def _on_mux_status(self, msg: String):
        try:
            self.mux_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_terrain_status(self, msg: String):
        try:
            self.terrain_status = json.loads(msg.data)
        except Exception:
            pass

    def get_tf_pose(self, parent: str, child: str) -> Optional[Tuple[float, float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            siny = 2.0 * (rot.w * rot.z + rot.x * rot.y)
            cosy = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny, cosy)
            return (tr.x, tr.y, yaw)
        except Exception:
            return None


def run_diagnostics(target_x=1.5, target_y=0.0, duration=15.0):
    rclpy.init()
    node = VelocityPipelineDiagnostic()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("=" * 80)
    print("🔍 PHASE 1: FULL VELOCITY PIPELINE DIAGNOSTIC PROBE")
    print("=" * 80)
    print("Waiting 3s for telemetry streams to initialize...")
    time.sleep(3.0)

    print(f"Initial Gazebo Pose:     {node.gz_pose}")
    print(f"Initial /odom Pose:      {node.odom_pose}")
    print(f"Initial TF map->base:    {node.get_tf_pose('map', 'base_footprint')}")
    print(f"Initial Wheel Velocities:{node.wheel_velocities}")

    # Wait for action server
    if not node.nav_client.wait_for_server(timeout_sec=5.0):
        print("❌ ERROR: /navigate_to_pose action server unavailable!")
        return

    print(f"\n🚀 Sending NavigateToPose goal: target=({target_x}, {target_y})")
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.header.frame_id = "map"
    goal_msg.pose.header.stamp = node.get_clock().now().to_msg()
    goal_msg.pose.pose.position.x = float(target_x)
    goal_msg.pose.pose.position.y = float(target_y)
    goal_msg.pose.pose.orientation.w = 1.0

    feedback_dist = [999.0]
    goal_completed = [False]
    goal_status = [None]

    def on_fb(fb_msg):
        fb = fb_msg.feedback
        if hasattr(fb, "distance_remaining"):
            feedback_dist[0] = fb.distance_remaining

    goal_handle_holder = [None]

    def on_goal_response(future):
        handle = future.result()
        goal_handle_holder[0] = handle
        if handle and handle.accepted:
            res_future = handle.get_result_async()
            res_future.add_done_callback(on_result)

    def on_result(future):
        goal_completed[0] = True
        goal_status[0] = future.result().status

    send_future = node.nav_client.send_goal_async(goal_msg, feedback_callback=on_fb)
    send_future.add_done_callback(on_goal_response)

    # Wait for goal acceptance via background spinner
    t_wait = time.time()
    while goal_handle_holder[0] is None and time.time() - t_wait < 5.0:
        time.sleep(0.1)

    goal_handle = goal_handle_holder[0]
    if not goal_handle or not goal_handle.accepted:
        print("❌ ERROR: Goal was rejected by Nav2 bt_navigator!")
        return

    print("✅ Goal accepted! Monitoring velocity pipeline at 4 Hz...\n")
    header = (
        f"{'Time':>5} | {'cmd_nav':>8} {'cmd_mux':>8} {'cmd_smth':>8} {'cmd_safe':>8} {'cmd_fin':>8} | "
        f"{'GZ_cmd':>8} | {'Whl_FL':>7} {'Whl_FR':>7} | {'Gz_X':>6} {'Od_X':>6} | {'RemDist':>7}"
    )
    print(header)
    print("-" * len(header))

    start_t = time.time()
    samples = []

    while time.time() - start_t < duration:
        t_now = time.time() - start_t
        v_nav = node.cmd_nav.linear.x if node.cmd_nav else 0.0
        v_mux = node.cmd_mux.linear.x if node.cmd_mux else 0.0
        v_smth = node.cmd_smoothed.linear.x if node.cmd_smoothed else 0.0
        v_safe = node.cmd_safe.linear.x if node.cmd_safe else 0.0
        v_fin = node.cmd_final.linear.x if node.cmd_final else 0.0
        gz_v = node.gz_cmd[0] if node.gz_cmd else 0.0

        w_fl = node.wheel_velocities.get("front_left_wheel_joint", 0.0)
        w_fr = node.wheel_velocities.get("front_right_wheel_joint", 0.0)

        gz_x = node.gz_pose[0] if node.gz_pose else 0.0
        od_x = node.odom_pose[0] if node.odom_pose else 0.0
        rem_d = feedback_dist[0]

        row = (
            f"{t_now:5.1f} | {v_nav:8.3f} {v_mux:8.3f} {v_smth:8.3f} {v_safe:8.3f} {v_fin:8.3f} | "
            f"{gz_v:8.3f} | {w_fl:7.2f} {w_fr:7.2f} | {gz_x:6.2f} {od_x:6.2f} | {rem_d:7.2f}"
        )
        print(row)

        samples.append({
            "t": t_now,
            "cmd_nav": v_nav,
            "cmd_mux": v_mux,
            "cmd_smth": v_smth,
            "cmd_safe": v_safe,
            "cmd_final": v_fin,
            "gz_cmd": gz_v,
            "wheel_fl": w_fl,
            "wheel_fr": w_fr,
            "gz_x": gz_x,
            "odom_x": od_x,
            "rem_dist": rem_d,
            "terrain_status": node.terrain_status,
            "mux_status": node.mux_status,
        })

        if rem_d <= 0.20 or goal_completed[0]:
            print(f"\n🎯 Reached goal distance: rem_dist={rem_d:.3f}m, status={goal_status[0]}")
            break

        time.sleep(0.25)

    print("\n" + "=" * 80)
    print("📊 DIAGNOSTIC SUMMARY")
    print("=" * 80)
    max_nav = max(s["cmd_nav"] for s in samples) if samples else 0.0
    max_mux = max(s["cmd_mux"] for s in samples) if samples else 0.0
    max_smth = max(s["cmd_smth"] for s in samples) if samples else 0.0
    max_safe = max(s["cmd_safe"] for s in samples) if samples else 0.0
    max_final = max(s["cmd_final"] for s in samples) if samples else 0.0
    max_gz_cmd = max(s["gz_cmd"] for s in samples) if samples else 0.0
    max_whl = max(abs(s["wheel_fl"]) for s in samples) if samples else 0.0
    total_gz_disp = samples[-1]["gz_x"] - samples[0]["gz_x"] if len(samples) > 1 else 0.0
    total_od_disp = samples[-1]["odom_x"] - samples[0]["odom_x"] if len(samples) > 1 else 0.0

    print(f"Max /cmd_vel_nav:       {max_nav:.4f} m/s")
    print(f"Max /cmd_vel_mux:       {max_mux:.4f} m/s")
    print(f"Max /cmd_vel_smoothed:  {max_smth:.4f} m/s")
    print(f"Max /cmd_vel_safe:      {max_safe:.4f} m/s")
    print(f"Max /cmd_vel:           {max_final:.4f} m/s")
    print(f"Max Gazebo /cmd_vel:    {max_gz_cmd:.4f} m/s")
    print(f"Max Wheel Rad/s:        {max_whl:.4f} rad/s")
    print(f"Total Gazebo X Motion:  {total_gz_disp:.4f} m")
    print(f"Total Odometry X Motion:{total_od_disp:.4f} m")
    print(f"Final Remaining Dist:   {samples[-1]['rem_dist'] if samples else 999.0:.4f} m")

    # Save diagnostic dump
    dump_path = Path("docs/validation/evidence/velocity_pipeline_diagnostic.json")
    dump_path.parent.mkdir(parents=True, exist_ok=True)
    dump_path.write_text(json.dumps(samples, indent=2))
    print(f"Saved diagnostic trace to: {dump_path}")

    # Cleanup
    if goal_handle and not goal_completed[0]:
        try:
            goal_handle.cancel_goal_async()
        except Exception:
            pass
    time.sleep(0.5)
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    os._exit(0)


if __name__ == "__main__":
    t_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
    t_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0
    run_diagnostics(target_x=t_x, target_y=t_y, duration=dur)
