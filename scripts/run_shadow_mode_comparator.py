#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase 12 Shadow Mode Comparator.
Executes shadow-mode comparison between DWB baseline and RL local planner:
  1. Strictly verifies hardware locks (MOTOR_OUTPUT_ENABLED = 0, PROTOCOL_CONFIRMED = False).
  2. Simultaneously receives DWB (/cmd_vel_nav) and RL (/cmd_vel_rl) commands.
  3. Computes discrepancy metrics, direction agreement, and motion smoothness.
  4. Generates shadow trace log without sending ANY commands to physical actuators.
"""

import sys
import os
import time
import math
import json
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def verify_hardware_safety_locks():
    """HARD CONSTRAINT 1: Verify hardware locks are strictly enforced."""
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config_h = os.path.join(repo_root, "firmware", "config.h")
    mcu_py = os.path.join(repo_root, "ros2_ws", "src", "gracemo_hardware", "gracemo_hardware", "mcu_transport.py")

    assert os.path.exists(config_h), f"Missing firmware config: {config_h}"
    with open(config_h, "r", encoding="utf-8") as f:
        content = f.read()
        assert "#define MOTOR_OUTPUT_ENABLED 0" in content, "SAFETY VIOLATION: MOTOR_OUTPUT_ENABLED is not 0!"

    assert os.path.exists(mcu_py), f"Missing mcu_transport: {mcu_py}"
    with open(mcu_py, "r", encoding="utf-8") as f:
        content = f.read()
        assert "PROTOCOL_CONFIRMED: bool = False" in content or "PROTOCOL_CONFIRMED = False" in content, "SAFETY VIOLATION: PROTOCOL_CONFIRMED is not False!"

    print("🔒 [HARDWARE SAFETY LOCK VERIFIED] MOTOR_OUTPUT_ENABLED = 0, PROTOCOL_CONFIRMED = False. Actuation strictly inhibited.")


class ShadowModeComparator(Node):
    def __init__(self, duration_sec: float = 20.0):
        super().__init__("shadow_mode_comparator")
        verify_hardware_safety_locks()

        self.duration_sec = duration_sec
        self.last_dwb_twist: Twist = Twist()
        self.last_rl_twist: Twist = Twist()
        self.dwb_received = False
        self.rl_received = False

        self.sub_dwb = self.create_subscription(Twist, "/cmd_vel_nav", self._on_dwb, 10)
        self.sub_rl = self.create_subscription(Twist, "/cmd_vel_rl", self._on_rl, 10)

        self.records = []
        self.timer = self.create_timer(0.1, self._sample_cycle)  # 10 Hz sampling
        self.start_time = time.time()

    def _on_dwb(self, msg: Twist):
        self.last_dwb_twist = msg
        self.dwb_received = True

    def _on_rl(self, msg: Twist):
        self.last_rl_twist = msg
        self.rl_received = True

    def _sample_cycle(self):
        now = time.time()
        if now - self.start_time > self.duration_sec:
            return

        v_dwb = self.last_dwb_twist.linear.x
        w_dwb = self.last_dwb_twist.angular.z

        v_rl = self.last_rl_twist.linear.x
        w_rl = self.last_rl_twist.angular.z

        err_v = abs(v_dwb - v_rl)
        err_w = abs(w_dwb - w_rl)

        # Sign agreement (both forward/backward and turning same direction)
        agree_v = (v_dwb * v_rl >= 0.0) or (abs(v_dwb) < 0.02 and abs(v_rl) < 0.02)
        agree_w = (w_dwb * w_rl >= 0.0) or (abs(w_dwb) < 0.05 and abs(w_rl) < 0.05)

        self.records.append({
            "timestamp": now - self.start_time,
            "v_dwb": round(v_dwb, 4),
            "w_dwb": round(w_dwb, 4),
            "v_rl": round(v_rl, 4),
            "w_rl": round(w_rl, 4),
            "err_v": round(err_v, 4),
            "err_w": round(err_w, 4),
            "agree_v": bool(agree_v),
            "agree_w": bool(agree_w),
        })

    def get_summary(self) -> dict:
        if not self.records:
            return {"status": "NO_DATA"}

        err_v_list = [r["err_v"] for r in self.records]
        err_w_list = [r["err_w"] for r in self.records]
        agree_v_count = sum(1 for r in self.records if r["agree_v"])
        agree_w_count = sum(1 for r in self.records if r["agree_w"])
        n = len(self.records)

        return {
            "samples": n,
            "duration_sec": self.duration_sec,
            "hardware_lock_verified": True,
            "mean_v_err": float(np.mean(err_v_list)),
            "max_v_err": float(np.max(err_v_list)),
            "mean_w_err": float(np.mean(err_w_list)),
            "max_w_err": float(np.max(err_w_list)),
            "v_sign_agreement_pct": float(agree_v_count / n * 100.0),
            "w_sign_agreement_pct": float(agree_w_count / n * 100.0),
            "dwb_packets_received": self.dwb_received,
            "rl_packets_received": self.rl_received,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0, help="Shadow mode logging duration (s)")
    parser.add_argument("--output", type=str, default="scratch/shadow_mode_trace.json")
    args = parser.parse_args()

    rclpy.init()
    node = ShadowModeComparator(duration_sec=args.duration)
    try:
        t0 = time.time()
        while time.time() - t0 < args.duration + 1.0:
            rclpy.spin_once(node, timeout_sec=0.1)

        summary = node.get_summary()
        summary["trace"] = node.records[:50]  # First 50 samples

        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(summary, f, indent=2)

        print("\nShadow Mode Execution Summary:")
        print(json.dumps(summary, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

