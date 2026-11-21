#!/usr/bin/env python3
"""
GRaCEmo ViRa — Rosbag Recording and Offline Replay Utility (Phase 12).
Records critical autonomy and telemetry topics for deterministic offline evaluation,
and supports headless playback into the local ROS 2 network.
"""

import os
import sys
import subprocess
import argparse
import time

TOPICS_TO_RECORD = [
    "/scan",
    "/odom",
    "/tf",
    "/tf_static",
    "/plan",
    "/cmd_vel_nav",
    "/cmd_vel_rl",
    "/cmd_vel_mux",
    "/cmd_vel_safe",
    "/collision_monitor_state",
    "/autonomy/state",
    "/autonomy/mission_status",
]


def record_bag(output_dir: str, duration_sec: int = 30):
    os.makedirs(output_dir, exist_ok=True)
    bag_name = os.path.join(output_dir, f"gracemo_bag_{time.strftime('%Y%m%d_%H%M%S')}")
    cmd = [
        "ros2", "bag", "record",
        "-o", bag_name,
        "--max-cache-size", "100000000",
    ] + TOPICS_TO_RECORD

    print(f"🎬 Starting ROS 2 bag recording to: {bag_name}")
    print(f"Topics: {TOPICS_TO_RECORD}")
    proc = subprocess.Popen(cmd)
    try:
        time.sleep(duration_sec)
    finally:
        proc.terminate()
        proc.wait()
    print(f"✅ Bag recorded successfully to {bag_name}")


def replay_bag(bag_path: str, rate: float = 1.0):
    if not os.path.exists(bag_path):
        raise FileNotFoundError(f"Bag file not found: {bag_path}")
    cmd = [
        "ros2", "bag", "play",
        bag_path,
        "--rate", str(rate),
    ]
    print(f"▶️ Replaying ROS 2 bag from {bag_path} at {rate}x speed...")
    subprocess.run(cmd)


def main():
    parser = argparse.ArgumentParser(description="GRaCEmo ViRa Rosbag Utility")
    subparsers = parser.add_subparsers(dest="command", required=True)

    rec_p = subparsers.add_parser("record")
    rec_p.add_argument("--output", default="rosbags", help="Output directory")
    rec_p.add_argument("--duration", type=int, default=15, help="Duration in seconds")

    play_p = subparsers.add_parser("play")
    play_p.add_argument("bag_path", help="Path to bag directory/file")
    play_p.add_argument("--rate", type=float, default=1.0, help="Playback speed multiplier")

    args = parser.parse_args()
    if args.command == "record":
        record_bag(args.output, args.duration)
    elif args.command == "play":
        replay_bag(args.bag_path, args.rate)


if __name__ == "__main__":
    main()

