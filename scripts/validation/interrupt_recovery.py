"""Observe one private Gazebo mission and interrupt it at a guard transition."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_gazebo_estop_smoke import GazeboEstopSmoke, rclpy, String


def magnitude(sample):
    return max(abs(sample["linear_x"]), abs(sample["angular_z"]))


def interrupt(node, target, result):
    def observe(message):
        event = json.loads(message.data)
        mission = (node.status or {}).get("mission") or {}
        if (event.get("state") != target or "request_wall" in result or
                not mission.get("id") or not node.odom or not node.clock_advances or
                not (node.terrain or {}).get("simulation_mode")):
            return
        result.update(mission_id=mission["id"], trigger=event, pose=dict(node.odom),
                      request_wall=time.monotonic(), request_sim=node.last_clock)
        node.publish_command({"intent": "STOP"})

    node.create_subscription(String, "/dynamic_guard/state", observe, 20)
    if not node.wait_for(lambda: "request_wall" in result, 75):
        return
    result["status"] = "FAIL"
    start = result["request_wall"]
    if not node.wait_for(lambda: node.estop_value is True and node.state == "EMERGENCY_STOP", 5):
        return
    latch = time.monotonic()
    result.update(latch_wall=latch,latch_sim=node.last_clock)
    node.spin_for(3)
    pose = result["pose"]
    odom = [(t,s) for t,s in node.odom_events if t >= start]
    stable = [magnitude(s) for t,s in odom if t >= time.monotonic()-1]
    distances = [math.hypot(s["x"]-pose["x"], s["y"]-pose["y"]) for _,s in odom]
    zeros = [t for t,c in node.cmd_events if t >= start and magnitude(c) <= .001]
    rest_start = next((t for t,s in odom if t > max((u for u,v in odom if magnitude(v)>.05),default=start)), None)
    result.update(zero_wall=zeros[0] if zeros else None, rest_wall=rest_start, zero_latency_s=zeros[0]-start if zeros else None,
                  max_displacement_m=max(distances, default=None),
                  final_second_speed=max(stable, default=None))
    if not (zeros and distances and max(distances)<=.15 and stable and max(stable)<=.05):
        return
    index = len(node.feedback)
    node.publish_command({"intent":"NAVIGATE_TO", "mission_id":"interrupt-rejection-probe",
                          "destination_coords":{"x":0., "y":0., "yaw":0., "frame_id":"map"}})
    result["rejected"] = node.wait_for(lambda: any(
        m.get("details",{}).get("error")=="estop_active" for m in node.feedback[index:]), 3)
    held = [magnitude(c) for t,c in node.cmd_events if t >= max(latch, zeros[0])]
    if not (result["rejected"] and held and max(held)<=.001 and node.estop_value and node.state=="EMERGENCY_STOP"):
        return
    reset = time.monotonic()
    node.publish_command({"intent":"RESET_ESTOP"})
    result["reset"] = node.wait_for(lambda: node.estop_value is False and node.state=="AUTONOMY_READY", 8)
    result.update(reset_wall=reset,reset_sim=node.last_clock)
    node.spin_for(3)
    commands = [magnitude(c) for t,c in node.cmd_events if t >= reset]
    speeds = [magnitude(s) for t,s in node.odom_events if t >= reset]
    result.update(post_reset_peak_command=max(commands,default=None),post_reset_peak_speed=max(speeds,default=None))
    if result["reset"] and commands and max(commands)<=.001 and speeds and max(speeds)<=.05:
        result["status"] = "PASS"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("state")
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    if not os.environ.get("GZ_PARTITION","").startswith("vira-dynamic-"):
        parser.error("requires a private fixture partition")
    rclpy.init()
    node = GazeboEstopSmoke()
    result = {"target":args.state, "status":"NOT TESTED"}
    try:
        interrupt(node, args.state, result)
    finally:
        args.out.write_text(json.dumps(result, indent=2)+"\n")
        node.destroy_node()
        rclpy.shutdown()
