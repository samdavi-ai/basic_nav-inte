"""Measure motion and stop from ROS independently of the speech adapter."""
import json
import math
from pathlib import Path
import re
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_gazebo_estop_smoke import GazeboEstopSmoke, rclpy


def main(folder):
    rclpy.init()
    node = GazeboEstopSmoke()
    result = {}
    mag = lambda s: max(abs(s["linear_x"]), abs(s["angular_z"]))
    try:
        if not node.wait_for(lambda: (node.terrain or {}).get("simulation_mode") and node.clock_advances
                             and node.state == "MISSION_EXECUTING" and node.odom and mag(node.odom) > .05, 90):
            raise RuntimeError("No simulated mission in motion")
        log = (folder / "mission.log").read_text()
        result.update(mission_id=re.search(r"Mission ID: (\S+)", log)[1],
                      trace_id=re.search(r"Trace ID: (\S+)", log)[1], motion_pose=node.odom)
        (folder / "moving.json").write_text(json.dumps(result))
        if not node.wait_for(lambda: (folder / "speech.json").exists(), 70):
            raise RuntimeError("No speech adapter result")
        speech = json.loads((folder / "speech.json").read_text())
        if "receipt" not in speech:
            raise RuntimeError("No kernel stop receipt")
        node.wait_for(lambda: node.estop_value and node.state == "EMERGENCY_STOP", 5)
        node.spin_for(3)
        t = speech["detected"]
        prev = [(u,s) for u,s in node.odom_events if u <= t]
        sample_time, pose = prev[-1]
        odom = [(u,s) for u,s in node.odom_events if u >= t]
        zero = next((u for u,s in node.cmd_events if u>=t and mag(s)<=.001), None)
        rest = next((u for u,s in odom if u>max((k for k,v in odom if mag(v)>.05),default=t)), None)
        distance = max(math.hypot(s["x"]-pose["x"],s["y"]-pose["y"]) for _,s in odom)
        stable = max(mag(s) for u,s in odom if u>=odom[-1][0]-1)
        result.update(speed_at_recognition=mag(pose), sensor_age_s=t-sample_time, zero_latency_s=zero-t if zero else None,
                      rest_latency_s=rest-t if rest else None, max_displacement_m=distance, stable_speed=stable,
                      state=node.state, latched=node.estop_value,passed=bool(mag(pose)>.05 and t-sample_time<.25 and zero and rest
                          and distance<=.15 and stable<=.05 and node.state=="EMERGENCY_STOP" and node.estop_value))
    finally:
        (folder / "interruption.json").write_text(json.dumps(result, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(Path(sys.argv[1]))
