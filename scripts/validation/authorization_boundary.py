"""Observe denied and authorized mission IDs on an isolated simulator's real ROS topic."""
import json
import sys
import time
import uuid
from pathlib import Path

import requests
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(folder, url):
    rclpy.init()
    node = Node("authorization_boundary_observer")
    observed = []
    node.create_subscription(String, "/coordinator/command",
                             lambda msg: observed.append(json.loads(msg.data)), 10)
    evidence = {"topic": "/coordinator/command", "commands": observed}
    try:
        deadline = time.monotonic() + 20
        while not node.count_publishers(evidence["topic"]):
            if time.monotonic() >= deadline:
                raise RuntimeError("No bridge publisher discovered")
            rclpy.spin_once(node, timeout_sec=0.1)
        mission_id, trace_id = str(uuid.uuid4()), str(uuid.uuid4())
        evidence.update(denied_mission_id=mission_id, denied_trace_id=trace_id)
        event = dict(id=str(uuid.uuid4()), trace_id=trace_id, timestamp=int(time.time()),
                     source="Brain", observed_by="StandaloneMissionRunner",
                     event_type=dict(type="ActionRequested", data=dict(action="StartMission",
                         params=dict(mission_id=mission_id, waypoints=[dict(coords=dict(x=0., y=0.))]))))
        reply = requests.post(url + "/actions/authorize", json=event, timeout=3)
        evidence["denial_http_status"] = reply.status_code
        if reply.status_code != 401:
            raise RuntimeError("Unauthenticated request did not receive 401")
        (folder / "boundary.ready").touch(exist_ok=False)
        deadline = time.monotonic() + 170
        while time.monotonic() < deadline and not (folder / "mission.json").exists():
            rclpy.spin_once(node, timeout_sec=0.05)
        result = json.loads((folder / "mission.json").read_text())
        evidence["positive_control"] = any(cmd.get("intent") == "START_MISSION"
            and cmd.get("mission_id") == result["mission_id"] and cmd.get("trace_id") == result["trace_id"]
            for cmd in observed)
        evidence["denied_command_count"] = sum(cmd.get("mission_id") == mission_id
            or cmd.get("trace_id") == trace_id for cmd in observed)
        if not evidence["positive_control"] or evidence["denied_command_count"]:
            raise RuntimeError("ROS dispatch observation failed")
    finally:
        (folder / "boundary.json").write_text(json.dumps(evidence, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(Path(sys.argv[1]), sys.argv[2])
