#!/usr/bin/env python3
"""Run a real, correlated multi-room mission through Kernel -> ROS 2 -> Nav2.

Requires the standalone simulation stack to be running, for example with
``./scripts/launch_full_simulation.sh``. No navigation success or detections are
simulated in this runner; completion is read from the ROS mission coordinator.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List

import requests

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT / "adapters/sdk"))
DEFAULT_KERNEL_URL = "http://127.0.0.1:7780"
DEFAULT_ROOMS = [
    "Kitchen & Dining",
    "Master Bedroom",
    "Home Study",
    "Living Room",
    "Central Hallway",
]


def _load_places() -> Dict[str, Dict[str, Any]]:
    path = ROOT / "vira_interaction" / "config" / "places.json"
    with path.open("r", encoding="utf-8") as handle:
        raw = json.load(handle)

    canonical_keys = {
        "kitchen": "Kitchen & Dining",
        "bedroom": "Master Bedroom",
        "study": "Home Study",
        "living room": "Living Room",
        "hallway": "Central Hallway",
    }
    result: Dict[str, Dict[str, Any]] = {}
    for key, value in raw.items():
        if key.startswith("_") or not isinstance(value, (list, tuple)) or len(value) < 3:
            continue
        canonical = canonical_keys.get(key.lower())
        if canonical:
            result[canonical.lower()] = {
                "name": canonical,
                "x": float(value[0]),
                "y": float(value[1]),
                "yaw": float(value[3]) if len(value) > 3 and value[3] is not None else 0.0,
            }
    return result


def _build_waypoints(rooms: List[str]) -> List[Dict[str, Any]]:
    places = _load_places()
    waypoints = []
    for requested in rooms:
        normalized = requested.strip().lower()
        aliases = {
            "kitchen": "kitchen & dining",
            "dining": "kitchen & dining",
            "bedroom": "master bedroom",
            "study": "home study",
            "office": "home study",
            "living": "living room",
            "hall": "central hallway",
            "hallway": "central hallway",
        }
        normalized = aliases.get(normalized, normalized)
        place = places.get(normalized)
        if place is None:
            known = ", ".join(p["name"] for p in places.values())
            raise ValueError(f"Unknown room {requested!r}; choose one of: {known}")
        waypoints.append({
            "name": place["name"],
            "coords": {
                "x": place["x"],
                "y": place["y"],
                "yaw": place["yaw"],
                "frame_id": "map",
            },
            "dwell_sec": 0.0,
        })
    return waypoints


class MissionMonitor:
    """Read mission history from the coordinator's ROS status topic."""

    def __init__(self, observe_clock=False):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        class _Node(Node):
            def __init__(self, parent):
                super().__init__("gracemo_standalone_mission_runner")
                self.parent = parent
                if observe_clock:
                    from rosgraph_msgs.msg import Clock
                    from rclpy.qos import qos_profile_sensor_data
                    self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
                self.nav_activity = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
                self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)

            def _on_clock(self, msg):
                with self.parent.clock_condition:
                    self.parent.sim_stamp_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
                    self.parent.clock_condition.notify_all()

            def _on_status(self, msg):
                try:
                    payload = json.loads(msg.data)
                except (TypeError, json.JSONDecodeError):
                    return
                with self.parent.lock:
                    self.parent.latest_status = payload
                    subsystems = payload.get("subsystems") or {}
                    robot = payload.get("robot") or {}
                    if (
                        payload.get("state") == "AUTONOMY_READY"
                        and subsystems.get("nav2_ready") is True
                        and subsystems.get("scan_fresh") is True
                        and subsystems.get("odom_fresh") is True
                        and robot.get("has_localization") is True
                        and subsystems.get("estop_active") is False
                    ):
                        self.parent.ready.set()
                    else:
                        self.parent.ready.clear()
                    mission = payload.get("mission") or {}
                    history = payload.get("history") or []
                    if mission.get("id") == self.parent.mission_id:
                        self.parent.active_seen.set()
                    for record in history:
                        if record.get("mission_id") == self.parent.mission_id:
                            self.parent.result = record
                            self.parent.finished.set()
                            break

        self.clock_condition = threading.Condition()
        self.sim_stamp_ns = None
        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.mission_id = ""
        self.latest_status: Dict[str, Any] = {}
        self.ready = threading.Event()
        self.result: Dict[str, Any] | None = None
        self.active_seen = threading.Event()
        self.finished = threading.Event()
        self.node = _Node(self)
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

    def close(self):
        # Stop and join the executor thread before destroying its node/context.
        # A short executor timeout here can leave the daemon spin thread unwinding
        # through rclpy after shutdown, which aborts the process after a valid run.
        self.executor.shutdown(timeout_sec=None)
        self.spin_thread.join()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()

    def wait_until_ready(self, timeout_sec: float) -> bool:
        """Wait for fresh sensors, localization, Nav2, and an unlatched safety gate."""
        from std_srvs.srv import Trigger

        deadline = time.monotonic() + timeout_sec
        client = self.node.nav_activity
        while (remaining := deadline - time.monotonic()) > 0 and self.ready.wait(remaining):
            if not client.wait_for_service(timeout_sec=min(0.5, max(0.0, deadline - time.monotonic()))):
                continue
            # Nav2 advertises its action servers while their lifecycle nodes are inactive.
            reply = client.call_async(Trigger.Request())
            received = threading.Event()
            reply.add_done_callback(lambda _, event=received: event.set())
            if received.wait(min(1.0, max(0.0, deadline - time.monotonic()))):
                if reply.exception() is None and reply.result().success and self.ready.is_set():
                    return True
            else:
                client.remove_pending_request(reply)
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
        return False


def _emit_action(kernel_url: str, action: Dict[str, Any], trace_id: str, adapter_id="StandaloneMissionRunner", parent=None):
    event = {
        "id": str(uuid.uuid4()),
        "trace_id": trace_id,
        "session_id": "standalone_autonomous_mission",
        "timestamp": int(time.time()),
        "source": "Brain",
        "observed_by": adapter_id,
        "event_type": {"type": "ActionRequested", "data": action},
        "parent_event_id": parent,
    }
    from gracemo_sdk.authorization import dispatch_action
    decision=dispatch_action(kernel_url,event)
    print("Kernel dispatch receipt "+json.dumps(decision["receipt"]),flush=True)
    return decision["receipt"]



def _wait_for_kernel(kernel_url: str, timeout_sec: float = 30.0) -> None:
    """Allow a standalone launcher to start the kernel and runner concurrently."""
    deadline = time.monotonic() + max(0.0, timeout_sec)
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            response = requests.get(f"{kernel_url.rstrip('/')}/health", timeout=2.0)
            response.raise_for_status()
            return
        except requests.RequestException as exc:
            last_error = exc
            time.sleep(0.25)
    raise requests.ConnectionError(
        f"Kernel did not become healthy at {kernel_url} within {timeout_sec:.0f}s"
        + (f": {last_error}" if last_error else "")
    )


def _cancel_mission(kernel_url: str, trace_id: str, adapter_id="StandaloneMissionRunner") -> None:
    try:
        _emit_action(kernel_url, {"action": "Cancel"}, trace_id, adapter_id)
    except requests.RequestException as exc:
        print(f"Could not send cancellation event: {exc}", file=sys.stderr)


def _wait_for_kernel_completion(kernel_url: str, trace_id: str, timeout_sec: float = 5.0) -> Dict[str, Any] | None:
    """Confirms the coordinator's correlated terminal event returned to the Kernel."""
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        try:
            response = requests.get(f"{kernel_url.rstrip('/')}/history", params={"trace_id": trace_id,"limit":100}, timeout=1.0)
            response.raise_for_status()
            for record in response.json():
                if record.get("trace_id") != trace_id:
                    continue
                raw_payload = record.get("payload", {})
                payload = json.loads(raw_payload) if isinstance(raw_payload, str) else raw_payload
                if not isinstance(payload, dict) or payload.get("type") != "AutonomyStateChanged":
                    continue
                data = payload.get("data", {})
                state = str(data.get("state", "")).upper()
                if state in {"MISSION_COMPLETED", "NAVIGATION_FAILURE", "PLANNING_FAILURE", "MISSION_CANCELLED"}:
                    return {**data, "trace_id": record["trace_id"]}
        except (requests.RequestException, ValueError, json.JSONDecodeError):
            pass
        time.sleep(0.25)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a real autonomous room tour in the active ROS/Gazebo stack.")
    parser.add_argument("--kernel-url", default=DEFAULT_KERNEL_URL)
    parser.add_argument("--readiness-timeout", type=float, help="Independent readiness deadline before dispatch, in seconds")
    parser.add_argument("--inspect-on-arrival",action="store_true")
    parser.add_argument("--adapter-id", default="StandaloneMissionRunner")
    parser.add_argument("--trace-id",type=uuid.UUID)
    parser.add_argument("--parent-event-id",type=uuid.UUID)
    parser.add_argument("--mission-name", default="Standalone apartment patrol")
    parser.add_argument("--timeout", type=float, default=300.0, help="Mission timeout in seconds")
    destinations = parser.add_mutually_exclusive_group()
    destinations.add_argument("--goal", nargs=2, type=float, metavar=("X","Y"), help="Single map-frame destination")
    destinations.add_argument("--rooms", nargs="+", default=DEFAULT_ROOMS, help="Ordered room names from places.json")
    parser.add_argument("--out", help="Optional path for a JSON acceptance record")
    args = parser.parse_args()
    if args.readiness_timeout is not None and (not math.isfinite(args.readiness_timeout) or args.readiness_timeout <= 0):
        parser.error("--readiness-timeout requires a positive finite duration")
    if args.goal is not None and not all(math.isfinite(value) for value in args.goal):
        parser.error("--goal requires finite map coordinates")

    try:
        waypoints = _build_waypoints(args.rooms) if args.goal is None else [{
            "name": "Simulation destination",
            "coords": {"x": args.goal[0], "y": args.goal[1], "yaw": 0., "frame_id": "map"},
            "dwell_sec": 0.,
        }]
        _wait_for_kernel(args.kernel_url)
    except (OSError, ValueError, requests.RequestException) as exc:
        print(f"Standalone mission preflight failed: {exc}", file=sys.stderr)
        return 2

    try:
        import rclpy
    except ImportError as exc:
        print(f"ROS 2 Python libraries are unavailable; source /opt/ros/jazzy/setup.bash first: {exc}", file=sys.stderr)
        return 2

    if not rclpy.ok():
        rclpy.init(args=None)
    monitor = MissionMonitor(observe_clock=args.inspect_on_arrival)
    monitor.mission_id = f"standalone-{uuid.uuid4()}"
    trace_id = str(args.trace_id or uuid.uuid4())
    action = {
        "action": "StartMission",
        "params": {
            "mission_id": monitor.mission_id,
            "mission_name": args.mission_name,
            "waypoints": waypoints,
        },
    }

    try:
        print(f"Mission ID: {monitor.mission_id}")
        print(f"Trace ID: {trace_id}")
        readiness_timeout = min(max(args.timeout, 1.0), 90.0) if args.readiness_timeout is None else args.readiness_timeout
        print(f"Waiting up to {readiness_timeout:.0f}s for localization, sensors, Nav2, and safety readiness")
        if not monitor.wait_until_ready(readiness_timeout):
            with monitor.lock:
                latest = monitor.latest_status
            print("Autonomy readiness gate did not pass; mission was not dispatched.", file=sys.stderr)
            if latest:
                print(json.dumps(latest, indent=2), file=sys.stderr)
            return 1
        print(f"Dispatching {len(waypoints)} real Nav2 waypoints through {args.kernel_url}")
        print("Waypoints: " + " -> ".join(wp["name"] for wp in waypoints))
        receipt=_emit_action(args.kernel_url, action, trace_id,args.adapter_id,str(args.parent_event_id) if args.parent_event_id else None)

        deadline = time.monotonic() + max(1.0, args.timeout)
        while time.monotonic() < deadline and not monitor.finished.wait(0.25):
            pass

        timed_out = monitor.result is None
        if timed_out:
            _cancel_mission(args.kernel_url, trace_id,args.adapter_id)
            with monitor.lock:
                latest = monitor.latest_status
            print("Mission did not reach a terminal coordinator result before timeout.", file=sys.stderr)
            if latest:
                print(json.dumps(latest, indent=2), file=sys.stderr)
            monitor.finished.wait(5.0)

        kernel_result = _wait_for_kernel_completion(args.kernel_url, trace_id)
        success = (
            not timed_out and monitor.result is not None and monitor.result.get("success") is True
            and monitor.result.get("mission_id") == monitor.mission_id
            and kernel_result is not None
            and kernel_result.get("mission_id") == monitor.mission_id
            and kernel_result.get("state") == "MISSION_COMPLETED"
        )
        report = {
            "success": success,
            "timed_out": timed_out,
            "mission_id": monitor.mission_id,
            "trace_id": trace_id,
            "rooms": [wp["name"] for wp in waypoints],
            "coordinator_result": monitor.result,
            "kernel_result": kernel_result,
            "timestamp": time.time(),
        }
        report["receipt"]=receipt
        if success and args.inspect_on_arrival:
            from gracemo_sdk.scene import current_scene, describe
            from gracemo_sdk import AdapterClient
            try:
                with monitor.clock_condition:
                    previous = monitor.sim_stamp_ns
                    advanced = monitor.clock_condition.wait_for(
                        lambda: monitor.sim_stamp_ns is not None and monitor.sim_stamp_ns != previous, timeout=2
                    )
                    if not advanced:
                        raise RuntimeError("Simulation clock did not advance after arrival")
                    barrier = monitor.sim_stamp_ns
                scene=current_scene(args.kernel_url,after_sim_ns=barrier,timeout=8)
                report["arrival_barrier_sim_ns"] = barrier
                report["scene"]=scene
                report["feedback"]=describe(scene)
                report["navigation_success"] = True
                report["inspection_status"] = "objects_localized" if scene["observations"] else "no_localized_objects"
                report["success"] = success = bool(scene["observations"])
                client=AdapterClient(args.adapter_id,args.kernel_url)
                if not client.emit("Custom", {"name":"mission_inspection","payload":{
                        "mission_id":monitor.mission_id,"scene":scene,"arrival_barrier_sim_ns":barrier,"inspection_status":report["inspection_status"],"feedback":report["feedback"]}},
                        trace_id=trace_id,parent_event_id=receipt["action_id"]):
                    raise RuntimeError("Kernel did not record the inspection result")
            except (RuntimeError, requests.RequestException) as exc:
                report["inspection_error"]=str(exc)
                report["success"]=success=False
        if args.out:
            evidence_path = Path(args.out)
            evidence_path.parent.mkdir(parents=True, exist_ok=True)
            evidence_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if success else 1
    except requests.RequestException as exc:
        print(f"Could not dispatch mission through the kernel: {exc}", file=sys.stderr)
        return 2
    finally:
        monitor.close()


if __name__ == "__main__":
    raise SystemExit(main())
