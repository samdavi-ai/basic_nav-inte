#!/usr/bin/env python3
"""Run a real, correlated multi-room mission through Kernel -> ROS 2 -> Nav2.

Requires the standalone simulation stack to be running, for example with
``./scripts/launch_full_simulation.sh``. No navigation success or detections are
simulated in this runner; completion is read from the ROS mission coordinator.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List

import requests

ROOT = Path(__file__).resolve().parent.parent
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

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        class _Node(Node):
            def __init__(self, parent):
                super().__init__("gracemo_standalone_mission_runner")
                self.parent = parent
                self.create_subscription(String, "/autonomy/mission_status", self._on_status, 10)

            def _on_status(self, msg):
                try:
                    payload = json.loads(msg.data)
                except (TypeError, json.JSONDecodeError):
                    return
                with self.parent.lock:
                    self.parent.latest_status = payload
                    mission = payload.get("mission") or {}
                    history = payload.get("history") or []
                    if mission.get("id") == self.parent.mission_id:
                        self.parent.active_seen.set()
                    for record in history:
                        if record.get("mission_id") == self.parent.mission_id:
                            self.parent.result = record
                            self.parent.finished.set()
                            break

        self.rclpy = rclpy
        self.lock = threading.Lock()
        self.mission_id = ""
        self.latest_status: Dict[str, Any] = {}
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
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


def _emit_action(kernel_url: str, action: Dict[str, Any], trace_id: str) -> None:
    event = {
        "id": str(uuid.uuid4()),
        "trace_id": trace_id,
        "session_id": "standalone_autonomous_mission",
        "timestamp": int(time.time()),
        "source": "Brain",
        "observed_by": "StandaloneMissionRunner",
        "event_type": {"type": "ActionRequested", "data": action},
        "parent_event_id": None,
    }
    response = requests.post(f"{kernel_url.rstrip('/')}/emit", json=event, timeout=3.0)
    response.raise_for_status()


def _cancel_mission(kernel_url: str, trace_id: str) -> None:
    try:
        _emit_action(kernel_url, {"action": "Cancel", "params": {}}, trace_id)
    except requests.RequestException as exc:
        print(f"Could not send cancellation event: {exc}", file=sys.stderr)


def _wait_for_kernel_completion(kernel_url: str, trace_id: str, timeout_sec: float = 5.0) -> Dict[str, Any] | None:
    """Confirms the coordinator's correlated terminal event returned to the Kernel."""
    deadline = time.monotonic() + max(0.0, timeout_sec)
    while time.monotonic() < deadline:
        try:
            response = requests.get(f"{kernel_url.rstrip('/')}/history?limit=100", timeout=1.0)
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
                    return data
        except (requests.RequestException, ValueError, json.JSONDecodeError):
            pass
        time.sleep(0.25)
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a real autonomous room tour in the active ROS/Gazebo stack.")
    parser.add_argument("--kernel-url", default=DEFAULT_KERNEL_URL)
    parser.add_argument("--mission-name", default="Standalone apartment patrol")
    parser.add_argument("--timeout", type=float, default=300.0, help="Mission timeout in seconds")
    parser.add_argument("--rooms", nargs="+", default=DEFAULT_ROOMS, help="Ordered room names from places.json")
    parser.add_argument("--out", help="Optional path for a JSON acceptance record")
    args = parser.parse_args()

    try:
        waypoints = _build_waypoints(args.rooms)
        health = requests.get(f"{args.kernel_url.rstrip('/')}/health", timeout=2.0)
        health.raise_for_status()
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
    monitor = MissionMonitor()
    monitor.mission_id = f"standalone-{uuid.uuid4()}"
    trace_id = str(uuid.uuid4())
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
        print(f"Dispatching {len(waypoints)} real Nav2 waypoints through {args.kernel_url}")
        print("Waypoints: " + " -> ".join(wp["name"] for wp in waypoints))
        _emit_action(args.kernel_url, action, trace_id)

        deadline = time.monotonic() + max(1.0, args.timeout)
        while time.monotonic() < deadline and not monitor.finished.wait(0.25):
            pass

        if monitor.result is None:
            _cancel_mission(args.kernel_url, trace_id)
            with monitor.lock:
                latest = monitor.latest_status
            print("Mission did not reach a terminal coordinator result before timeout.", file=sys.stderr)
            if latest:
                print(json.dumps(latest, indent=2), file=sys.stderr)
            return 1

        kernel_result = _wait_for_kernel_completion(args.kernel_url, trace_id)
        success = (
            monitor.result.get("success") is True
            and kernel_result is not None
            and kernel_result.get("state") == "MISSION_COMPLETED"
        )
        report = {
            "success": success,
            "mission_id": monitor.mission_id,
            "trace_id": trace_id,
            "rooms": [wp["name"] for wp in waypoints],
            "coordinator_result": monitor.result,
            "kernel_result": kernel_result,
            "timestamp": time.time(),
        }
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
