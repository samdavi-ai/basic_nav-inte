"""Ground inspection and destination requests in the running Gazebo mission."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

from gracemo_sdk.scene import current_scene, describe


_INSPECTION = re.compile(
    r"((?:navigate|go|take me) to .+?)[, ]+(?:(?:and then|and|then) )?"
    r"(?:inspect|identify|look for) (?:the |any )?(?:environment|surroundings|objects)"
    r"(?:[, ]+(?:and )?identify (?:the )?objects)?(?:[, ]+(?:and )?report.*)?"
)


def _visual_query(bus, query):
    import requests
    response = requests.post(bus.base_url + "/k/execute", json={
        "expression": f'memory::recall[query:"{query}"]'},timeout=2)
    if not response.ok:
        raise RuntimeError("No current visual answer")
    return response.json()["result"]["stage_results"][0]["output"]


def _describe_current_view(bus):
    _visual_query(bus,"visible_objects")
    return describe(current_scene(bus.base_url))


def _describe_people(bus, utterance):
    if utterance == "is a person present":
        present = _visual_query(bus,"person_present")["Bool"]
        return ("A person was detected in the current camera view." if present else
                "I did not detect a person in the current camera view.")
    if utterance == "where is the detected person":
        scene = current_scene(bus.base_url)
        people = [obj for obj in scene["observations"] if obj["class_name"]=="person"]
        if len(people)!=1:
            return "I do not have one unambiguous, localized person in view."
        point = people[0]["position_world"]
        return f"The detected person is at approximately x {point['x']:.1f}, y {point['y']:.1f} metres in the map."
    return "A camera track or appearance match alone cannot verify a person's identity."


def handle(text, gateway):
    if os.environ.get("VIRA_GAZEBO_INTERACTION") != "1":
        return None
    utterance = text.lower().strip(" .?!")
    bus = gateway.event_bus
    if utterance in {"what do you see", "what can you see", "inspect the environment", "identify the objects"}:
        return _describe_current_view(bus)
    if utterance == "what changed in the room":
        changes = _visual_query(bus,"scene_changes")["Map"]
        entered = [v["String"] for v in changes["entered_view"]["List"]]
        left = [v["String"] for v in changes["left_view"]["List"]]
        parts = []
        if entered:
            parts.append("Newly visible: " + ", ".join(entered))
        if left:
            parts.append("No longer visible: " + ", ".join(left))
        return ". ".join(parts) + "." if parts else "The same object classes appeared in both recent camera views."
    if utterance in {"is a person present","where is the detected person","is this the same person detected earlier"}:
        return _describe_people(bus,utterance)
    inspection = _INSPECTION.fullmatch(utterance)
    if not inspection and not re.fullmatch(r"(?:navigate|go|take me) to [\w &-]+", utterance):
        return None
    navigation = inspection[1] if inspection else utterance
    phrase = re.sub(r"^(?:navigate|go|take me) to (?:the )?", "", navigation)
    if phrase not in gateway.fast_path.place_resolver.places:
        return "Please give me one named destination."
    intent = gateway.fast_path.match(navigation)
    if not intent.matched or intent.intent != "NAVIGATE" or len(intent.semantic_actions) != 1:
        return "I could not resolve that destination."
    destination = intent.semantic_actions[0]["params"]["target_name"]
    request = bus.create_envelope("Custom", {
        "name": "navigation_inspection_request" if inspection else "navigation_request",
        "payload": {"destination": destination},
    })
    if not bus.emit_envelope(request):
        return "The kernel could not record the navigation request."
    script = Path(__file__).resolve().parents[2] / "scripts/run_autonomous_mission.py"
    with tempfile.TemporaryDirectory(prefix="vira-inspection-") as work:
        report = Path(work) / "result.json"
        command = [
            os.getenv("VIRA_ROS_PYTHON", "/usr/bin/python3"), str(script),
            "--kernel-url", bus.base_url, "--adapter-id", bus.observed_by,
            "--trace-id", request["trace_id"], "--parent-event-id", request["id"],
            "--mission-name", "Navigation and inspection" if inspection else "Spoken destination",
            "--rooms", destination, "--out", str(report),
        ]
        if inspection:
            command.append("--inspect-on-arrival")
        result = subprocess.run(command, stdout=subprocess.DEVNULL, timeout=450)
        if not report.exists():
            return "I could not verify completion of the mission."
        outcome = json.loads(report.read_text())
        if inspection and outcome.get("inspection_status") == "no_localized_objects":
            return f"I reached {destination}. " + outcome["feedback"]
        if result.returncode or not outcome.get("success"):
            return "I could not verify arrival."
        return f"I reached {destination}." + (" " + outcome["feedback"] if inspection else "")
