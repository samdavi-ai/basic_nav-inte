"""Read-only K visual grounding against a private kernel; injected telemetry fixture."""
import sys
import json
import os
import time
from uuid import uuid4
from pathlib import Path

import requests
from gracemo_sdk import AdapterClient

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/"scripts/validation"))
from kernel_context_demo import kernel


def test_missing_current_and_stale(tmp_path):
    with kernel(tmp_path) as url:
        client = AdapterClient("unit-vision",url)
        def query(name):
            return requests.post(url+"/k/execute",json={"expression":f'memory::recall[query:"{name}"]'},timeout=2)
        assert not query("person_present").ok
        sample = dict(ready=True,captured_at=time.time(),observations=[dict(class_name="person")],person_tracks=[dict(track_id="t")])
        def emit():
            assert client.emit("Custom",dict(name="rgbd_perception_status",payload=sample),source="Vision")
        emit()
        assert query("person_present").json()["result"]["stage_results"][0]["output"]=={"Bool":True}
        assert query("visible_objects").json()["result"]["stage_results"][0]["output"]=={"List":[{"String":"person"}]}
        sample["captured_at"]-=20
        emit()
        assert not query("person_present").ok
        assert not requests.get(url+"/perception",timeout=2).json()["current"]


def test_view_changes_need_comparable_samples(tmp_path):
    with kernel(tmp_path) as url:
        client = AdapterClient("unit-vision",url)
        assert client.emit("RobotPosition",dict(x=0.,y=0.,theta=0.,speed=0.))
        now = time.time()
        for offset, names in ((-.4, ["chair"]),(-.1, ["person"])):
            sample = dict(ready=True,captured_at=now+offset,camera_frame="camera_optical_link",
                          observations=[dict(class_name=name) for name in names])
            assert client.emit("Custom",dict(name="rgbd_perception_status",payload=sample),source="Vision")
        response = requests.post(url+"/k/execute",json={"expression":'memory::recall[query:"scene_changes"]'},timeout=2)
        response.raise_for_status()
        changes = response.json()["result"]["stage_results"][0]["output"]["Map"]
        assert changes["entered_view"] == {"List":[{"String":"person"}]}
        assert changes["left_view"] == {"List":[{"String":"chair"}]}


def test_historical_location_after_restart(tmp_path):
    with kernel(tmp_path) as url:
        client = AdapterClient("test-vision",url)
        assert client.emit("ObjectDetected",dict(class_name="fixture",confidence=.75,
            x=1.5,y=2.5,z=.9,frame_id="map"),source="Vision")
    with kernel(tmp_path) as url:
        response = requests.post(url+"/k/execute",json={
            "expression":'memory::recall[query:"last_observation",entity:"fixture"]'},timeout=2)
        response.raise_for_status()
        result = response.json()["result"]["stage_results"][0]["output"]["Map"]
        assert result["frame"] == {"String":"map"}
        assert result["event_time"]["Number"] > 0
        assert result["last_position"]["Coordinates"]["x"] == 1.5


def test_visual_archive_authorized_delete_survives_restart(tmp_path, monkeypatch):
    token = "visual-memory-test-token-" + "x" * 40
    grants = tmp_path / "grants.json"
    with open(grants, "x", opener=lambda path, flags: os.open(path, flags, 0o600)) as stream:
        json.dump([dict(id="visual-operator", token=token,
                        actions=["ReadVisualMemory", "DeleteVisualMemory"])], stream)
    monkeypatch.setenv("VIRA_ADAPTER_GRANTS", str(grants))
    headers = {"Authorization": "Bearer " + token}
    observation_id = str(uuid4())
    trace_id = str(uuid4())
    event = dict(id=observation_id, trace_id=trace_id, timestamp=int(time.time()),
                 source="Vision", observed_by="synthetic-rgbd", parent_event_id=None,
                 event_type=dict(type="ObjectDetected", data=dict(
                     class_name="archive_fixture", confidence=.91, x=2.0, y=1.0, z=.4,
                     frame_id="map", detection_id="det-1", track_id="track-4",
                     camera_id="front-rgbd", image_stamp_ns=123456789)))
    with kernel(tmp_path) as url:
        assert requests.get(url+"/visual/observations", timeout=2).status_code == 401
        assert requests.post(url+"/adapters/connect", headers=headers, json={}, timeout=2).ok
        assert requests.post(url+"/emit", json=event, timeout=2).ok
        records = requests.get(url+"/visual/observations", headers=headers, timeout=2).json()["observations"]
        assert len(records) == 1 and records[0]["observation_id"] == observation_id
        assert records[0]["trace_id"] == trace_id
        assert records[0]["track_id"] == "track-4"
        assert requests.post(url+f"/visual/observations/{observation_id}/delete",
                             headers=headers, json={"reason":"test privacy deletion"}, timeout=2).status_code == 204
        assert requests.get(url+"/visual/observations", headers=headers, timeout=2).json()["observations"] == []
        assert requests.get(url+"/world/objects?class=archive_fixture", timeout=2).json()["count"] == 0
    with kernel(tmp_path) as url:
        assert requests.post(url+"/adapters/connect", headers=headers, json={}, timeout=2).ok
        assert requests.get(url+"/visual/observations?include_history=true", headers=headers,
                            timeout=2).json()["observations"] == []
        history = requests.get(url+"/history", timeout=2).json()
        assert not any(row["id"] == observation_id for row in history)
