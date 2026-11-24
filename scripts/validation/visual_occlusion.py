"""Occlude the live camera during a stationary, private saved-map diagnostic."""
import json
import os
from pathlib import Path
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from gz.transport13 import Node


def run(folder):
    if (not os.environ.get("GZ_PARTITION", "").startswith("vira-restart-")
            or os.environ.get("VIRA_TEST_STATIONARY_RELOAD") != "1"
            or os.environ.get("VIRA_TEST_OFFSET_START") != "1"):
        raise ValueError("Stationary offset reload in a private Gazebo partition required")
    node = Node()
    name = "registration_occlusion_panel"
    geometry = "<geometry><box><size>0.12 3 2.6</size></box></geometry>"
    model = (f"<sdf version='1.9'><model name='{name}'><static>true</static><pose>1.2 0 1.3 0 0 0</pose>"
             f"<link name='panel'><collision name='c'>{geometry}</collision><visual name='v'>{geometry}"
             "<material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>"
             "</visual></link></model></sdf>")
    def mark():
        # Observer RGB timestamps bracket the service response to one camera interval.
        with (folder / "registration.jsonl").open() as stream:
            stamps = [json.loads(line)["t"] for line in stream
                      if line.endswith("\n") and '"kind": "rgb"' in line]
        return stamps[-1]

    events = {}
    time.sleep(5)
    ok, reply = node.request("/world/gracemo_home/create", EntityFactory(sdf=model), EntityFactory, Boolean, 5000)
    if not ok or not reply.data:
        raise RuntimeError("Gazebo did not insert the panel")
    try:
        events["inserted_rgb_stamp"] = mark()
        time.sleep(6)
    finally:
        ok, reply = node.request("/world/gracemo_home/remove", Entity(name=name, type=Entity.MODEL),
                                 Entity, Boolean, 5000)
        if not ok or not reply.data:
            raise RuntimeError("Gazebo did not remove the panel")
    events["removed_rgb_stamp"] = mark()
    (folder / "occlusion.json").write_text(json.dumps(events, indent=2))
    time.sleep(24)


if __name__ == "__main__":
    run(Path(sys.argv[1]))
