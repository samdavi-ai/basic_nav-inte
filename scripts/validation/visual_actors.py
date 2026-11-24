"""Cross two animated actors in front of the stationary camera in a private Gazebo world."""
from collections import OrderedDict
import json
import threading
import tempfile

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image, RGB_INT8
import os
from pathlib import Path
import sys
import time
from uuid import uuid4
from xml.sax.saxutils import escape

import requests
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.entity_pb2 import Entity
from capture_routing import GzNode, Pose_V, stamp


def run(folder, url, mesh):
    if not os.environ.get("GZ_PARTITION", "").startswith("vira-restart-"):
        raise ValueError("Private validation world required")
    gz = GzNode()
    original = mesh.resolve(strict=True).read_text()
    sweater = '0.09837106 0.254824 0.07500499 1'
    if sweater not in original:
        raise ValueError("Expected the Gazebo Classic green sweater material")
    meshes = tempfile.TemporaryDirectory(prefix="vira-actors-")
    images = OrderedDict()
    lock = threading.Lock()
    def capture(msg):
        if msg.pixel_format_type != RGB_INT8:
            return
        ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nsec
        with lock:
            images[ns] = np.ndarray((msg.height,msg.width,3),np.uint8,buffer=msg.data,
                                   strides=(msg.step,3,1)).copy()
            while len(images) > 40:
                images.popitem(last=False)
    gz.subscribe(Image,"/camera/image_raw",capture)
    frames = folder/"actor_frames"
    frames.mkdir()

    names = [f"walking_{i}_{uuid4().hex[:8]}" for i in range(2)]
    world = "/world/gracemo_home"
    created = []
    samples = {}
    with (folder/"actor_truth.jsonl").open("x") as truth:
        def observe(msg):
            for pose in msg.pose:
                if pose.name in names or pose.name == "gracemo_vira":
                    p = pose.position
                    print(json.dumps([stamp(msg.header.stamp),pose.name,p.x,p.y,p.z]), file=truth,flush=True)
        gz.subscribe(Pose_V, world+"/pose/info", observe)
        try:
            for _ in range(12):
                online, _scene = gz.request(world+"/scene/info",Empty(),Empty,Scene,1000)
                if online:
                    break
                time.sleep(.2)
            for i,name in enumerate(names):
                derived = Path(meshes.name)/f"walking_{i}.dae"
                color = ("0.95 0.015 0.75 1", "0.01 0.85 0.85 1")[i]
                derived.write_text(original.replace(sweater,color))
                mesh_uri = escape(str(derived))
                x, y = (2.5, -1.) if i == 0 else (3.1, 1.)
                heading = 1.571 if y < 0 else -1.571
                waypoints = "".join(f"<waypoint><time>{t}</time><pose>{x} {lat} 1 0 0 {yaw}</pose></waypoint>"
                                    for t,lat,yaw in [(0,y,heading),(4,-y,heading),(5,-y,-heading),
                                                      (9,y,-heading),(10,y,heading)])
                sdf = f"""<sdf version="1.9"><actor name="{name}"><skin><filename>{mesh_uri}</filename></skin>
                <animation name="walk"><filename>{mesh_uri}</filename><interpolate_x>true</interpolate_x></animation>
                <script><loop>true</loop><auto_start>true</auto_start><trajectory id="0" type="walk">
                {waypoints}</trajectory></script></actor></sdf>"""
                ok, reply = gz.request(world+"/create", EntityFactory(sdf=sdf),EntityFactory,Boolean,5000)
                if not ok or not reply.data:
                    ok, scene = gz.request(world+"/scene/info",Empty(),Empty,Scene,2000)
                    if not ok or not any(actor.name == name for actor in scene.model):
                        raise RuntimeError(f"Actor creation unconfirmed: {name}")
                created.append(name)
            deadline = time.monotonic()+22
            while time.monotonic() < deadline:
                data = requests.get(url+"/perception",timeout=2).json()
                if data.get("current"):
                    s = data["sample"]
                    ns = s["image_stamp_ns"]
                    with lock:
                        frame = images.get(ns)
                    if frame is not None and not (frames/f"{ns}.png").exists():
                        cv2.imwrite(str(frames/f"{ns}.png"),cv2.cvtColor(frame,cv2.COLOR_RGB2BGR))
                    samples[s["image_stamp_ns"]] = dict(stamp_ns=s["image_stamp_ns"],
                        people=s.get("person_tracks", []), tracking_ms=s.get("tracking_ms"),
                        cpu=s.get("cpu_percent"),rss=s.get("process_rss_bytes"))
                time.sleep(.15)
        finally:
            removed = []
            for name in created:
                ok, reply = gz.request(world+"/remove", Entity(name=name,type=Entity.ACTOR),Entity,Boolean,5000)
                if ok and reply.data:
                    removed.append(name)
            gz.unsubscribe(world+"/pose/info")
            gz.unsubscribe("/camera/image_raw")
            meshes.cleanup()
            (folder/"actors.json").write_text(json.dumps(dict(created=created,removed=removed,
                                                    samples=list(samples.values())),indent=2))
            if removed != created:
                raise RuntimeError("Actor removal unconfirmed")


if __name__ == "__main__":
    run(Path(sys.argv[1]),sys.argv[2],Path(os.environ["VIRA_WALK_MESH"]))
