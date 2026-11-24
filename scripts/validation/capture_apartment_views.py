"""Acquire stationary camera observations; this renderer contains no robot or motion systems."""
import argparse
import copy
import functools
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

ROOT = Path(__file__).resolve().parents[2]
# Static sensor origins and headings. A different sensor represents each view.
VIEWS = {
    "hallway": (0, 0, 0), "kitchen": (-4, -2.5, -1.57),
    "bedroom": (-4.8, 2.8, 2.4), "study": (4.5, 2.8, 1.57),
    "living": (4, -2.5, -.32), "monitor_near": (7.25, -3.43, -.40),
}


def capture(out, lighting, occlusion):
    out.mkdir(parents=True, exist_ok=False)
    os.environ.update(GZ_PARTITION="vira-static-views-"+uuid.uuid4().hex, GZ_IP="127.0.0.1")
    tree = ET.parse(ROOT/"ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world")
    world = tree.find("world")
    robot = world.find("model[@name='gracemo_vira']")
    sensors = [copy.deepcopy(robot.find(f".//sensor[@name='{n}']")) for n in ("camera","depth_camera")]
    world.remove(robot)
    for parent in world.iter():
        for child in list(parent):
            if child.tag in ("sensor","gui","state") or (child.tag=="plugin" and "Sensors" not in child.get("name","")):
                parent.remove(child)
    if lighting == "dim":
        world.find("light/diffuse").text = ".23 .225 .22 1"
    if occlusion:
        panel = ET.fromstring("""<model name="screen_occluder"><static>true</static><pose>8.2 -4.40 1 0 0 0</pose><link name="panel"><visual name="slat"><geometry><box><size>.04 .30 .9</size></box></geometry><material><diffuse>.5 .5 .5 1</diffuse></material></visual></link></model>""")
        world.append(panel)
    topics = {}
    for room, (x,y,yaw) in VIEWS.items():
        for index, offset in enumerate((0, .7, 2.6)):
            view = f"{room}_{index}"
            model = ET.SubElement(world,"model",name=view)
            ET.SubElement(model,"static").text="true"
            ET.SubElement(model,"pose").text=f"{x} {y} 1.05 0 0 {yaw+offset}"
            link=ET.SubElement(model,"link",name="mount")
            for name, template in zip(("rgb","depth"),sensors):
                sensor = copy.deepcopy(template)
                sensor.find("pose").text="0 0 0 0 0 0"
                topic=f"/vira_static/{view}/{name}"
                sensor.find("topic").text=topic
                sensor.find("update_rate").text="1"
                link.append(sensor)
                topics[topic]=(view,name)
    fuel=Path.home()/".gz/fuel/fuel.gazebosim.org/openrobotics/models"
    os.environ["GZ_SIM_RESOURCE_PATH"]=os.pathsep.join(map(str,[ROOT/"ros2_ws/src/gracemo_gazebo/models",fuel,Path("/opt/ros/jazzy/share")]))
    node=Node()
    pending=set(topics)
    lock=threading.Lock()
    manifest={}
    def receive(topic, msg):
        with lock:
            if topic not in pending:
                return
            view,name=topics[topic]
            buf=np.frombuffer(msg.data,np.uint8)
            if name=="rgb":
                a=buf.reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
                if not cv2.imwrite(str(out/f"{view}.png"),cv2.cvtColor(a,cv2.COLOR_RGB2BGR)):
                    raise IOError(view)
            else:
                a=buf.reshape(msg.height,msg.step)[:,:msg.width*4].copy().view('<f4')
                np.save(out/f"{view}.npy",a)
            manifest[topic]=dict(stamp=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9,width=msg.width,height=msg.height)
            pending.remove(topic)
    callbacks=[functools.partial(receive,t) for t in topics]
    for t,cb in zip(topics,callbacks):
        node.subscribe(Image,t,cb)
    with tempfile.TemporaryDirectory() as tmp, (out/"renderer.log").open("w") as log:
        path=Path(tmp)/"cameras.sdf";tree.write(path)
        proc=subprocess.Popen(["gz","sim","-s","-r","--headless-rendering",str(path)],stdout=log,stderr=log,start_new_session=True)
        try:
            deadline=time.monotonic()+90
            while pending and time.monotonic()<deadline and proc.poll() is None:
                time.sleep(.1)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)
    (out/"capture.json").write_text(json.dumps(dict(lighting=lighting,partial_occlusion=occlusion,views=VIEWS,
                                                   captured=manifest,missing=sorted(pending)),indent=2))
    if pending:raise RuntimeError(f"{len(pending)} camera topics missing")


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("out",type=Path)
    parser.add_argument("--lighting",choices=("original","dim"),default="original")
    parser.add_argument("--partial-occlusion",action="store_true")
    args=parser.parse_args()
    capture(args.out,args.lighting,args.partial_occlusion)
