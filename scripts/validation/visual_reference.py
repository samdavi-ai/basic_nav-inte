"""Stationary RGB-D target in a private Gazebo world, removed before dispatch."""
import json
import os
from pathlib import Path
import sys
import time
from uuid import uuid4
from xml.sax.saxutils import escape

import numpy as np
import requests
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.transport13 import Node
from capture_routing import Pose_V, stamp
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from scipy.spatial.transform import Rotation

if not os.environ.get("GZ_PARTITION","").startswith("vira-restart-"):
    raise RuntimeError("Expected private validation partition")
out,url = Path(sys.argv[1]),sys.argv[2]
image = Path(os.environ["VIRA_REFERENCE_IMAGE"]).resolve(strict=True)
name = "visual_reference_"+uuid4().hex
# A Gazebo board depicts people. This tests imagery, never identity or live actors.
sdf = f"""<sdf version="1.9"><model name="{name}"><static>true</static><pose>2 0 1.02 0 0 0</pose>
<link name="board"><visual name="image"><geometry><box><size>.04 2.2 1.24</size></box></geometry>
<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{escape(image.as_uri())}</albedo_map><metalness>0</metalness><roughness>1</roughness>
</metal></pbr></material></visual></link></model></sdf>"""
gz = Node()
rclpy.init()
ros = rclpy.create_node("visual_reference_observer")
values = []
comparisons = []
poses = []
buffer = Buffer()
listener = TransformListener(buffer,ros)
def truth(message):
    for obj in message.pose:
        if obj.name == "gracemo_vira":
            p,q = obj.position,obj.orientation
            poses.append((stamp(message.header.stamp),np.array([p.x,p.y,p.z]),Rotation.from_quat([q.x,q.y,q.z,q.w])))
            del poses[:-100]
            break
gz.subscribe(Pose_V,"/world/gracemo_home/dynamic_pose/info",truth)

def depth(msg):
    if msg.encoding=="32FC1":
        data = np.ndarray((msg.height,msg.width),dtype=">f4" if msg.is_bigendian else "<f4",
                          buffer=msg.data,strides=(msg.step,4))
        values.append(float(np.median(data[msg.height//2-2:msg.height//2+3,msg.width//2-2:msg.width//2+3])))
        try:
            ts = stamp(msg.header.stamp)
            if not poses:
                return
            gps,base,attitude = min(poses.copy(),key=lambda row:abs(row[0]-ts))
            tf = buffer.lookup_transform("base_footprint",msg.header.frame_id,Time()).transform
            mount,q = tf.translation,tf.rotation
            origin = base + attitude.apply([mount.x,mount.y,mount.z])
            direction = attitude.apply(Rotation.from_quat([q.x,q.y,q.z,q.w]).apply([0,0,1]))
            expected = (1.98-origin[0])/direction[0]
            hit = origin + direction*expected
            if abs(ts-gps)<.10 and abs(hit[1])<1.1 and abs(hit[2]-1.02)<.62 and time.monotonic()-start>2:
                comparisons.append(dict(stamp=ts, expected=float(expected), measured=values[-1]))
        except TransformException:
            pass
ros.create_subscription(Image,"/camera/depth_image",depth,qos_profile_sensor_data)
evidence = []
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
for _ in range(10):
    ready,_ = gz.request("/world/gracemo_home/scene/info",Empty(),Empty,Scene,1000)
    if ready:
        break
    time.sleep(.2)
ok,reply = gz.request("/world/gracemo_home/create",EntityFactory(sdf=sdf),EntityFactory,Boolean,5000)
if not ok or not reply.data:
    visible,scene = gz.request("/world/gracemo_home/scene/info",Empty(),Empty,Scene,2000)
    if not visible or not any(model.name==name for model in scene.model):
        raise RuntimeError("Target insertion unconfirmed")
try:
    start = time.monotonic()
    end = start+12
    while time.monotonic()<end:
        rclpy.spin_once(ros,timeout_sec=.3)
        time.sleep(.06)
        sample=requests.get(url+"/perception",timeout=2).json()
        if sample.get("current"):
            evidence.append(sample["sample"])
finally:
    ok,reply=gz.request("/world/gracemo_home/remove",Entity(name=name,type=Entity.MODEL),Entity,Boolean,5000)
    ros.destroy_node()
    rclpy.shutdown()
    (out/"reference.json").write_text(json.dumps(dict(medians_m=values,observations=evidence,
        comparisons=comparisons,tolerance_m=.05,nominal_front_x=1.98,nominal_camera_x=.112,removed=bool(ok and reply.data)),indent=2))
    if not ok or not reply.data:
        raise RuntimeError("Target removal unconfirmed")
