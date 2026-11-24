"""Measure the existing visual backend with a fixed Gazebo camera and no robot or navigation."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

from capture_apartment_views import ROOT, VIEWS
from launcher_recovery import private_domain, validation_pythonpath


def run(args):
    out=args.out.resolve();out.mkdir(parents=True,exist_ok=False)
    domain,lease=private_domain()
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
             GZ_PARTITION="vira-restart-static-"+uuid.uuid4().hex,GZ_IP="127.0.0.1",VIRA_DETECTION_HZ="10",
             VIRA_KERNEL_URL="http://127.0.0.1:9",PYTHONUNBUFFERED="1")
    env["PYTHONPATH"]=validation_pythonpath(ROOT,env.get("PYTHONPATH"))
    env["GZ_SIM_RESOURCE_PATH"]=os.pathsep.join([str(ROOT/"ros2_ws/src/gracemo_gazebo/models"),
                                   str(Path.home()/".gz/fuel/fuel.gazebosim.org/openrobotics/models")])
    env["ROS_LOG_DIR"]=str(out/"ros-logs")
    tree=ET.parse(ROOT/"ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world");world=tree.find("world")
    robot=world.find("model[@name='gracemo_vira']")
    sensors=[copy.deepcopy(robot.find(f".//sensor[@name='{n}']")) for n in ("camera","depth_camera")]
    world.remove(robot)
    for parent in world.iter():
        for child in list(parent):
            if child.tag in ("gui","sensor","state") or (child.tag=="plugin" and "Sensors" not in child.get("name","")):
                parent.remove(child)
    x,y,yaw=VIEWS[args.room]; x+=args.offset;yaw+=args.heading
    model=ET.SubElement(world,"model",name="stationary_rig")
    ET.SubElement(model,"static").text="true"
    ET.SubElement(model,"pose").text=f"{x} {y} 0.025 0 0 {yaw}"
    link=ET.SubElement(model,"link",name="mount")
    for sensor,info in zip(sensors,("/camera/camera_info","/camera/depth/camera_info")):
        sensor.find("pose").text=".112 0 1.02 0 0 0"
        sensor.find("update_rate").text="10"
        ET.SubElement(sensor,"gz_frame_id").text="camera_optical_link"
        ET.SubElement(sensor.find("camera"),"camera_info_topic").text=info
        link.append(sensor)
    commands={};children={};files=[]
    def start(name,argv):
        log=(out/(name+".log")).open("w"); files.append(log)
        children[name]=subprocess.Popen(argv,env=env,stdout=log,stderr=log,start_new_session=True)
        commands[name]=argv
    status=dict(room=args.room,fixture_xyz=[x,y,.025],yaw=yaw,domain=domain,partition=env["GZ_PARTITION"],duration_requested=args.seconds)
    (out/"process_lifecycle_evidence.json").write_text(json.dumps([dict(launcher_pid=os.getpid())]))
    with tempfile.TemporaryDirectory(prefix="vira-fixed-rig-") as tmp:
        path=Path(tmp)/"fixed.sdf";tree.write(path)
        try:
            start("renderer",["gz","sim","-s","-r","--headless-rendering",str(path)])
            start("bridge",["ros2","run","ros_gz_bridge","parameter_bridge",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"])
            start("mount",["ros2","run","tf2_ros","static_transform_publisher","--x",".112","--z","1.02",
                      "--roll","-1.5707963268","--yaw","-1.5707963268","--frame-id","base_footprint","--child-frame-id","camera_optical_link"])
            launch=["ros2","launch",str(ROOT/"ros2_ws/src/gracemo_nav2/launch/gazebo_rgbd_mapping.launch.py"),
                    f"database:={out/'map.db'}","visual_odometry:=true","benchmark_camera_rate:=true","distributed_features:=true"]
            if args.source:
                with args.source.open("rb") as f:sha=hashlib.file_digest(f,"sha256").hexdigest()
                launch.extend([f"source_map:={args.source.resolve()}",f"source_sha256:={sha}","blind_localization:=true"])
            start("1_gracemo_rgbd_slam",launch)
            start("recorder",[sys.executable,"scripts/validation/record_registration.py",str(out)])
            start("status_recorder",[sys.executable,"scripts/validation/record_visual_status.py",str(out/"visual_status.jsonl")])
            start("perception",[sys.executable,"-m","adapters.perception.perception_service","--ros-args","-p","use_sim_time:=true"])
            deadline=time.monotonic()+45;trace=out/"registration.jsonl"
            while time.monotonic()<deadline and all(c.poll() is None for c in children.values()):
                if trace.exists() and '"kind": "info"' in trace.read_text(): break
                time.sleep(.5)
            else:raise RuntimeError("No live camera odometry before startup deadline")
            start_time=time.monotonic();status["start_monotonic"]=start_time
            while time.monotonic()-start_time<args.seconds:
                for name,c in children.items():
                    if c.poll() is not None:raise RuntimeError(f"{name} exited during observation")
                time.sleep(.5)
            status["wall_seconds"]=time.monotonic()-start_time
        finally:
            for name,c in reversed(list(children.items())):
                if c.poll() is None:
                    os.killpg(c.pid,signal.SIGINT)
                    try:c.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(c.pid,signal.SIGTERM);c.wait(timeout=10)
            for f in files:f.close()
            status["exit_codes"]={n:c.returncode for n,c in children.items()}
            (out/"trial.json").write_text(json.dumps(status,indent=2))
            lease.close()


if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("out",type=Path)
    ap.add_argument("--seconds",type=int,required=True);ap.add_argument("--room",choices=VIEWS,default="study")
    ap.add_argument("--offset",type=float,default=0);ap.add_argument("--heading",type=float,default=0)
    ap.add_argument("--source",type=Path);run(ap.parse_args())
