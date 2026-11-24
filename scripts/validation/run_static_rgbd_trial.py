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
            if child.tag in ("gui","sensor","state") or (child.tag=="plugin" and child.get("name","").split("::")[-1] not in ("Sensors","SceneBroadcaster")):
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
        if sensor.get("type")=="depth_camera":
            # Harmonic derives its depth calibration topic from the parent
            # namespace of the depth image, even with camera_info_topic set.
            sensor.find("topic").text="/camera/depth/image"
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
                "/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "--ros-args","-r","/camera/depth/image:=/camera/depth_image"])
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
                if trace.exists():
                    # A partially written last line is possible with a live recorder.
                    lines=trace.read_bytes().split(b"\n")[:-1]
                    rows=[json.loads(line) for line in lines]
                    rgb={(r["width"],r["height"]) for r in rows if r["kind"]=="/camera/camera_info"}
                    depth={(r["width"],r["height"]) for r in rows if r["kind"]=="/camera/depth/camera_info"}
                    if rgb=={(320,240)} and depth=={(160,120)} and any(r["kind"]=="info" for r in rows):
                        break
                time.sleep(.5)
            else:raise RuntimeError("No live camera odometry before startup deadline")
            if args.source:
                # Read the actual, immutable rig pose from Gazebo in the private partition.
                # This subprocess has no ROS interface and cannot supply the estimator.
                result=subprocess.run([sys.executable, __file__, "--read-scene-pose"],
                                      env=env, capture_output=True, text=True, check=True, timeout=12)
                status["gazebo_pose"]=json.loads(result.stdout)
                status["source_sha256"]=sha
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
    if sys.argv[1:] == ["--read-scene-pose"]:
        from gz.transport13 import Node
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.scene_pb2 import Scene
        node=Node()
        for attempt in range(4):
            ok,scene=node.request("/world/gracemo_home/scene/info",Empty(),Empty,Scene,2000)
            rig=next((m for m in scene.model if m.name=="stationary_rig"),None) if ok else None
            if rig is not None:
                p,q=rig.pose.position,rig.pose.orientation
                print(json.dumps(dict(xyz=[p.x,p.y,p.z],q=[q.x,q.y,q.z,q.w])))
                sys.exit(0)
        sys.exit("Fixed sensor model absent from live Gazebo scene")
    ap=argparse.ArgumentParser();ap.add_argument("out",type=Path)
    ap.add_argument("--seconds",type=int,required=True);ap.add_argument("--room",choices=VIEWS,default="study")
    ap.add_argument("--offset",type=float,default=0);ap.add_argument("--heading",type=float,default=0)
    ap.add_argument("--source",type=Path);run(ap.parse_args())
