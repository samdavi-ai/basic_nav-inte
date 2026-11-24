"""Read-only ROS/Gazebo observer for private RTAB-Map trials."""
import json
import os
from pathlib import Path
import sys
import threading
import time

import psutil

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, JointState
from rtabmap_msgs.msg import Info, OdomInfo
from capture_routing import GzNode, Pose_V, stamp


def main(folder):
    if not os.environ.get('GZ_PARTITION','').startswith('vira-restart-'):
        raise ValueError('Private validation partition required')
    rclpy.init()
    node = rclpy.create_node('registration_evaluator')
    cv2.setNumThreads(1)
    orb = cv2.ORB_create()
    previous = None
    lock = threading.Lock()
    with (folder/'registration.jsonl').open('x', buffering=1) as stream:
        def record(kind, t, **values):
            with lock:
                print(json.dumps(dict(kind=kind,t=t,**values)), file=stream)

        def rgb(m):
            nonlocal previous
            if m.encoding not in ('rgb8','bgr8'):
                return
            a=np.ndarray((m.height,m.width,3),np.uint8,buffer=m.data,strides=(m.step,3,1))
            gray=cv2.cvtColor(a,cv2.COLOR_RGB2GRAY if m.encoding=='rgb8' else cv2.COLOR_BGR2GRAY)
            points = orb.detect(gray)
            grid = np.zeros((3, 4), dtype=int)
            for point in points:
                x, y = point.pt
                grid[int(3*y/m.height), int(4*x/m.width)] += 1
            record('rgb',stamp(m.header.stamp),frame=m.header.frame_id,encoding=m.encoding,
                   features=len(points), feature_grid=grid.tolist(), brightness=float(gray.mean()),
                   laplacian_variance=float(cv2.Laplacian(gray, cv2.CV_32F).var()),
                   delta=None if previous is None else float(cv2.absdiff(gray,previous).mean()))
            previous=gray

        def depth(m):
            if m.encoding!='32FC1':
                return
            a=np.ndarray((m.height,m.width),'>f4' if m.is_bigendian else '<f4',buffer=m.data,strides=(m.step,4))
            v=a[np.isfinite(a)&(a>0)]
            record('depth',stamp(m.header.stamp),frame=m.header.frame_id,encoding=m.encoding,
                   valid=v.size/a.size,quantiles=np.percentile(v,[0,25,50,75,100]).tolist() if v.size else [])

        def pose(kind,t,p):
            record(kind,t,xyz=[p.position.x,p.position.y,p.position.z],q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])

        gz=GzNode()
        def truth(m):
            for p in m.pose:
                if p.name=='gracemo_vira':
                    pose('truth',stamp(m.header.stamp),p)
                    break
        gz.subscribe(Pose_V,'/world/gracemo_home/dynamic_pose/info',truth)
        def neck(m):
            angles = {name: angle for name, angle in zip(m.name, m.position)
                      if name in ('neck_pan_joint', 'neck_tilt_joint')}
            if angles:
                record('neck', stamp(m.header.stamp), angles=angles)
        node.create_subscription(JointState, '/joint_states', neck, qos_profile_sensor_data)
        for cls,topic,cb in [(Image,'/camera/image_raw',rgb),(Image,'/camera/depth_image',depth),
                (OdomInfo,'/visual_slam/odom_info',lambda m:record('info',stamp(m.header.stamp),
                  **{k:getattr(m,k) for k in ('lost','matches','inliers','features','key_frame_added','time_estimation')})),
                (PoseWithCovarianceStamped,'/visual_slam/localization_pose',lambda m:pose('localization',stamp(m.header.stamp),m.pose.pose)),
                (Info,'/visual_slam/info',lambda m:record('graph',stamp(m.header.stamp),ref=m.ref_id,
                  loop=m.loop_closure_id,proximity=m.proximity_detection_id,
                  stats=dict(zip(m.stats_keys,m.stats_values)))),
                (Odometry,'/visual_slam/odom',lambda m:pose('odom',stamp(m.header.stamp),m.pose.pose))]:
            node.create_subscription(cls,topic,cb,qos_profile_sensor_data)
        for topic in ('/camera/camera_info','/camera/depth/camera_info'):
            node.create_subscription(CameraInfo,topic,lambda m,k=topic:record(k,stamp(m.header.stamp),
                frame=m.header.frame_id,k=list(m.k),width=m.width,height=m.height,model=m.distortion_model),qos_profile_sensor_data)
        watched = {}
        def resources():
            processes = []
            launcher = json.loads((folder/'process_lifecycle_evidence.json').read_text())[-1]['launcher_pid']
            for proc in psutil.Process(launcher).children(recursive=True):
                try:
                    if proc.name() not in ('rgbd_odometry','rtabmap'):
                        continue
                    if proc.pid not in watched:
                        watched[proc.pid] = proc
                        proc.cpu_percent()
                        continue
                    sample = watched[proc.pid]
                    processes.append(dict(name=proc.name(),rss=proc.memory_info().rss,cpu=sample.cpu_percent()))
                except (psutil.NoSuchProcess,psutil.AccessDenied):
                    continue
            database = folder/'map.db'
            record('resources',time.monotonic(),processes=processes,
                   database_bytes=database.stat().st_size if database.exists() else None,
                   available_bytes=psutil.virtual_memory().available)
        node.create_timer(2.,resources)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            gz.unsubscribe('/world/gracemo_home/dynamic_pose/info')
            node.destroy_node()
            rclpy.try_shutdown()


if __name__=='__main__':
    main(Path(sys.argv[1]))
