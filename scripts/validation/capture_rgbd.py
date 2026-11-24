"""Record stamped, calibrated RGB-D pairs from the private Gazebo graph."""
import argparse
import json
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def ns(msg):
    return msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec


class Recorder(Node):
    def __init__(self, folder):
        super().__init__('validation_rgbd_capture')
        self.folder = folder
        folder.mkdir()
        self.count = 0
        self.sources = [message_filters.Subscriber(self, kind, topic,qos_profile=qos_profile_sensor_data)
                        for kind,topic in [(Image,'/camera/image_raw'),(Image,'/camera/depth_image'),
                                           (CameraInfo,'/camera/camera_info'),(CameraInfo,'/camera/depth/camera_info')]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.sources,10,.025)
        self.sync.registerCallback(self.record)

    def record(self, rgb, depth, color_info, depth_info):
        if (rgb.encoding != 'rgb8' or depth.encoding != '32FC1' or
                len({m.header.frame_id for m in (rgb, depth, color_info, depth_info)}) != 1 or
                (rgb.width,rgb.height) != (color_info.width,color_info.height) or
                (depth.width,depth.height) != (depth_info.width,depth_info.height)):
            return
        color = np.ndarray((rgb.height,rgb.width,3),np.uint8,buffer=rgb.data,strides=(rgb.step,3,1))
        metric = np.ndarray((depth.height,depth.width),'>f4' if depth.is_bigendian else '<f4',
                            buffer=depth.data,strides=(depth.step,4))
        prefix = f'{self.count:06d}'
        if not cv2.imwrite(str(self.folder/(prefix+'.png')),cv2.cvtColor(color,cv2.COLOR_RGB2BGR)):
            raise IOError('Could not save RGB image')
        np.save(self.folder/(prefix+'.npy'),metric)
        with (self.folder/'frames.jsonl').open('a') as out:
            print(json.dumps(dict(prefix=prefix,rgb_ns=ns(rgb),depth_ns=ns(depth),
                                  frame=rgb.header.frame_id,K=list(color_info.k),Kd=list(depth_info.k))),file=out)
        self.count += 1


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=Recorder(args.folder)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
