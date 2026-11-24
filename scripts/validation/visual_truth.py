"""Independent evaluator. Collect simulated base poses and the camera mounting transform."""
import json
from pathlib import Path
import sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from capture_routing import GzNode, Pose_V, stamp

rclpy.init()
ros = rclpy.create_node("visual_truth_evaluator")
buffer = Buffer()
listener = TransformListener(buffer,ros)
node = GzNode()
output_path = Path(sys.argv[1])
with output_path.open("x") as output:
    def sample(message):
        for obj in message.pose:
            if obj.name == "gracemo_vira":
                q, p = obj.orientation, obj.position
                print(json.dumps([stamp(message.header.stamp),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]]),file=output,flush=True)
                break
    node.subscribe(Pose_V,"/world/gracemo_home/dynamic_pose/info",sample)
    with output_path.with_name("camera_mount.jsonl").open("x") as mounts:
        while rclpy.ok():
            rclpy.spin_once(ros,timeout_sec=.1)
            try:
                tf = buffer.lookup_transform("base_footprint","camera_optical_link",Time())
            except TransformException:
                continue
            p,q = tf.transform.translation,tf.transform.rotation
            print(json.dumps([stamp(tf.header.stamp),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]]),file=mounts,flush=True)
