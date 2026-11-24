#!/usr/bin/env python3
"""Read-only ROS/Gazebo recorder for the apartment routing acceptance runs."""
import argparse
import base64
import gzip
import json
import math
import signal
import time
import zlib
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from gz.msgs10.laserscan_pb2 import LaserScan as GzScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp(s):
    return s.sec + (s.nanosec if hasattr(s, "nanosec") else s.nsec) * 1e-9


def pose(p):
    q = p.orientation
    return [p.position.x, p.position.y,
            math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


class Recorder(Node):
    def __init__(self, output, dynamic=False, truth_file=None):
        super().__init__("routing_measurements")
        self.dynamic = dynamic
        if dynamic:
            from dwb_msgs.msg import LocalPlanEvaluation
            self.create_subscription(LocalPlanEvaluation, '/evaluation', self.on_evaluation, 10)
            self.create_subscription(Twist, '/validation/policy_proposal', lambda m:
                                     self.emit('policy_proposal',linear=m.linear.x,angular=m.angular.z),10)
            from nav2_msgs.msg import CollisionMonitorState
            import tf2_ros
            from action_msgs.msg import GoalStatusArray
            from nav2_msgs.action import ComputePathToPose
            self.result_type = ComputePathToPose.Impl.GetResultService
            self.plan_result = self.create_client(self.result_type, '/compute_path_to_pose/_action/get_result')
            self.seen_goals = set()
            self.create_subscription(GoalStatusArray,'/compute_path_to_pose/_action/status',self.plan_status,10)
            self.pending_depth = []
            self.create_subscription(String, "/dynamic_guard/state", lambda m:
                                     self.emit("guard", **json.loads(m.data)), 10)
            self.buffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buffer, self)
            self.create_subscription(CollisionMonitorState, "/collision_monitor_state", lambda m:
                self.emit("collision_monitor", action=m.action_type, polygon=m.polygon_name),10)
        self.file = output
        self.now = 0.
        self.truth_file = truth_file
        self.truth = None
        self.fixture_truth = None
        self.velocity = {}
        self.transforms = {}
        self.last_cloud = self.last_scan = -1.
        self.gz = GzNode()
        self.native_scan = None
        self.gz.subscribe(GzScan, "/scan", self.on_native_scan)
        self.gz.subscribe(Pose_V, "/world/gracemo_home/dynamic_pose/info", self.on_truth)
        self.create_subscription(Clock, "/clock", self.clock_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(RosPath, "/plan", self.on_plan, 10)
        self.create_subscription(RosPath, "/plan_smoothed", self.on_smoothed_plan, 10)
        self.create_subscription(RosPath, "/local_plan", self.on_local_plan, 10)
        self.create_subscription(TFMessage, "/tf", self.tf_cb, 30)
        self.create_subscription(String, "/autonomy/mission_status", self.status_cb, 10)
        self.create_subscription(PointCloud2, "/camera/depth/points", self.cloud_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        for name in ("cmd_vel_nav", "cmd_vel_guarded_nav", "cmd_vel_mux", "cmd_vel_smoothed", "cmd_vel_safe", "cmd_vel"):
            self.create_subscription(Twist, "/"+name, lambda m,k=name: self.on_command(k,m), 10)
        for name in ("local", "global"):
            prefix = f"/{name}_costmap/"
            self.create_subscription(Costmap, prefix+"costmap_raw", lambda m,k=name:self.cost_cb(k,m), 10)
            self.create_subscription(PolygonStamped, prefix+"published_footprint", lambda m,k=name:self.foot_cb(k,m), 10)

    def emit(self, kind, **fields):
        self.file.write(json.dumps(dict(t=self.now, kind=kind, **fields),separators=(',',':'))+'\n')

    def clock_cb(self, m):
        self.now = stamp(m.clock)
        if self.dynamic and self.pending_depth:
            pending, self.pending_depth = self.pending_depth, []
            for msg, points in pending:
                self.record_hits(msg, points, "depth")

    def on_evaluation(self, msg):
        policy = [s.raw_score for s in msg.twists[msg.best_index].scores
                  if s.name == 'PolicyPreference'] if msg.best_index < len(msg.twists) else []
        self.emit('dwb_evaluation', stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                  evaluated=len(msg.twists), rejected=sum(t.total < 0 for t in msg.twists),
                  best_policy_score=policy[0] if policy else None)

    def on_truth(self, m):
        for p in m.pose:
            if p.name == "gracemo_vira":
                self.truth = [stamp(m.header.stamp), *pose(p)]
                if self.truth_file is not None:
                    self.truth_file.write(json.dumps(self.truth)+'\n')
            elif self.dynamic and p.name == "routing_fixture":
                self.fixture_truth = [stamp(m.header.stamp), *pose(p), p.position.z]

    def tf_cb(self, m):
        for tf in m.transforms:
            if tf.header.frame_id == "map" and tf.child_frame_id == "odom":
                q = tf.transform.rotation
                self.transforms['map_odom'] = [tf.transform.translation.x, tf.transform.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))]

    def on_odom(self,m):
        if self.dynamic and self.fixture_truth is not None:
            self.emit('fixture_truth', pose=self.fixture_truth)
        self.emit('motion', stamp=stamp(m.header.stamp), odom=pose(m.pose.pose), truth=self.truth,
                  map_odom=self.transforms.get('map_odom'), velocities=self.velocity,
                  twist=[m.twist.twist.linear.x,m.twist.twist.angular.z])

    def on_command(self,name,m):
        self.velocity[name] = [m.linear.x,m.angular.z]
        if self.dynamic:
            self.emit("command",topic=name,twist=self.velocity[name])

    def record_hits(self,msg,points,source):
        import tf2_ros
        from rclpy.time import Time
        from scipy.spatial.transform import Rotation
        try:
            tf = self.buffer.lookup_transform("base_footprint",msg.header.frame_id,Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException:
            if source == "depth" and self.now - stamp(msg.header.stamp) < .5 and len(self.pending_depth)<8:
                self.pending_depth.append((msg, points))
            else:
                self.emit("missing_sensor_tf",source=source,stamp=stamp(msg.header.stamp))
            return
        q = tf.transform.rotation
        t = tf.transform.translation
        points = Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(points) + [t.x,t.y,t.z]
        if source=="depth":
            points = points[(points[:,2]>=.15)&(points[:,2]<=1.8)]
        # Quantize XY at 2 cm for offline association with measured world geometry.
        cells=np.unique(np.rint(points[:,:2]*50).astype("<i2"),axis=0)
        self.emit("sensor_hits",source=source,stamp=stamp(msg.header.stamp),
                  xy_zlib_b64=base64.b64encode(zlib.compress(cells.tobytes())).decode())

    def plan_status(self, msg):
        for status in msg.status_list:
            goal = status.goal_info
            key = bytes(goal.goal_id.uuid).hex()
            if key in self.seen_goals or not self.plan_result.service_is_ready():
                continue
            self.seen_goals.add(key)
            accepted = stamp(goal.stamp)
            query = self.result_type.Request(goal_id=goal.goal_id)
            def record(future, goal_id=key, start=accepted):
                if future.exception() is None:
                    response = future.result()
                    self.emit('planning_result', uuid=goal_id, accepted=start,
                              path_stamp=stamp(response.result.path.header.stamp),
                              duration=stamp(response.result.planning_time), status=response.status,
                              error_code=response.result.error_code)
            self.plan_result.call_async(query).add_done_callback(record)

    def on_plan(self,m):
        self.emit('plan', stamp=stamp(m.header.stamp), frame=m.header.frame_id, points=[pose(p.pose) for p in m.poses])

    def on_smoothed_plan(self,m):
        self.emit("smoothed_plan", stamp=stamp(m.header.stamp), frame=m.header.frame_id,
                  points=[pose(p.pose) for p in m.poses])

    def on_local_plan(self,m):
        self.emit("local_plan", stamp=stamp(m.header.stamp), frame=m.header.frame_id,
                  points=[pose(p.pose) for p in m.poses])

    def status_cb(self,m):
        status=json.loads(m.data)
        self.emit('status',state=status.get('state'),mission=status.get('mission'),robot=status.get('robot'))

    def cost_cb(self,name,m):
        md=m.metadata
        self.emit(name+'_costmap',frame=m.header.frame_id,stamp=stamp(md.update_time), published=stamp(m.header.stamp),
                  origin=pose(md.origin),size=[md.size_x,md.size_y],resolution=md.resolution,
                  cells_zlib_b64=base64.b64encode(zlib.compress(bytes(m.data))).decode())

    def foot_cb(self,name,m):
        self.emit(name+'_footprint',frame=m.header.frame_id,stamp=stamp(m.header.stamp),
                  corners=[[p.x,p.y] for p in m.polygon.points])

    def on_native_scan(self,m):
        self.native_scan = m

    def scan_cb(self,m):
        if self.dynamic:
            ranges=np.asarray(m.ranges)
            angle=m.angle_min+np.arange(len(ranges))*m.angle_increment
            valid=np.isfinite(ranges)&(ranges>=m.range_min)&(ranges<=m.range_max)
            self.record_hits(m,np.column_stack([ranges[valid]*np.cos(angle[valid]),
                            ranges[valid]*np.sin(angle[valid]),np.zeros(valid.sum())]),"scan")
        if self.now-self.last_scan < .5: return
        self.last_scan = self.now
        native = self.native_scan
        if native is not None:
            self.emit('gazebo_scan', stamp=stamp(native.header.stamp), sensor_pose=pose(native.world_pose),
                      angle_min=native.angle_min, step=native.angle_step,
                      ranges=[round(x,3) if math.isfinite(x) else None for x in native.ranges])
        self.emit('scan', stamp=stamp(m.header.stamp), angle_min=m.angle_min,step=m.angle_increment,
                  ranges=[round(x,3) if math.isfinite(x) else None for x in m.ranges])

    def cloud_cb(self,m):
        if self.dynamic:
            offsets={f.name:f.offset for f in m.fields}
            points=np.stack([np.ndarray((m.height,m.width),dtype=">f4" if m.is_bigendian else "<f4",
                      buffer=m.data,offset=offsets[k],strides=(m.row_step,m.point_step))[::3,::3]
                      for k in "xyz"],axis=-1).reshape(-1,3)
            self.record_hits(m,points[np.isfinite(points).all(axis=1)],"depth")
        if self.now-self.last_cloud < .5: return
        self.last_cloud = self.now
        fields={f.name:f.offset for f in m.fields}
        xyz=np.stack([np.ndarray((m.height,m.width),np.dtype('>f4' if m.is_bigendian else '<f4'),
                  buffer=m.data,offset=fields[k],strides=(m.row_step,m.point_step))[::12,::16] for k in 'xyz'],axis=-1)
        self.emit('cloud',stamp=stamp(m.header.stamp),frame=m.header.frame_id,
                  pixels=[[i*12,j*16,*map(float,p)] for i,row in enumerate(xyz) for j,p in enumerate(row) if np.isfinite(p).all()])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace',type=Path)
    parser.add_argument('--truth-file',type=Path)
    parser.add_argument('--duration',type=float,default=240.)
    parser.add_argument('--dynamic',action='store_true')
    args=parser.parse_args()
    rclpy.init()
    running=True
    def stop(*_):
        nonlocal running
        running=False
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    with gzip.open(args.trace,'xt') as output:
        truth_file=gzip.open(args.truth_file,'xt') if args.truth_file else None
        node=Recorder(output,args.dynamic,truth_file)
        try:
            deadline=time.monotonic()+args.duration
            while running and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
        finally:
            node.gz.unsubscribe('/scan')
            node.gz.unsubscribe('/world/gracemo_home/dynamic_pose/info')
            if truth_file is not None:
                truth_file.close()
            node.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    main()
