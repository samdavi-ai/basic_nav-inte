#!/usr/bin/env python3
"""
GRaCEmo ViRa — Native Gazebo Harmonic to ROS 2 Physical Sensor Bridge Node
Authoritatively bridges:
  1. LiDAR:               Gazebo /scan               -> ROS 2 /scan (sensor_msgs/msg/LaserScan)
  2. TF & Odometry:       Gazebo /odom               -> ROS 2 /odom (nav_msgs/msg/Odometry) & TF (odom -> base_footprint, world -> odom)
  3. Joint States:        Gazebo joint_state (Model) -> ROS 2 /joint_states (sensor_msgs/msg/JointState)
  4. IMU:                 Gazebo /imu/data           -> ROS 2 /imu/data (sensor_msgs/msg/Imu)
  5. RGB Camera & Info:   Gazebo /camera/image_raw   -> ROS 2 /camera/image_raw & /camera/camera_info
  6. Depth Camera & Info: Gazebo /camera/depth_image -> ROS 2 /camera/depth_image & /camera/depth/camera_info
  7. PointCloud2:         Gazebo .../points          -> ROS 2 /camera/depth/points (sensor_msgs/msg/PointCloud2)
  8. Velocity Commands:   ROS 2 /cmd_vel             -> Gazebo /cmd_vel (gz.msgs.Twist)
"""

import math
import time
from typing import Optional
import numpy as np

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import (
    LaserScan as RosLaserScan,
    JointState as RosJointState,
    Imu as RosImu,
    Image as RosImage,
    CameraInfo as RosCameraInfo,
    PointCloud2 as RosPointCloud2,
    PointField as RosPointField,
)
from rosgraph_msgs.msg import Clock as RosClock
from nav_msgs.msg import Odometry as RosOdometry
from geometry_msgs.msg import Twist as RosTwist, TransformStamped
import tf2_ros

# Gazebo Harmonic Transport & Messages
import gz.transport13 as gz_transport
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.odometry_pb2 import Odometry as GzOdometry
from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
from gz.msgs10.image_pb2 import Image as GzImage
from gz.msgs10.imu_pb2 import IMU as GzIMU
from gz.msgs10.model_pb2 import Model as GzModel
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked as GzPointCloudPacked
from gz.msgs10.clock_pb2 import Clock as GzClock
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__("gracemo_sensor_bridge")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        if not self.has_parameter("publish_world_tf"):
            self.declare_parameter("publish_world_tf", False)

        # ── 1. Setup ROS 2 Publishers ──
        self.pub_clock = self.create_publisher(RosClock, "/clock", 10)
        self.pub_scan = self.create_publisher(RosLaserScan, "/scan", 10)
        self.pub_joint_states = self.create_publisher(RosJointState, "/joint_states", 20)
        self.pub_odom = self.create_publisher(RosOdometry, "/odom", 20)
        self.pub_imu = self.create_publisher(RosImu, "/imu/data", 20)
        self.pub_rgb_image = self.create_publisher(RosImage, "/camera/image_raw", 10)
        self.pub_rgb_info = self.create_publisher(RosCameraInfo, "/camera/camera_info", 10)
        self.pub_depth_image = self.create_publisher(RosImage, "/camera/depth_image", 10)
        self.pub_depth_info = self.create_publisher(RosCameraInfo, "/camera/depth/camera_info", 10)
        self.pub_depth_points = self.create_publisher(RosPointCloud2, "/camera/depth/points", 10)

        # ── 2. Setup TF Broadcasters ──
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Ground-truth tracking for dynamic world -> odom alignment & authoritative odometry
        self._spawn_world_x: Optional[float] = None
        self._spawn_world_y: Optional[float] = None
        self._spawn_world_z: Optional[float] = None
        self._spawn_world_yaw: float = 0.0
        self._spawn_world_q = (0.0, 0.0, 0.0, 1.0)

        self._world_x: float = 0.0
        self._world_y: float = 0.0
        self._world_z: float = 0.025
        self._world_yaw: float = 0.0
        self._world_q = (0.0, 0.0, 0.0, 1.0)
        self._has_world_pose: bool = False

        self._last_odom_x: float = 0.0
        self._last_odom_y: float = 0.0
        self._last_odom_yaw: float = 0.0
        self._last_odom_time_sec: float = 0.0
        self._last_published_tf_time: float = 0.0
        self._calc_vx: float = 0.0
        self._calc_vy: float = 0.0
        self._calc_wz: float = 0.0

        # ── 3. Setup ROS 2 Subscribers ──
        self.sub_cmd_vel = self.create_subscription(
            RosTwist, "/cmd_vel", self._on_ros_cmd_vel, 10
        )

        # ── 4. Precompute Optical Camera Intrinsics & PointCloud Fields ──
        # RGB: 320x240, hfov=1.15 rad (matches the Gazebo apartment model)
        self.rgb_info_msg = self._build_camera_info(
            width=320, height=240, hfov=1.15, frame_id="camera_optical_link"
        )
        # Depth: 160x120, hfov=1.15 rad (matches the Gazebo apartment model)
        self.depth_info_msg = self._build_camera_info(
            width=160, height=120, hfov=1.15, frame_id="camera_optical_link"
        )
        # Precomputed PointField descriptors (FLOAT32 = 7)
        self.pc_fields = []
        for name, offset in [("x", 0), ("y", 4), ("z", 8), ("rgb", 16)]:
            pf = RosPointField()
            pf.name = name
            pf.offset = offset
            pf.datatype = RosPointField.FLOAT32
            pf.count = 1
            self.pc_fields.append(pf)

        self._last_js_time = 0.0
        self._last_imu_time = 0.0
        self._last_pc_time = 0.0
        rgb_hz = float(self.declare_parameter("rgb_publish_hz",10.).value)
        depth_hz = float(self.declare_parameter("depth_image_publish_hz",5.).value)
        if not all(.1 <= rate <= 60 for rate in (rgb_hz,depth_hz)):
            raise ValueError("Camera publication rates must be between 0.1 and 60 Hz")
        self.rgb_period_ns = round(1_000_000_000/rgb_hz)
        self.depth_image_period_ns = round(1_000_000_000/depth_hz)
        self._last_depth_ns = None
        self._last_rgb_ns = None

        # ── 5. Setup Gazebo Harmonic Transport Subscriptions ──
        self.gz_node = gz_transport.Node()

        # Clock
        self.gz_node.subscribe(GzClock, "/clock", self._on_gz_clock)

        # Velocity commanding
        self.gz_cmd_pub = self.gz_node.advertise("/cmd_vel", GzTwist)

        # Sensor callbacks
        self.gz_node.subscribe(GzLaserScan, "/scan", self._on_gz_scan)
        self.gz_node.subscribe(
            GzModel,
            "/world/gracemo_home/model/gracemo_vira/joint_state",
            self._on_gz_joint_state,
        )
        self.gz_node.subscribe(GzOdometry, "/odom", self._on_gz_odom)
        self.gz_node.subscribe(GzIMU, "/imu/data", self._on_gz_imu)
        self.gz_node.subscribe(GzImage, "/camera/image_raw", self._on_gz_rgb_image)
        self.gz_node.subscribe(GzImage, "/camera/depth_image", self._on_gz_depth_image)
        self.gz_node.subscribe(
            GzPointCloudPacked,
            "/camera/depth_image/points",
            self._on_gz_depth_points,
        )
        self.gz_node.subscribe(
            GzPoseV,
            "/world/gracemo_home/dynamic_pose/info",
            self._on_gz_pose_v,
        )

        self.get_logger().info("✓ GRaCEmo Sensor Bridge initialized: All physical Gazebo sensors mapped to ROS 2!")

    def _to_ros_time(self, gz_header) -> Time:
        """Extract ROS 2 timestamp from Gazebo Header with fallback to node clock."""
        if gz_header and gz_header.HasField("stamp") and (gz_header.stamp.sec > 0 or gz_header.stamp.nsec > 0):
            return Time(seconds=gz_header.stamp.sec, nanoseconds=gz_header.stamp.nsec).to_msg()
        return self.get_clock().now().to_msg()

    def _on_gz_pose_v(self, msg: GzPoseV):
        """Authoritatively tracks true world pose of gracemo_vira model from Gazebo."""
        for p in msg.pose:
            if p.name == "gracemo_vira":
                px = float(p.position.x)
                py = float(p.position.y)
                pz = float(p.position.z)
                q = p.orientation
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)

                if self._spawn_world_x is None:
                    self._spawn_world_x = px
                    self._spawn_world_y = py
                    self._spawn_world_z = pz
                    self._spawn_world_yaw = yaw
                    self._spawn_world_q = (float(q.x), float(q.y), float(q.z), float(q.w))
                    self._last_odom_x = 0.0
                    self._last_odom_y = 0.0
                    self._last_odom_yaw = 0.0
                    self.get_logger().info(
                        f"📍 Authoritative spawn pose locked: x={px:.3f}, y={py:.3f}, z={pz:.3f}, yaw={yaw:.3f}"
                    )

                self._world_x = px
                self._world_y = py
                self._world_z = pz
                self._world_yaw = yaw
                self._world_q = (float(q.x), float(q.y), float(q.z), float(q.w))
                self._has_world_pose = True
                break

    def _build_camera_info(self, width: int, height: int, hfov: float, frame_id: str) -> RosCameraInfo:
        """Construct static CameraInfo matching pinhole camera optics."""
        info = RosCameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        fx = (width / 2.0) / math.tan(hfov / 2.0)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = frame_id
        return info

    # ── Gazebo Callbacks ──

    def _on_gz_scan(self, msg: GzLaserScan):
        ros_scan = RosLaserScan()
        ros_scan.header.stamp = self._to_ros_time(msg.header)
        ros_scan.header.frame_id = "lidar_link"

        ros_scan.angle_min = float(msg.angle_min)
        ros_scan.angle_max = float(msg.angle_max)
        ros_scan.angle_increment = float(msg.angle_step)
        ros_scan.time_increment = 0.0
        ros_scan.scan_time = 1.0 / 15.0
        ros_scan.range_min = float(msg.range_min)
        ros_scan.range_max = float(msg.range_max)

        # Self-exclusion filter: Eliminate self-body reflections from robot chassis, wheels, and torso
        # Robot footprint bounds in base_footprint frame: [-0.35, 0.35] x [-0.35, 0.35]
        # LiDAR origin in base_footprint frame: x = 0.11m, y = 0.00m
        raw_ranges = [float(r) for r in msg.ranges]
        filtered_ranges = []
        angle = ros_scan.angle_min
        for r in raw_ranges:
            if math.isinf(r) or math.isnan(r) or r < 0.18:
                filtered_ranges.append(float("inf"))
            else:
                x_bf = 0.11 + r * math.cos(angle)
                y_bf = r * math.sin(angle)
                if -0.35 <= x_bf <= 0.35 and -0.35 <= y_bf <= 0.35:
                    filtered_ranges.append(float("inf"))
                else:
                    filtered_ranges.append(r)
            angle += ros_scan.angle_increment

        ros_scan.ranges = filtered_ranges
        ros_scan.intensities = [float(i) for i in msg.intensities]

        self.pub_scan.publish(ros_scan)

    def _on_gz_clock(self, msg: GzClock):
        ros_clock = RosClock()
        ros_clock.clock.sec = int(msg.sim.sec)
        ros_clock.clock.nanosec = int(msg.sim.nsec)
        self.pub_clock.publish(ros_clock)

    def _on_gz_joint_state(self, msg: GzModel):
        now = time.time()
        if now - self._last_js_time < 0.033:  # Throttle to 30 Hz
            return
        self._last_js_time = now

        ros_js = RosJointState()
        ros_js.header.stamp = self._to_ros_time(msg.header)
        ros_js.header.frame_id = ""

        for j in msg.joint:
            ros_js.name.append(j.name)
            if j.HasField("axis1"):
                ros_js.position.append(float(j.axis1.position))
                ros_js.velocity.append(float(j.axis1.velocity))
            else:
                ros_js.position.append(0.0)
                ros_js.velocity.append(0.0)
            ros_js.effort.append(0.0)

        self.pub_joint_states.publish(ros_js)

    def _on_gz_odom(self, msg: GzOdometry):
        stamp = self._to_ros_time(msg.header)
        curr_time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

        # Filter out backwards or duplicate timestamps to guarantee strictly monotonic TF stream
        if self._last_published_tf_time > 0.0:
            if curr_time_sec < self._last_published_tf_time - 2.0:
                # Simulation restarted or jumped back in time; rebase
                self._last_published_tf_time = curr_time_sec
            elif curr_time_sec <= self._last_published_tf_time:
                # Duplicate or out-of-order packet; skip to prevent TF_OLD_DATA warnings
                return
        self._last_published_tf_time = curr_time_sec

        # Authoritatively derive odom pose and velocity from Gazebo physical model kinematics
        if self._has_world_pose and self._spawn_world_x is not None:
            dx = self._world_x - self._spawn_world_x
            dy = self._world_y - self._spawn_world_y
            dz = self._world_z - self._spawn_world_z

            syaw = self._spawn_world_yaw
            cos_s = math.cos(syaw)
            sin_s = math.sin(syaw)
            odom_x = cos_s * dx + sin_s * dy
            odom_y = -sin_s * dx + cos_s * dy
            odom_z = dz

            odom_yaw = self._world_yaw - self._spawn_world_yaw
            odom_yaw = (odom_yaw + math.pi) % (2.0 * math.pi) - math.pi

            qx = 0.0
            qy = 0.0
            qz = math.sin(odom_yaw / 2.0)
            qw = math.cos(odom_yaw / 2.0)

            # Compute true physical velocities
            curr_time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            dt = curr_time_sec - self._last_odom_time_sec
            if 0.005 <= dt <= 0.5:
                vx_w = (odom_x - self._last_odom_x) / dt
                vy_w = (odom_y - self._last_odom_y) / dt
                cos_o = math.cos(odom_yaw)
                sin_o = math.sin(odom_yaw)
                self._calc_vx = cos_o * vx_w + sin_o * vy_w
                self._calc_vy = -sin_o * vx_w + cos_o * vy_w
                dyaw = odom_yaw - self._last_odom_yaw
                dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi
                self._calc_wz = dyaw / dt
            elif dt > 0.5 or dt < 0:
                self._calc_vx = 0.0
                self._calc_vy = 0.0
                self._calc_wz = 0.0

            self._last_odom_x = odom_x
            self._last_odom_y = odom_y
            self._last_odom_yaw = odom_yaw
            self._last_odom_time_sec = curr_time_sec

        else:
            # Fallback before first dynamic_pose/info packet
            odom_x = float(msg.pose.position.x)
            odom_y = float(msg.pose.position.y)
            odom_z = float(msg.pose.position.z)
            qx = float(msg.pose.orientation.x)
            qy = float(msg.pose.orientation.y)
            qz = float(msg.pose.orientation.z)
            qw = float(msg.pose.orientation.w)
            self._calc_vx = float(msg.twist.linear.x)
            self._calc_vy = float(msg.twist.linear.y)
            self._calc_wz = float(msg.twist.angular.z)

        # 1. Publish Odometry message
        ros_odom = RosOdometry()
        ros_odom.header.stamp = stamp
        ros_odom.header.frame_id = "odom"
        ros_odom.child_frame_id = "base_footprint"

        ros_odom.pose.pose.position.x = float(odom_x)
        ros_odom.pose.pose.position.y = float(odom_y)
        ros_odom.pose.pose.position.z = float(odom_z)
        ros_odom.pose.pose.orientation.x = float(qx)
        ros_odom.pose.pose.orientation.y = float(qy)
        ros_odom.pose.pose.orientation.z = float(qz)
        ros_odom.pose.pose.orientation.w = float(qw)

        ros_odom.pose.covariance = [
            1e-4, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-4, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-4, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2,
        ]

        ros_odom.twist.twist.linear.x = float(self._calc_vx)
        ros_odom.twist.twist.linear.y = float(self._calc_vy)
        ros_odom.twist.twist.linear.z = 0.0
        ros_odom.twist.twist.angular.x = 0.0
        ros_odom.twist.twist.angular.y = 0.0
        ros_odom.twist.twist.angular.z = float(self._calc_wz)

        ros_odom.twist.covariance = [
            1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e-2,
        ]

        self.pub_odom.publish(ros_odom)

        # 2. Broadcast TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = float(odom_x)
        t.transform.translation.y = float(odom_y)
        t.transform.translation.z = float(odom_z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

        # 3. Broadcast TF: world -> odom aligning odometry frame with physical Gazebo world
        if self.get_parameter("publish_world_tf").value and self._spawn_world_x is not None:
            tw = TransformStamped()
            tw.header.stamp = stamp
            tw.header.frame_id = "world"
            tw.child_frame_id = "odom"
            tw.transform.translation.x = float(self._spawn_world_x)
            tw.transform.translation.y = float(self._spawn_world_y)
            tw.transform.translation.z = float(self._spawn_world_z)
            tw.transform.rotation.x = float(self._spawn_world_q[0])
            tw.transform.rotation.y = float(self._spawn_world_q[1])
            tw.transform.rotation.z = float(self._spawn_world_q[2])
            tw.transform.rotation.w = float(self._spawn_world_q[3])
            self.tf_broadcaster.sendTransform(tw)

    def _on_gz_imu(self, msg: GzIMU):
        now = time.time()
        if now - self._last_imu_time < 0.025:  # Throttle to 40 Hz
            return
        self._last_imu_time = now

        ros_imu = RosImu()
        ros_imu.header.stamp = self._to_ros_time(msg.header)
        ros_imu.header.frame_id = "imu_link"

        if msg.HasField("orientation"):
            ros_imu.orientation.x = float(msg.orientation.x)
            ros_imu.orientation.y = float(msg.orientation.y)
            ros_imu.orientation.z = float(msg.orientation.z)
            ros_imu.orientation.w = float(msg.orientation.w)

        if msg.HasField("angular_velocity"):
            ros_imu.angular_velocity.x = float(msg.angular_velocity.x)
            ros_imu.angular_velocity.y = float(msg.angular_velocity.y)
            ros_imu.angular_velocity.z = float(msg.angular_velocity.z)

        if msg.HasField("linear_acceleration"):
            ros_imu.linear_acceleration.x = float(msg.linear_acceleration.x)
            ros_imu.linear_acceleration.y = float(msg.linear_acceleration.y)
            ros_imu.linear_acceleration.z = float(msg.linear_acceleration.z)

        self.pub_imu.publish(ros_imu)

    def _on_gz_rgb_image(self, msg: GzImage):
        if self.pub_rgb_image.get_subscription_count() == 0 and self.pub_rgb_info.get_subscription_count() == 0:
            return
        stamp = self._to_ros_time(msg.header)
        capture_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self._last_rgb_ns is not None and 0 <= capture_ns - self._last_rgb_ns < self.rgb_period_ns:
            return
        self._last_rgb_ns = capture_ns

        if int(msg.width) != self.rgb_info_msg.width or int(msg.height) != self.rgb_info_msg.height:
            self.rgb_info_msg = self._build_camera_info(
                width=int(msg.width), height=int(msg.height), hfov=1.15, frame_id="camera_optical_link"
            )

        # Image message
        ros_img = RosImage()
        ros_img.header.stamp = stamp
        ros_img.header.frame_id = "camera_optical_link"
        ros_img.width = int(msg.width)
        ros_img.height = int(msg.height)
        ros_img.encoding = "rgb8"
        ros_img.is_bigendian = 0
        ros_img.step = int(msg.step)
        ros_img.data = bytes(msg.data)
        self.pub_rgb_image.publish(ros_img)

        # CameraInfo message
        self.rgb_info_msg.header.stamp = stamp
        self.pub_rgb_info.publish(self.rgb_info_msg)

    def _on_gz_depth_image(self, msg: GzImage):
        if self.pub_depth_image.get_subscription_count() == 0 and self.pub_depth_info.get_subscription_count() == 0:
            return
        stamp = self._to_ros_time(msg.header)
        capture_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self._last_depth_ns is not None and 0 <= capture_ns - self._last_depth_ns < self.depth_image_period_ns:
            return
        self._last_depth_ns = capture_ns

        if int(msg.width) != self.depth_info_msg.width or int(msg.height) != self.depth_info_msg.height:
            self.depth_info_msg = self._build_camera_info(
                width=int(msg.width), height=int(msg.height), hfov=1.15, frame_id="camera_optical_link"
            )

        ros_img = RosImage()
        ros_img.header.stamp = stamp
        ros_img.header.frame_id = "camera_optical_link"
        ros_img.width = int(msg.width)
        ros_img.height = int(msg.height)
        ros_img.encoding = "32FC1"
        ros_img.is_bigendian = 0
        ros_img.step = int(msg.step)
        ros_img.data = bytes(msg.data)
        self.pub_depth_image.publish(ros_img)

        self.depth_info_msg.header.stamp = stamp
        self.pub_depth_info.publish(self.depth_info_msg)

    def _on_gz_depth_points(self, msg: GzPointCloudPacked):
        if self.pub_depth_points.get_subscription_count() == 0:
            return
        now = time.time()
        if now - self._last_pc_time < 0.20:  # 5 Hz; keep depth obstacle sensing while limiting bridge load
            return
        self._last_pc_time = now

        stamp = self._to_ros_time(msg.header)

        p_step = int(msg.point_step)
        raw_data = bytearray(msg.data)
        is_dense = bool(msg.is_dense)

        # Geometrically accurate 3D robot-body self-exclusion filter
        # Camera is located at [0.112, 0.0, 1.020] in base_footprint frame.
        # Optical transform: x_bf = 0.112 + z_opt, y_bf = -x_opt, z_bf = 1.020 - y_opt
        # Robot physical bounding box: [-0.35, 0.35] x [-0.35, 0.35] x [0.00, 1.05]
        if len(raw_data) >= p_step:
            dtype = np.dtype([
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32),
                ("rest", np.uint8, p_step - 12)
            ])
            arr = np.frombuffer(raw_data, dtype=dtype)
            x_opt = arr["x"]
            y_opt = arr["y"]
            z_opt = arr["z"]

            # Gazebo packs forward/left/up. The ROS camera optical frame is right/down/forward.
            forward = x_opt.copy()
            x_opt[:] = -y_opt
            y_opt[:] = -z_opt
            z_opt[:] = forward

            x_bf = 0.112 + z_opt
            y_bf = -x_opt
            z_bf = 1.020 - y_opt

            self_mask = (
                (x_bf >= -0.35) & (x_bf <= 0.35) &
                (y_bf >= -0.35) & (y_bf <= 0.35) &
                (z_bf >= 0.00)  & (z_bf <= 1.05) &
                np.isfinite(z_opt)
            )

            if np.any(self_mask):
                x_opt[self_mask] = np.nan
                y_opt[self_mask] = np.nan
                z_opt[self_mask] = np.nan
                is_dense = False

        ros_cloud = RosPointCloud2()
        ros_cloud.header.stamp = stamp
        ros_cloud.header.frame_id = "camera_optical_link"
        ros_cloud.height = int(msg.height)
        ros_cloud.width = int(msg.width)
        ros_cloud.is_bigendian = False
        ros_cloud.point_step = p_step
        ros_cloud.row_step = int(msg.row_step)
        ros_cloud.is_dense = is_dense
        ros_cloud.data = bytes(raw_data)
        ros_cloud.fields = self.pc_fields
        self.pub_depth_points.publish(ros_cloud)

    def shutdown_gazebo_transport(self):
        """Stop transport callbacks before Python destroys the Gazebo node."""
        gz_node = getattr(self, "gz_node", None)
        if gz_node is None:
            return
        try:
            for topic in list(gz_node.subscribed_topics()):
                gz_node.unsubscribe(topic)
        except Exception as exc:
            self.get_logger().warning(f"Gazebo transport unsubscribe during shutdown failed: {exc}")
        self.gz_cmd_pub = None
        self.gz_node = None

    def _on_ros_cmd_vel(self, msg: RosTwist):
        """Forward ROS 2 /cmd_vel to Gazebo diff_drive."""
        if self.gz_cmd_pub:
            gz_tw = GzTwist()
            gz_tw.linear.x = float(msg.linear.x)
            gz_tw.angular.z = float(msg.angular.z)
            self.get_logger().debug(f"Forwarding cmd_vel: lin={gz_tw.linear.x:.2f}, ang={gz_tw.angular.z:.2f}")
            self.gz_cmd_pub.publish(gz_tw)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGINT can invalidate the context while the executor allocates its wait set.
        if node.context.ok():
            raise
    finally:
        try:
            node.shutdown_gazebo_transport()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
