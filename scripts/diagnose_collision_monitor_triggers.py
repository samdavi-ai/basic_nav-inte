#!/usr/bin/env python3
"""
Diagnostic tool to identify exact sensor points triggering Nav2 Collision Monitor.
Transforms points from /scan and /camera/depth/points into base_footprint frame
and tests containment in PolygonStop and PolygonSlow.
"""

import sys
import math
import struct
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class CollisionMonitorDiagnoser(Node):
    def __init__(self):
        super().__init__("collision_monitor_diagnoser")

        # Polygons from nav2_params.yaml
        # PolygonStop: [[0.40, 0.38], [0.40, -0.38], [-0.38, -0.38], [-0.38, 0.38]]
        self.stop_x_min = -0.38
        self.stop_x_max = 0.40
        self.stop_y_min = -0.38
        self.stop_y_max = 0.38

        # PolygonSlow: [[0.65, 0.48], [0.65, -0.48], [-0.42, -0.48], [-0.42, 0.48]]
        self.slow_x_min = -0.42
        self.slow_x_max = 0.65
        self.slow_y_min = -0.48
        self.slow_y_max = 0.48

        # Footprint: [[0.35, 0.34], [0.35, -0.34], [-0.35, -0.34], [-0.35, 0.34]]
        self.fp_x_min = -0.35
        self.fp_x_max = 0.35
        self.fp_y_min = -0.34
        self.fp_y_max = 0.34

        # Height limits from nav2_params.yaml collision_monitor:
        # scan: min_height: 0.05, max_height: 2.0
        # pointcloud: min_height: 0.05, max_height: 1.8
        self.scan_z_min = 0.05
        self.scan_z_max = 2.0
        self.pc_z_min = 0.05
        self.pc_z_max = 1.8

        # Transform from camera_optical_link to base_footprint:
        # Camera link in base_footprint: [0.112, 0.0, 1.020]
        # Optical frame: X_opt -> -Y_bf, Y_opt -> -Z_bf, Z_opt -> +X_bf
        self.cam_pos = np.array([0.112, 0.0, 1.020])

        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.pc_sub = self.create_subscription(PointCloud2, "/camera/depth/points", self.on_pointcloud, 10)

        self.scan_done = False
        self.pc_done = False

    def on_scan(self, msg: LaserScan):
        if self.scan_done:
            return
        self.scan_done = True
        self.get_logger().info(f"Received LaserScan with {len(msg.ranges)} beams.")

        # Lidar is at [0.11, 0, 0.14] in base_link, and base_link is at z=0.115 in base_footprint
        # Total lidar in base_footprint: [0.11, 0.0, 0.255]
        lidar_x = 0.11
        lidar_y = 0.0
        lidar_z = 0.255

        stop_points = []
        slow_points = []
        footprint_points = []

        angle = msg.angle_min
        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r) and r > 0.01:
                x = lidar_x + r * math.cos(angle)
                y = lidar_y + r * math.sin(angle)
                z = lidar_z

                if self.scan_z_min <= z <= self.scan_z_max:
                    if self.fp_x_min <= x <= self.fp_x_max and self.fp_y_min <= y <= self.fp_y_max:
                        footprint_points.append((x, y, z, r, angle))

                    if self.stop_x_min <= x <= self.stop_x_max and self.stop_y_min <= y <= self.stop_y_max:
                        stop_points.append((x, y, z, r, angle))
                    elif self.slow_x_min <= x <= self.slow_x_max and self.slow_y_min <= y <= self.slow_y_max:
                        slow_points.append((x, y, z, r, angle))

            angle += msg.angle_increment

        print("\n" + "="*60)
        print("--- LASERSCAN COLLISION MONITOR ANALYSIS ---")
        print(f"Total valid beams: {len(msg.ranges)}")
        print(f"Points inside Robot Footprint: {len(footprint_points)}")
        print(f"Points inside PolygonStop:     {len(stop_points)}")
        print(f"Points inside PolygonSlow:     {len(slow_points)}")
        if stop_points:
            print("Sample PolygonStop LiDAR points (x, y, z, range, angle_deg):")
            for p in stop_points[:10]:
                print(f"  x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}, r={p[3]:.3f}m, angle={math.degrees(p[4]):.1f}°")
        print("="*60 + "\n")

    def on_pointcloud(self, msg: PointCloud2):
        if self.pc_done:
            return
        self.pc_done = True
        self.get_logger().info(f"Received PointCloud2 with {msg.width * msg.height} points.")

        # Read x, y, z in camera_optical_link
        points_opt = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.get_logger().info(f"Valid finite points: {len(points_opt)}")

        stop_points = []
        slow_points = []
        footprint_points = []

        for p_opt in points_opt:
            # Transform to base_footprint
            x_opt, y_opt, z_opt = p_opt[0], p_opt[1], p_opt[2]
            x_bf = self.cam_pos[0] + z_opt
            y_bf = self.cam_pos[1] - x_opt
            z_bf = self.cam_pos[2] - y_opt

            # Check height bounds
            if self.pc_z_min <= z_bf <= self.pc_z_max:
                if self.fp_x_min <= x_bf <= self.fp_x_max and self.fp_y_min <= y_bf <= self.fp_y_max:
                    footprint_points.append((x_bf, y_bf, z_bf, z_opt))

                if self.stop_x_min <= x_bf <= self.stop_x_max and self.stop_y_min <= y_bf <= self.stop_y_max:
                    stop_points.append((x_bf, y_bf, z_bf, z_opt))
                elif self.slow_x_min <= x_bf <= self.slow_x_max and self.slow_y_min <= y_bf <= self.slow_y_max:
                    slow_points.append((x_bf, y_bf, z_bf, z_opt))

        print("\n" + "="*60)
        print("--- POINTCLOUD2 COLLISION MONITOR ANALYSIS ---")
        print(f"Total valid points: {len(points_opt)}")
        print(f"Points inside Robot Footprint: {len(footprint_points)}")
        print(f"Points inside PolygonStop:     {len(stop_points)}")
        print(f"Points inside PolygonSlow:     {len(slow_points)}")
        if stop_points:
            print("Sample PolygonStop PointCloud points (x_bf, y_bf, z_bf, depth_z):")
            for p in stop_points[:15]:
                print(f"  x_bf={p[0]:.3f}, y_bf={p[1]:.3f}, z_bf={p[2]:.3f}, depth_z={p[3]:.3f}m")
        print("="*60 + "\n")

def main():
    rclpy.init()
    node = CollisionMonitorDiagnoser()
    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.scan_done and node.pc_done:
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

