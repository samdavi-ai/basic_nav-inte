#!/usr/bin/env python3
"""
GRaCEmo ViRa — AT-02 Reproduction & Forensic Failure Diagnostic.
Captures complete robotic state at the exact moment of failure or arrival:
  - Nav2 action result and controller status
  - Costmap cell values around robot pose
  - Collision monitor stop/slow state
  - Velocity pipeline: /cmd_vel_nav, /cmd_vel_smoothed, /cmd_vel_safe
  - Poses: Gazebo ground truth, /odom, /amcl_pose, TF map->base_footprint
"""

import sys
import time
import math
import json
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool, Empty
from nav2_msgs.msg import CollisionMonitorState
import tf2_ros

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.pose_pb2 import Pose as GzPose
from gz.msgs10.boolean_pb2 import Boolean as GzBoolean


class AT02Diagnoser(Node):
    def __init__(self):
        super().__init__("at02_diagnoser")

        # Pose & State
        self.gz_pose: Optional[Dict[str, float]] = None
        self.odom_pose: Optional[Dict[str, float]] = None
        self.amcl_pose: Optional[Dict[str, float]] = None
        self.supervisor_state: str = "UNKNOWN"
        self.mission_status: Optional[Dict[str, Any]] = None

        # Velocities across pipeline
        self.vel_nav = {"vx": 0.0, "wz": 0.0}
        self.vel_smoothed = {"vx": 0.0, "wz": 0.0}
        self.vel_safe = {"vx": 0.0, "wz": 0.0}

        # Collision monitor
        self.col_action = 0
        self.col_poly = ""
        self.stop_count = 0
        self.slow_count = 0

        # Costmaps
        self.local_costmap: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[OccupancyGrid] = None

        # Subscriptions
        self.create_subscription(String, "/autonomy/state", self._on_state, 10)
        self.create_subscription(String, "/autonomy/mission_status", self._on_mission_status, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self._on_vel(m, "nav"), 10)
        self.create_subscription(Twist, "/cmd_vel_smoothed", lambda m: self._on_vel(m, "smoothed"), 10)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: self._on_vel(m, "safe"), 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_col, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_global_costmap, 10)

        # Publishers
        self.pub_cmd = self.create_publisher(String, "/coordinator/command", 10)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo Transport
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(GzPoseV, "/world/gracemo_home/dynamic_pose/info", self._on_gz_pose)

    def _on_state(self, msg: String):
        self.supervisor_state = msg.data

    def _on_mission_status(self, msg: String):
        try:
            self.mission_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = {"x": p.x, "y": p.y, "yaw": yaw}

    def _on_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.amcl_pose = {"x": p.x, "y": p.y, "yaw": yaw}

    def _on_vel(self, msg: Twist, stage: str):
        v = {"vx": msg.linear.x, "wz": msg.angular.z}
        if stage == "nav":
            self.vel_nav = v
        elif stage == "smoothed":
            self.vel_smoothed = v
        elif stage == "safe":
            self.vel_safe = v

    def _on_col(self, msg: CollisionMonitorState):
        self.col_action = msg.action_type
        self.col_poly = msg.polygon_name
        if msg.action_type == 1:
            self.stop_count += 1
        elif msg.action_type == 2:
            self.slow_count += 1

    def _on_local_costmap(self, msg: OccupancyGrid):
        self.local_costmap = msg

    def _on_global_costmap(self, msg: OccupancyGrid):
        self.global_costmap = msg

    def _on_gz_pose(self, msg: GzPoseV):
        for p in msg.pose:
            if p.name == "gracemo_vira":
                q = p.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                self.gz_pose = {"x": float(p.position.x), "y": float(p.position.y), "z": float(p.position.z), "yaw": float(yaw)}
                break

    def get_tf_map_pose(self) -> Optional[Dict[str, float]]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            r = t.transform.rotation
            yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y), 1.0 - 2.0 * (r.y * r.y + r.z * r.z))
            return {"x": float(tr.x), "y": float(tr.y), "yaw": float(yaw)}
        except Exception:
            return None

    def query_costmap_at(self, costmap: Optional[OccupancyGrid], x: float, y: float, radius_cells: int = 2) -> Dict[str, Any]:
        if not costmap:
            return {"available": False}
        info = costmap.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        col = int((x - ox) / res)
        row = int((y - oy) / res)
        w, h = info.width, info.height

        window = []
        for dr in range(-radius_cells, radius_cells + 1):
            row_vals = []
            for dc in range(-radius_cells, radius_cells + 1):
                c = col + dc
                r = row + dr
                if 0 <= c < w and 0 <= r < h:
                    row_vals.append(int(costmap.data[r * w + c]))
                else:
                    row_vals.append(-1)
            window.append(row_vals)

        center_val = window[radius_cells][radius_cells] if window else -1
        return {
            "available": True,
            "center_cell": (col, row),
            "center_cost": center_val,
            "window": window,
            "resolution": res,
        }

    def reset_clean(self, x=0.0, y=0.0, yaw=0.0):
        # Cancel & Reset supervisor
        msg = String()
        msg.data = json.dumps({"intent": "CANCEL"})
        self.pub_cmd.publish(msg)
        time.sleep(0.3)
        msg.data = json.dumps({"intent": "RESET"})
        self.pub_cmd.publish(msg)
        time.sleep(0.3)

        # Teleport Gazebo
        try:
            req = GzPose()
            req.name = "gracemo_vira"
            req.position.x = float(x)
            req.position.y = float(y)
            req.position.z = 0.04
            req.orientation.z = math.sin(yaw / 2.0)
            req.orientation.w = math.cos(yaw / 2.0)
            self.gz_node.request("/world/gracemo_home/set_pose", req, GzPose, GzBoolean, 2000)
        except Exception as e:
            print(f"Warning: GZ set_pose failed: {e}")

        # Stop velocities
        z_twist = Twist()
        for _ in range(5):
            self.pub_teleop.publish(z_twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Initialize AMCL localization
        loc_cmd = {"intent": "INITIALIZE_LOCALIZATION", "coords": {"x": float(x), "y": float(y), "yaw": float(yaw)}}
        loc_msg = String()
        loc_msg.data = json.dumps(loc_cmd)
        self.pub_cmd.publish(loc_msg)

        # Clear costmaps
        time.sleep(0.5)
        clr_msg = String()
        clr_msg.data = json.dumps({"intent": "CLEAR_COSTMAPS"})
        self.pub_cmd.publish(clr_msg)

        # Settle
        t_settle = time.time() + 3.0
        while time.time() < t_settle:
            rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    diag = AT02Diagnoser()

    print("\n============================================================")
    print("   AT-02 FORENSIC DIAGNOSTIC SUITE")
    print("============================================================")

    # 1. Reset clean state
    print("Resetting robot to clean initial state (0, 0)...")
    diag.reset_clean(0.0, 0.0, 0.0)

    # 2. Capture starting state
    start_tf = diag.get_tf_map_pose()
    start_gz = diag.gz_pose
    print(f"Starting state: TF_map={start_tf}, Gazebo={start_gz}")

    # 3. Dispatch AT-02: Kitchen (-5.0, -2.5)
    target_x, target_y = -5.0, -2.5
    cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
    msg = String()
    msg.data = json.dumps(cmd)
    diag.pub_cmd.publish(msg)
    print(f"Dispatched mission to Kitchen ({target_x}, {target_y}). Tracking execution...")

    t0 = time.time()
    max_wait = 50.0
    terminal_state = None
    terminal_snapshot = {}

    while time.time() - t0 < max_wait:
        rclpy.spin_once(diag, timeout_sec=0.1)
        cur_tf = diag.get_tf_map_pose()
        cur_gz = diag.gz_pose

        if cur_tf:
            dist_to_goal = math.hypot(cur_tf["x"] - target_x, cur_tf["y"] - target_y)
        else:
            dist_to_goal = 999.0

        if diag.supervisor_state in ("DESTINATION_REACHED", "MISSION_COMPLETED"):
            terminal_state = diag.supervisor_state
            print(f"\n[EVENT] Destination reached signal! Elapsed: {time.time()-t0:.1f}s")
            break

        if diag.supervisor_state in ("NAVIGATION_FAILURE", "PLANNING_FAILURE", "LOCALIZATION_LOST"):
            terminal_state = diag.supervisor_state
            print(f"\n[EVENT] Terminal failure: {terminal_state}! Elapsed: {time.time()-t0:.1f}s")
            break

    # Immediate Snapshot Capture
    cur_tf = diag.get_tf_map_pose()
    cur_gz = diag.gz_pose
    cur_odom = diag.odom_pose
    cur_amcl = diag.amcl_pose

    rob_x = cur_tf["x"] if cur_tf else 0.0
    rob_y = cur_tf["y"] if cur_tf else 0.0

    local_cost_info = diag.query_costmap_at(diag.local_costmap, rob_x, rob_y, radius_cells=2)
    global_cost_info = diag.query_costmap_at(diag.global_costmap, rob_x, rob_y, radius_cells=2)

    snapshot = {
        "timestamp": time.time(),
        "elapsed_sec": time.time() - t0,
        "terminal_state": terminal_state or "TIMEOUT",
        "supervisor_mission_status": diag.mission_status,
        "poses": {
            "tf_map": cur_tf,
            "gazebo_ground_truth": cur_gz,
            "odom": cur_odom,
            "amcl": cur_amcl,
        },
        "velocity_pipeline": {
            "cmd_vel_nav": diag.vel_nav,
            "cmd_vel_smoothed": diag.vel_smoothed,
            "cmd_vel_safe": diag.vel_safe,
        },
        "collision_monitor": {
            "last_action": diag.col_action,
            "last_polygon": diag.col_poly,
            "total_stops": diag.stop_count,
            "total_slows": diag.slow_count,
        },
        "costmap_at_robot": {
            "local_costmap": local_cost_info,
            "global_costmap": global_cost_info,
        }
    }

    snap_file = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/at02_failure_snapshot.json"
    with open(snap_file, "w") as f:
        json.dump(snapshot, f, indent=2)

    print("\n" + "="*60)
    print("   AT-02 FORENSIC SNAPSHOT SUMMARY")
    print("="*60)
    print(f"Terminal State:          {snapshot['terminal_state']}")
    print(f"Elapsed Time:            {snapshot['elapsed_sec']:.1f} s")
    print(f"Gazebo Ground Truth:     {cur_gz}")
    print(f"TF Map Pose:             {cur_tf}")
    print(f"Odom Pose:               {cur_odom}")
    print(f"AMCL Pose:               {cur_amcl}")
    if cur_tf and cur_gz:
        gz_tf_err = math.hypot(cur_tf["x"] - cur_gz["x"], cur_tf["y"] - cur_gz["y"])
        print(f"TF vs Gazebo Error:      {gz_tf_err:.3f} m")
        dist_to_goal_tf = math.hypot(cur_tf["x"] - target_x, cur_tf["y"] - target_y)
        dist_to_goal_gz = math.hypot(cur_gz["x"] - target_x, cur_gz["y"] - target_y)
        print(f"Distance to Goal (TF):   {dist_to_goal_tf:.3f} m")
        print(f"Distance to Goal (GZ):   {dist_to_goal_gz:.3f} m")
    print(f"Velocity Pipeline:")
    print(f"  /cmd_vel_nav:          vx={diag.vel_nav['vx']:.3f}, wz={diag.vel_nav['wz']:.3f}")
    print(f"  /cmd_vel_smoothed:     vx={diag.vel_smoothed['vx']:.3f}, wz={diag.vel_smoothed['wz']:.3f}")
    print(f"  /cmd_vel_safe:         vx={diag.vel_safe['vx']:.3f}, wz={diag.vel_safe['wz']:.3f}")
    print(f"Collision Monitor:")
    print(f"  Action={diag.col_action} (1=STOP, 2=SLOW, 0=NONE), Polygon='{diag.col_poly}'")
    print(f"  Stops={diag.stop_count}, Slows={diag.slow_count}")
    print(f"Costmap at Robot Pose:")
    print(f"  Local Center Cost:     {local_cost_info.get('center_cost')}")
    print(f"  Global Center Cost:    {global_cost_info.get('center_cost')}")
    print(f"Saved complete snapshot to: {snap_file}")
    print("="*60 + "\n")

    diag.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

