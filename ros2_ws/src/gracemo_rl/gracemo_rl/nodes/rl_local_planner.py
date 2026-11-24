#!/usr/bin/env python3
"""
RL Local Planner Node for GRaCEmo ViRa.
Loads trained policy via ONNX Runtime and tracks Nav2 global /plan.
Publishes velocity commands to /cmd_vel_rl at 10 Hz with safety watchdog.
"""

import os
import math
import time
from typing import Optional, Tuple, Dict, Any
import numpy as np

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    import onnxruntime as ort
except ImportError:
    ort = None

from gracemo_rl.common.obs import build_observation, downsample_scan_ranges, compute_relative_goal
from gracemo_rl.common.limits import load_nav2_limits


class RLLocalPlannerNode(Node):
    def __init__(self):
        super().__init__("rl_local_planner")

        nav_limits = load_nav2_limits()

        # Declare parameters with authoritative defaults from nav2_params.yaml
        self.declare_parameter("model_path", "")
        self.declare_parameter("inference_threads", 0)
        self.declare_parameter("lookahead_dist", 0.80)        # meters (0.6 - 1.0 m)
        self.declare_parameter("max_linear_vel", nav_limits.max_linear_vel)        # m/s
        self.declare_parameter("max_angular_vel", nav_limits.max_angular_vel)       # rad/s
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("min_reverse_vel", nav_limits.min_reverse_vel)      # m/s
        self.declare_parameter("min_scan_range", nav_limits.rl_min_scan_range)
        self.declare_parameter("max_scan_range", nav_limits.rl_max_scan_range)
        self.declare_parameter("watchdog_timeout_sec", 0.50)  # seconds
        self.declare_parameter("control_rate_hz", 10.0)       # 10 Hz
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_rl")
        self.declare_parameter("enable_obstacle_speed_limiter", True)
        self.declare_parameter("obstacle_stop_dist", 0.35)
        self.declare_parameter("obstacle_slow_dist", 0.70)
        self.declare_parameter("enable_forward_bias", True)

        self.model_path = str(self.get_parameter("model_path").value)
        self.lookahead_dist = float(self.get_parameter("lookahead_dist").value)
        self.max_linear_vel = float(self.get_parameter("max_linear_vel").value)
        self.max_angular_vel = float(self.get_parameter("max_angular_vel").value)
        self.allow_reverse = bool(self.get_parameter("allow_reverse").value)
        self.min_reverse_vel = float(self.get_parameter("min_reverse_vel").value)
        self.min_scan_range = float(self.get_parameter("min_scan_range").value)
        self.max_scan_range = float(self.get_parameter("max_scan_range").value)
        self.watchdog_timeout_sec = float(self.get_parameter("watchdog_timeout_sec").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.enable_obstacle_speed_limiter = bool(self.get_parameter("enable_obstacle_speed_limiter").value)
        self.obstacle_stop_dist = float(self.get_parameter("obstacle_stop_dist").value)
        self.obstacle_slow_dist = float(self.get_parameter("obstacle_slow_dist").value)
        self.enable_forward_bias = bool(self.get_parameter("enable_forward_bias").value)

        # Internal state
        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_time: float = 0.0

        self.robot_x: float = 0.0
        self.robot_y: float = 0.0
        self.robot_yaw: float = 0.0
        self.robot_vx: float = 0.0
        self.robot_wz: float = 0.0
        self.last_odom_time: float = 0.0
        self.last_valid_map_pose: Optional[Tuple[float, float, float]] = None

        self.active_path: Optional[Path] = None
        self.last_path_time: float = 0.0

        self.last_action = np.zeros(2, dtype=np.float32)
        self.downsampled_beams = 72

        # Subscriptions
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.on_odom, qos_profile_sensor_data
        )
        self.sub_plan = self.create_subscription(
            Path, "/plan", self.on_plan, 10
        )
        self.sub_received_plan = self.create_subscription(
            Path, "/received_global_plan", self.on_plan, 10
        )
        self.sub_state = self.create_subscription(
            String, "/autonomy/state", self.on_state, 10
        )
        self.sub_feedback = self.create_subscription(
            String, "/coordinator/feedback", self.on_feedback, 10
        )

        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # TF2 Buffer and Listener for accurate map-to-base_footprint tracking
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load ONNX Model
        self.ort_session = None
        self._load_onnx_model()

        # Control Loop Timer
        timer_period = 1.0 / max(1.0, self.control_rate_hz)
        self.timer = self.create_timer(timer_period, self.on_control_cycle)

        self.get_logger().info(
            f"🧠 RL Local Planner active. Output: '{self.cmd_vel_topic}' @ {self.control_rate_hz}Hz | "
            f"Lookahead: {self.lookahead_dist}m | Watchdog: {self.watchdog_timeout_sec}s"
        )

    def _load_onnx_model(self):
        if not self.model_path or not os.path.exists(self.model_path):
            # Attempt to locate default exported model in share dir or local source
            candidate_paths = []
            try:
                from ament_index_python.packages import get_package_share_directory
                share_dir = get_package_share_directory("gracemo_rl")
                candidate_paths.append(os.path.join(share_dir, "models", "gracemo_sac_policy.onnx"))
            except Exception:
                pass

            candidate_paths.append(os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "models", "gracemo_sac_policy.onnx"
            ))
            # Also check repository models path directly
            candidate_paths.append(os.path.join(
                os.getcwd(), "ros2_ws", "src", "gracemo_rl", "gracemo_rl", "models", "gracemo_sac_policy.onnx"
            ))

            for p in candidate_paths:
                if os.path.exists(p):
                    self.model_path = os.path.abspath(p)
                    break

        if self.model_path and os.path.exists(self.model_path) and ort is not None:
            try:
                options = ort.SessionOptions()
                options.intra_op_num_threads = self.get_parameter("inference_threads").value
                self.ort_session = ort.InferenceSession(
                    self.model_path,
                    sess_options=options, providers=["CPUExecutionProvider"]
                )
                self.input_name = self.ort_session.get_inputs()[0].name
                self.get_logger().info(f"✓ Loaded ONNX model from: {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load ONNX model: {e}")
                self.ort_session = None
        else:
            self.get_logger().warn(
                f"No ONNX model loaded (path: '{self.model_path}'). Operating in safety fallback mode."
            )

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg
        self.last_scan_time = self._get_time_sec()

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.robot_x = p.x
        self.robot_y = p.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.robot_vx = msg.twist.twist.linear.x
        self.robot_wz = msg.twist.twist.angular.z
        self.last_odom_time = self._get_time_sec()

    def on_plan(self, msg: Path):
        self.active_path = msg
        self.last_path_time = self._get_time_sec()

    def on_state(self, msg: String):
        if msg.data in ("AUTONOMY_READY", "MISSION_COMPLETED", "DESTINATION_REACHED", "MISSION_CANCELLED", "NAVIGATION_FAILURE"):
            self.active_path = None

    def on_feedback(self, msg: String):
        # Monitor overrides or e-stop announcements
        pass

    def _get_time_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _extract_lookahead_point(self, current_x: Optional[float] = None, current_y: Optional[float] = None) -> Optional[Tuple[float, float]]:
        """Finds waypoint at ~lookahead_dist along the global plan."""
        if not self.active_path or len(self.active_path.poses) == 0:
            return None

        if current_x is None:
            current_x = self.robot_x
        if current_y is None:
            current_y = self.robot_y

        # Find closest path point
        min_idx = 0
        min_dist = float("inf")
        for i, pose_stamped in enumerate(self.active_path.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            d = math.hypot(px - current_x, py - current_y)
            if d < min_dist:
                min_dist = d
                min_idx = i

        # Step forward along path until reaching lookahead distance
        accum_dist = 0.0
        lookahead_idx = min_idx
        for i in range(min_idx, len(self.active_path.poses) - 1):
            p1 = self.active_path.poses[i].pose.position
            p2 = self.active_path.poses[i + 1].pose.position
            seg_len = math.hypot(p2.x - p1.x, p2.y - p1.y)
            accum_dist += seg_len
            if accum_dist >= self.lookahead_dist:
                lookahead_idx = i + 1
                break
        else:
            lookahead_idx = len(self.active_path.poses) - 1

        target_pose = self.active_path.poses[lookahead_idx].pose.position
        return (target_pose.x, target_pose.y)

    def _downsample_scan(self) -> np.ndarray:
        ranges = self.latest_scan.ranges if self.latest_scan else None
        return downsample_scan_ranges(
            ranges,
            downsampled_beams=self.downsampled_beams,
            min_range=self.min_scan_range,
            max_range=self.max_scan_range
        )

    def on_control_cycle(self):
        # If no active path or plan completed, suppress publishing so cmd_vel_mux can fall back
        if self.active_path is None or len(self.active_path.poses) == 0:
            return

        now = self._get_time_sec()

        # Watchdog checks (sensor staleness)
        scan_stale = (now - self.last_scan_time) > self.watchdog_timeout_sec
        odom_stale = (now - self.last_odom_time) > self.watchdog_timeout_sec
        plan_stale = (now - self.last_path_time) > 45.0  # Allow active global trajectory to execute

        if scan_stale or odom_stale or plan_stale:
            # Safety zero-velocity halt when active path is lost or stale
            self.pub_cmd_vel.publish(Twist())
            if plan_stale:
                self.active_path = None
            return

        # Obtain robot coordinates in the frame of the active path (default: map)
        frame_id = self.active_path.header.frame_id if self.active_path.header.frame_id else "map"
        rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw
        try:
            t = self.tf_buffer.lookup_transform(frame_id, "base_footprint", rclpy.time.Time())
            rx = float(t.transform.translation.x)
            ry = float(t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            ryaw = float(math.atan2(siny_cosp, cosy_cosp))
            self.last_valid_map_pose = (rx, ry, ryaw)
        except Exception:
            if self.last_valid_map_pose is not None:
                rx, ry, ryaw = self.last_valid_map_pose

        # Verify distance to final destination in the path
        final_pose = self.active_path.poses[-1].pose.position
        dist_to_final = math.hypot(final_pose.x - rx, final_pose.y - ry)
        if dist_to_final < 0.35:
            # Goal successfully reached
            self.pub_cmd_vel.publish(Twist())
            self.active_path = None
            return

        lookahead_target = self._extract_lookahead_point(rx, ry)
        if lookahead_target is None:
            self.pub_cmd_vel.publish(Twist())
            self.active_path = None
            return

        gx, gy = lookahead_target

        # Authoritative 79-D observation vector
        obs_vec = build_observation(
            scan_ranges=self.latest_scan.ranges if self.latest_scan else None,
            robot_x=rx,
            robot_y=ry,
            robot_yaw=ryaw,
            goal_x=gx,
            goal_y=gy,
            last_action=self.last_action,
            robot_vx=self.robot_vx,
            robot_wz=self.robot_wz,
            max_linear_vel=self.max_linear_vel,
            max_angular_vel=self.max_angular_vel,
            min_scan_range=self.min_scan_range,
            max_scan_range=self.max_scan_range,
            downsampled_beams=self.downsampled_beams
        )
        obs = obs_vec.reshape(1, -1)

        # Model inference
        if self.ort_session is not None:
            try:
                outputs = self.ort_session.run(None, {self.input_name: obs})
                action = outputs[0].ravel()
            except Exception as e:
                self.get_logger().error(f"Inference error: {e}")
                action = np.zeros(2, dtype=np.float32)
        else:
            # Fallback pure pursuit controller if ONNX not yet loaded
            sin_b = float(obs_vec[73])
            cos_b = float(obs_vec[74])
            rel_bearing = math.atan2(sin_b, cos_b)
            action = np.array([
                0.5 * cos_b if cos_b > 0 else 0.0,
                np.clip(rel_bearing * 1.2, -1.0, 1.0)
            ], dtype=np.float32)

        # Scale action to physical limits
        a_v, a_w = float(action[0]), float(action[1])

        # Forward bias remediation: prevent reverse oscillation trap when goal is ahead and path is clear
        if self.enable_forward_bias:
            cos_b = float(obs_vec[74])
            if cos_b > 0.0:
                scan_m = obs_vec[:72] * self.max_scan_range
                front_clr = float(np.min(scan_m[30:42]))
                if front_clr > 0.40:
                    a_v = max(0.15, a_v)

        if self.allow_reverse:
            v = (a_v + 1.0) / 2.0 * (self.max_linear_vel - self.min_reverse_vel) + self.min_reverse_vel
        else:
            v = max(0.0, (a_v + 1.0) / 2.0 * self.max_linear_vel)
        w = a_w * self.max_angular_vel

        # Obstacle distance speed limiter: scale speed before triggering collision monitor stop
        if self.enable_obstacle_speed_limiter:
            scan_m = obs_vec[:72] * self.max_scan_range
            front_clr = float(np.min(scan_m[30:42]))
            if front_clr < self.obstacle_stop_dist:
                v = min(0.0, v)
            elif front_clr < self.obstacle_slow_dist:
                scale = (front_clr - self.obstacle_stop_dist) / (self.obstacle_slow_dist - self.obstacle_stop_dist)
                v = min(v, max(0.05, self.max_linear_vel * float(scale)))

        # Clamp output
        v = float(np.clip(v, self.min_reverse_vel if self.allow_reverse else 0.0, self.max_linear_vel))
        w = float(np.clip(w, -self.max_angular_vel, self.max_angular_vel))

        self.last_action = np.array([a_v, a_w], dtype=np.float32)

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RLLocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
