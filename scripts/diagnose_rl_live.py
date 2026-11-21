#!/usr/bin/env python3
"""
GRaCEmo ViRa — Phase B: RL Local Planner & Telemetry Pipeline Diagnostic Probe
Records 20 seconds of synchronized live telemetry across the entire navigation stack:
1. Raw /scan statistics (inf/NaN counts, min/max ranges, angle order)
2. 79-D observation vector (72 LiDAR sectors, dist, sin/cos bearing, last action, velocities)
3. Policy action [a_v, a_w] from ONNX model
4. Scaled & clamped velocity commands
5. Cmd_vel_mux received topics & selected active source
6. Downstream safety outputs (/cmd_vel_smoothed, /collision_monitor_state, /cmd_vel_safe, /cmd_vel, /odom)
7. Distribution comparison against KinematicSimulation (79-D observation histograms)
"""

import os
import sys
import time
import math
import json
from typing import Dict, List, Any, Optional
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String as RosString
from nav2_msgs.msg import CollisionMonitorState

try:
    import onnxruntime as ort
except ImportError:
    ort = None


class RLLiveDiagnosticProbe(Node):
    def __init__(self):
        super().__init__("rl_live_diagnostic_probe")

        # Telemetry logs
        self.scan_records: List[Dict[str, Any]] = []
        self.obs_records: List[np.ndarray] = []
        self.action_records: List[Dict[str, Any]] = []
        self.mux_status_records: List[Dict[str, Any]] = []
        self.monitor_records: List[Dict[str, Any]] = []
        self.cmd_rl_records: List[Dict[str, Any]] = []
        self.cmd_mux_records: List[Dict[str, Any]] = []
        self.cmd_safe_records: List[Dict[str, Any]] = []
        self.cmd_final_records: List[Dict[str, Any]] = []
        self.odom_records: List[Dict[str, Any]] = []

        # Current state
        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_plan: Optional[Path] = None
        self.latest_mux_status: Optional[str] = None
        self.latest_monitor_state: Optional[CollisionMonitorState] = None

        self.last_action = np.zeros(2, dtype=np.float32)

        # Subscribers
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(RosString, "/cmd_vel_mux/status", self.on_mux_status, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.on_monitor, 10)

        self.create_subscription(Twist, "/cmd_vel_rl", lambda m: self.cmd_rl_records.append(self._twist_to_dict(m)), 10)
        self.create_subscription(Twist, "/cmd_vel_mux", lambda m: self.cmd_mux_records.append(self._twist_to_dict(m)), 10)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: self.cmd_safe_records.append(self._twist_to_dict(m)), 10)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.cmd_final_records.append(self._twist_to_dict(m)), 10)

        self.cmd_pub = self.create_publisher(RosString, "/coordinator/command", 10)

        # Load ONNX model for parity and independent verification
        self.ort_session = None
        model_path = os.path.join(
            os.getcwd(), "ros2_ws", "src", "gracemo_rl", "gracemo_rl", "models", "gracemo_sac_policy.onnx"
        )
        if os.path.exists(model_path) and ort is not None:
            self.ort_session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
            self.input_name = self.ort_session.get_inputs()[0].name
            self.get_logger().info(f"Loaded diagnostic ONNX model from {model_path}")
        else:
            self.get_logger().warn(f"ONNX model not found at {model_path}")

    def _twist_to_dict(self, msg: Twist) -> Dict[str, float]:
        return {
            "time": self.get_clock().now().nanoseconds / 1e9,
            "vx": float(msg.linear.x),
            "vy": float(msg.linear.y),
            "wz": float(msg.angular.z)
        }

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_odom(self, msg: Odometry):
        self.latest_odom = msg
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_records.append({
            "time": self.get_clock().now().nanoseconds / 1e9,
            "x": float(p.x),
            "y": float(p.y),
            "yaw": float(yaw),
            "vx": float(msg.twist.twist.linear.x),
            "wz": float(msg.twist.twist.angular.z)
        })

    def on_plan(self, msg: Path):
        self.latest_plan = msg

    def on_mux_status(self, msg: RosString):
        try:
            d = json.loads(msg.data)
            d["time"] = self.get_clock().now().nanoseconds / 1e9
            self.mux_status_records.append(d)
        except Exception:
            pass

    def on_monitor(self, msg: CollisionMonitorState):
        self.latest_monitor_state = msg
        self.monitor_records.append({
            "time": self.get_clock().now().nanoseconds / 1e9,
            "action": msg.action_type,
            "polygon": msg.polygon_name
        })

    def record_tick(self):
        """Samples telemetry and computes observation & ONNX action synchronously."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self.latest_scan is None or self.latest_odom is None:
            return

        # 1. Raw /scan analysis
        ranges = np.array(self.latest_scan.ranges, dtype=np.float32)
        inf_mask = np.isinf(ranges)
        nan_mask = np.isnan(ranges)
        finite_mask = np.isfinite(ranges)
        valid_ranges = ranges[finite_mask]

        scan_stat = {
            "time": now,
            "num_beams": len(ranges),
            "angle_min": float(self.latest_scan.angle_min),
            "angle_max": float(self.latest_scan.angle_max),
            "angle_increment": float(self.latest_scan.angle_increment),
            "inf_count": int(np.sum(inf_mask)),
            "nan_count": int(np.sum(nan_mask)),
            "finite_count": int(np.sum(finite_mask)),
            "min_range": float(np.min(valid_ranges)) if len(valid_ranges) > 0 else float("inf"),
            "max_range": float(np.max(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
            "mean_range": float(np.mean(valid_ranges)) if len(valid_ranges) > 0 else 0.0,
        }
        self.scan_records.append(scan_stat)

        # 2. Downsample scan to 72 beams (matching rl_local_planner & nav_env)
        raw_clean = ranges.copy()
        raw_clean[inf_mask | nan_mask] = 10.0
        n_raw = len(raw_clean)
        step = n_raw // 72
        downsampled = np.empty(72, dtype=np.float32)
        for i in range(72):
            downsampled[i] = np.min(raw_clean[i * step: (i + 1) * step])
        norm_scan = np.clip(downsampled, 0.10, 10.0) / 10.0

        # 3. Compute target lookahead from plan or fallback
        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y
        q = self.latest_odom.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        ryaw = math.atan2(siny_cosp, cosy_cosp)
        vx = self.latest_odom.twist.twist.linear.x
        wz = self.latest_odom.twist.twist.angular.z

        # Target point (lookahead or default waypoint)
        gx, gy = -5.0, -2.5  # Kitchen default if plan empty
        if self.latest_plan and len(self.latest_plan.poses) > 0:
            # find closest then lookahead 0.8m
            pts = [(p.pose.position.x, p.pose.position.y) for p in self.latest_plan.poses]
            min_i = min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - rx, pts[i][1] - ry))
            accum = 0.0
            tgt_i = min_i
            for i in range(min_i, len(pts) - 1):
                accum += math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
                if accum >= 0.80:
                    tgt_i = i + 1
                    break
            gx, gy = pts[tgt_i]

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        norm_dist = float(np.tanh(dist / 5.0))

        world_b = math.atan2(dy, dx)
        rel_b = (world_b - ryaw + np.pi) % (2.0 * np.pi) - np.pi
        sin_b = float(math.sin(rel_b))
        cos_b = float(math.cos(rel_b))

        norm_v = float(np.clip(vx / 0.50, -1.0, 1.0))
        norm_w = float(np.clip(wz / 1.50, -1.0, 1.0))

        obs_79 = np.concatenate([
            norm_scan.astype(np.float32),
            np.array([norm_dist, sin_b, cos_b], dtype=np.float32),
            self.last_action.astype(np.float32),
            np.array([norm_v, norm_w], dtype=np.float32)
        ]).astype(np.float32)
        self.obs_records.append(obs_79)

        # 4. Policy inference
        if self.ort_session is not None:
            out = self.ort_session.run(None, {self.input_name: obs_79.reshape(1, -1)})
            action = out[0].ravel()
        else:
            action = np.zeros(2, dtype=np.float32)

        a_v, a_w = float(action[0]), float(action[1])
        scaled_v = (a_v + 1.0) / 2.0 * (0.50 - (-0.20)) + (-0.20)
        scaled_w = a_w * 1.50
        clamped_v = float(np.clip(scaled_v, -0.20, 0.50))
        clamped_w = float(np.clip(scaled_w, -1.50, 1.50))

        self.last_action = np.array([a_v, a_w], dtype=np.float32)

        self.action_records.append({
            "time": now,
            "raw_av": a_v,
            "raw_aw": a_w,
            "scaled_v": scaled_v,
            "scaled_w": scaled_w,
            "clamped_v": clamped_v,
            "clamped_w": clamped_w,
            "goal_dist": dist,
            "rel_bearing_deg": math.degrees(rel_b)
        })


def compare_with_kinematic(live_obs: np.ndarray) -> Dict[str, Any]:
    """Generates kinematic simulation observations and performs distribution comparison."""
    from gracemo_rl.envs.nav_env import GracemoNavEnv, NavEnvConfig

    config = NavEnvConfig()
    env = GracemoNavEnv(mode="kinematic", config=config)
    kinematic_obs_list = []

    # Run 5 episodes in kinematic env to gather representative observation distribution
    for ep in range(5):
        obs, _ = env.reset(seed=42 + ep)
        kinematic_obs_list.append(obs)
        for _ in range(50):
            action = env.action_space.sample()
            obs, _, term, trunc, _ = env.step(action)
            kinematic_obs_list.append(obs)
            if term or trunc:
                break

    kin_obs = np.array(kinematic_obs_list)  # Shape: (N, 79)
    live_obs_arr = np.array(live_obs)        # Shape: (M, 79)

    # Per-dimension summary statistics: [mean, std, min, max]
    kin_mean = np.mean(kin_obs, axis=0)
    kin_std = np.std(kin_obs, axis=0)
    live_mean = np.mean(live_obs_arr, axis=0)
    live_std = np.std(live_obs_arr, axis=0)

    # LiDAR range stats (dims 0..71)
    kin_scan_mean = float(np.mean(kin_mean[:72]))
    live_scan_mean = float(np.mean(live_mean[:72]))
    kin_scan_min = float(np.min(kin_obs[:, :72]))
    live_scan_min = float(np.min(live_obs_arr[:, :72]))

    # Target features stats (dims 72..74)
    # 72: dist, 73: sin, 74: cos
    dist_gap = float(abs(live_mean[72] - kin_mean[72]))
    bearing_gap = float(math.hypot(live_mean[73] - kin_mean[73], live_mean[74] - kin_mean[74]))

    # Velocity stats (dims 77..78)
    vel_gap = float(abs(live_mean[77] - kin_mean[77]))

    # Histogram comparison (L1 divergence on sector distances)
    hist_kin, bins = np.histogram(kin_obs[:, :72].ravel(), bins=10, range=(0.0, 1.0), density=True)
    hist_live, _ = np.histogram(live_obs_arr[:, :72].ravel(), bins=bins, density=True)
    l1_divergence = float(0.5 * np.sum(np.abs(hist_kin - hist_live)) * (bins[1] - bins[0]))

    return {
        "kin_samples": len(kin_obs),
        "live_samples": len(live_obs_arr),
        "kin_scan_mean": kin_scan_mean,
        "live_scan_mean": live_scan_mean,
        "kin_scan_min": kin_scan_min,
        "live_scan_min": live_scan_min,
        "dist_dim_gap": dist_gap,
        "bearing_dim_gap": bearing_gap,
        "vel_dim_gap": vel_gap,
        "scan_l1_divergence": l1_divergence,
        "dim_mismatches": {
            "lidar_min_clip": bool(live_scan_min < 0.05),
            "bearing_norm_valid": bool(np.all((live_obs_arr[:, 73] >= -1.01) & (live_obs_arr[:, 73] <= 1.01))),
            "distance_tanh_valid": bool(np.all((live_obs_arr[:, 72] >= 0.0) & (live_obs_arr[:, 72] <= 1.01))),
        }
    }


def main():
    rclpy.init()
    probe = RLLiveDiagnosticProbe()

    print("\n" + "="*60)
    print("  PHASE B: RL LOCAL PLANNER & VELOCITY PIPELINE PROBE")
    print("="*60)
    print("Awaiting initial telemetry (/scan, /odom, /cmd_vel_mux/status)...")

    t_start = time.time()
    while time.time() - t_start < 10.0:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if probe.latest_scan is not None and probe.latest_odom is not None:
            break

    if probe.latest_scan is None or probe.latest_odom is None:
        print("ERROR: Telemetry timeout. Gazebo simulation or bridge is not publishing.")
        probe.destroy_node()
        rclpy.shutdown()
        return

    print("✓ Telemetry received. Dispatching AT-02 Kitchen goal to coordinator...")
    cmd = {"intent": "NAVIGATE_TO", "destination": "kitchen"}
    msg = RosString()
    msg.data = json.dumps(cmd)
    probe.cmd_pub.publish(msg)
    time.sleep(0.5)

    print("Beginning 20-second high-rate telemetry trace on AT-02...")
    t_end = time.time() + 20.0
    tick_interval = 0.10  # 10 Hz recording
    last_tick = time.time()

    while time.time() < t_end:
        rclpy.spin_once(probe, timeout_sec=0.02)
        if time.time() - last_tick >= tick_interval:
            probe.record_tick()
            last_tick = time.time()

    print(f"\nTrace complete! Recorded {len(probe.obs_records)} ticks over 20.0s.")

    # -------------------------------------------------------------
    # Forensics & Diagnostics Synthesis
    # -------------------------------------------------------------
    scans = probe.scan_records
    actions = probe.action_records
    muxs = probe.mux_status_records
    monitors = probe.monitor_records

    # Raw Scan Statistics
    avg_inf = np.mean([s["inf_count"] for s in scans])
    avg_nan = np.mean([s["nan_count"] for s in scans])
    min_range = np.min([s["min_range"] for s in scans])
    max_range = np.max([s["max_range"] for s in scans])
    angle_min = scans[0]["angle_min"]
    angle_max = scans[0]["angle_max"]
    angle_inc = scans[0]["angle_increment"]

    print("\n--- 1. RAW /scan TELEMETRY FORENSICS ---")
    print(f"Beams: {scans[0]['num_beams']} | Angle range: [{angle_min:.3f}, {angle_max:.3f}] rad (inc: {angle_inc:.4f})")
    print(f"Angle ordering: Index 0 = {angle_min:.3f} rad (-pi rear), Index 180 = ~0.0 rad (front)")
    print(f"Inf beams per scan (mean): {avg_inf:.1f} / {scans[0]['num_beams']} ({avg_inf/scans[0]['num_beams']*100:.1f}%)")
    print(f"NaN beams per scan (mean): {avg_nan:.1f}")
    print(f"Observed range envelope: [{min_range:.3f}m, {max_range:.3f}m]")

    # Policy Action & Clamping Forensics
    raw_avs = [a["raw_av"] for a in actions]
    raw_aws = [a["raw_aw"] for a in actions]
    clamped_vs = [a["clamped_v"] for a in actions]
    clamped_ws = [a["clamped_w"] for a in actions]

    print("\n--- 2. RL POLICY & CLAMPING FORENSICS ---")
    print(f"Raw action a_v: mean={np.mean(raw_avs):.3f}, min={np.min(raw_avs):.3f}, max={np.max(raw_avs):.3f}")
    print(f"Raw action a_w: mean={np.mean(raw_aws):.3f}, min={np.min(raw_aws):.3f}, max={np.max(raw_aws):.3f}")
    print(f"Clamped linear vel: mean={np.mean(clamped_vs):.3f} m/s, max={np.max(clamped_vs):.3f} m/s")
    print(f"Clamped angular vel: mean={np.mean(clamped_ws):.3f} rad/s, max={np.max(clamped_ws):.3f} rad/s")

    # Pipeline Velocity Transmission Forensics
    print("\n--- 3. VELOCITY TRANSMISSION PIPELINE FORENSICS ---")
    print(f"/cmd_vel_rl published count: {len(probe.cmd_rl_records)}")
    if probe.cmd_rl_records:
        rl_vx = [c["vx"] for c in probe.cmd_rl_records]
        rl_wz = [c["wz"] for c in probe.cmd_rl_records]
        print(f"  /cmd_vel_rl linear.x: mean={np.mean(rl_vx):.3f} m/s, max={np.max(rl_vx):.3f} m/s")
        print(f"  /cmd_vel_rl angular.z: mean={np.mean(rl_wz):.3f} rad/s, max={np.max(rl_wz):.3f} rad/s")

    print(f"/cmd_vel_mux published count: {len(probe.cmd_mux_records)}")
    if probe.cmd_mux_records:
        mux_vx = [c["vx"] for c in probe.cmd_mux_records]
        print(f"  /cmd_vel_mux linear.x: mean={np.mean(mux_vx):.3f} m/s, max={np.max(mux_vx):.3f} m/s")

    # Mux status analysis
    if muxs:
        sources = [m.get("active_source", "UNKNOWN") for m in muxs]
        from collections import Counter
        counts = Counter(sources)
        print(f"cmd_vel_mux source distribution: {dict(counts)}")

    # Collision Monitor Forensics
    print("\n--- 4. COLLISION MONITOR SAFETY FORENSICS ---")
    if monitors:
        actions_list = [m["action"] for m in monitors]
        from collections import Counter
        m_counts = Counter(actions_list)
        print(f"Collision monitor states observed: {dict(m_counts)}")
    else:
        print("Collision monitor states observed: {'none': 100% (No safety intervention triggered)}")

    print(f"/cmd_vel_safe published count: {len(probe.cmd_safe_records)}")
    print(f"/cmd_vel (final motor output) published count: {len(probe.cmd_final_records)}")

    # Distribution comparison with Kinematic Simulation
    print("\n--- 5. OBSERVATION DISTRIBUTION: GAZEBO LIVE vs KINEMATIC SIM ---")
    comparison = compare_with_kinematic(np.array(probe.obs_records))
    print(f"Kinematic samples: {comparison['kin_samples']} | Live Gazebo samples: {comparison['live_samples']}")
    print(f"LiDAR scan mean: Live={comparison['live_scan_mean']:.3f} | Kinematic={comparison['kin_scan_mean']:.3f}")
    print(f"LiDAR min clearance: Live={comparison['live_scan_min']:.3f}m | Kinematic={comparison['kin_scan_min']:.3f}m")
    print(f"Feature gap (normalized distance tanh): {comparison['dist_dim_gap']:.4f}")
    print(f"Feature gap (bearing vector sin/cos): {comparison['bearing_dim_gap']:.4f}")
    print(f"Feature gap (velocity): {comparison['vel_dim_gap']:.4f}")
    print(f"Sector LiDAR L1 divergence: {comparison['scan_l1_divergence']:.4f}")
    print(f"Dimension validity checks: {comparison['dim_mismatches']}")

    # Check where command is lost or zeroed
    print("\n--- 6. STALL ROOT CAUSE DIAGNOSIS ---")
    if len(probe.cmd_rl_records) == 0:
        print("⚠️ COMMAND LOST AT SOURCE: /cmd_vel_rl received 0 messages!")
        print("   Reason: rl_local_planner suppresses publishing when self.active_path is None or plan_stale (>2.0s).")
    elif all(c["vx"] == 0.0 and c["wz"] == 0.0 for c in probe.cmd_rl_records):
        print("⚠️ ZERO COMMAND GENERATED: /cmd_vel_rl published zero twist.")
    elif muxs and all(m.get("active_source") != "RL" for m in muxs):
        print("⚠️ ARBITRATION OVERRIDE: cmd_vel_mux never selected 'RL' as active source.")
    elif any(m["action"] == "stop" for m in monitors):
        print("⚠️ COLLISION SHIELD HALT: collision_monitor PolygonStop halted output.")
    else:
        print("✓ Velocity transmission active across pipeline.")

    # Save complete diagnostic json report
    out_report = {
        "scan_forensics": {
            "num_beams": scans[0]["num_beams"],
            "angle_min": angle_min,
            "angle_max": angle_max,
            "angle_inc": angle_inc,
            "avg_inf_beams": avg_inf,
            "avg_nan_beams": avg_nan,
            "min_range": min_range,
            "max_range": max_range
        },
        "policy_forensics": {
            "mean_av": float(np.mean(raw_avs)),
            "mean_aw": float(np.mean(raw_aws)),
            "mean_clamped_v": float(np.mean(clamped_vs)),
            "mean_clamped_w": float(np.mean(clamped_ws))
        },
        "pipeline_forensics": {
            "cmd_rl_count": len(probe.cmd_rl_records),
            "cmd_mux_count": len(probe.cmd_mux_records),
            "cmd_safe_count": len(probe.cmd_safe_records),
            "cmd_final_count": len(probe.cmd_final_records)
        },
        "distribution_comparison": comparison
    }

    report_path = "scripts/rl_live_diagnosis_report.json"
    with open(report_path, "w") as f:
        json.dump(out_report, f, indent=2)
    print(f"\nDetailed diagnostic report saved to: {report_path}")

    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

