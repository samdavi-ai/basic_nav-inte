#!/usr/bin/env python3
"""
GRaCEmo ViRa: Sweep Policy Gate
Authoritative verification gate for RL local planner policy:
1. Goal bearing grid from -180 deg to +180 deg (30 deg steps) + fine grid around 0 deg (-10 to +10 deg)
2. Goal distances: 1 m, 3 m, 6 m
3. Free space test:
   - |w| < 0.15 rad/s when |bearing| < 10 deg
   - sign(w) == sign(bearing) when |bearing| >= 5 deg
   - dw/dbearing < 3.0 rad/s/rad
4. Wall response test:
   - v lower near a 0.4 m front wall than in free space
Must achieve 100% PASS to qualify for Gazebo deployment.
"""

import os
import sys
import math
import json
from typing import Dict, List, Tuple, Any
import numpy as np

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
sys.path.insert(0, os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl"))

import onnxruntime as ort
from gracemo_rl.common.obs import build_observation
from gracemo_rl.common.limits import (
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_REVERSE_VEL,
    RL_MIN_SCAN_RANGE,
    RL_MAX_SCAN_RANGE
)


def evaluate_policy_point(
    session: ort.InferenceSession,
    input_name: str,
    bearing_rad: float,
    dist_m: float,
    front_obstacle_dist: float = 10.0,
    v_max: float = RL_MAX_LINEAR_VEL,
    w_max: float = RL_MAX_ANGULAR_VEL,
    v_min: float = RL_MIN_REVERSE_VEL
) -> Tuple[float, float, float, float]:
    """
    Evaluates policy at synthetic state:
    Robot at (0, 0, 0), goal at (dist * cos(bearing), dist * sin(bearing)).
    """
    gx = dist_m * math.cos(bearing_rad)
    gy = dist_m * math.sin(bearing_rad)

    # 72 beams across [-pi, pi], angle 0 (straight ahead) is index 36
    # Angles: i * 2pi / 72 - pi
    # Indices 30 to 42 are front [-30 deg, +30 deg]
    scan_ranges = np.full(72, 10.0, dtype=np.float32)
    if front_obstacle_dist < 10.0:
        for idx in range(30, 42):
            scan_ranges[idx] = float(front_obstacle_dist)

    obs = build_observation(
        scan_ranges=scan_ranges,
        robot_x=0.0,
        robot_y=0.0,
        robot_yaw=0.0,
        goal_x=gx,
        goal_y=gy,
        last_action=np.zeros(2, dtype=np.float32),
        robot_vx=0.0,
        robot_wz=0.0,
        max_linear_vel=v_max,
        max_angular_vel=w_max,
        min_scan_range=RL_MIN_SCAN_RANGE,
        max_scan_range=RL_MAX_SCAN_RANGE
    ).reshape(1, -1)

    out = session.run(None, {input_name: obs})[0].ravel()
    a_v, a_w = float(out[0]), float(out[1])

    # Convert to physical cmd velocities matching rl_local_planner.py
    v = (a_v + 1.0) / 2.0 * (v_max - v_min) + v_min
    w = a_w * w_max
    v = float(np.clip(v, v_min, v_max))
    w = float(np.clip(w, -w_max, w_max))

    return a_v, a_w, v, w


def run_sweep_gate(
    model_path: str = "ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_policy.onnx",
    output_report: str = "scratch/post_bc_sweep_grid_results.json"
) -> Dict[str, Any]:
    print(f"\n==================================================")
    print(f"       GRaCEmo ViRa: Sweep Policy Gate Check       ")
    print(f"==================================================")
    assert os.path.exists(model_path), f"ONNX model not found: {model_path}"
    print(f"Loading ONNX policy: {model_path}...")
    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    distances = [1.0, 3.0, 6.0]
    # Standard grid -180 to +180 deg in 30 deg steps
    coarse_bearings_deg = list(range(-180, 181, 30))
    # Fine grid around 0 deg (-10 to +10 deg in 2 deg steps)
    fine_bearings_deg = [-10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 8.0, 10.0]

    all_bearings_deg = sorted(list(set(coarse_bearings_deg + fine_bearings_deg)))

    gate_passes = True
    failure_reasons = []

    results_by_dist = {}

    total_checks = 0
    passed_checks = 0

    print("\n--- 1. Evaluating Bearing Grid & Smoothness ---")
    for d in distances:
        dist_results = []
        prev_w = None
        prev_b_rad = None

        for b_deg in all_bearings_deg:
            b_rad = math.radians(b_deg)
            a_v, a_w, v, w = evaluate_policy_point(session, input_name, b_rad, d)

            check_zero_heading = True
            check_sign = True
            check_deriv = True

            # Criterion 1: |w| < 0.15 rad/s when |bearing| < 10 deg
            if abs(b_deg) < 10.0:
                total_checks += 1
                if abs(w) >= 0.15:
                    check_zero_heading = False
                    gate_passes = False
                    reason = f"Fail |w| < 0.15 at d={d}m, bearing={b_deg:+.1f} deg: w={w:+.4f} rad/s"
                    failure_reasons.append(reason)
                    print(f"  ❌ {reason}")
                else:
                    passed_checks += 1

            # Criterion 2: sign(w) == sign(bearing) when |bearing| >= 5 deg
            if abs(b_deg) >= 5.0 and abs(b_deg) < 175.0:
                total_checks += 1
                if (b_deg > 0 and w <= 0.0) or (b_deg < 0 and w >= 0.0):
                    check_sign = False
                    gate_passes = False
                    reason = f"Fail sign(w) == sign(bearing) at d={d}m, bearing={b_deg:+.1f} deg: w={w:+.4f}"
                    failure_reasons.append(reason)
                    print(f"  ❌ {reason}")
                else:
                    passed_checks += 1

            # Criterion 3: dw/dbearing < 3.0 rad/s/rad
            if prev_w is not None and prev_b_rad is not None:
                delta_b = b_rad - prev_b_rad
                if delta_b > 1e-4:
                    dw_db = abs(w - prev_w) / delta_b
                    total_checks += 1
                    if dw_db >= 3.0:
                        check_deriv = False
                        gate_passes = False
                        reason = f"Fail dw/db < 3.0 at d={d}m, between {math.degrees(prev_b_rad):.1f} and {b_deg:.1f} deg: dw/db={dw_db:.3f}"
                        failure_reasons.append(reason)
                        print(f"  ❌ {reason}")
                    else:
                        passed_checks += 1

            prev_w = w
            prev_b_rad = b_rad

            dist_results.append({
                "bearing_deg": b_deg,
                "bearing_rad": b_rad,
                "a_v": a_v,
                "a_w": a_w,
                "v": v,
                "w": w,
                "pass_zero_heading": check_zero_heading,
                "pass_sign": check_sign,
                "pass_deriv": check_deriv
            })

        results_by_dist[str(d)] = dist_results

    # Print representative table at d=3m
    print("\nRepresentative Grid at d = 3.0 m:")
    print(f"  {'Bearing (deg)':>14} | {'a_v':>7} | {'a_w':>7} | {'v (m/s)':>8} | {'w (rad/s)':>10} | {'Status'}")
    print("  " + "-" * 60)
    for res in results_by_dist["3.0"]:
        b = res["bearing_deg"]
        if b in coarse_bearings_deg or abs(b) in (0.0, 5.0, 10.0):
            status = "PASS" if (res["pass_zero_heading"] and res["pass_sign"] and res["pass_deriv"]) else "FAIL"
            print(f"  {b:>14.1f} | {res['a_v']:>7.3f} | {res['a_w']:>7.3f} | {res['v']:>8.3f} | {res['w']:>10.4f} | {status}")

    # Criterion 4: Wall response test at 0.4 m front wall vs free space
    print("\n--- 2. Evaluating Wall Response Test (front wall 0.40 m vs free space) ---")
    wall_test_results = {}
    for d in distances:
        _, _, v_free, w_free = evaluate_policy_point(session, input_name, 0.0, d, front_obstacle_dist=10.0)
        _, _, v_wall, w_wall = evaluate_policy_point(session, input_name, 0.0, d, front_obstacle_dist=0.40)

        total_checks += 1
        pass_wall = (v_wall < v_free) and (v_wall <= 0.15)
        if pass_wall:
            passed_checks += 1
            print(f"  ✓ Dist {d}m: Free v={v_free:.3f} m/s -> Wall v={v_wall:.3f} m/s (speed reduced by {(v_free - v_wall)/v_free*100.0:.1f}%) [PASS]")
        else:
            gate_passes = False
            reason = f"Fail wall slowdown at d={d}m: Free v={v_free:.3f}, Wall v={v_wall:.3f}"
            failure_reasons.append(reason)
            print(f"  ❌ {reason}")

        wall_test_results[str(d)] = {
            "v_free": v_free,
            "w_free": w_free,
            "v_wall": v_wall,
            "w_wall": w_wall,
            "pass": pass_wall
        }

    overall_status = "PASS" if gate_passes else "FAIL"
    pass_rate = passed_checks / max(1, total_checks) * 100.0

    print(f"\n==================================================")
    print(f"  Sweep Gate Result: {overall_status} ({passed_checks}/{total_checks} checks passed, {pass_rate:.1f}%)")
    print(f"==================================================")

    out_data = {
        "timestamp": math.floor(os.path.getmtime(model_path)),
        "model_path": model_path,
        "overall_status": overall_status,
        "total_checks": total_checks,
        "passed_checks": passed_checks,
        "pass_rate_pct": pass_rate,
        "failure_reasons": failure_reasons,
        "wall_test": wall_test_results,
        "grid_results": results_by_dist
    }

    os.makedirs(os.path.dirname(os.path.abspath(output_report)), exist_ok=True)
    with open(output_report, "w") as f:
        json.dump(out_data, f, indent=2)
    print(f"Saved sweep gate results to: {output_report}")

    return out_data


if __name__ == "__main__":
    run_sweep_gate()
