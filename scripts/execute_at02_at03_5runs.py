#!/usr/bin/env python3
"""
GRaCEmo ViRa — AT-02 and AT-03 5-Run Acceptance Runner on New Map
Runs AT-02 (Kitchen) and AT-03 (Circuit) 5 times each with AMCL on, static off.
Logs dual-source ground-truth arrival, collision monitor counts, door-relative distance,
and invalid counts. Writes full results to scratch/at02_at03_new_map_results.json.
"""

import os
import sys
import time
import json
import math
import rclpy

# Add scripts directory to path to import NavigationAcceptanceTester
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from execute_navigation_acceptance_suite import NavigationAcceptanceTester, wrap_pi

def run_5runs_evaluation(out_filename="at02_at03_new_map_results.json"):
    rclpy.init()
    log_name = out_filename.replace(".json", ".log")
    log_file = os.path.join(SCRIPT_DIR, "..", "scratch", log_name)
    tester = NavigationAcceptanceTester(log_file=log_file)

    tester.log("\n=======================================================")
    tester.log(f"  ACCEPTANCE TESTING: AT-02 & AT-03 (5 RUNS EACH) -> {out_filename}")
    tester.log("=======================================================\n")

    # Wait for subsystems
    t0 = time.time()
    tester.log("Waiting for subsystems to initialize...")
    while time.time() - t0 < 15.0:
        rclpy.spin_once(tester, timeout_sec=0.2)
        if tester.has_odom and tester.has_gz and tester.cmd_pub.get_subscription_count() > 0:
            break

    results = {
        "at02_runs": [],
        "at03_runs": [],
        "summary": {}
    }

    # 1. AT-02 Kitchen Navigation (5 Runs)
    tester.log("\n>>> STARTING AT-02 KITCHEN NAVIGATION (5 RUNS) <<<")
    at02_passes = 0
    at02_invalids = 0

    for r in range(1, 6):
        tester.log(f"\n--- AT-02 Run {r}/5 ---")
        status, meta = tester.run_at02_kitchen_navigation(max_wait_sec=90.0)
        
        # Calculate door relative metrics (Kitchen door opening: X in [-5.60, -4.40], Y = -1.0)
        gx, gy = tester.gz_x, tester.gz_y
        dist_west_jamb = math.hypot(gx - (-5.60), gy - (-1.00))
        dist_east_jamb = math.hypot(gx - (-4.40), gy - (-1.00))
        dist_door_center = math.hypot(gx - (-5.00), gy - (-1.00))

        run_record = {
            "test": "AT-02",
            "run": r,
            "status": status,
            "supervisor_state": tester.supervisor_state,
            "stop_count": tester.collision_stop_count,
            "slow_count": tester.collision_slow_count,
            "gz_pose": [round(tester.gz_x, 3), round(tester.gz_y, 3), round(tester.gz_yaw, 3)],
            "tf_pose": [round(p, 3) for p in (tester.get_tf_map_pose() or [0, 0, 0])],
            "meta": meta,
            "door_relative": {
                "dist_west_jamb_m": round(dist_west_jamb, 3),
                "dist_east_jamb_m": round(dist_east_jamb, 3),
                "dist_door_center_m": round(dist_door_center, 3)
            }
        }
        results["at02_runs"].append(run_record)
        if status == "PASS":
            at02_passes += 1
        elif status == "INVALID":
            at02_invalids += 1

    # 2. AT-03 Multi-Waypoint Circuit (5 Runs)
    tester.log("\n>>> STARTING AT-03 MULTI-WAYPOINT CIRCUIT (5 RUNS) <<<")
    at03_passes = 0
    at03_invalids = 0

    for r in range(1, 6):
        tester.log(f"\n--- AT-03 Run {r}/5 ---")
        status, meta = tester.run_at03_multi_waypoint_circuit(max_wait_sec=90.0)
        
        run_record = {
            "test": "AT-03",
            "run": r,
            "status": status,
            "supervisor_state": tester.supervisor_state,
            "stop_count": tester.collision_stop_count,
            "slow_count": tester.collision_slow_count,
            "gz_pose": [round(tester.gz_x, 3), round(tester.gz_y, 3), round(tester.gz_yaw, 3)],
            "tf_pose": [round(p, 3) for p in (tester.get_tf_map_pose() or [0, 0, 0])],
            "meta": meta
        }
        results["at03_runs"].append(run_record)
        if status == "PASS":
            at03_passes += 1
        elif status == "INVALID":
            at03_invalids += 1

    results["summary"] = {
        "at02_pass_count": at02_passes,
        "at02_invalid_count": at02_invalids,
        "at02_total": 5,
        "at03_pass_count": at03_passes,
        "at03_invalid_count": at03_invalids,
        "at03_total": 5
    }

    out_json = os.path.join(SCRIPT_DIR, "..", "scratch", out_filename)
    with open(out_json, "w") as f:
        json.dump(results, f, indent=2)

    tester.log(f"\nSaved test results to: {out_json}")
    tester.log(f"AT-02 Result: {at02_passes}/5 PASS (Invalids: {at02_invalids})")
    tester.log(f"AT-03 Result: {at03_passes}/5 PASS (Invalids: {at03_invalids})")

    tester.destroy_node()
    rclpy.shutdown()
    return results

if __name__ == "__main__":
    out_file = sys.argv[1] if len(sys.argv) > 1 else "at02_at03_new_map_results.json"
    run_5runs_evaluation(out_file)

