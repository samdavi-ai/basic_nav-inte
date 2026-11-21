#!/usr/bin/env python3
"""
GRaCEmo ViRa — Gate C: 10 Consecutive Runs of AT-01 through AT-07.
Logs:
  - Per-test pass/fail/invalid status across 10 complete iterations
  - Exact failure reasons with timestamps and logs
  - For AT-02: exact displacement, distance to door jambs, collision monitor triggers, and Nav2 BT status
  - Gate C target: AT-02 >= 9/10, AT-03 >= 9/10.
Outputs results to scratch/gate_c_10_runs_results.json.
"""

import os
import sys
import time
import json
import math
import rclpy

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from execute_navigation_acceptance_suite import NavigationAcceptanceTester

def run_gate_c_10_iterations():
    rclpy.init()
    log_file = os.path.join(SCRIPT_DIR, "..", "scratch", "gate_c_10_runs_execution.log")
    tester = NavigationAcceptanceTester(log_file=log_file)

    tester.log("\n=======================================================")
    tester.log("      GATE C: 10 CONSECUTIVE RUNS OF AT-01..AT-07")
    tester.log("=======================================================\n")

    # Wait for subsystems
    t0 = time.time()
    tester.log("Waiting for subsystems to initialize...")
    while time.time() - t0 < 15.0:
        rclpy.spin_once(tester, timeout_sec=0.2)
        if tester.has_odom and tester.has_gz and tester.cmd_pub.get_subscription_count() > 0:
            break

    test_keys = ["AT-01", "AT-02", "AT-03", "AT-04", "AT-05", "AT-06", "AT-07"]
    matrix_results = {k: [] for k in test_keys}
    at02_detailed_logs = []

    for run_idx in range(1, 11):
        tester.log(f"\n>>>>>>>>>>>>>>> RUN {run_idx}/10 <<<<<<<<<<<<<<<")

        # AT-01
        st_01, meta_01 = tester.run_at01_basic_waypoint_navigation()
        matrix_results["AT-01"].append({"run": run_idx, "status": st_01, "meta": meta_01})

        # AT-02
        st_02, meta_02 = tester.run_at02_kitchen_navigation(max_wait_sec=90.0)
        gx, gy = tester.gz_x, tester.gz_y
        dist_west = math.hypot(gx - (-5.60), gy - (-1.00))
        dist_east = math.hypot(gx - (-4.40), gy - (-1.00))
        dist_center = math.hypot(gx - (-5.00), gy - (-1.00))

        at02_record = {
            "run": run_idx,
            "status": st_02,
            "meta": meta_02,
            "gz_pose": [round(tester.gz_x, 3), round(tester.gz_y, 3), round(tester.gz_yaw, 3)],
            "tf_pose": [round(p, 3) for p in (tester.get_tf_map_pose() or [0, 0, 0])],
            "collision_stops": tester.collision_stop_count,
            "collision_slows": tester.collision_slow_count,
            "door_distances": {
                "dist_west_jamb_m": round(dist_west, 3),
                "dist_east_jamb_m": round(dist_east, 3),
                "dist_door_center_m": round(dist_center, 3)
            }
        }
        matrix_results["AT-02"].append({"run": run_idx, "status": st_02, "meta": meta_02})
        at02_detailed_logs.append(at02_record)

        # AT-07 (dwell after AT-02 arrival)
        st_07, meta_07 = tester.run_at07_goal_dwell_and_anti_drift()
        matrix_results["AT-07"].append({"run": run_idx, "status": st_07, "meta": meta_07})

        # AT-03
        st_03, meta_03 = tester.run_at03_multi_waypoint_circuit(max_wait_sec=90.0)
        matrix_results["AT-03"].append({"run": run_idx, "status": st_03, "meta": meta_03})

        # AT-04
        st_04, meta_04 = tester.run_at04_emergency_stop_test()
        matrix_results["AT-04"].append({"run": run_idx, "status": st_04, "meta": meta_04})

        # AT-05
        st_05, meta_05 = tester.run_at05_obstacle_recovery_and_replanning()
        matrix_results["AT-05"].append({"run": run_idx, "status": st_05, "meta": meta_05})

        # AT-06
        st_06, meta_06 = tester.run_at06_rotation_and_reverse()
        matrix_results["AT-06"].append({"run": run_idx, "status": st_06, "meta": meta_06})

    # Summary table
    summary_table = {}
    for k in test_keys:
        passes = sum(1 for x in matrix_results[k] if x["status"] == "PASS")
        fails = sum(1 for x in matrix_results[k] if x["status"] == "FAIL")
        invalids = sum(1 for x in matrix_results[k] if x["status"] == "INVALID")
        summary_table[k] = {
            "pass_count": passes,
            "fail_count": fails,
            "invalid_count": invalids,
            "pass_rate_pct": round((passes / 10.0) * 100.0, 1),
            "gate_passed": (passes >= 9) if k in ("AT-02", "AT-03") else (passes >= 8)
        }

    full_output = {
        "summary": summary_table,
        "matrix_results": matrix_results,
        "at02_detailed_diagnostics": at02_detailed_logs
    }

    out_file = os.path.join(SCRIPT_DIR, "..", "scratch", "gate_c_10_runs_results.json")
    with open(out_file, "w") as f:
        json.dump(full_output, f, indent=2)

    tester.log("\n=======================================================")
    tester.log("      GATE C: 10-RUN FINAL SUMMARY")
    tester.log("=======================================================")
    for k, v in summary_table.items():
        tester.log(f"  {k}: {v['pass_count']}/10 PASS ({v['pass_rate_pct']}%) | Fails: {v['fail_count']}, Invalids: {v['invalid_count']}")
    tester.log("=======================================================")
    tester.log(f"Results saved to: {out_file}")

    tester.destroy_node()
    rclpy.shutdown()
    return full_output

if __name__ == "__main__":
    run_gate_c_10_iterations()

    run_gate_c_10_iterations()

