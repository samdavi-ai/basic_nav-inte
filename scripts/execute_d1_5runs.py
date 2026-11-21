#!/usr/bin/env python3
"""
GRaCEmo ViRa — D1 Sign-Off Verification: 5 Consecutive Acceptance Suite Runs (DWB + AMCL)
Definition of Done D1: DWB passes AT-01..AT-07 (hardened versions) 5/5 runs, with AMCL ON, static localization OFF.
"""

import os
import sys
import time
import json
import math
import subprocess

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
SCRATCH_DIR = os.path.join(REPO_ROOT, "scratch")
os.makedirs(SCRATCH_DIR, exist_ok=True)


def ensure_sim_running():
    res = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True)
    if "/autonomy/state" in res.stdout:
        print("[D1 Verification] Headless Gazebo & Nav2 already running.")
        return None

    print("[D1 Verification] Launching headless Gazebo & Nav2 stack (AMCL ON, static_loc=false)...")
    sim_log = open(os.path.join(SCRATCH_DIR, "d1_sim.log"), "w")
    proc = subprocess.Popen([
        "ros2", "launch", "gracemo_gazebo", "vira_complete.launch.py",
        "use_rviz:=false",
        "use_rl:=false",
        "use_static_localization:=false"
    ], stdout=sim_log, stderr=subprocess.STDOUT)

    for i in range(45):
        time.sleep(1)
        chk = subprocess.run(["ros2", "topic", "list"], capture_output=True, text=True)
        if "/autonomy/state" in chk.stdout:
            print(f"[D1 Verification] Stack fully active after {i+1}s.")
            time.sleep(2)
            return proc

    raise RuntimeError("Failed to launch simulation and Nav2 stack within 45s.")


def run_5_acceptance_runs():
    sim_proc = ensure_sim_running()
    suite_script = os.path.join(REPO_ROOT, "scripts/execute_navigation_acceptance_suite.py")

    all_runs = []
    print("\n============================================================")
    print("      EXECUTING D1 VERIFICATION: 5 CONSECUTIVE RUNS")
    print("============================================================")

    for run_num in range(1, 6):
        print(f"\n>>> STARTING ACCEPTANCE RUN {run_num} OF 5 <<<")
        t0 = time.time()
        res = subprocess.run([sys.executable, suite_script, "--controller", "DWB"], capture_output=True, text=True)
        dur = time.time() - t0

        output = res.stdout + "\n" + res.stderr
        print(output[-400:] if len(output) > 400 else output)

        # Parse test results from stdout
        run_results = {}
        for line in output.splitlines():
            line_str = line.strip()
            for t_id in ["AT-01", "AT-02", "AT-03", "AT-04", "AT-05", "AT-06", "AT-07"]:
                if f"{t_id} Result:" in line_str or f"  {t_id}: [" in line_str:
                    status = "PASS" if "PASS" in line_str else ("INVALID" if "INVALID" in line_str else "FAIL")
                    run_results[t_id] = status

        pass_count = sum(1 for s in run_results.values() if s == "PASS")
        print(f">>> RUN {run_num}/5 RESULT: {pass_count}/7 PASSED (Duration: {dur:.1f}s) <<<")

        all_runs.append({
            "run": run_num,
            "duration_sec": round(dur, 1),
            "pass_count": pass_count,
            "total_tests": 7,
            "results": run_results,
            "all_passed": (pass_count == 7)
        })

    if sim_proc:
        try:
            sim_proc.terminate()
            sim_proc.wait(timeout=5)
        except Exception:
            sim_proc.kill()

    out_file = os.path.join(SCRATCH_DIR, "d1_five_runs_verification.json")
    with open(out_file, "w") as f:
        json.dump({
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
            "runs": all_runs,
            "success_rate": sum(1 for r in all_runs if r["all_passed"]) / 5.0,
            "mean_duration_sec": sum(r["duration_sec"] for r in all_runs) / 5.0
        }, f, indent=2)

    print("\n================== D1 VERIFICATION SUMMARY ==================")
    print(f"Total Runs: 5")
    perfect_runs = sum(1 for r in all_runs if r["all_passed"])
    print(f"7/7 Perfect Runs: {perfect_runs}/5")
    for r in all_runs:
        print(f"  Run {r['run']}: {r['pass_count']}/7 | {r['results']}")
    print(f"Report saved to: {out_file}")
    print("=============================================================")
    return 0 if perfect_runs == 5 else 1


if __name__ == "__main__":
    sys.exit(run_5_acceptance_runs())

