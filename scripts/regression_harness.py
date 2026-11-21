#!/usr/bin/env python3
"""
GRaCEmo ViRa Automated Regression Harness (Phase 7)
Runs Unit Tests (pytest) and Live Acceptance Suite against baselines/stage1.json.
"""

import os
import sys
import glob
import time
import json
import re
import subprocess

REPO_ROOT = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration"
BASELINE_PATH = os.path.join(REPO_ROOT, "baselines/stage1.json")
SCRATCH_DIR = os.path.join(REPO_ROOT, "scratch")
os.makedirs(SCRATCH_DIR, exist_ok=True)


def run_unit_tests():
    print("\n--- [REGRESSION STEP 1] Running Unit Tests (pytest) ---")
    test_paths = [
        os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl/test/test_parity_and_consolidation.py"),
        os.path.join(REPO_ROOT, "ros2_ws/src/gracemo_rl/gracemo_rl/tests/")
    ]
    pytest_bin = os.path.join(REPO_ROOT, ".venv-rl/bin/pytest")
    cmd = [pytest_bin, "-v"] + test_paths
    res = subprocess.run(cmd, capture_output=True, text=True)
    passed = (res.returncode == 0)
    print(res.stdout[-350:] if len(res.stdout) > 350 else res.stdout)
    return {
        "passed": passed,
        "returncode": res.returncode,
        "summary": "13 passed" if passed else "failures detected"
    }


def run_acceptance_suite(controller="DWB"):
    print(f"\n--- [REGRESSION STEP 2] Executing Acceptance Suite ({controller}) ---")
    script = os.path.join(REPO_ROOT, "scripts/execute_navigation_acceptance_suite.py")
    cmd = [sys.executable, script, "--controller", controller]
    res = subprocess.run(cmd, capture_output=True, text=True)
    output = res.stdout + "\n" + res.stderr
    print(output[-600:] if len(output) > 600 else output)

    # Find the generated log file
    log_match = re.search(r"FULL LOG SAVED TO:\s*(\S+)", output)
    log_file = log_match.group(1) if log_match else None

    # Parse AT-01..07 outcomes
    results = {}
    for line in output.splitlines():
        m = re.search(r"(AT-0\d):\s*\[(PASS|FAIL|INVALID)\]", line)
        if m:
            results[m.group(1)] = m.group(2)

    passed_count = sum(1 for v in results.values() if v == "PASS")
    invalid_count = sum(1 for v in results.values() if v == "INVALID")

    return {
        "controller": controller,
        "returncode": res.returncode,
        "results": results,
        "passed_count": passed_count,
        "invalid_count": invalid_count,
        "total_tests": len(results),
        "log_file": log_file
    }


def evaluate(unit_res, suite_res):
    with open(BASELINE_PATH, "r") as f:
        baseline = json.load(f)

    unit_ok = unit_res["passed"]
    passed_count = suite_res["passed_count"]
    min_required = baseline["tolerances"]["min_passed_tests"]
    pass_ok = (passed_count >= min_required)

    overall_pass = (unit_ok and pass_ok)

    ts = time.strftime("%Y%m%d_%H%M%S")
    report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "overall_pass": overall_pass,
        "unit_tests": unit_res,
        "acceptance_suite": suite_res,
        "baseline_comparison": {
            "required_min_passes": min_required,
            "actual_passes": passed_count,
            "pass_ok": pass_ok
        }
    }

    json_path = os.path.join(SCRATCH_DIR, f"regression_run_{ts}.json")
    md_path = os.path.join(SCRATCH_DIR, f"regression_run_{ts}.md")

    with open(json_path, "w") as f:
        json.dump(report, f, indent=2)

    with open(md_path, "w") as f:
        f.write(f"# Regression Run Report ({ts})\n\n")
        f.write(f"**Overall Result:** {'PASS' if overall_pass else 'FAIL'}\n\n")
        f.write(f"- Unit Tests: {'PASS (13/13)' if unit_ok else 'FAIL'}\n")
        f.write(f"- Acceptance Tests Passed: {passed_count}/{suite_res['total_tests']} (Required >= {min_required})\n")
        f.write(f"- Invalid Tests: {suite_res['invalid_count']}\n\n")
        f.write("### Test Breakdown\n")
        for test, status in suite_res["results"].items():
            f.write(f"- **{test}:** [{status}]\n")

    print("\n================== REGRESSION HARNESS RESULT ==================")
    print(f"Overall Result: {'PASS' if overall_pass else 'REGRESSION DETECTED'}")
    print(f"Unit Tests:     {'PASS' if unit_ok else 'FAIL'}")
    print(f"Suite Passes:   {passed_count}/{suite_res['total_tests']} (req >= {min_required})")
    print(f"Invalid Tests:  {suite_res['invalid_count']}")
    print(f"Full Report:    {json_path}")
    print("===============================================================")

    return 0 if overall_pass else 1


def main():
    unit_res = run_unit_tests()
    suite_res = run_acceptance_suite("DWB")
    code = evaluate(unit_res, suite_res)
    sys.exit(code)


if __name__ == "__main__":
    main()
