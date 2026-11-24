# VIRA VIR-VISUAL-ALL-03 — Final Execution Report

**Date:** 23 November 2026  
**Environment:** ROS 2 Jazzy + Gazebo Harmonic, desktop CPU  
**Physical robot / motor output:** Not used  
**Jetson hardware:** Excluded as directed  
**Release decision:** **HOLD — mandatory acceptance criteria remain unmet**

## Executive result

This execution repaired three concrete defects, reran the integrated Gazebo visual-goal flow, reran the five-room patrol and idle-zero regression, and measured the visual estimator against independent Gazebo truth. The robot completed the patrol and reached a live object observation pose through the existing perception → K → authorized adapter → Nav2 route. That validates those specific integrated behaviors.

It does not complete visual autonomy. The latest five-room visual-odometry result was **1.656 m translation RMSE, 26.68° rotation RMSE, and 35.7% sampled availability**, versus targets of ≤0.20 m, ≤5°, and ≥95%. A subsequent map-loaded restart recorded a real keyframe relocalization and completed a controlled Nav2 route, but measured **0.776 m translation RMSE and 89.3% tracking availability**. One of ten restart routes completed; a separate map-loaded attempt failed Nav2 readiness. RGB-D loop closure, the person/identity metrics, MNSE lifecycle, operator workflows, nine remaining restart routes, and E-stop during visual recovery are not accepted. Nav2 continues to use AMCL.

## Dependency-aware plan and execution

| Order | Workstream / dependency | Execution and outcome |
|---:|---|---|
| 1 | Preserve existing repository state and inspect ROS/runtime/evidence | Kept all pre-existing changes. Confirmed ROS Jazzy and Gazebo Harmonic packages. `slam_toolbox` exists, but it is LiDAR based; RTAB-Map, ORB-SLAM3, GTSAM, g2o and pygmo are unavailable in the active runtime. |
| 2 | Stabilize RGB-D frame handling before integration | Fixed invalid face ROI handling and NaN depth feature projection. Added regressions. |
| 3 | Stabilize validation runtime imports | Fixed the isolated launcher’s child `PYTHONPATH` so interaction subprocesses can import the repository SDK and adapters. Added a path regression. |
| 4 | Run live integrated Gazebo behaviors | Repeated the 11-waypoint five-room patrol, idle-zero, and visual-to-K-to-authorized-Nav2 object goal. |
| 5 | Measure visual localization against independent truth | Ran the timestamp-matched Gazebo truth evaluator. Results fail the localization gates; do not switch Nav2 from AMCL. |
| 6 | Run saved-map restart campaign | Two independent map-loaded launches were attempted. One relocalized but failed Nav2 readiness; the second relocalized, passed readiness, and completed a controlled route. One of ten route trials completed. |
| 7 | Complete data-dependent and operator acceptance | Blocked by absent moving-actor mesh/test identity set and unavailable GUI operator session; retained prior limited evidence without upgrading its status. |
| 8 | Close release report | Classified each capability below. Release remains HOLD. |

## Capability matrix

Statuses follow the requested vocabulary. `IMPLEMENTED BUT NOT ACCEPTED` means the code path exists, but its required target/evidence has not passed.

| Capability | Status | Evidence / reason |
|---|---|---|
| RGB-D perception and synchronization | IMPLEMENTED BUT NOT ACCEPTED | Live Gazebo camera data was processed in patrol and visual-goal runs; the visual-goal run recorded 55 ready status rows with 0.0 ms sampled RGB/depth skew. No expanded motion, range, occlusion or calibration matrix was run. |
| Object detection and 3D localization | IMPLEMENTED BUT NOT ACCEPTED | Live monitor detection, K graph location, authorized action dispatch, and arrival succeeded. This is a bounded scene test, not general object/range acceptance. |
| Moving-person detection/tracking | IMPLEMENTED BUT NOT ACCEPTED | Earlier two-actor evaluation measured precision 1.00, recall 0.710, IDF1 0.226 and seven ID switches. ALL-03 fixes appearance-candidate status persistence, but no fresh actor run or required threshold campaign was possible. |
| Face detection | IMPLEMENTED BUT NOT ACCEPTED | YuNet/reference and prior Gazebo evidence remain limited. Invalid ROI no longer aborts the synchronized frame and result status remains attached to a live track. No current multi-distance, orientation, lighting and occlusion trial. |
| Consent-based face recognition | PARTIAL | Local consent-gated gallery and deletion code exist. No consenting enrollment/held-out/unknown identity set was available; no FAR/FRR/accuracy is claimed. |
| Person Re-ID | IMPLEMENTED BUT NOT ACCEPTED | Current appearance histogram path is a candidate mechanism only. Earlier genuine/impostor results fail acceptance; no independent cross-camera dataset or fresh ALL-03 Re-ID run. |
| Visual odometry | IMPLEMENTED BUT NOT ACCEPTED | Live poses publish, but current route measured 1.656 m RMSE, 26.68° rotation RMSE and 35.7% availability. It is not fit for Nav2 localization. |
| RGB-D Visual SLAM | MISSING | Current RGB-D estimator does not implement accepted trajectory-wide SLAM. No available RGB-D SLAM backend was present in this ROS environment. `slam_toolbox` is not an RGB-D substitute. |
| Loop closure and trajectory optimization | MISSING | No verified loop-closure event or trajectory-wide optimization implementation/evidence. |
| Map persistence and relocalization | IMPLEMENTED BUT NOT ACCEPTED | Versioned, hashed map protections remain. Two map-loaded runs recorded frame-1 relocalization in their trajectories. One completed a controlled route (1/10); its translation RMSE and availability still fail. No 9/10 or 10/10 campaign. |
| MNSE visual observation lifecycle | PARTIAL | Kernel persists event history and latest world-object state; the required per-observation correction/supersession, retention and deletion propagation workflow is incomplete. |
| K visual reasoning | PARTIAL | Fresh visible-object and object-location queries were exercised live. The full requested current/historical person and object questions, freshness semantics and lifecycle are not complete. |
| PySide6 visual control center | PARTIAL | Existing visual panel and offscreen telemetry check remain. Operator-tested map controls, consent management, memory review/deletion and loss/relocalization workflows were not demonstrated in the live application. |
| Full pipeline desktop performance | IMPLEMENTED BUT NOT ACCEPTED | Short visual-goal run: 8.24 FPS median, 8.70 FPS p95, 55 ready rows, 55.3 ms median detector time, 0.016 ms median tracker time, 571 MB median perception RSS and ~167% process CPU. Patrol processing was 8.94 FPS. These are not a sustained full-pipeline acceptance benchmark; dropped pending frames occurred. |
| E-stop during visual recovery | NOT TESTED | No ALL-03 E-stop injection during tracking loss/relocalization was performed. Existing E-stop systems were not bypassed or changed. |
| Ten independent saved-map restarts | PARTIAL | Two map-loaded trials attempted; both recorded frame-1 relocalization. One passed Nav2 readiness and completed a route; the other failed readiness. No 10/10 campaign. |
| Five-room patrol and idle-zero | IMPLEMENTED AND ACCEPTED for the tested Gazebo run | Post-fix run completed 11/11 waypoints in 185.6 s. Idle-zero passed: command peaks 0.0 on `cmd_vel`, mux and smoothed output; odometry displacement 0.00792 m. |
| Jetson deployment | NOT TESTED / OUT OF SCOPE | Explicitly excluded by the mission. |

## Implementations and root causes repaired

### Face ROI failure

The face stage previously skipped an empty clipped person crop, then indexed a sample that did not exist. That could fail the whole RGB-D worker frame. `_update_faces` now clips the ROI to the image bounds, records `FACE_ROI_INVALID` with an unknown face count, and preserves the active track’s face result on subsequent frames. It does not invent an identity.

### Invalid depth feature arithmetic

Visual odometry projected ORB feature coordinates by multiplying image coordinates by depth before rejecting NaN/out-of-range depth. Gazebo logs showed a NumPy invalid-value warning. Projection now initializes a bounded point array and computes x/y only for finite depth features in the valid 0.1–10 m interval. A warnings-as-errors regression passes. This removes the arithmetic warning; it does not repair VO drift.

### Appearance association state reset

An accepted appearance-gallery candidate was returned for the creation frame, but active-track updates initialized state to `unknown` each frame. Track state now retains `appearance_candidate` while the association remains active. A regression verifies stable track ID and state over successive frames. This is still not a Re-ID acceptance result.

### Validation subprocess import failure

The live patrol had completed, but the wrapper failed in its follow-up interaction subprocess because `gracemo_sdk` was absent from that child’s `PYTHONPATH`. The isolated launcher now supplies repository, SDK, interaction and ROS bridge paths while preserving inherited paths. A regression passes, and the subsequent post-fix patrol’s interaction stage completed.

## Live Gazebo evidence

### Five-room patrol and idle-zero

Evidence: [`patrol_after_fixes/`](visual_all_03/patrol_after_fixes/)

- `patrol.json`: success, 11/11 waypoints, 185.6 s.
- Mission ID: `standalone-f05684a1-9666-4f2d-9638-55defe888ac6`.
- Trace ID: `ab9de22b-2654-4827-85b1-68f1eb5424a2`.
- `idle_zero.json`: passed, zero command peaks, 0.007915 m displacement.
- `interaction.log`: post-patrol runtime interaction completed (“I reached Living Room”), proving the repaired child import path in the live launcher.
- `process_lifecycle_evidence.json`: clean launcher/kernel shutdown.

### Visual object to authorized Nav2 goal

Evidence: [`visual_goal_live_01/`](visual_all_03/visual_goal_live_01/)

- `staging.json`: authorized staging navigation completed.
- `visual_goal.json`: live monitor detection, K returned its world location, registered adapter dispatched `NavigateTo`, Nav2 reached the monitor observation pose.
- Target: `monitor_002`; action trace `8963e8b0-a0e1-4896-b1bc-5cbd4f77d064`; outcome `MISSION_COMPLETED`.
- `visual_status.jsonl`: 55 ready rows, sampled RGB/depth skew 0.0 ms.
- Short run performance summaries are reported above. This does not prove sustained throughput or localization quality.

### Visual localization against independent Gazebo truth

Evidence: [`patrol_after_fixes/measurements.json`](visual_all_03/patrol_after_fixes/measurements.json), [`visual_status.jsonl`](visual_all_03/patrol_after_fixes/visual_status.jsonl), [`visual_truth.jsonl`](visual_all_03/patrol_after_fixes/visual_truth.jsonl), and [`camera_mount.jsonl`](visual_all_03/patrol_after_fixes/camera_mount.jsonl).

- Unique status captures: 199.
- Tracking availability: 35.68%.
- Ground-truth matched fraction: 33.17% (66 matched poses).
- Translation RMSE: 1.6555 m.
- Rotation RMSE: 26.6804°.
- Processed rate: 8.937 FPS.
- The trajectory and availability targets fail. No current visual pose is used by Nav2.

The earlier patrol before the NaN arithmetic fix measured 1.077 m translation RMSE, 21.20° rotation RMSE and 23.78% sampled availability. This earlier data is retained in [`patrol_live_02/measurements.json`](visual_all_03/patrol_live_02/measurements.json); it is not the post-fix result.

### Saved-map restart trials

Evidence: [`restart_trials/trial_01/`](visual_all_03/restart_trials/trial_01/).

- Fresh Gazebo partition and kernel PID; saved patrol map passed to the visual node.
- `visual_status.jsonl`: current-state rows reported `tracking`; the first stored trajectory entry marked frame 1 as `relocalized`. This distinction matters because the status callback observed the frame after the relocalization event.
- `1_gracemo_readiness.log`: AMCL pose/TF checks passed, but Nav2 lifecycle activation was only 2/10; `/navigate_to_pose` was unavailable and coordinator remained in `SYSTEM_HEALTH_CHECK`.
- Launcher readiness deadline expired; no controlled Nav2 route or ground-truth restart evaluation was completed. This is an incomplete/failed route trial.
- Shutdown reaped the owned processes. This result is a failed/incomplete trial, not evidence of successful map relocalization.

The second independent map-loaded run is [`restart_trials/trial_02_loaded_map/`](visual_all_03/restart_trials/trial_02_loaded_map/). It recorded frame-1 `relocalized`, passed all readiness gates including 10/10 Nav2 lifecycle nodes, and completed the `Living Room` interaction route. Its `measurements.json` reports 28 unique status captures, 22 truth-matched poses, 0.7757 m translation RMSE, 3.9758° rotation RMSE, 89.29% tracking availability, and 8.78 FPS. Rotation passes this individual run; translation and availability fail. **Successful independent restart routes: 1/10.** The separate [`trial_02_no_map/`](visual_all_03/restart_trials/trial_02_no_map/) control also passed readiness and interaction; it is not counted as a saved-map trial.

## Test results and failed attempts

| Test | Result |
|---|---|
| Expanded component and ROS suite, including new face/VO/tracker/runner tests | **63 passed** |
| Kernel visual-memory E2E, local isolated kernel access enabled | **3 passed** |
| Rust workspace regression | **18 passed** (5 K runtime + 10 kernel + 1 benchmark + 2 types) |
| `git diff --check` | Passed |
| ALL-03 patrol | Passed 11/11; wrapper follow-up interaction passed after harness repair |
| Visual object goal | Passed live in Gazebo through authorized adapter/Nav2 path |
| Saved-map restart trial 1 | **Failed/incomplete route**: trajectory contains frame-1 relocalized event, but Nav2 readiness stopped at 2/10 lifecycle nodes |
| Saved-map restart trial 2 | **One successful route out of ten**: frame-1 relocalized, 10/10 Nav2 nodes ready, interaction route completed; localization accuracy/availability failed |
| Initial patrol wrapper attempt | Patrol JSON succeeded, then wrapper failed: `ModuleNotFoundError: gracemo_sdk`; fixed and verified in the later run |
| Initial launcher attempt without ROS workspace overlay | Failed before simulator startup because `gracemo_gazebo` package was not discoverable; no Gazebo process started |
| First E2E invocation without SDK path / loopback | Collection or isolated kernel startup failed; rerun with repository SDK path and local loopback passed 3/3 |

Reproduction from repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PWD/vira_interaction:$PWD/ros2_ws/src/gracemo_bridge:$PYTHONPATH"
python3 -m pytest -q tests/test_edge_vision.py tests/test_visual_validation_runner.py tests/test_perception_pipeline.py tests/test_depth_self_filter.py tests/ros2/test_approach_guard.py tests/ros2/test_clearance_oracle.py tests/ros2/test_dynamic_routing_guard.py tests/ros2/test_gazebo_depth_frame.py tests/ros2/test_generation_guard.py tests/ros2/test_routing_geometry.py tests/ros2/test_standalone_readiness.py
VIRA_TEST_PATROL=1 VIRA_DETECTION_HZ=10 VIRA_TEST_DEPTH_10HZ=1 VIRA_VISUAL_ODOMETRY=1 python3 scripts/validation/launcher_recovery.py docs/validation/visual_all_03/patrol_after_fixes --vision
VIRA_DETECTION_HZ=10 VIRA_TEST_DEPTH_10HZ=1 VIRA_VISUAL_ODOMETRY=1 python3 scripts/validation/launcher_recovery.py docs/validation/visual_all_03/visual_goal_live_01 --vision --visual-goal
python3 scripts/validation/evaluate_visual_run.py docs/validation/visual_all_03/patrol_after_fixes
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PYTHONPATH"
python3 -m pytest -q tests/e2e/test_visual_memory.py
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
```

The E2E command launches temporary kernel listeners on `127.0.0.1`; it requires local loopback permissions. The output directories above already contain the corresponding captured evidence.

## Modified files in ALL-03

- `adapters/perception/perception_service.py` — bounded invalid face ROI handling and persistent face result attachment.
- `adapters/perception/visual_odometry.py` — finite-depth-only ORB point projection.
- `adapters/perception/person_tracks.py` — persistent appearance candidate state for a continuing track.
- `scripts/validation/launcher_recovery.py` — reliable repository SDK/adapter import paths for subprocesses.
- `tests/test_edge_vision.py` — regressions for face ROI, face-state persistence, NaN depth and appearance-state persistence.
- `tests/test_visual_validation_runner.py` — validation environment path regression.
- `docs/validation/VIR-VISUAL-ALL-03-FINAL-REPORT.md` — this report.

All earlier uncommitted changes remain in the working tree. No reset, stash, commit or push was performed. All robot movement described here was in private Gazebo simulation partitions. No physical robot or motor output was commanded.

## Remaining blockers and next dependency order

1. Provide/install one supported RGB-D SLAM backend in the active ROS environment; integrate camera calibration, synchronized depth, TF, loop closure, pose graph optimization and map persistence. The existing estimator fails its accuracy and availability targets, so it remains advisory.
2. Investigate the intermittent Nav2 startup failure seen in map-loaded trial 1. Frame-1 relocalization was observed in both map-loaded trials, and trial 2 completed the controlled route. Continue independent restarts until the 10/10 route campaign passes with ground-truth metrics; one failed startup must not be counted as successful.
3. Provide reproducible animated actor geometry and ground-truth trajectories for crossings, occlusion, re-entry, speeds, camera motion and lighting. The earlier limited actor run misses recall and IDF1 thresholds.
4. Provide explicitly consenting enrollment, held-out and unknown face samples before identity verification acceptance. Maintain local-only biometric processing and deletion auditing.
5. Complete MNSE observation IDs/provenance, correction/supersession, expiration/retention and deletion invalidation; demonstrate move, restart, retrieval and delete end-to-end.
6. Finish and test actual PySide6 operator flows for maps, visual health/loss/recovery, consented identities and visual-memory deletion. Offscreen widget checks do not satisfy operator acceptance.
7. Run a sustained full-pipeline benchmark with queue/drop counts, stage latency, CPU/memory and tracking availability; short 8 FPS runs are not acceptance.
8. Run E-stop during visual loss, stopping, relocalization and resumption; verify latch and zero command and require explicit authorized reset. No safety thresholds were altered.
9. After those changes, rerun the complete five-room patrol, idle-zero, object goal and the full ten-trial restart matrix.

**Final status: VIR-VISUAL-ALL-03 is not complete. Release remains HOLD.**
