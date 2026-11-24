# VIRA Full Live Visual Intelligence — VIR-VISUAL-ALL-02

**Date:** 23 November 2026  
**Runtime:** ROS 2 Jazzy, Gazebo Harmonic, desktop CPU  
**Physical robot / motor output:** Not used  
**Jetson hardware:** Explicitly out of scope  
**Release decision:** **HOLD**

## Executive summary

The previous VIR-VISUAL-ALL-01 work remains the baseline. This continuation fixed a face-stage failure mode: an invalid or clipped person region could previously skip sample creation and then be indexed, raising an exception in the synchronized RGB-D frame worker. The face stage now reports `FACE_ROI_INVALID` for that track, keeps the rest of the frame pipeline alive, and continues to attach a valid prior face result to a continuing track. Two regressions cover these behaviors.

The requested full live visual-intelligence milestone is **not complete**. The most important blockers remain visual localization accuracy, loop closure, moving-person association, consented held-out face evaluation, cross-camera Re-ID, durable MNSE observation lifecycle, operator workflow acceptance, and the dedicated E-stop/restart campaigns. The system must continue to use AMCL as Nav2's primary localization source. The earlier 11/11 patrol is a real Gazebo regression result, not evidence that visual localization is acceptable.

## Initial baseline and current status

| Capability | VIR-VISUAL-ALL-01 evidence | VIR-VISUAL-ALL-02 status |
|---|---|---|
| RGB-D acquisition | 2.18 mm median error in a 48-sample static-depth check; synchronized patrol/reload traces | Prior desktop functional evidence retained. No new range/motion/occlusion matrix run. |
| Object detection and metric location | Monitor detected, K retrieved its location, authorized Nav2 observation goal reached | Prior live Gazebo integration retained; broader class/range acceptance remains open. |
| Person detection and tracking | Precision 1.00, recall 0.710, IDF1 0.226, seven ID switches in limited two-actor scene | **Not accepted**; no tracker replacement or acceptance campaign this sprint. |
| Face detection | Public reference replay detected one face; moving-actor run detected no faces in 22 person observations | Invalid-ROI frame failure repaired and regression-tested. Live distance/orientation/lighting/occlusion acceptance remains open. |
| Face recognition | Consent-gated local gallery implementation; no held-out identities | **Not accepted**; no consenting enrollment and held-out/unknown evaluation set was available. |
| Person Re-ID | At threshold 0.94, 38/103 genuine comparisons accepted and 4/97 impostor comparisons accepted | **Not accepted**; no cross-view test set or replacement embedding model. |
| Visual odometry/localization | Patrol: 1.082 m translation RMSE, 9.90° rotation RMSE, 32.9% sampled availability. Reload: 0.909 m, 5.36° | **Not accepted**; no new estimator or accuracy campaign. |
| Loop closure / trajectory optimization | No trajectory-wide optimizer or verified loop closure | **Not implemented/accepted.** |
| Map persistence/relocalization | Version 3 integrity checks; one successful keyframe-match restart | **Partial**; no ten independent restart trials. |
| MNSE visual memory | Latest-object state and K latest-observation query; restart retrieval tested | **Partial**; append-only per-observation provenance, correction/supersession, retention and deletion propagation are not complete. |
| K visual reasoning | Freshness-aware object/person/current-view queries and historical object lookup | **Partial**; not all requested grounded queries or lifecycle semantics are implemented. |
| PySide6 monitoring | Live kernel telemetry and 197 trajectory items in offscreen integration check | **Partial**; not a completed operator workflow, with no map, consent, memory deletion, or subsystem controls acceptance. |
| Desktop throughput | 9.31 processed FPS on temporary 10 Hz profile; peak perception RSS 565 MB | Previous result retained; no sustained full-pipeline benchmark with per-stage latency/queue/drop measurements this sprint. |
| Nav2 patrol / idle-zero | 11/11 waypoints; idle-zero commands 0.0 and 0.0081 m displacement | Prior Gazebo regression retained; no new patrol in this continuation. |
| E-stop during visual recovery | Not rerun in VIR-VISUAL-ALL-01 | **Not run in VIR-VISUAL-ALL-02.** |
| Jetson hardware | Not tested | Explicitly excluded. |

## Root cause and repair in this continuation

In `PerceptionService.process_frame`, face sampling skipped an empty person crop with `continue`, but then unconditionally read `self.face_samples[track_id]`. A clipped or malformed box therefore had no stored sample and could raise `KeyError`, failing the worker's whole RGB-D frame. The repair is isolated in `_update_faces`: crop bounds are clipped to the image, invalid regions yield a timestamped `FACE_ROI_INVALID` status with unknown face count, and the per-frame face result remains attached to active tracks. The change does not infer identity, relax consent, alter tracking, or touch the navigation safety chain.

This is a bounded robustness repair, not completion of face recognition or of the visual-intelligence mission.

## SLAM and localization findings

The earlier saved-map bug fix remains in place: a saved keyframe is a search reference, not an assumed current pose; an unmatched map does not publish a fabricated pose. Earlier tests also cover map digest corruption, sensor-geometry mismatch, stale timestamps and moving-person masking. Those repairs prevent false-positive localization, but the recorded trajectory error still exceeds the target by a wide margin.

The desktop ROS installation includes `slam_toolbox`, which is a LiDAR localization/mapping package. It does not satisfy this assignment's RGB-D SLAM and visual loop-closure requirement. RTAB-Map and ORB-SLAM executables/packages were not present in the available runtime; Python checks also found no GTSAM, g2o, or pygmo backend. No new pose-graph optimizer was implemented, and no loop closure is claimed. Introducing an uninstalled backend or labeling the existing bounded RGB-D odometry as SLAM would not constitute live acceptance.

**Nav2 remains on AMCL.** The measured VO errors and availability are not safe grounds to switch the accepted localization source.

## Integration and test results

| Check | Result | Evidence / limits |
|---|---:|---|
| Face invalid-ROI regression | Pass | In `tests/test_edge_vision.py`; verifies bounded state instead of worker exception. |
| Face result persistence regression | Pass | Verifies `FACE_DETECTED` remains attached on a following frame for the same active track. |
| Expanded component and ROS suite | **60 passed** | Exact command below. These are component/ROS tests, not full visual acceptance. |
| Kernel visual-memory E2E | **3 passed** | Required local loopback access to start temporary isolated kernel processes. |
| Rust workspace | **18 passed** | `cargo test --workspace --offline --manifest-path kernel/Cargo.toml`. |
| `git diff --check` | Pass | Run after changes. |
| Gazebo full mission / visual recovery E-stop | Not rerun | Prior 11/11 patrol remains the regression evidence; the E-stop recovery matrix is an outstanding gate. |

Reproduction commands from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PWD/vira_interaction:$PWD/ros2_ws/src/gracemo_bridge:$PYTHONPATH"
python3 -m pytest -q tests/test_edge_vision.py tests/test_perception_pipeline.py tests/test_depth_self_filter.py tests/ros2/test_approach_guard.py tests/ros2/test_clearance_oracle.py tests/ros2/test_dynamic_routing_guard.py tests/ros2/test_gazebo_depth_frame.py tests/ros2/test_generation_guard.py tests/ros2/test_routing_geometry.py tests/ros2/test_standalone_readiness.py
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PYTHONPATH"
python3 -m pytest -q tests/e2e/test_visual_memory.py
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
```

The E2E test starts temporary isolated kernel processes and requires local loopback bind/connect access. Without that access, it fails before exercising K. The passing result above used that local test allowance.

## Changes in this continuation

- `adapters/perception/perception_service.py` — bounds face ROI handling and emits explicit invalid-ROI status without aborting a frame.
- `tests/test_edge_vision.py` — adds invalid-ROI and active-track result-persistence regressions.
- `docs/validation/VIR-VISUAL-ALL-02-FINAL-REPORT.md` — this milestone's capability assessment and evidence summary.

Earlier implementation changes and evidence remain documented in [VIR-VISUAL-ALL-01](../../VIRA_COMPLETE_VISUAL_INTELLIGENCE_ACCEPTANCE_REPORT.md) and under [`docs/validation/visual_all_01/`](visual_all_01/). Key files include `patrol_isolated/measurements.json`, `patrol_isolated/patrol.json`, `rate_trial/actors.json`, `rate_trial/tracking.json`, `reload_monitored/measurements.json`, `reload_monitored/visual_status.jsonl`, `goal/visual_goal.json`, `kernel_integration.log`, `final_regression.log`, and `rust_final.log`.

## Required remaining work and release gates

1. Select and install one supported live RGB-D SLAM backend, integrate it with the actual Gazebo camera and TF graph, then demonstrate loop closure and trajectory correction against independent ground truth. Do not change Nav2 from AMCL until the stated accuracy and availability thresholds pass.
2. Repair and evaluate person tracking on actual Gazebo actor scenarios with ground-truth IDs, crossings, occlusion, re-entry, camera motion and multiple actors. Required precision, recall and IDF1 thresholds are unmet.
3. Run live face detection across range/view/light/occlusion conditions. Face recognition acceptance additionally requires explicit consent, separated enrollment/held-out/unknown samples, and audited deletion. No identity evaluation is claimed without those samples.
4. Replace or validate Re-ID with independent cross-camera genuine/impostor data; current histogram evidence fails the assignment's intent.
5. Implement durable observation IDs and provenance in MNSE, correction/supersession, expiry/retention and deletion invalidation; complete a move-object/restart/delete live demonstration. Current K and world-object state do not implement that lifecycle.
6. Finish the existing PySide6 operator workflow for map save/load, tracking/loss, identity uncertainty, memory and authorized deletion, then test it with the live application and kernel.
7. Run a sustained full-pipeline benchmark with FPS, each stage's latency, CPU/RAM, dropped frames, queue depth and availability.
8. Run ten independent saved-map restart trials and controlled relocalization trials; each must have separate run IDs and independent ground-truth comparisons.
9. Run the requested Gazebo E-stop-during-visual-recovery sequence and verify latch, zero command, displacement and authorized reset/resumption behavior. Do not bypass the safety gate.
10. Re-run the five-room 11/11 patrol and idle-zero after the integrated visual changes.

No physical robot, motor output, Jetson device or unconsented biometric samples were used. Existing working-tree modifications were preserved; no changes were committed or pushed. **VIR-VISUAL-ALL-02 remains HOLD.**
