# VIR-VISUAL-ALL-04 — Visual Intelligence Programme Report

**Date:** 23 November 2026  
**Environment:** ROS 2 Jazzy + Gazebo Harmonic desktop simulation  
**Jetson / physical robot:** Excluded; no physical robot or motor output was used.  
**Final recommendation:** **HOLD — programme is not complete.**

## Executive summary

The repository contains a connected but incomplete visual stack. I added durable, per-observation MNSE records to the existing Rust kernel, carried camera/detection/track/frame metadata across the perception event contract, added K retrieval for visual history, and added authenticated read plus capability-gated deletion. The standalone baseline architecture was retained.

Evidence this turn:

- A fresh live Gazebo RGB-D run processed synchronized camera pairs, detected and localized the monitor, ingested the live observation with mission and trace IDs, and completed the authorized visual-object-to-Nav2 goal. The run captured 56 status samples over 58.6 seconds, median processing 8.25 FPS, median detector time 53.5 ms, 18 localized detection samples, one pending-frame drop, and no rejected input. The output is [`rgbd_memory_live_01`](visual_all_04/rgbd_memory_live_01/).
- The complete offline Rust workspace suite passed: **19 tests**.
- The isolated kernel visual-memory REST E2E passed: **4 tests**, including authenticated reads, deletion authorization, removal from public history, and restart persistence. It required local loopback/file-lock access; the sandbox initially denied those operations. No lock or safety guard was weakened.
- Relevant perception and ROS/Gazebo depth tests passed: **30 tests**.

These results do not accept the full visual programme. The live run was a short integration demonstration, not a sustained full SLAM/perception/control-center benchmark. The VO estimator still fails its required trajectory accuracy, no RGB-D SLAM and loop closure backend is integrated, the independent saved-map route campaign remains **1/10**, and several operator, actor, biometric and E-stop recovery gates remain unpassed.

## Architecture map

| Stage | Existing implementation and data flow | Status / gap |
|---|---|---|
| Gazebo sensors | Gazebo RGB-D camera; bridge topics include `/camera/image_raw`, `/camera/camera_info`, `/camera/depth_image`, `/camera/depth/camera_info`, and `/camera/depth/points`. | Live RGB-D publication observed in this turn. Camera processing rate varied during startup and settled near 8.7 FPS. |
| ROS sensor pipeline and transforms | Perception subscribes to RGB/depth and calibration; approximate timestamp synchronization; optical frame is `camera_optical_link`; world route uses `world`/`map`, robot uses `odom` and `base_footprint`; transforms convert camera XYZ into world/map coordinates. | Live sync skew in this run was 0 ms in all 56 status samples. Full coverage of lighting, occlusion, ranges, and orientations is not established. |
| Perception / tracking | Existing detector, depth estimator, coordinate transformer, person tracker, face path, and visual odometry live in `adapters/perception` and `adapters/vision`. Perception publishes status and emits `ObjectDetected` to the kernel. | Object localization worked live. Person/face/Re-ID metrics do not meet acceptance or lack evaluation inputs. VO is advisory and fails accuracy/availability targets. |
| Kernel / MNSE | Rust kernel ingests the event into the graph and append-only ledger. The graph supplies latest world objects; ledger supplies event history and trace references. New `visual_observations` SQLite archive stores each detection and its provenance, camera stamp, track/detection IDs, mission, trace, status and supersession link. | The live goal confirms the existing perception-to-kernel-to-K route. Archive query/deletion/restart behavior passed an isolated synthetic API E2E; a live Gazebo archive query was not captured during this run. Correction and map-version association remain incomplete. |
| K reasoning | `memory::recall` reads current graph/perception state. `last_observation` now includes the durable observation ID; `visual_history` returns stored prior observations with trace, mission and status. | Component and isolated API checks pass. K query tests against live detections and all requested uncertainty/person/identity queries are incomplete. |
| Adapter and motion path | Registered adapter grants and connected sessions constrain authorized requests; ROS interaction bridge/coordinator routes accepted goals to Nav2; Nav2 sends commands through existing safety layers to Gazebo. | Fresh live visual monitor goal dispatched through this route and reached `MISSION_COMPLETED`. No alternate direct visual movement path was introduced. |
| PySide6 operator surface | Existing control center and ROS manager show runtime camera/robot state; visual panel presents a bounded latest-observation view. | Actual operator walkthrough and requested workflows for consent, memory edit/delete, map persistence, recovery and identities are not accepted. |

## Phase results and acceptance matrix

| Capability | Previous baseline | Work / evidence this turn | Required target | Final status |
|---|---|---|---|---|
| RGB-D synchronization and geometry | Partial | Live camera path with zero reported pair skew; metric monitor localization and authorized goal succeeded. See `visual_status.jsonl`, `visual_goal.json`. | Multiple classes, distances, orientations, occlusions, lighting, freshness/unknown-state validation. | **IMPLEMENTED BUT NOT ACCEPTED** |
| Object detection and 3D localization | Partial | Live monitor detection at `map` coordinates; Nav2 goal completed. 18 status-sample detections; detector median 53.5 ms. | Full multi-condition accuracy/uncertainty acceptance. | **IMPLEMENTED BUT NOT ACCEPTED** |
| Moving-person detection/tracking | Partial | Existing tracker retained. | Precision ≥95%, recall ≥90%, IDF1 ≥80%; multiple independently moving actors. Existing prior actor run was below target (recall .710, IDF1 .226, seven ID switches). | **NOT TESTED** to new acceptance matrix |
| Face detection | Partial | Existing face path retained; relevant component regression tests included. | Multiple pose/range/lighting/occlusion acceptance. | **IMPLEMENTED BUT NOT ACCEPTED** |
| Consent-based face recognition | Partial infrastructure | No enrollment or held-out/unknown consented image set available in this desktop-only task context; no identity claim made. | FAR/FRR and unknown rejection on consented held-out set; local template protection, review, revoke/delete. | **BLOCKED** |
| Person Re-ID | Partial histogram candidate | No independent cross-view genuine/impostor data or supported learned representation validated. | Independent comparisons with precision/recall/false-match results. | **PARTIAL** |
| Visual odometry | Existing implementation | Prior live results remain authoritative: patrol translation RMSE **1.6555 m**, rotation RMSE **26.6804°**, availability **35.68%**; latest map-loaded trial was 0.7757 m, 3.9758°, 89.29%. | ≤0.20 m, ≤5°, ≥95%. | **IMPLEMENTED BUT NOT ACCEPTED** |
| RGB-D SLAM, loop closure, optimization | Missing backend | Environment inspection in ALL-03 found no RTAB-Map, ORB-SLAM3, GTSAM, g2o or pygmo integration; installed `slam_toolbox` is LiDAR SLAM and is not a visual SLAM substitute. No safe compatible backend was available to integrate and verify in this turn. | Live revisit, verified loop closure, optimized trajectory and independent improvement. | **MISSING** |
| Map persistence / relocalization | Partial | Prior map-loaded trials: one readiness failure and one controlled route; both recorded first-frame `relocalized`. | 10/10 independent successful routes, integrity and no previous-pose leakage. | **IMPLEMENTED BUT NOT ACCEPTED** — 1/10 routes |
| MNSE visual memory | Partial latest state | Added append-only per-detection archive, active/latest vs superseded view, retention expiration, provenance, IDs, mission/trace, camera stamp, and deletion tombstones. Unit tests cover supersession and deletion. E2E covers authorized read/delete and restart. | Full live lifecycle including correction, map version, restart, stale handling and deletion audit. | **IMPLEMENTED BUT NOT ACCEPTED** |
| K visual reasoning | Partial | Added durable last observation ID and `visual_history` query. E2E verifies current location persists after restart; archive-specific API tests pass. | Live grounded tests for freshness, change, person/identity uncertainty and evidence attribution. | **IMPLEMENTED BUT NOT ACCEPTED** |
| PySide6 control center | Partial | Existing control center remained in live launcher; prior files retained. | Actual operator walkthrough of all requested visual and destructive/sensitive workflows. | **NOT TESTED** |
| Performance and reliability | Partial | Live 58.6 s camera pipeline: median 8.25 FPS, last 8.72 FPS; median detector 53.5 ms, tracker 0.017 ms; median RSS 378 MB, peak 378.4 MB; median reported CPU 158.65%; 1 pending drop and 0 rejected inputs. | Sustained full pipeline ≥5 FPS with SLAM, latency/drop/queue/resource/stability evidence. | **IMPLEMENTED BUT NOT ACCEPTED** |
| E-stop during visual recovery | Not tested | No E-stop thresholds changed and no recovery bypass introduced. | E-stop through visual loss, stopping, relocalization and resume with zero command/latch evidence. | **NOT TESTED** |
| Full-system integration | Partial | Fresh complete launcher; readiness 11/11, AMCL and 10/10 Nav2 active, live RGB-D object-to-authorized-goal through kernel/adapter/ROS/Nav2/Gazebo completed. | Same-runtime visual SLAM, memory, K, operator visibility and safety recovery evidence. | **IMPLEMENTED BUT NOT ACCEPTED** |
| Five-room patrol / idle-zero baseline | Accepted baseline | This turn used a single live visual goal, not the full patrol. Baseline report records patrol 11/11 in 185.6 s, idle command peaks 0 and displacement 0.007915 m. | Re-run after integration with all capabilities active. | **IMPLEMENTED BUT NOT ACCEPTED** for this programme's new regression run |

## MNSE and K implementation

The new `visual_observations` table records `observation_id` (kernel Event UUID), aggregate `object_id`, class, capture time, image stamp, camera, coordinate frame, XYZ, confidence, detection/track IDs, mission/session ID, trace ID, source/observer, optional map version, status and supersession pointer. The graph updates remain the latest-world-state view; individual events are no longer collapsed into only a latest object record.

New observations supersede the current active record for the associated aggregate object. History retrieval can include active and superseded rows; deleted/expired rows are excluded. Retention defaults to 30 days and accepts `VIRA_VISUAL_MEMORY_RETENTION_DAYS` only within 1–3650 days. Expiration runs when world-object or archive queries occur, not via a background scheduler. Map version is currently null because no map-version field is emitted by the accepted perception pipeline. Correction/replacement APIs are not implemented yet.

The read endpoint requires an authenticated connected grant with `ReadVisualMemory`. Deletion requires `DeleteVisualMemory`, a live session, and a non-empty reason; it tombstones the private row, hides it from public history/context/K retrieval, and removes the active graph object when no active observation remains. The test checks that this behavior survives restart. The underlying append-only ledger remains as internal audit history; deleted visual event payloads therefore remain in that audit store and are filtered from public retrieval views. This distinction should be revisited against the product's final privacy/deletion policy before release.

## Quantitative live evidence

Live evidence directory: [`docs/validation/visual_all_04/rgbd_memory_live_01`](visual_all_04/rgbd_memory_live_01/). The compact [`metrics summary`](visual_all_04/rgbd_memory_live_01/metrics_summary.json) and [`visual goal result`](visual_all_04/rgbd_memory_live_01/visual_goal.json) are suitable for Git review; the high-volume `visual_status.jsonl` remains local and is excluded by the repository artifact policy.

| Measurement | Result |
|---|---:|
| Launcher readiness | 11/11 stages; Nav2 lifecycle 10/10 |
| RGB-D status samples | 56 |
| Observation window | 58.62 s |
| Synchronized depth/RGB pairs at last sample | 554 |
| Pair skew | 0 ms median and max reported |
| Localized detections in status samples | 18 across 16 samples |
| RGB-D processing rate | 8.25 FPS median; 8.72 FPS last sample |
| Detector latency | 53.54 ms median; 72.53 ms max |
| Tracker latency | 0.0169 ms median; 0.0286 ms max |
| Peak perception RSS | 378,359,808 bytes |
| CPU reported by perception process | 158.65% median after startup; 165.5% max |
| Pending-pair drops / rejected input | 1 / 0 |
| Visual goal | Authorized dispatch; monitor observation pose reached; `MISSION_COMPLETED` |
| Mission / trace association | Present on observations and navigation evidence |

This is a short live visual-goal demonstration; it does not include a SLAM backend or constitute the required sustained full-pipeline benchmark. The system's mission completion is the actual coordinator outcome, not an inferred position-only result.

## Test results and exact reproduction commands

| Test | Result |
|---|---|
| Rust kernel workspace: `cargo test --workspace --offline --manifest-path kernel/Cargo.toml` | **19 passed** (5 K, 11 kernel, 1 benchmark, 2 types) |
| Perception / pipeline / Gazebo depth test subset | **30 passed** |
| `tests/test_edge_vision.py` | **17 passed** |
| `tests/e2e/test_visual_memory.py` with loopback/file-lock access | **4 passed** |
| Live authorized RGB-D visual goal | **Passed**, with evidence above |
| Full restart, SLAM loop closure, person, biometric, GUI walkthrough, E-stop recovery | Not accepted / blocked as listed |

From the repository root:

```bash
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PWD/vira_interaction:$PWD/ros2_ws/src/gracemo_bridge:$PYTHONPATH"
python3 -m pytest -q tests/test_edge_vision.py tests/test_perception_pipeline.py tests/ros2/test_gazebo_depth_frame.py
PYTHONPATH="$PWD/adapters/sdk:$PYTHONPATH" python3 -m pytest -q tests/e2e/test_visual_memory.py
VIRA_DETECTION_HZ=10 VIRA_TEST_DEPTH_10HZ=1 python3 scripts/validation/launcher_recovery.py docs/validation/visual_all_04/rgbd_memory_live_01 --vision --visual-goal
```

The isolated-kernel E2E opens loopback HTTP and takes an OS file lock. A restricted sandbox invocation failed with `Operation not permitted`; the same tests passed when run with authorized local-process access. The Gazebo launcher run used a private simulation partition and exited with recorded process cleanup. No physical target was connected.

## Files changed for this programme turn

- `kernel/gracemo-types/src/lib.rs` — backward-compatible optional observation metadata on `ObjectDetected`.
- `adapters/perception/models.py` — include detection, track, camera and image-stamp fields in the canonical event.
- `kernel/gracemo-kernel/src/graph.rs` — observation archive, supersession, retention, deletion, hidden-view checks, and lifecycle unit test.
- `kernel/gracemo-kernel/src/authorization.rs` — require a live authenticated session and scoped capability for visual-memory access.
- `kernel/gracemo-kernel/src/api.rs` — authenticated archive retrieval and scoped deletion endpoint; filter hidden rows from history.
- `kernel/gracemo-kernel/src/context_compiler.rs` — filter deleted/expired observations from compiled context.
- `kernel/gracemo-kernel/src/k_execution.rs` — durable observation identity and visual-history query.
- `tests/e2e/test_visual_memory.py` — authorization, deletion, audit-view hiding and restart persistence E2E.
- `docs/validation/VIR-VISUAL-ALL-04-FINAL-REPORT.md` — this report.

## Remaining blockers and dependency order

1. Select and integrate an actual RGB-D SLAM backend compatible with ROS 2 Jazzy, then validate mapping, loop closure, pose-graph optimization, map integrity and reload. Until then AMCL remains the primary Nav2 localization source.
2. Repair VO drift/availability and show independent RMSE ≤0.20 m, ≤5°, and ≥95% availability before using visual localization for navigation.
3. Add observation correction and explicit map-version semantics; capture the live archive query/delete lifecycle with live RGB-D data. Decide whether immutable ledger audit payload retention satisfies the intended deletion policy.
4. Provide animated independent Gazebo actor paths and achieve person tracking precision/recall/IDF1 thresholds. Provide consenting enrollment, held-out and unknown biometric data for FAR/FRR tests; do not invent these results.
5. Implement and operator-test control-center workflows for map, visual health/recovery, consent, visual history and deletion.
6. Run a sustained ≥5 FPS integrated SLAM pipeline benchmark with queue, drops, per-stage latency, CPU, memory and runtime stability.
7. Complete the independent map-loaded route campaign at 10/10; prior record is 1/10, with one Nav2 readiness failure.
8. Test E-stop through visual loss, stopping, relocalization and recovery. Demonstrate latch, zero-command, authorization and no stale command replay.
9. After integration, rerun the five-room 11/11 patrol and idle-zero gates with the full visual runtime.

No existing uncommitted changes were reset, stashed, discarded, committed or pushed. Existing Nav2 safety settings, footprint, collision monitor, command mux and E-stop protections were not weakened. **VIR-VISUAL-ALL-04 remains incomplete; release status is HOLD.**
