# VIRA Visual Intelligence Recovery and Acceptance Report

**Mission:** VIR-VISUAL-ALL-01  
**Environment:** ROS 2 Jazzy, Gazebo Harmonic, desktop CPU  
**Release decision:** **HOLD**  
**Robot movement:** Gazebo only

## Executive summary

This sprint repaired a saved-map localization flaw, added fail-closed map validation, exercised two animated Gazebo people, verified visual-to-K-to-authorized-Nav2 navigation, and completed a five-room patrol with the perception pipeline enabled. The patrol completed 11/11 waypoints in 217.5 seconds with matching coordinator and kernel mission IDs. The idle-zero observation passed.

Visual localization remains unfit to control Nav2. On the 11-waypoint route, it produced 1.082 m translation RMSE, 9.90° rotation RMSE, and 32.9% sampled pose availability. After a saved-map restart, it reported a successful keyframe relocalization and stayed available throughout the short room run, while translation RMSE remained 0.909 m. AMCL remains the Nav2 localization source.

The temporary 10 Hz RGB-D simulation profile raised integrated processing to 9.31 FPS during the patrol, above the 5 FPS desktop target. The profile used a temporary copy of the Gazebo world and did not change the accepted robot footprint, costmaps, planner, controller, collision monitor, command mux, or E-stop setup.

## Capability status

Statuses use the assignment's release vocabulary.

| Capability | Status | Evidence and remaining gap |
|---|---|---|
| RGB-D acquisition and synchronization | IMPLEMENTED AND ACCEPTED for tested desktop configuration | The patrol and reload runs reported zero RGB/depth timestamp skew. A 48-sample static-depth reference run had 2.18 mm median absolute error. This does not validate every distance, camera motion, occlusion, or TF failure case. |
| Object detection and metric observations | IMPLEMENTED AND ACCEPTED for functional Gazebo scenarios | Live perception identified a monitor, K retrieved its map location, the registered adapter authorized a Nav2 observation goal, and the robot reached that goal. Mission trace `5c816d14-9551-4034-a0c2-61dba0ec69aa`. Broad object classes, lighting, and range acceptance remain limited to the saved tests. |
| Moving-person detection and tracking | IMPLEMENTED BUT NOT ACCEPTED | Two animated actors crossed the camera view. Independent colored-garment labels across 21 sampled frames yielded 1.00 precision, 0.710 recall, 0.226 IDF1, and seven ID switches including re-entry. Three-or-more actor, sustained occlusion, variable lighting, and camera-motion acceptance remain open. |
| Face detection | IMPLEMENTED BUT NOT ACCEPTED | YuNet was repaired for the installed OpenCV 4.6 runtime. It detected one face in public reference-image replay at 0.927 confidence and 25.8 ms. In the moving-actor run, none of 22 detected person observations contained a detected face. A live multi-distance and orientation matrix remains untested. |
| Consent-based face recognition | PARTIAL | SFace model provisioning, local embeddings, consent-required enrollment, encrypted local templates, deletion, and an optional owner-only external key are implemented. Unit coverage passes. No consenting enrollment set, held-out identities, unknown identities, or ambiguous identity acceptance set was supplied or tested. No biometric images or embeddings were sent externally. |
| Person Re-ID | IMPLEMENTED BUT NOT ACCEPTED | Local color-histogram candidates were evaluated on the two moving actors. At threshold 0.94, 38/103 genuine comparisons exceeded threshold and 4/97 impostor comparisons also exceeded it. This does not meet cross-camera Re-ID acceptance. |
| Visual odometry and localization | IMPLEMENTED BUT NOT ACCEPTED | Dense RGB-D ICP, depth-consistency checks, moving-person masks, explicit loss states, keyframe search, and timestamps are present. The full patrol exceeded both localization error targets and fell below the availability target. No visual pose feeds Nav2. |
| Map save, integrity, and relocalization | IMPLEMENTED BUT NOT ACCEPTED | Version 3 maps carry calibration/sensor-geometry fingerprints and SHA-256 integrity checks. Corrupt, incompatible, or unmatched maps return `map_unavailable` or `lost` without a current pose. The saved-map restart recorded a relocalized keyframe in its trajectory and continued tracking. It was one Gazebo restart, not the required 10/10 campaign. |
| MNSE and K visual memory | PARTIAL | K now retrieves the latest persisted object observation with timestamp, position, confidence, frame, age, and freshness fields. Three kernel integration tests passed, including persistence over restart. Visual observation correction, retention policy, durable per-observation IDs, and deletion propagation through MNSE remain incomplete. |
| PySide6 visual monitoring | IMPLEMENTED BUT NOT ACCEPTED | The supported control center read current kernel perception and rendered 197 trajectory items in the live visual tab. RGB and depth previews use the existing sensor view. Full operator workflow evaluation for face status, identity uncertainty, loss/recovery, and memory deletion remains untested. |
| Desktop edge throughput | IMPLEMENTED AND ACCEPTED for the tested simulation profile | The 10 Hz temporary camera profile produced 9.31 processed FPS on the patrol. Peak perception RSS was 565 MB. Peak sampled launcher-tree PSS was 2.71 GB including Gazebo. This is desktop CPU evidence and does not demonstrate Jetson performance. |
| Jetson Orin Nano deployment | NOT TESTED | CPU, constrained, and TensorRT profiles are available. TensorRT cache paths are keyed by runtime version, model digest, and precision. No Jetson hardware, thermal, power, FP16, or INT8 acceptance run was available. |
| Five-room patrol and idle-zero regression | IMPLEMENTED AND ACCEPTED for this Gazebo run | The standalone adapter → kernel → ROS 2 → Nav2 → Gazebo mission completed all 11 waypoints. Idle-zero passed with all measured commands at 0.0 and 0.0081 m odometry displacement. |
| Full visual-autonomy campaign | PARTIAL | The patrol, visual goal, actor crossing, saved-map restart, memory queries, and live control-center telemetry ran. Consented identity, full Re-ID, deletion workflow, 10 restart trials, repeated routes, and Jetson hardware gates remain open. |

## Localization diagnosis and repair

The estimator combines ORB features and PnP with aligned RGB-D ICP at the depth-camera resolution. It uses current depth measurements to reject inconsistent feature matches. The evaluator aligns only the first timestamp-matched pose to Gazebo camera truth. Ground truth never enters the estimator.

The saved-map loader previously set the last stored keyframe as the robot's current pose. Dense frame-to-frame tracking could then continue from that assumed pose before the restarted camera had matched any saved keyframe. This made map loading look successful without proving relocalization. The loader now treats saved keyframes only as search references. It publishes a current pose after a valid keyframe match. An unmatched map stays `lost`. Stale or non-monotonic timestamps do not produce current poses.

The route-wide error remains high after that repair. The new map-reload run confirms the camera can match a saved keyframe, but the 0.909 m translation RMSE and 5.36° rotation RMSE remain above the 0.20 m and 5° targets. The patrol run's 1.082 m and 9.90° errors also fail. Depth residual consistency alone does not prevent accumulated drift. The implementation has no trajectory-wide loop closure or independent multi-route accuracy acceptance.

| Run | Localization availability | Translation RMSE | Rotation RMSE | Truth-matched poses | Processing |
|---|---:|---:|---:|---:|---:|
| Five-room patrol, saved map built during run | 32.9% of 228 unique status captures | 1.082 m | 9.90° | 71 | 9.315 FPS |
| Saved-map restart, short room route | 100% of 27 unique status captures | 0.909 m | 5.36° | 24/27 | 8.498 FPS |

The reload status began at simulation image stamp 0.8 s. Its first stored trajectory entry was marked `relocalized`. The first 50 recorded status messages all carried a current pose. This is one restart result, not evidence for the 10/10 gate. The first matched camera pose is alignment origin for RMSE, so the table measures subsequent drift rather than absolute map-frame calibration error.

## Moving actors, faces, and Re-ID

Gazebo actors used two distinct sweater colors. The evaluator matched visible garment pixels to detector boxes with one-to-one assignment and at least 60% garment-pixel containment. It sampled 21 synchronized images from a 22-second crossing and re-entry sequence. Of 31 visible actor instances, 22 were matched to detections. There were 22 person predictions and seven track-ID switches under the evaluator's actor-to-track association. The sequence is useful functional evidence, but it covers only two actors in one camera scene.

Appearance descriptors are two-part HSV histograms. The 103 genuine and 97 impostor comparisons were derived from the labeled moving-actor observations. At the existing 0.94 similarity threshold, the descriptor missed most genuine pairs and still passed four impostor pairs. Keep Re-ID unaccepted.

YuNet 2023mar is 232,589 bytes with SHA-256 `8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4`. Its model license is MIT. SFace 2021dec is 38,696,353 bytes with SHA-256 `0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79`. Its model license is Apache 2.0. The provisioning script verifies size and digest before use. [YuNet model license](https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/LICENSE) · [SFace model license](https://github.com/opencv/opencv_zoo/blob/main/models/face_recognition_sface/LICENSE)

The gallery defaults to a key in its private owner-only directory. `VIRA_FACE_KEY_FILE` enables an owner-only key stored outside the gallery directory. This reduces the risk of copying the encrypted gallery and key together. It does not protect a running user's unlocked process or replace an operator backup and retention policy.

## Integration, safety, and memory

The five-room patrol used the existing standalone mission dispatch and safety chain. Coordinator and kernel reported the same mission ID `standalone-4cee4a71-3b43-4fe7-80ba-a4f1ef3b963c` and trace ID `a140b23f-19ba-4c07-93b8-16ef3e1105ff`. It completed Kitchen & Dining, Master Bedroom, Home Study, Living Room, and Central Hallway. The visual-target mission used the existing authorized navigation adapter and reached monitor observation waypoint `monitor_001` under trace `5c816d14-9551-4034-a0c2-61dba0ec69aa`.

No Nav2 costmap, footprint, inflation, controller, collision-monitor, terrain E-stop, or command-multiplexer parameters were changed by this sprint. The 10 Hz camera trial wrote a temporary world copy in the private launcher directory. The source Gazebo world was not altered for that test. The separate E-stop during visual recovery matrix was not rerun in this sprint. Earlier E-stop evidence remains in the existing validation report.

K's `memory::recall[query:"last_observation",entity:"monitor"]` returns the most recent graph observation and an explicit `fresh_within_3s` value. The visual memory suite passed current/stale checks, comparable-view checks, and a restart retrieval check. Deletion of persistent graph observations and retention enforcement are not implemented as a complete workflow.

The live control-center check read the current `/perception` response and confirmed the configured camera frame `camera_optical_link`, 491 processed frames, 12 map keyframes, and 197 rendered trajectory items. It is an offscreen integration check, not an operator acceptance test.

## Test results and evidence

**Component and ROS tests:** 58 passed.  
**Kernel visual-memory integration tests:** 3 passed after enabling the local port access needed by the isolated test kernels.  
**Rust workspace tests:** 18 passed.  
**Gazebo patrol:** 11/11 waypoints completed.  
**Idle-zero:** passed, with 0.0 command peaks and 0.0081 m displacement.  
**Visual navigation:** reached the monitor observation pose using authorized adapter dispatch.  
**Hardware tests:** none.

Reproduction commands from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PWD/vira_interaction:$PWD/ros2_ws/src/gracemo_bridge:$PYTHONPATH"
python3 -m pytest -q tests/test_edge_vision.py tests/test_perception_pipeline.py tests/test_depth_self_filter.py tests/ros2/test_approach_guard.py tests/ros2/test_clearance_oracle.py tests/ros2/test_dynamic_routing_guard.py tests/ros2/test_gazebo_depth_frame.py tests/ros2/test_generation_guard.py tests/ros2/test_routing_geometry.py tests/ros2/test_standalone_readiness.py
python3 -m pytest -q tests/e2e/test_visual_memory.py
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
VIRA_TEST_PATROL=1 VIRA_DETECTION_HZ=10 VIRA_TEST_DEPTH_10HZ=1 VIRA_VISUAL_ODOMETRY=1 python3 scripts/validation/launcher_recovery.py docs/validation/visual_all_01/patrol_isolated --vision
VIRA_DETECTION_HZ=10 VIRA_TEST_DEPTH_10HZ=1 VIRA_VISUAL_ODOMETRY=1 VIRA_VISUAL_MAP_INPUT="$PWD/docs/validation/visual_all_01/patrol_isolated/visual_map.npz" python3 scripts/validation/launcher_recovery.py docs/validation/visual_all_01/reload_monitored --vision
```

Gazebo movement stayed inside isolated partitions. The actor scene was removed by the validation script. No physical robot, motor driver, Jetson, or biometric enrollment set was used. No commit or push was made.

| Evidence | Contents |
|---|---|
| [`patrol_isolated/patrol.json`](docs/validation/visual_all_01/patrol_isolated/patrol.json) | 11/11 patrol result, mission ID, trace ID, kernel and coordinator outcome |
| [`patrol_isolated/measurements.json`](docs/validation/visual_all_01/patrol_isolated/measurements.json) | Ground-truth pose errors, availability, FPS, memory, detections |
| [`patrol_isolated/idle_zero.json`](docs/validation/visual_all_01/patrol_isolated/idle_zero.json) | Command-chain idle-zero measurement |
| [`reload_monitored/visual_status.jsonl`](docs/validation/visual_all_01/reload_monitored/visual_status.jsonl) | Startup relocalization trajectory and current-pose states, no camera images |
| [`reload_monitored/measurements.json`](docs/validation/visual_all_01/reload_monitored/measurements.json) | Saved-map route accuracy and runtime measurements |
| [`actors_ready/actors.json`](docs/validation/visual_all_01/actors_ready/actors.json) | First crossing trial insertion, removal, and track samples |
| [`rate_trial/actors.json`](docs/validation/visual_all_01/rate_trial/actors.json) | Color-labeled actor samples |
| [`rate_trial/tracking.json`](docs/validation/visual_all_01/rate_trial/tracking.json) | Tracking and histogram comparison metrics |
| [`goal/visual_goal.json`](docs/validation/visual_all_01/goal/visual_goal.json) | Authorized object-location navigation and outcome |
| [`reload_monitored/desktop_visual.json`](docs/validation/visual_all_01/reload_monitored/desktop_visual.json) | Live PySide6 visual-tab check |
| [`kernel_integration.log`](docs/validation/visual_all_01/kernel_integration.log) | Kernel/K memory integration test output |
| [`final_regression.log`](docs/validation/visual_all_01/final_regression.log) | 58 component and ROS tests |
| [`rust_final.log`](docs/validation/visual_all_01/rust_final.log) | 18 Rust workspace tests |
| [`evr02/face_replay.json`](docs/validation/evr02/face_replay.json) | YuNet public-reference replay measurements |

## Changes made

The direct implementation changes for this sprint cover:

- `adapters/perception/visual_odometry.py`: depth-consistent RGB-D motion, person-region masking, monotonic capture-time checks, safe map reload, calibration fingerprints, version 3 map digest, explicit lost/relocalized states, and bounded keyframes.
- `adapters/perception/perception_service.py` and `adapters/perception/person_tracks.py`: persistent face-detection status per live track, stale-frame rejection, person-track metrics, and non-monotonic frame protection.
- `adapters/vision/gracemo_vision/yunet.py`, `face_id.py`, `face_gallery.py`, `scripts/validation/provision_face_models.py`, and `adapters/perception/detector.py`: OpenCV 4.6 YuNet ONNX fallback, pinned model checksums, optional external gallery key, and TensorRT cache configuration.
- `ros2_ws/src/gracemo_bridge/gracemo_bridge/sensor_bridge.py` and `ros2_ws/src/gracemo_gazebo/launch/sim.launch.py`: capture-time sensor-rate gating and a private, temporary 10 Hz depth-camera test mode.
- `kernel/gracemo-kernel/src/k_execution.rs`: grounded latest-observation K retrieval with an explicit age and freshness result.
- `gracemo_control_center/modules/cognitive/visual_panel.py` and `cognitive_view.py`: live trajectory rendering in the existing visual-perception tab.
- `scripts/validation/visual_actors.py`, `evaluate_visual_actors.py`, `record_visual_status.py`, `evaluate_visual_run.py`, `visual_view.py`, and `launcher_recovery.py`: Gazebo actors, independent color-mask scoring, startup status capture, trajectory evaluation, and integration evidence.
- `tests/test_edge_vision.py` and `tests/e2e/test_visual_memory.py`: regressions for corrupted or mismatched maps, stale inputs, consent storage, unmatched-map safety, and K memory after restart.

The working tree contains many existing uncommitted project changes from earlier work. They were preserved. No unrelated file was reset, stashed, or overwritten. All changes remain uncommitted.

## Remaining release blockers

1. Visual odometry misses the 0.20 m translation target by 0.709 m on the reload run and has no loop closure.
2. Full-route tracking availability was 32.9%, below the 95% requirement.
3. Moving-person recall and IDF1 were 0.710 and 0.226, with seven ID switches in this limited scene.
4. Face recognition lacks a consenting held-out test set. Actual moving-actor faces were not detected in the sampled sequence.
5. Person Re-ID's current appearance descriptor missed most genuine pairs and accepted impostor pairs.
6. Visual-memory correction, retention, and deletion propagation remain incomplete.
7. The control center has offscreen integration evidence but no operator workflow acceptance.
8. Ten independent map-restart trials, the full scenario matrix, and Jetson tests have not been run.
9. E-stop during visual recovery was not repeated as part of this sprint. Existing E-stop baseline evidence remains separate.

The accepted AMCL/Nav2 navigation path remains operational in Gazebo. Visual odometry remains advisory. The project release stays **HOLD** until the localization, tracking, consented identity, memory lifecycle, and hardware gates receive reproducible passing evidence.
