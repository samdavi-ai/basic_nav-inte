# VIRA visual intelligence acceptance

**Release decision — HOLD for full edge autonomy.** On desktop Gazebo, VIRA found a monitor from live RGB-D input, retrieved its position through K and MNSE, dispatched an authorized navigation action, and reached the requested observation pose. Visual odometry lost tracking during the same type of room transition. Nav2 continues to use AMCL. The operator selected desktop Gazebo only for this sprint. [Navigation action and terminal event](visual_goal/visual_goal.json) · [Localization measurements](final_profile/measurements.json)

## Implementation and acceptance matrix

| Capability | Status | Evidence and limits |
|---|---|---|
| Calibrated RGB-D and metric depth | IMPLEMENTED AND ACCEPTED for the planar fixture | 43 timestamp-matched samples. Median absolute optical-depth error 2.65 mm, maximum 3.52 mm against a 50 mm test tolerance. Projected person detections on the panel had at most 23 mm map-x error. These measurements cover one plane and range. [Reference evidence](reference_depth/reference.json) · [Summary](summary.json) |
| Object detection | IMPLEMENTED AND ACCEPTED for functional simulation | One local YOLO11n session provided monitor, sofa and chair detections across runs. Current view can also legitimately be empty. Labeled scene-wide accuracy and the throughput target remain open. [Live measurements](gazebo/measurements.json) |
| Person detection and tracking | PARTIAL | Two people depicted on a stationary panel received distinct IDs in 11 of 13 sampled camera frames. An independent image replay preserved four IDs after input order reversal and a short gap. Moving actors, crossings and sustained 15–30 Hz tracking remain unverified. [Panel](reference_depth/reference.json) · [Replay](appearance.json) |
| Face detection | IMPLEMENTED BUT NOT ACCEPTED for integrated people scenes | Local OpenCV cascade detected two faces in an image replay in 46.9 ms. Both were unknown. The fixture did not establish reliable face acquisition from robot-distance crops. [Replay](face_fallback.json) |
| Consent-based recognition | PARTIAL | Explicit enrollment, encrypted templates, deletion, ambiguous and unknown outcomes have component coverage. YuNet and SFace provisioning failed certificate verification. No consented enrollment or held-out identity set was supplied. Local gallery encryption uses a key in the same owner-only directory and does not protect against compromise of that account. [Component results](verified_tests.log) |
| Person Re-ID | PARTIAL | Event-triggered normalized color histograms reuse detector crops. On photometric and crop variations from one photograph, 10 true matches, 2 misses and 0 false matches across 36 negative comparisons gave 83.3% recall and 100% precision. This measures within-image consistency. Real cross-camera accuracy, clothing changes and long occlusion remain open. [Evaluation](appearance.json) |
| Visual odometry and mapping | IMPLEMENTED BUT NOT ACCEPTED for navigation | Bounded ORB keyframes, metric depth checks, pose estimation, tracking-loss reporting, map persistence and reload ran locally. In the final reload run, 20 of 26 sampled statuses reported tracking and six reported loss. Translation RMSE was 0.526 m over 16 matched poses after first-pose alignment. Trajectory-wide loop closure is missing. [Final map](final_profile/visual_map.npz) · [Pose evaluation](final_profile/measurements.json) |
| MNSE observations and K | PARTIAL | Live class observations, map coordinates, confidence and mission trace entered the kernel. K can query visible classes, person presence, unambiguous coordinates and changes between comparable views. Missing or stale samples yield unknown/error. Historic enrolled-person identification, operator correction and a durable visual-event retention policy remain open. [Mission history](final_profile/inspection_trace.json) · [K component checks](verified_tests.log) |
| Perception-driven navigation | IMPLEMENTED AND ACCEPTED for one inanimate target | K located monitor_001. The SDK submitted NavigateTo through the local authorized adapter. The coordinator reported MISSION_COMPLETED after reaching the observation waypoint. This test did not establish new route-clearance or dynamic-obstacle guarantees. [Trace](visual_goal/visual_goal.json) |
| Control center | PARTIAL | Existing RGB/depth previews remain available. A new visual-perception tab displayed live kernel values in an offscreen PySide6 test. No dedicated trajectory/map renderer or biometric identity history view was validated. [Actual widget response](final_profile/desktop_visual.json) |
| Resource-constrained operation | PARTIAL | A separate constrained profile ran in Gazebo. Inference scheduling, stale-input rejection, bounded buffering and resource diagnostics operate locally. Thermal throttling under measured hardware load, rail power and GPU utilization remain unverified. [Constrained run](constrained/measurements.json) |
| Jetson / TensorRT | NOT TESTED | The available host was x86-64 with CPU ONNX Runtime. NVIDIA lists Orin Nano 8 GB and Orin NX 16 GB. The requested 12 GB application budget needs a 16 GB-class target for acceptance. [NVIDIA module specifications](https://developer.nvidia.com/embedded/faq) |

## Architecture and model choices

The simulation sensor bridge publishes RGB, depth in metres and CameraInfo. Synchronization uses six-message input buffers and a 100 ms pairing tolerance. The ROS node replaces the waiting pair when a newer pair arrives and permits one inference job in flight. YOLO letterboxing serves both runtime and calibration. One detector supplies objects and people. The tracker associates boxes using predicted motion and overlap. It bounds active tracks and appearance candidates and expires them by capture time. Face matching and appearance comparisons run on new tracks when enabled. Resource pressure defers those comparisons. The tracker currently runs at the detector cadence. [Perception node](../../../adapters/perception/perception_service.py) · [Tracker](../../../adapters/perception/person_tracks.py)

Depth processing rejects invalid ranges and missing calibration or transform. It back-projects through the measured intrinsics and applies image-time camera-to-map TF. The reported depth-patch median absolute deviation characterizes local spread. Full 3D covariance remains unknown. IDs for detections, short-lived tracks, appearance candidates and enrolled identities have distinct meanings. The desktop face adapter's default-name assignment and its fabricated person event on camera loss were removed. The live Gemini image path rejects external image submission. [Depth processing](../../../adapters/perception/depth_estimator.py) · [Face matching](../../../adapters/vision/gracemo_vision/face_id.py)

| Model | Provisioning and licensing | Decision |
|---|---|---|
| YOLO11n FP32 ONNX | Local 10,701,564-byte file. SHA-256 54f0395fd0f92170dccc0ca7d2c4ae6793ac568d54526bc19fa76170f13b20d0. [Ultralytics licensing](https://docs.ultralytics.com/models/yolo11) | Keep active. The baseline and optimized desktop FP32 runs agreed on the detections in the eight-frame dataset. |
| Per-channel INT8 convolution trial | About 3.1 MiB. Calibrated on alternating archived frames. | Reject for this profile. It lost the FP32 positive on a held-out frame and was slower. This is agreement with FP32, not labeled accuracy. [Replay](heldout_int8.json) |
| Frontal-face cascade | 930,127 bytes, SHA-256 0f7d4527844eb514d4a4948e822da90fbb16a34a0bbbbc6adc6498747a5aafb0. License notice accompanies the XML. | Local detection-only fallback. |
| YuNet and SFace | Model LFS digests checked by the provisioning script. HTTPS trust prevented download in this run. [YuNet MIT](https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/LICENSE) · [SFace Apache 2.0](https://github.com/opencv/opencv_zoo/blob/main/models/face_recognition_sface/LICENSE) | Lazy optional face detection and embedding. Real identity acceptance pending. |
| ORB and PnP | OpenCV; no additional neural weights. Bounded map stores descriptors and 3D features locally. | Experimental observer. The first image-only PnP trial produced a catastrophic degenerate pose. Independent present-frame depth consistency rejects many such fits at the cost of more reported tracking losses. [Original failure](mapping/measurements.json) · [Depth-checked trial](depth_checked/measurements.json) |
| RTAB-Map | Prior separate mapping run using RGB-D and wheel odometry. [Earlier evaluation](../autonomy_mapping/REPORT.md) | Target-hardware comparison with ORB-SLAM3 and RTAB-Map remains untested. |

## Performance

All values below are desktop measurements. The live rates use wall time; simulation-clock scheduling and the world's 10 Hz RGB / 5 Hz depth sensor limits influence those rates. The replay used eight saved Gazebo images, three passes and one warm-up. The three measurements in each row are median latency, p95 latency and process peak RSS increase including session creation and inference. [Baseline](baseline.json) · [Optimized](optimized.json) · [INT8](heldout_int8.json)

| Detector replay | Median | p95 | Peak RSS increase |
|---|---:|---:|---:|
| Original FP32 | 45.8 ms | 49.2 ms | 140,408 KiB |
| Optimized FP32 | 42.9 ms | 51.0 ms | 138,136 KiB |
| INT8 | 59.1 ms | 66.5 ms | 139,132 KiB |

The final integrated run processed **3.34 frames/s** and acquired **3.37 synchronized pairs/s**. Detector latency was 61.5 ms median and 130.3 ms p95. Median perception CPU was 102.1% on the one-core-equals-100% scale. Peak perception RSS was 549 MB. Peak sampled proportional set size of the private launcher process tree was 2.44 GB, including Gazebo. Individual process groups are recorded in the raw samples. Shared pages are apportioned; snapshots can miss short peaks. This does not establish edge memory headroom. [Resource samples](final_profile/visual_resources.jsonl)

Median visual-odometry step time was 3.18 ms. Median sampled last successful ingestion acknowledgement was 4.42 ms. The K HTTP query took 2.45 ms median and 4.83 ms p95. The tracking-loop median of 0.018 ms in this run includes empty views. Full acoustic-to-visual reasoning latency, per-person embedding latency, and power or thermal behavior were not measured. [Summary](summary.json) · [Final run](final_profile/measurements.json)

## Operation and reproduction

Run from the project root. The ROS node can run against a calibrated, aligned local RGB-D driver without Gazebo. Supply RGB at /camera/image_raw, depth at /camera/depth_image, intrinsics at /camera/camera_info, timestamped TF to map, and an existing kernel endpoint via VIRA_KERNEL_URL. Sensor-driver acquisition and VIRA_DETECTION_HZ are independent. For Gazebo, the bridge also exposes rgb_publish_hz and depth_image_publish_hz. Inference rejects incompatible fixed ONNX sizes. Provision a matching square model before selecting VIRA_DETECTOR_SIZE. A provider request can fall back to CPU; confirm the actual providers in /perception/status.

The CPU environment requires Jazzy rclpy, sensor_msgs, std_msgs, tf2_ros, message_filters, NumPy, OpenCV, requests, psutil and ONNX Runtime. Gallery operations additionally require cryptography. The evaluator uses SciPy. Install the matching architecture build of ONNX Runtime on the deployment machine. The Jetson profile requests TensorRT when available. Hardware-specific TensorRT engine building, cache sizing and hard process-memory enforcement remain deployment work. Persist maps to a **new** private file using VIRA_VISUAL_MAP and reload a compatible map via VIRA_VISUAL_MAP_INPUT. The node drains its worker on shutdown. Camera calibration must match the saved map.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PWD/vira_interaction:$PYTHONPATH"
cargo build --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
out=$(mktemp -d)
python3 scripts/validation/visual_replay.py docs/validation/autonomy_mapping/corridor/visual.db "$out/fp32.json"
python3 scripts/validation/launcher_recovery.py "$out/standard" --vision
VIRA_DETECTION_HZ=3 VIRA_VISUAL_ODOMETRY=1 python3 scripts/validation/launcher_recovery.py "$out/constrained" --vision
VIRA_VISUAL_ODOMETRY=1 python3 scripts/validation/launcher_recovery.py "$out/map" --vision
python3 scripts/validation/evaluate_visual_run.py "$out/map"
python3 scripts/validation/launcher_recovery.py "$out/goal" --visual-goal
```

For the reference-board test, set VIRA_REFERENCE_IMAGE to a local copy of the Ultralytics zidane.jpg test asset, enable VIRA_FACE=1 and VIRA_REID=1, and run the same --vision harness with a fresh output path. The harness removes the fixture before mission dispatch. Use scripts/validation/appearance_replay.py with a local bus.jpg for the separate controlled appearance comparison. The quantizer takes the database, FP32 model and new output path as arguments and requires ONNX and the ONNX Runtime quantization package. Select that candidate only for replay via VIRA_DETECTOR_ONNX.

For deployment, start one sensor consumer with scripts/launch_edge_perception.sh cpu, constrained or jetson after the local camera, TF and kernel are available. Leave face and Re-ID disabled unless needed. Check /perception for current=true, the configured camera frame, observed throughput and actual execution provider before accepting the profile. The private SDK credential is supplied by the operator for authorized movement; the perception node itself publishes no velocity commands.

For consented face enrollment, first provision both local models with scripts/validation/provision_face_models.py in a trusted HTTPS environment. Set VIRA_FACE_GALLERY to a private directory owned by the operator. Enroll with the subject's explicit consent and a local image containing one sufficiently large, sharp face. The identity is a local operator label.

```bash
python3 -m adapters.vision.gracemo_vision.face_id enroll "$label" --gallery "$VIRA_FACE_GALLERY" --image "$image" --consent
python3 -m adapters.vision.gracemo_vision.face_id delete "$label" --gallery "$VIRA_FACE_GALLERY"
```

Deletion removes the active template. Removal from operator backups requires separate handling. No real individual was enrolled in this sprint. Confirm unknown, ambiguous, deleted and low-quality identities against an explicitly consented held-out set before accepting recognition.

## Release gates

The final selection passed **62 Python/ROS tests and 18 Rust tests**. Component tests include synthetic embeddings and synthetic visual telemetry. Integrated runs used the supported launcher, real Gazebo camera messages, the kernel, ROS navigation and the existing PySide6 widget. A physical robot and a Jetson were outside this validation. [Python/ROS](verified_tests.log) · [Rust](rust_queries.log) · [Gazebo lifecycle](visual_goal/process_lifecycle_evidence.json)

The next evidence needed for release is sustained tracking and relocalization through low-texture turns, a moving-person sequence with independent identity labels, a consented face evaluation, real hardware measurements against the inference-rate and memory budgets, and an operator-tested retention/deletion policy for stored observations. The separate patrol-clearance and dynamic-obstacle safety gates also remain open. [Navigation baseline](../unified_simulation/REPORT.md)
