  # VIRA simulation integration decision
  
  **HOLD for full voice-driven autonomy.** The accepted Gazebo stack completed the five-room patrol, two static corridor repetitions and a typed navigation-and-inspection sequence. All movement in this investigation used Gazebo. The safety-critical moving-obstacle candidate remains private.
  
  ## Seven mission gates
  
  | Mission | Result | Evidence and boundary |
  |---|---|---|
  | 1. Spoken destination | Partial | The desktop routes supported room requests through the existing authorized mission runner and waits for coordinator and kernel completion before announcing arrival. The normal microphone-to-destination-to-audible-arrival chain still needs a successful trial. |
  | 2. Environment understanding | Partial | A live, post-arrival camera frame produced a monitor detection (model class “tv”, project alias “monitor”), 4.665 m optical depth and confidence 0.638. The observation includes camera and map coordinates. Identity was not independently verified against simulator ground truth. Persistent tracking and audible answers remain open. [Detection trace](typed_inspection/inspection_trace.json) |
  | 3. Navigate, inspect and report | Typed sequence demonstrated | The desktop's post-transcription handler dispatched a scoped StartMission. The coordinator reached the destination. Frame 127 supplied the reported detection. A second run used the room's configured heading and reported no localized objects. Both runs have causal links between the interaction request, authorized action and inspection. The final implementation distinguishes arrival from successful identification when the scene is empty. [First result](typed_inspection/interaction.log) · [Named-room trace](named_inspection/inspection_trace.json) |
  | 4. Obstacle response | Static corridor passed twice. Moving recovery remains open. | The padded footprint measures 0.72 × 0.70 m. The 0.35 m square obstacle was centred at (2, −0.5). The two executed swept-clearance lower bounds were 0.138400 and 0.139082 m. Under the analyzer's additional speed assumptions the bounds were 0.065441 and 0.055693 m. Goal errors were 0.23958 and 0.21693 m. Both runs had zero DWB cycles rejecting all candidates. Existing negative controls emitted zero coordinator commands. [Run 1](corridor_1/geometry.json) · [Run 2](corridor_2/geometry.json) · [Prior dynamic failures](../release_closure/DYNAMIC_RECOVERY_REPORT.md) |
  | 5. Spoken emergency interruption | Open | An earlier instrumented microphone client achieved a verified stop. In the first normal-launcher trial this round, the microphone supplied samples during motion but no stop receipt arrived within 58 seconds. A timeout supplies no stopping latency or proof of an acoustic cause. [Normal trial](voice_restart/voice_1_diagnostics.json) · [Earlier measurements](../security_voice_recovery/voice_interruption_report.md) |
  | 6. Provider and adapter outage | Partial | A deliberately unconfigured Gemini adapter failed explicitly during the executing five-room patrol. It reported zero actions and the mission/trace stayed active. The bounded history query omitted the original action at both checkpoints, so those two empty lists alone cannot prove lifetime absence of duplicates. Live model inference, a mid-request network outage and service failover remain unverified. [Outage sample](patrol/provider_outage.json) |
  | 7. Restart and reconciliation | Passed for interrupted execution and authorized abandonment | Two fresh supported-launcher processes used one persistent private state directory. The interrupted mission required review. A new dispatch was rejected (409), the runner could not approve abandonment (403), and the authorized reviewer cleared it (204). Restart peak command was zero at the mux, smoother and output. The three-second observer measured 0.00659 m odometry displacement. [Review decision](full_restart/operator_decision.json) · [Idle measurement](full_restart/restart_idle.json) · [Desktop view](full_restart/desktop_review.json) |
  
  **Patrol clearance failed.** At Gazebo pose (−5.151, 0.883) with yaw 5.081 radians the analyzer identified the northwest hallway wall as the limiting obstacle. The minimum signed clearance at a recorded pose was −0.005999 m. The interpolated swept lower bound was −0.011902 m and the conservative intersample speed bound was −0.089649 m. The padded envelope overlaps the modeled wall even at a recorded pose. Actual chassis contact was not measured. [Patrol geometry](patrol/geometry.json)
  
  The separate functional patrol completed **11/11 in 181 seconds**. The coordinator and Rust kernel agreed on mission ID and terminal state. A subsequent independent Gazebo E-stop test passed latching, rejection, explicit reset and post-reset idle checks. Maximum post-stop drift was 0.0246 m and final-second maximum odometry speed was 0.0162. [E-stop](patrol/estop.json) · [Idle](patrol/idle.json) The earlier visual mapping run used RTAB-Map alongside AMCL. Navigation localized solely from the saved visual map and full-room loop closure are still open. [Patrol record](patrol/mission.json) · [Mapping evidence](../autonomy_mapping/REPORT.md)
  
  ### Final camera and microphone trials
  
  At the configured living-room heading, frame 110 was captured at 44.600 s on the Gazebo clock. The first clock tick sampled after verified arrival was 44.008 s. The 0.592 s difference confirms acquisition after arrival. The application reported arrival and explicitly recorded no_localized_objects. Navigation completion alone did not satisfy object identification. [Final inspection trace](final_inspection/inspection_trace.json)
  
  In a second cued normal-launcher trial the priority listener processed at least 970,752 signed PCM samples, with a maximum interval peak of 31,799 on a 32,768 scale. The decoder finalized nine nonempty utterances. None matched an accepted stop phrase and the app logged no dispatch attempt. These aggregate diagnostics distinguish missing capture from an unmatched decode. They cannot establish the words spoken, microphone source isolation or the exact recognition error. The earlier successful instrumented stop does not qualify the normal launcher. [Input levels](voice_levels_trial/voice_1_input_levels.json) · [App diagnostics](voice_levels_trial/voice_1_diagnostics.json)
  
  An offline replay used two synthetic English voices with four stop phrases and five negative phrases per voice. The production decoder matched 7/8 intended stops and triggered on 0/10 negatives. The private grammar matched 8/8 and triggered on 0/10 negatives, but the comparison phrases appear in its vocabulary; this is an in-vocabulary check. It remains test-only until independently evaluated on held-out speech and actual microphone input. [Controlled results](controlled_stop.json)
  
  ## Integration changes
  
  - The K interpreter requires a live executor for each stage. Mission admission and dispatch read the MNSE autonomy state. Graph location rejects absent or ambiguous observations. The previous fixed example coordinates and unconditional skill completions could produce false success. The active-mission query returned MISSION_EXECUTING. K still lacks general skill execution. [Live K result](full_restart/live_k.json)
  - The launcher starts the RGB-D service. It synchronizes color and depth, requires camera intrinsics, and transforms localized detections using TF at image time. A missing transform discards the localization. The kernel tracks perception health. Gemini image access checks scoped file freshness. The local ignored ONNX asset is required for reproducibility on another machine. It was exported on CPU from the official YOLO11n weights using Ultralytics 8.3.162 and ONNX opset 17. [Artifact hashes](model_hashes.txt)
  - The normal simulation interaction handler resolves named destinations and reuses the mission runner, Rust authorization and coordinator. An inspection checks a newer frame after navigation completion and records feedback under the same trace. The integrated interaction bridge requires the coordinator. The operator view exposes registered adapter capabilities and live lease state. [Leases](full_restart/adapters.json) · [Inspection trace](typed_inspection/inspection_trace.json)
  - The scene-question handler can ask Gemini to rank multiple measured observations. Sensor records supply the facts, and the application validates the returned index before using it. Provider participation in a successful live answer remains unverified. Navigation can run offline.
  - The microphone listener records aggregate sample counts, peak levels and decoded utterance counts for diagnosis. No raw audio is retained by the trial harness. The harness distinguishes simulated motion, operator cue, detection, authenticated receipt and rest.
  
  ## Verification and outstanding gates
  
  The Rust workspace passed 18 tests. The final Python and ROS selection passed 75 tests including four fault-handling tests, including component tests with simulated transports and integration tests with a private Rust kernel. Earlier runs exposed an obsolete camera-file mock and reliability fixtures missing Nav2 readiness, fresh transforms, sensor inputs and the kernel action echo. Those fixtures were updated and the failing logs retained. These counts provide no physical validation. [Rust](final_rust_tests.log) · [Python and ROS](final_validation.log) · [Initial reliability failures](final_python_tests.log)
  
  The operator can see MNSE state, history, restart review and scoped adapter connections. A single live view verifying readiness of **all** dependencies, general K skill execution, live provider failover, microphone removal/reconnection, persistence-write and ROS-bridge failure during motion, and complete dynamic recovery remain release blockers. The authenticated restart reviewer can abandon an interrupted mission. Automatic redispatch is prohibited. No evidence here approves resuming from an arbitrary partial trajectory. The Agent Bridge queue was empty at discovery and supplied no task ID to publish against. [Initial subsystem inventory](architecture.md)
  
  ## Reproduction
  
  From the project root, use new evidence directories for every run. The validation harness supplies separate local kernel ports, ROS domains and Gazebo partitions. The spoken run requires a human cue after moving.json and speak.ready appear. Record that cue using time.monotonic() in operator.cue and say “stop” at the development microphone.
  
  ```bash
  source /opt/ros/jazzy/setup.bash
  source ros2_ws/install/setup.bash
  export PYTHONPATH="$PWD:$PWD/vira_interaction:$PWD/adapters/sdk:$PYTHONPATH"
  cargo build --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel
  cargo test --offline --workspace --manifest-path kernel/Cargo.toml
  ROS_DOMAIN_ID=198 python3 -m pytest -q tests/test_perception_pipeline.py tests/test_gemini_robotics.py tests/e2e/test_kernel_authorization.py tests/e2e/test_mission_memory.py tests/ros2/test_interaction_bridge.py tests/test_voice_navigation_e2e.py tests/test_master_system_reliability.py tests/fault/test_fault_recovery.py
  out=$(mktemp -d)
  python3 scripts/validation/dynamic_cases.py "$out/corridor" --regression original --authorization-check
  python3 scripts/validation/analyze_routing.py "$out/corridor"
  python3 scripts/validation/dynamic_cases.py "$out/patrol" --regression patrol --mission-timeout 300
  python3 scripts/validation/analyze_routing.py "$out/patrol"
  python3 scripts/validation/launcher_recovery.py "$out/restart"
  python3 scripts/validation/launcher_recovery.py "$out/inspection" --inspection
  python3 scripts/validation/launcher_recovery.py "$out/spoken" --spoken
  vira_interaction/.venv/bin/python scripts/validation/stop_decoder_comparison.py "$out/controlled_stop.json"
  ```
