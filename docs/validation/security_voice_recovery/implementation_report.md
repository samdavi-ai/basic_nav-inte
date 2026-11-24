# Implementation and reproduction

The Rust kernel owns adapter grants, scoped receipts, the dispatch transaction, and the review of interrupted missions. ROS continues to own execution and the hardware-independent safety latch. The navigation footprint, inflation, controller and speed limits were preserved.

| Files | Change in this milestone |
|---|---|
| kernel/gracemo-kernel/Cargo.toml and src/{authorization,api,main,lib,ledger,inspector_state}.rs | Register kernel HTTP routes, validate requests, bind receipts and safety state, record decisions, and reconstruct mission history. |
| adapters/sdk/gracemo_sdk/{authorization,client}.py and adapters/gemini_robotics/tools.py | Route consequential actions through the authenticated SDK. |
| vira_interaction/core/{event_bus,gateway,offline_stop,safety_gate}.py, vira_interaction/{model.py,requirements.txt} | Remove speculative local movement dispatch, recognize stop locally, and correlate the authenticated request. |
| ros2_ws/src/gracemo_bridge/gracemo_bridge/interaction_bridge.py | Receive executable actions through the kernel stream. |
| ros2_ws/src/gracemo_nav2/gracemo_nav2/coordinator_node.py | Record mission cancellation while preserving the emergency safety state. |
| run_gracemo.sh, scripts/{verify_system_readiness,run_autonomous_mission}.py | Support isolated complete launcher runs, address the selected simulator and provision the standalone adapter. |
| gracemo_control_center/main.py and modules/cognitive/cognitive_view.py | Preserve ROS isolation and show kernel restart review. |
| scripts/validation/{authorization_boundary,dynamic_cases,launcher_recovery,microphone_stop,observe_voice_stop,recovery_view}.py | Capture denied and authorized ROS commands, voice timing and restart evidence. |
| tests/e2e/test_kernel_authorization.py, tests/{test_gemini_robotics,test_master_system_reliability,test_voice_navigation_e2e}.py and tests/ros2/test_interaction_bridge.py | Check kernel permissions, adapt the mock dispatch contract and guard cancellation during E-stop, and exercise bridge translation using an explicitly simulated kernel echo. The first legacy test run had 10 failures because those tests assumed uncredentialed action dispatch. |

Run from the project root with unique output directories. The supported voice application needs the Vosk English acoustic model in vira_interaction/.venv/share/vosk-model-small-en-us-0.15 or a model directory specified in VIRA_OFFLINE_STT_MODEL. Production credentials must be provisioned independently. Validation scripts provision short-lived, least-privilege credentials in private temporary files.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD:$PWD/adapters/sdk:$PYTHONPATH"
cargo build --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
python3 -m pytest -q tests/e2e/test_kernel_authorization.py tests/e2e/test_mission_memory.py tests/test_gemini_robotics.py
ROS_DOMAIN_ID=201 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 -m pytest -q tests/test_master_system_reliability.py -k 'cancel_during_estop or cancellation_idempotency or estop_lockout'
out=$(mktemp -d)
for n in 1 2; do
  python3 scripts/validation/dynamic_cases.py "$out/corridor_$n" --regression original --authorization-check
  python3 scripts/validation/analyze_routing.py "$out/corridor_$n" > "$out/corridor_$n/geometry.json"
done
python3 scripts/validation/launcher_recovery.py "$out/restart"
```

For the instrumented microphone trial use `PYTHONUNBUFFERED=1 python3 scripts/validation/dynamic_cases.py "$out/voice" --regression patrol --mission-timeout 100 --live-voice`. A second operator terminal creates `$out/voice/operator.cue` after `speak.ready` appears and the operator then says “stop”. For the complete launcher use `python3 scripts/validation/launcher_recovery.py "$out/combined" --spoken`. Record the cue with `python3 -c 'import pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text(str(time.monotonic()))' "$out/combined/operator.cue"` once its speak.ready file appears. The scripts require actual ROS discovery and microphone access on the development host.

The Linux voice runtime opens the shared desktop capture source for priority recognition independently of conversational STT and playback. The priority listener never synthesizes speech. Keyboard microphone disable also disables this listener. Both conversational actions and offline stops use the same kernel authorization and ROS dispatch.
