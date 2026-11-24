# Simulation readiness

**Partial system demonstration.** A real kernel/adapter persistence test and a real Gazebo mission passed in separate runs. No run demonstrated a spoken command, live provider reasoning, obstacle recovery and spoken completion together.

The existing full-system launcher now requires healthy kernel/MNSE read endpoints and exits on navigation readiness failure. Its child PID list is per invocation, and shutdown preserves failure exit status. The supported headless integration attempt detected an existing session and exited before startup. No existing process was terminated. A fresh supported-launcher run, full GUI verification, service reconnection and voice hardware acceptance remain open. [Launcher attempt](master_launch.log)

Reproduce from the repository root using fresh output paths:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cargo build --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
python3 scripts/validation/kernel_context_demo.py
python3 -m pytest -q tests/contract tests/test_gemini_robotics.py tests/test_perception_pipeline.py tests/test_voice_navigation_e2e.py tests/integration/test_interaction_pipeline.py tests/fault/test_fault_recovery.py tests/safety
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=173 python3 -m pytest -q tests/ros2/test_interaction_bridge.py
out=$(mktemp -d)
python3 scripts/validation/dynamic_cases.py "$out/corridor" --regression original
python3 scripts/validation/analyze_routing.py "$out/corridor"
./run_gracemo.sh --headless --duration 10
```

The first mixed Python invocation could not access the ROS log directory or localhost. The bridge tests subsequently ran on a private DDS domain. The test hard-coded to the shared kernel on port 7780 was not rerun against that session. The isolated restart probe supplies its own independent evidence. [Initial results](python.xml) · [ROS rerun](ros.xml) · [Summary](results.json)
