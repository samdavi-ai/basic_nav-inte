# VIRA end-to-end acceptance

**Hold release. Zero of seven workflows meets every mandatory stage.** A static corridor run and its mission-memory restart check succeeded. The dedicated integration suite verified correlation, unknown location and explicit provider failure. The real microphone path was unavailable. The supported-launcher restart, active mission reconciliation, authenticated action authorization and moving-obstacle recovery remain release blockers.

| Scenario | Verdict | Remaining gate |
|---|---|---|
| [E2E-01](E2E_01_SPOKEN_NAVIGATION_REPORT.md) | PARTIAL | A real spoken request and audible terminal confirmation were unavailable. No authenticated adapter authorization receipt was demonstrated. |
| [E2E-02](E2E_02_CONTEXT_QUESTIONS_REPORT.md) | PARTIAL | Tracking and locating the human who spoke, fresh spatial answers with source and spoken output remain unverified. |
| [E2E-03](E2E_03_OBSTACLE_AWARENESS_REPORT.md) | PARTIAL | Dynamic insertion and full stop-reassess-resume remain unqualified. Previous front insertion exhausted retries. |
| [E2E-04](E2E_04_SPOKEN_INTERRUPTION_REPORT.md) | PARTIAL | No microphone-to-stop trial on a moving Gazebo mission was demonstrated. |
| [E2E-05](E2E_05_ORDERED_SKILLS_REPORT.md) | PARTIAL | A complete navigate-observe-report chain with actual scene verification has no passing integrated run. |
| [E2E-06](E2E_06_PROVIDER_FAILOVER_REPORT.md) | PARTIAL | Active mission preservation and selection of a second live provider during an outage remain unverified. |
| [E2E-07](E2E_07_RESTART_MEMORY_REPORT.md) | PARTIAL | This mission completed before shutdown and originated from the programmatic adapter. Voice input, reconciliation of interrupted actions and complete supported-launcher restart remain open. |

## Changes and evidence

The Rust ledger migrates existing databases, preserves session and parent IDs and serves trace-filtered history. The SDK exposes that query and optional parent ID on event submission. The private Gazebo runner stops producers before comparing durable history. The first comparison failed because producers appended two legitimate rows during shutdown. It is retained in initial_history_comparison.json. The initial repository-path launch exceeded the Unix socket path limit. The final runner keeps the socket and transient databases under its short temporary overlay.

The Gemini client and orchestrator explicitly fail on provider outages and failed tools. The MNSE inspector, context compiler and adapter distinguish an unknown pose from the map origin. Accepted navigation parameters and movement ownership were preserved.

## Reproduction

From the repository root with Jazzy, Gazebo and the workspace installed:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cargo test --workspace --offline --manifest-path kernel/Cargo.toml
cargo build --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel
python3 -m pytest -q tests/e2e/test_mission_memory.py
python3 -m pytest -q tests/test_gemini_robotics.py tests/test_voice_navigation_e2e.py tests/test_perception_pipeline.py tests/fault/test_fault_recovery.py tests/safety
out=$(mktemp -d)
PYTHONPATH="$PWD/adapters/sdk:$PYTHONPATH" python3 scripts/validation/dynamic_cases.py "$out/corridor" --regression original --verify-memory
python3 scripts/validation/analyze_routing.py "$out/corridor"
```

[Machine-readable verdicts and evidence](../../../artifacts/e2e/e2e_acceptance_summary.json)

The final repository-path repeat recovered 60 identical trace rows after restart. The kernel terminal state was MISSION_COMPLETED. Its executed interpolated clearance lower bound was 0.133977 m. [Repeat evidence](../../../artifacts/e2e/final_corridor/restart.json)
