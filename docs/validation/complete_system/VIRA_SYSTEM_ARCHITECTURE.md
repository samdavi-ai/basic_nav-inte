# VIRA system architecture and capability inventory

| Component | Existing feature | Interface | Simulation demonstration | Verification |
|---|---|---|---|---|
| Rust types, kernel and K | IMPLEMENTED. Typed event envelope, Tokio bus, K lexer/parser/planner/runtime | HTTP, SSE, newline-delimited Unix socket | Local kernel, validated action rejection, streamed observation | [Kernel coverage](KERNEL_FEATURE_COVERAGE.md) |
| MNSE | IMPLEMENTED. InspectorState, persistent event ledger and relational graph, context compiler | Snapshot, history, graph, context and world routes | Fixture observation retrieved by Gemini context provider after ingestion | [MNSE coverage](MNSE_FEATURE_COVERAGE.md) |
| SDK and reasoning | PARTIALLY IMPLEMENTED end to end. AdapterClient, tool registry and orchestrator | REST, canonical actions, provider client | Local tests and live context read | [Adapter coverage](ADAPTER_FEATURE_COVERAGE.md) |
| ROS bridge, coordinator and Nav2 | IMPLEMENTED for static navigation, PARTIALLY IMPLEMENTED for dynamic recovery | Kernel events, ROS actions and sensor topics | Apartment corridor with physical simulation | [Navigation](ROS2_GAZEBO_INTEGRATION.md) |
| Sensor and perception adapters | PARTIALLY IMPLEMENTED integration | RGB-D, scan, odometry and scene events | Raw sensor capture and historical visual mapping | [Perception](PERCEPTION_AND_VOICE_REPORT.md) |
| Voice | PARTIALLY IMPLEMENTED | STT, intent, typed actions and TTS | Phrase and safety tests, device chain open | [Voice coverage](PERCEPTION_AND_VOICE_REPORT.md) |
| Control center | PARTIALLY IMPLEMENTED | PySide6 modules for autonomy, cognition, diagnostics, interaction, localization, logs, navigation, overview, safety, sensors, settings and simulation | Added live snapshot, context, graph and historical events to cognitive view | Four tabs constructed offscreen; operator display inspection pending |
| Agent Bridge | IMPLEMENTED queue and task approval interfaces | MCP | Queue queried, zero pending tasks | Separate from per-action kernel authorization |
| General adapter registry and per-action authorization receipt | MISSING from inspected HTTP router | AdapterConnected/Heartbeat are events | No demonstrated authoritative registration or permission handshake | Release blocker |

The supported simulation launcher is [run_gracemo.sh](../../../run_gracemo.sh). MNSE is implemented within the Rust kernel process. The accepted mission route passes through the ROS interaction bridge, autonomous coordinator, Nav2 and Gazebo. The experimental planner remains private. No claim of physical hardware acceptance follows from this inventory.
