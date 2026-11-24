# Existing interfaces

The supported launcher is run_gracemo.sh. The isolated dynamic_cases.py runner exercises a separate local kernel, Gazebo partition, ROS graph, coordinator and motion safety chain. The Rust kernel accepts canonical events over HTTP and Unix socket, broadcasts SSE, updates InspectorState and persists ledger and graph data. The Python SDK and Gemini context provider read these APIs. The cognitive PySide6 view displays current state and historical records.

| Capability | Status at entry | Evidence or gap |
|---|---|---|
| Adapter to Nav2 mission | IMPLEMENTED | Static corridor and five-room functional baseline |
| Dynamic recovery | BROKEN in tested traversable cases | Braking vetoes exhaust retries |
| Voice and TTS | PARTIALLY IMPLEMENTED | Text intents tested; real spoken round trip unverified |
| Context and perception | PARTIALLY IMPLEMENTED | RGB-D sensors and persistent graph available; human localization and grounded speech incomplete |
| Skill ordering and failover | PARTIALLY IMPLEMENTED | Tool registry and orchestrator exist; verified completion across failures missing |
| Per-action identity verification | MISSING on generic event ingestion | Payload validation does not authenticate the sender |
| Full-system restart | NOT TESTED | Kernel-only observation persistence demonstrated |
| Historical mission correlation | PARTIALLY IMPLEMENTED | Trace IDs stored, recent history capped, session and parent links previously omitted |

[Source inventory and prior component evidence](../complete_system/VIRA_SYSTEM_ARCHITECTURE.md)
