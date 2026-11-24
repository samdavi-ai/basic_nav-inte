# Audit before changes

| Component | Current code and evidence | Status | Gap |
|---|---|---|---|
| Registry and identity | Event source and observed_by are client-supplied. AdapterConnected/Heartbeat are telemetry. | MISSING | Trusted operator grants and authenticated principals |
| Kernel authorization | Both /emit and newline Unix input call process_incoming_event. RobotAction.validate checks payload constraints. | PARTIALLY IMPLEMENTED | No capability decision |
| Receipts | Ledger stores event, session, parent and trace IDs. | MISSING | No atomic authorization decision and replay barrier |
| Dispatch | SSE feeds interaction and kernel bridge; ROS can also accept local inputs. | PARTIALLY IMPLEMENTED | Consistent authenticated ingress and receiver boundary |
| E-stop and cancellation | Existing ROS terrain gate and coordinator latch; private eight-state probes. | IMPLEMENTED for measured profiles | Real spoken interruption during motion |
| Microphone and STT | Voice adapter and STT runtime exist. /dev/snd absent in this session. | NOT TESTED | Actual operator microphone and transcript |
| Mission persistence | Trace-scoped history survived isolated kernel restarts in two prior trials. | IMPLEMENTED | Incomplete mission reconciliation |
| Supported launcher | run_gracemo.sh uses owned process groups, default ROS domain 68 and partition gracemo-68. | PARTIALLY IMPLEMENTED | Full interrupted-mission restart and voice-provider provisioning |

Reference commands and baseline traces are in ../e2e_milestone/VIRA_E2E_ACCEPTANCE_REPORT.md. Main at a7945a8d has pre-existing uncommitted changes. The accepted Nav2 configuration is outside the intended change set. Agent Bridge returned zero pending items.
