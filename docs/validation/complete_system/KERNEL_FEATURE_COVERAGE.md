# Kernel feature coverage

| Feature | Implemented | Demonstrated | Tested | Evidence |
|---|---|---|---|---|
| Typed publication and SSE | IMPLEMENTED | Synthetic Vision producer to real HTTP kernel and SSE consumer | Same event ID and trace ID on receive | [Restart evidence](context_adapter_restart.json) |
| Action validation | IMPLEMENTED for constraints in RobotAction::validate | Invalid NavigateToRoom rejected before state update or dispatch | HTTP 422 and absent from ledger. Unix input uses same validation path | [Executable probe](../../../scripts/validation/kernel_context_demo.py) |
| Schema validation | IMPLEMENTED | Malformed envelope rejected | HTTP client error | [Probe output](context_adapter_restart.json) |
| Durable history | IMPLEMENTED | Existing state directory reopened by new process | Same ID/trace retrieved | [Persistence evidence](context_adapter_restart.json) |
| Mission execution feedback | IMPLEMENTED for navigation | Coordinator and kernel terminal records | Real Gazebo fixture | [Mission](gazebo_control/mission.json) |
| Parent IDs, duplicate handling, ordering, acknowledgements | PARTIALLY IMPLEMENTED | Envelope supports parent ID, SQLite primary key rejects duplicate rows, history orders by timestamp | No strict ordering or exactly-once delivery test. Kernel currently ignores ledger insert errors, so a duplicate may broadcast again | kernel/gracemo-types/src/lib.rs and kernel/gracemo-kernel/src/ledger.rs |
| Authorization and adapter lifecycle | PARTIALLY IMPLEMENTED | Higher-level policy gates and heartbeat events exist | No proof that arbitrary POST /emit clients are authorized | [Safety tests](offline.xml) |

K execution returns a runtime summary. It does not certify ROS action execution. The public event payload and recent-history projection differ; the latter does not expose every envelope field.
