# Documentation index

Use this page to find the current guides and acceptance evidence. Historical reports are retained as dated evidence; the newest status report takes precedence where results differ.

## Start here

- [Repository overview and simulation quick start](../README.md)
- [Project gaps and simulation status](PROJECT_GAPS_AND_SIMULATION_STATUS.md)
- [Autonomous operation architecture](AUTONOMOUS_OPERATION_ARCHITECTURE.md)
- [Autonomous operation user guide](AUTONOMOUS_OPERATION_USER_GUIDE.md)
- [Safety gates](AUTONOMOUS_SAFETY_GATES.md)

## Development and integration

- [ROS 2 integration](ROS2_INTEGRATION.md)
- [Gazebo integration](GAZEBO_INTEGRATION.md)
- [Navigation flow](NAVIGATION_FLOW.md)
- [Robot bridge](ROBOT_BRIDGE.md)
- [Event contracts](VIRA_EVENT_CONTRACTS.md)
- [Test architecture](VIRA_TEST_ARCHITECTURE.md)
- [Contributing guide](../CONTRIBUTING.md)

## Operations and deployment

- [Operator manual](OPERATOR_MANUAL.md)
- [Deployment guide](DEPLOYMENT_GUIDE.md)
- [Fault recovery manual](FAULT_RECOVERY_MANUAL.md)
- [Safety operations](SAFETY_OPERATIONS.md)
- [Hardware deployment gates](hardware_activation_gates.md)

Physical hardware guides describe interfaces and gates. This branch's acceptance target is Gazebo desktop simulation; a hardware guide does not imply physical validation.

## Validation reports

| Programme | Scope | Report |
|---|---|---|
| System status | Current measured gaps and simulation state | [Project status](PROJECT_GAPS_AND_SIMULATION_STATUS.md) |
| VIR-VISUAL-ALL-02 | Visual capability audit | [Report](validation/VIR-VISUAL-ALL-02-FINAL-REPORT.md) |
| VIR-VISUAL-ALL-03 | Visual implementation and acceptance | [Report](validation/VIR-VISUAL-ALL-03-FINAL-REPORT.md) |
| VIR-VISUAL-ALL-04 | Complete visual intelligence programme | [Report](validation/VIR-VISUAL-ALL-04-FINAL-REPORT.md) |
| Corridor routing | Static corridor obstacle routing diagnosis | [Report](validation/corridor_routing/REPORT.md) |
| Dynamic obstacles | Dynamic obstacle routing and recovery | [Report](validation/dynamic_obstacles/REPORT.md) |
| Edge vision | Desktop perception validation | [Report](validation/edge_vision/REPORT.md) |
| Unified simulation | End-to-end simulation integration | [Report](validation/unified_simulation/REPORT.md) |

Validation runs may produce image frames, maps, and high-volume JSONL traces. The concise reports and result summaries are the primary review artifacts; large raw telemetry is regenerated locally and ignored by Git.
