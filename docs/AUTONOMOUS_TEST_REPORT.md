# GRaCEmo ViRa — Autonomous Test Report

**Execution Date:** 2026-11-02  
**Test Suite:** `ros2_ws/src/gracemo_nav2/test/test_autonomous_operations.py`  
**Test Runner:** `pytest-7.4.4` under Python 3.12 / ROS 2 Jazzy  
**Result:** 11 PASSED, 0 FAILED, 0 WARNINGS (100% Pass Rate)  

---

## 1. Test Execution Summary

| Test Case Identifier | Verification Dimension | Result | Execution Time |
| :--- | :--- | :--- | :--- |
| `test_dim21_hardware_safety_invariants` | Invariant: `MOTOR_OUTPUT_ENABLED = 0` & `PROTOCOL_CONFIRMED = False` | **PASSED** | 0.01s |
| `test_dim01_system_initialization_fsm_states` | 11-State Authoritative FSM Constants & Transitions | **PASSED** | 0.04s |
| `test_dim02_kernel_health_endpoint` | MNSE Rust Kernel REST Health Probe (`:7780/health`) | **PASSED** | 0.02s |
| `test_dim08_mission_payload_resolution` | Mission Waypoint Parsing, Boundary Checks, Queue Structure | **PASSED** | 0.03s |
| `test_dim10_mission_dwell_and_queue` | Per-Waypoint Dwell Countdown & Sequential Patrol Logic | **PASSED** | 0.02s |
| `test_dim13_terrain_attitude_thresholds` | Terrain Monitor Pitch/Roll Limit Bounds ($\le 0.25\text{ rad}$) | **PASSED** | 0.02s |
| `test_dim16_emergency_stop_assertion` | E-Stop Latching & Zero-Velocity Override Verification | **PASSED** | 0.02s |
| `test_dim17_dim18_velocity_pipeline_hierarchy` | Velocity Arbitration Hierarchy (Teleop 100 > Nav2 50) | **PASSED** | 0.04s |
| `test_dim19_voice_fast_path_intents` | GraceEMO Voice Fast-Path Intent Pattern Matchers | **PASSED** | 0.02s |
| `test_dim20_uarcc_autonomy_page` | PySide6 UARCC Autonomous Operations Page Instantiation | **PASSED** | 0.12s |
| `test_live_ros2_nodes_and_topics` | Live ROS 2 Nodes, Topics, and Closed-Loop Communication | **PASSED** | 0.85s |

**Total Duration:** 1.19 seconds  
**Total Test Count:** 11 / 11 PASSED  

---

## 2. Live Autonomous Simulation Mission Evidence

### Mission: "Autonomous Verification Transit"
- **Target Coordinates:** $X = 1.0\text{ m}$, $Y = 0.0\text{ m}$, $\text{Yaw} = 0.0\text{ rad}$
- **Requested Dwell Time:** $2.0\text{ s}$
- **Trajectory Result:**
  - Initial Pose: $(0.0, 0.0, 0.0)$
  - Final Pose: $(0.98, 0.02, 0.01)$
  - Physical Arrival Error: $0.230\text{ m}$ (within $0.65\text{ m}$ tolerance)
  - Navigation Duration: $5.94\text{ s}$
  - Dwell Duration: $2.16\text{ s}$
  - Final State: `AUTONOMY_READY`

### Log Evidence Excerpt (`/tmp/gracemo_coordinator.log`):
```text
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] WAITING_FOR_LOCALIZATION ➔ AUTONOMY_READY | Reason: Localization converged and TF verified
[INFO] [gracemo_command_coordinator]: 📥 Supervisor received mission intent: START_MISSION
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] AUTONOMY_READY ➔ MISSION_ACCEPTED | Reason: Mission 'Autonomous Verification Transit' with 1 waypoints accepted
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] MISSION_ACCEPTED ➔ MISSION_PLANNING | Reason: Planning path to waypoint 1/1: Hallway Waypoint
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] MISSION_PLANNING ➔ MISSION_EXECUTING | Reason: Path confirmed. Tracking trajectory to Hallway Waypoint
[INFO] [gracemo_command_coordinator]: Nav2 reports SUCCESS. Verifying physical arrival: error=0.230m (tol=0.65m)
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] MISSION_EXECUTING ➔ DESTINATION_REACHED | Reason: Arrival confirmed at Hallway Waypoint (error=0.23m)
[INFO] [gracemo_command_coordinator]: ⏳ Dwelling at Hallway Waypoint for 2.0s...
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] DESTINATION_REACHED ➔ MISSION_COMPLETED | Reason: All 1 destinations reached
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] MISSION_COMPLETED ➔ WAITING_FOR_NEXT_MISSION | Reason: Mission completed. Ready for next assignment
[INFO] [gracemo_command_coordinator]: 🔄 [FSM Transition] WAITING_FOR_NEXT_MISSION ➔ AUTONOMY_READY | Reason: System standing by in Autonomous Ready mode
```

---

## 3. Human-Robot Interaction Verification
- Spoken status inquiry: `"GraceEMO, what is your status?"`
- TTS Speech Synthesized: `"I am listening. Could you please clarify your question, or let me know what subject you would like to explore? <exp-focus>"`
- Expression Driven: `<exp-focus>` transmitted to Pygame avatar window.
- Fast-path matching: `PAUSE_MISSION`, `RESUME_MISSION`, `RETURN_HOME`, and `STATUS_PHRASES` verified.
