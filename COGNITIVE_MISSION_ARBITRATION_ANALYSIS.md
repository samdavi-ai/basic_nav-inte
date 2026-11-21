# GRaCEmo ViRa — Cognitive Mission & Safety Arbitration Analysis

**Project:** GRaCEmo ViRa Autonomous Humanoid Service Robot  
**Auditor:** Principal Robotics Research Scientist & Autonomous Systems Architect  
**Modules Investigated:** `vira_interaction/`, `kernel/`, `ros2_ws/src/gracemo_nav2/`, `ros2_ws/src/gracemo_bridge/`  
**Focus:** Area A (Cognitive Mission Arbitration) & Area D (Cross-Layer Safety Arbitration)

---

## 1. Executive Summary

This report performs an in-depth reverse-engineering of the decision-making pathways within GRaCEmo ViRa. It tracks how a high-level cognitive instruction (e.g. user voice command, operator UI override, or cognitive kernel plan) propagates through the system, interacts with active autonomous missions, and resolves conflicts against safety monitoring constraints.

### Key Findings
1. **Decision Architecture:** The system employs a **hybrid two-stage arbitration model**:
   - *Stage 1 (Cognitive Gateway):* Regex fast-path intent matching with Groq cloud LLM fallback and safety-gate rule filtering (`vira_interaction/`).
   - *Stage 2 (Supervisory Coordinator):* Explicit hierarchical Finite State Machine (FSM) managing mission lifecycles, waypoint queues, and Nav2 action goals (`ros2_ws/.../coordinator_node.py`).
2. **Current Arbitration Nature:** The arbitration logic is **strictly deterministic, rule-based, and priority-multiplexed**. It does not currently compute multi-criteria utility functions or dynamic probabilistic Pareto frontiers.
3. **Novelty Evaluation:** While architecturally clean, robust, and highly modular, the current implementation is **conventional systems engineering**. To establish patentable novelty, the cognitive arbitration must be upgraded from static priority switching to the proposed **Autonomous Mission Integrity Index ($\mathcal{I}_{mission}$)** arbitration engine.

---

## 2. End-to-End Decision Execution Trace

The lifecycle of an incoming instruction travels through the following precise path:

```
Step 1: Audio Ingestion & Transcription
   └── Module: vira_interaction/ (SpeechRecognition / Whisper / Groq API)
   └── Output: Raw text string (e.g., "ViRa, pause navigation and wait here")
   └── Latency: ~180 ms - 450 ms

Step 2: Gateway & Fast-Path Intent Routing
   └── Module: vira_interaction/core/gateway.py -> fast_path.py
   └── Function: InteractionGateway.process_text(text) -> FastPathMatcher.match(text)
   └── Evaluation:
       ├── 2.1 Emergency regex check: r"\b(emergency\s+stop|e-?stop|abort|freeze|shutdown)\b"
       ├── 2.2 Intent regex evaluation: MATCHES INTENT_PAUSE
       └── 2.3 Result: FastPathResult(intent="PAUSE_MISSION", params={})
   └── Latency: ~1.2 ms

Step 3: Landmark & Parameter Validation
   └── Module: vira_interaction/core/place_resolver.py
   └── Function: PlaceResolver.resolve_place(query)
   └── Evaluation: For "NAVIGATE", resolves target coordinates against places.json;
       for "PAUSE", no spatial parameters required.
   └── Latency: ~0.4 ms

Step 4: Safety Gate Evaluation
   └── Module: vira_interaction/core/safety_gate.py
   └── Function: SafetyGate.evaluate_action(act)
   └── Invariant Checks:
       ├── E-stop latch status: Clear (PASS)
       ├── Forbidden keywords: None detected (PASS)
       ├── Command rate limit: > 0.2 s since last command (PASS)
       └── Battery level: Nominal (> 20%) (PASS)
   └── Latency: ~0.8 ms

Step 5: Event Bus Dispatch
   └── Module: vira_interaction/core/event_bus.py
   └── Function: EventBus.publish(Event(type="ActionRequested", payload={...}))
   └── Listeners:
       ├── Pygame Face Engine: Triggers eye squint / listening expression
       └── ROS Interaction Bridge: Translates event into ROS message
   └── Latency: ~0.5 ms

Step 6: ROS 2 Interaction Bridge Ingestion
   └── Module: ros2_ws/src/gracemo_bridge/gracemo_bridge/interaction_bridge.py
   └── Function: InteractionBridgeNode.handle_action_event()
   └── Action Mapping:
       ├── Action "PAUSE_MISSION" -> coordinator_cmd = "pause"
       └── ROS Publisher: /gracemo/coordinator/command (std_msgs/msg/String)
   └── Latency: ~1.5 ms (DDS transport)

Step 7: Command Coordinator Arbitration & FSM Transition
   └── Module: ros2_ws/src/gracemo_nav2/gracemo_nav2/coordinator_node.py
   └── Function: CommandCoordinatorNode._on_coordinator_command(msg)
   └── Execution:
       ├── Current state: STATE_MISSION_EXECUTING
       ├── Received command: "pause"
       ├── Evaluates preconditions: Is actively executing? Yes.
       ├── Executes: self._abort_active_navigation_goal()
       │   └── Nav2 Action Client: self.active_goal_handle.cancel_goal_async()
       ├── Preserves mission state:
       │   ├── self.waypoints remains intact
       │   └── self.current_waypoint_idx remains at active index
       └── Transitions state: self.transition_to(STATE_MISSION_PAUSED)
   └── Latency: ~8.0 ms (Nav2 cancellation handshake)

Total End-to-End Latency: ~192 ms - 462 ms
```

---

## 3. Decision & Priority Arbitration Matrices

### 3.1 Cognitive Intent vs. Robot Operational State Matrix

The following decision matrix is implemented in `coordinator_node.py` and `interaction_bridge.py`:

| Incoming Instruction | State: IDLE | State: PLANNING | State: EXECUTING | State: PAUSED | State: OBSTACLE_HANDLING | State: LOCALIZATION_LOST | State: EMERGENCY_STOP |
|---|---|---|---|---|---|---|---|
| **EMERGENCY_STOP** | Latch E-stop, publish zero twist | Abort plan, Latch E-stop | Abort goal, Latch E-stop, zero twist | Latch E-stop | Abort recovery, Latch E-stop | Abort recovery, Latch E-stop | Already latched (No-op) |
| **NAVIGATE (New Goal)** | Accept, plan path, transition to PLANNING | Reject (Planning in progress) | Abort current, clear queue, start new mission | Abort paused, clear queue, start new mission | Abort recovery, clear queue, start new mission | Reject (Localization unhealthy) | Reject (Hardware locked) |
| **PAUSE** | Ignore (No active mission) | Cancel planner, transition to PAUSED | Cancel Nav2 goal, preserve waypoint, transition to PAUSED | Already paused (No-op) | Cancel retry timer, transition to PAUSED | Transition to PAUSED | Reject (Locked) |
| **RESUME** | Reject (Queue empty) | Ignore | Already executing (No-op) | Verify terrain, query current TF pose, re-dispatch goal, transition to EXECUTING | Resume recovery retry | Reject (Localization unhealthy) | Reject (Locked) |
| **CANCEL** | No-op | Cancel planner, clear queue, transition to IDLE | Cancel goal, clear queue, transition to IDLE | Clear queue, transition to IDLE | Cancel recovery, clear queue, transition to IDLE | Clear queue, transition to IDLE | Reset state to IDLE (Keep E-stop latched until explicit reset) |

---

### 3.2 Cross-Layer Safety Velocity Arbitration (The 5-Stage Cascade)

When an active mission is in `STATE_MISSION_EXECUTING`, velocity commands pass through the serialized 5-stage safety cascade:

```
[ Downstream Nav2 Path Follower ]
         │ /cmd_vel_nav (e.g. vx = 0.35 m/s, wz = 0.0 rad/s)
         ▼
[ Stage 1: Priority Mux (cmd_vel_mux_node) ]
   ├── Input A: /cmd_vel_teleop (Priority 1)
   ├── Input B: /cmd_vel_nav (Priority 2)
   ├── Timeout Window: 500 ms
   └── Rule: If teleop received within 500 ms, SUPPRESS /cmd_vel_nav completely.
         │ /cmd_vel_mux
         ▼
[ Stage 2: Velocity Smoother (nav2_velocity_smoother) ]
   ├── Kinematic Limits: ax,max = 1.0 m/s^2, alphaz,max = 1.5 rad/s^2
   └── Rule: Profile acceleration to prevent motor jerk and inertial slip.
         │ /cmd_vel_smoothed
         ▼
[ Stage 3: Collision Monitor (nav2_collision_monitor) ]
   ├── Input: 2D LiDAR Pointcloud (/scan)
   ├── Stop Zone: Polygon [x: -0.28 to +0.28, y: -0.28 to +0.28] -> Output v = 0
   └── Slowdown Zone: Polygon [x: -0.50 to +0.50, y: -0.45 to +0.45] -> Output 0.3 * v
         │ /cmd_vel_safe
         ▼
[ Stage 4: Terrain Monitor (gracemo_terrain_monitor) ]
   ├── Input: IMU (/imu/data) Roll (phi) and Pitch (theta)
   ├── Rule 4.1: If |phi| > 25 deg or |theta| > 20 deg -> TRIGGER EMERGENCY STOP
   ├── Rule 4.2: If |phi| > 15 deg or |theta| > 12 deg -> SCALE VELOCITY by (1 - excess/10 deg)
   └── Rule 4.3: Waist Pitch Servo = -0.85 * theta (Active tip-over counterbalance)
         │ /cmd_vel
         ▼
[ Stage 5: Hardware Lockout (mcu_transport / firmware) ]
   ├── Check 5.1: MOTOR_OUTPUT_ENABLED == 0 (Strict simulation lock)
   ├── Check 5.2: PROTOCOL_CONFIRMED == False (Serial MCU handshake inhibited)
   └── Rule: Zero physical voltage delivered to physical motor H-bridges.
```

---

## 4. Technical Deficiencies of Current Arbitration

1. **Static Priority Without Contextual Weighing:**
   If a user says "Navigate to the classroom" while the robot is navigating to the library, the system performs a hard cancellation of the existing mission and immediately starts the new mission. It does not compute whether the new destination is en route, evaluate topological detour costs, or confirm human intent.
2. **Open-Loop Progress Assumption in Velocity Mux:**
   Stage 1 (`cmd_vel_mux`) blindly forwards commands if messages arrive within 500 ms. It has zero awareness of whether physical wheels are slipping or whether the robot chassis is progressing.
3. **Absence of Unified Risk-Utility Function:**
   Safety decisions (terrain tilt, LiDAR proximity, AMCL covariance) operate as independent trip-wires. If the robot experiences mild terrain tilt ($13^\circ$), mild localization uncertainty ($\sigma = 2.0$), and an approaching obstacle at 0.48 m simultaneously, none of the individual trip-wires fire, yet the **compound risk** is catastrophic.

---

## 5. Architectural Specification for the Advanced Cognitive Arbitration Engine

To establish patent novelty, the system must replace disjoint trip-wires with the **Compound Risk & Mission Integrity Arbitration Engine**:

$$\mathcal{R}_{compound}(t) = 1.0 - \mathcal{I}_{mission}(t)$$

Where the decision protocol is governed by:

$$\text{Arbitrate}(\text{Cmd}_{new}, \text{Mission}_{active}, \mathcal{R}_{compound}) = \begin{cases}
\text{EXECUTE\_IMMEDIATE}(\text{Cmd}_{new}) & \text{if } \text{Cmd}_{new} = \text{E-STOP} \\
\text{SPLICE\_WAYPOINT}(\text{Cmd}_{new}) & \text{if } \text{Cmd}_{new} = \text{NAV} \land \mathcal{R}_{compound} \le 0.25 \land \text{IsOnRoute}(\text{Cmd}_{new}) \\
\text{HOLD\_AND\_CLARIFY}(\text{Cmd}_{new}) & \text{if } \text{Cmd}_{new} = \text{NAV} \land \mathcal{R}_{compound} > 0.25 \\
\text{ATOMIC\_PAUSE\_CHECKPOINT} & \text{if } \mathcal{R}_{compound} \ge 0.50 \lor \text{Cmd}_{new} = \text{PAUSE}
\end{cases}$$

This compound formulation provides the exact technical effect required to satisfy both US non-obviousness (§ 103) and Indian technical-effect standards (Section 3(k)).

