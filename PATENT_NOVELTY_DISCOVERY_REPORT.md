# GRaCEmo ViRa — Comprehensive Patent Novelty & Technical Invention Discovery Report

**Project:** GRaCEmo ViRa Autonomous Humanoid Service Robot  
**Location:** `~/projects/VIRA-agent-bridge-migration`  
**Investigation Date:** November 4, 2026  
**Auditor:** Principal Robotics Research Scientist & Patent Prior-Art Research Analyst  
**Focus Area:** Safe Cognitive Mission Arbitration and Recovery for Autonomous Mobile Robots  
**Hardware & Safety Constraint:** Simulation-Only (`MOTOR_OUTPUT_ENABLED = 0`, `PROTOCOL_CONFIRMED = False`)

---

## 1. Executive Summary & Purpose of Audit

This investigation was commissioned to conduct an exhaustive, scientifically rigorous, and legally grounded audit of the GRaCEmo ViRa codebase. The primary objective is to determine whether GRaCEmo ViRa has developed any genuinely original, patentable technical mechanisms in the coordination of:
1. Cognitive decision-making
2. Voice-based human interruption
3. Active autonomous missions
4. Mission-state preservation
5. Robot localization
6. Navigation execution
7. Multi-layer safety monitoring
8. Emergency interruption
9. Mission recovery
10. Physical-progress verification

### The Core Finding
**Honest Scientific & Legal Conclusion:**  
GRaCEmo ViRa is a highly sophisticated, production-grade integration of modern open-source robotics frameworks (ROS 2 Jazzy, Gazebo Harmonic, Nav2, AMCL, Pygame ViRa Interaction Gateway, and Rust-based MNSE Kernel). The engineering remediation achieved in November 2026 successfully resolved critical simulation-to-localization synchronization defects (including the stationary robot/advancing RViz divergence).

However, **in its current as-implemented state, the codebase consists predominantly of standard robotics architectural patterns, known Finite State Machine (FSM) semantics, conventional ROS 2 topic/action abstractions, and standard threshold-based watchdogs.** Taken individually, none of the existing components exhibit worldwide algorithmic novelty that would withstand scrutiny under 35 U.S.C. § 102/103 (US), Article 54/56 EPC (Europe), or Section 3(k) of the Indian Patents Act, 1970 (which bars algorithms and computer programs per se).

### The Viable Invention Pathway
While individual modules are conventional, **the system contains a distinct architectural synthesis** developed during the November 2026 remediation:
> **The Candidate Invention:**  
> *"Grounded Multi-Modal Mission Supervision Architecture with Closed-Loop Physical Invariant Verification and Voice-Preserved Checkpointing for Autonomous Service Robots"*

Specifically, this mechanism coordinates:
- Independent dual-threshold supervisory progress monitoring ($\Delta d_{map} \ge 0.08\text{ m}$ within $14.0\text{ s}$ window) that decouples high-level mission state verification from downstream Nav2 controller feedback loops.
- Serialized 5-tier safety invariant cascading (`cmd_vel_mux` $\to$ `velocity_smoother` $\to$ `collision_monitor` $\to$ `terrain_monitor` $\to$ `mcu_transport`).
- Voice-interrupted mission checkpointing with pose-relative goal re-dispatch and AMCL covariance-bounded relocalization.

To elevate this candidate from a "patent-ineligible aggregation of known components" (*KSR International Co. v. Teleflex Inc.*) into a **globally defensible, patentable invention**, this report defines the necessary algorithmic leap: a **Multi-Sensor Kinematic Consistency Index ($\Gamma$) and Autonomous Mission Integrity Metric ($\mathcal{I}$)** that mathematically proves physical progress and prevents ghost odometry across physical, simulated, and degraded sensing environments.

---

## 2. Complete Runtime Architecture & Data Flow

GRaCEmo ViRa's autonomy stack spans three execution tiers:
1. **Cognitive & Voice Interaction Tier (Python / Pygame / REST):** User speech capture, wake-word detection, intent classification, safety filtering, and UI rendering.
2. **Supervisory Autonomy & Coordination Tier (ROS 2 Jazzy):** Mission lifecycle management, waypoint queue sequencing, AMCL health monitoring, trajectory watchdog, and bridge translation.
3. **Physical & Simulation Substrate Tier (ROS 2 / Gazebo Harmonic / MCU):** Sensor processing (LiDAR, IMU, Depth, Odometry), Nav2 costmap/controller pipelines, velocity safety multiplexing, and hardware-locked MCU transport.

### 2.1 Complete Architectural Data-Flow Diagram

```
[ Human Voice / UI ]
        │ Audio Stream (16 kHz PCM)
        ▼
[ Pygame ViRa Voice Engine ] ── (SpeechRecognition / Whisper / Groq API)
        │ Raw Transcript ("Go to the library", "Stop", "Pause")
        ▼
[ InteractionGateway.process_text() ] ── (vira_interaction/core/gateway.py)
   ├── Regex FastPathMatcher (fast_path.py)
   ├── PlaceResolver (place_resolver.py -> config/places.json)
   ├── Groq Cloud Fallback (model.py)
   └── SafetyGate.evaluate_action() (safety_gate.py)
        │ Validated Action Event {type, params, priority}
        ▼
[ EventBus.publish() ] ── (vira_interaction/core/event_bus.py)
        │
        ├──> [ Pygame Face GUI ] (Eye animation, state rendering)
        │
        └──> [ ROS Interaction Bridge ] (gracemo_bridge/interaction_bridge.py)
                 │ ROS 2 Topics: /gracemo/coordinator/command
                 │               /emergency_stop
                 │               /cmd_vel_teleop
                 ▼
[ CommandCoordinatorNode ] ── (gracemo_nav2/coordinator_node.py)
   ├── Supervisory State Machine (IDLE, PLANNING, EXECUTING, PAUSED, etc.)
   ├── Waypoint Queue Manager (Index preservation across interruptions)
   ├── Localization Health Monitor (/amcl_pose covariance check)
   ├── Trajectory Watchdog (_check_navigation_watchdog: disp >= 0.08m / 14s)
   └── Recovery Executive (Costmap clearing, nomotion_update, goal retry)
        │
        │ Action Goal: /navigate_to_pose
        ▼
[ Nav2 Autonomy Stack ] (nav2_bringup / nav2_params.yaml)
   ├── BT Navigator / NavfnPlanner (Global Path: /plan)
   ├── DWB Local Controller (Trajectory Generation: /cmd_vel_nav)
   └── Costmap2D Server (Global & Local Obstacle Layers)
        │
        │ /cmd_vel_nav (Nominal Navigation Velocity)
        ▼
[ 5-Stage Safety Invariant Velocity Chain ]
   Stage 1: [ cmd_vel_mux_node ] (Arbitrates /cmd_vel_teleop vs /cmd_vel_nav)
            │ /cmd_vel_mux
   Stage 2: [ nav2_velocity_smoother ] (Enforces accel/jerk bounds)
            │ /cmd_vel_smoothed
   Stage 3: [ nav2_collision_monitor ] (LiDAR polygonal footprint safety stop/slowdown)
            │ /cmd_vel_safe
   Stage 4: [ gracemo_terrain_monitor ] (IMU roll/pitch tilt throttling & waist servo)
            │ /cmd_vel
   Stage 5: [ gracemo_hardware / MCU ] (MOTOR_OUTPUT_ENABLED = 0 lock & heartbeat)
            │
            ▼
[ Gazebo Harmonic 8.8.0 Physics Substrate ] (DiffDrive, Joints, Wheels, Sensors)
   ├── /clock (Gazebo Simulation Clock)
   ├── /odom (gz-sim-diff-drive-system Odometry)
   ├── /imu/data (Gazebo IMU Sensor)
   ├── /scan (Gazebo 2D LiDAR Ray Sensor)
   └── /camera/* (Gazebo RGB-D Sensor)
```

---

## 3. Systematic Component Audit

| Component | File Path | Primary Class / Function | Inputs | Outputs | Internal State | Topics / Services / Actions | Error Handling | Maturity |
|---|---|---|---|---|---|---|---|---|
| **Voice Fast Path** | `vira_interaction/core/fast_path.py` | `FastPathMatcher.match()` | Text string from VAD/ASR | `FastPathResult(action, params)` | Pre-compiled regex patterns | None (Pure Python) | Graceful fallback to LLM | Implemented & Tested |
| **Place Resolver** | `vira_interaction/core/place_resolver.py` | `PlaceResolver.resolve_place()` | Landmark query string | Coordinates `(x, y, yaw)` | Landmark registry from `places.json` | None (Pure Python) | Unknown landmark returns `None` | Implemented & Tested |
| **Safety Gate** | `vira_interaction/core/safety_gate.py` | `SafetyGate.evaluate_action()` | Structured action candidate | Boolean decision + reason | Command frequency history, E-stop latch | None (In-memory checks) | Command rejection with audit log | Implemented & Tested |
| **Interaction Gateway** | `vira_interaction/core/gateway.py` | `InteractionGateway.process_text()` | Raw user utterance | Dispatched action event | Gateway mode, session context | Emits to `EventBus` | Regex $\to$ LLM $\to$ Help message | Implemented & Tested |
| **Interaction Bridge** | `ros2_ws/.../gracemo_bridge/interaction_bridge.py` | `InteractionBridgeNode` | `EventBus` events or `/vira/voice_intent` | ROS messages on coordinator / cmd topics | ROS node handle | Publishes `/gracemo/coordinator/command`, `/emergency_stop` | Topic retry, logging | Implemented & Tested |
| **Command Coordinator** | `ros2_ws/.../gracemo_nav2/coordinator_node.py` | `CommandCoordinatorNode` | Coordinator commands, `/amcl_pose`, TF transforms | Nav2 action goals, recovery triggers | FSM state, waypoint queue, progress pose, timers | Subscribes `/amcl_pose`, `/clock`; Actions: `/navigate_to_pose`; Srv: clear costmaps | Retries up to 3x, aborts to safe state | Implemented & Tested |
| **Trajectory Watchdog** | `ros2_ws/.../gracemo_nav2/coordinator_node.py` | `_check_navigation_watchdog()` | TF `map` $\to$ `base_footprint`, ROS clock | Cancel goal, transition to obstacle recovery | `last_progress_pose`, `last_progress_time` | None (Internal timer method) | Cancels goal, triggers costmap clear | Implemented & Tested |
| **Command Mux** | `ros2_ws/.../gracemo_nav2/cmd_vel_mux_node.py` | `CmdVelMuxNode` | `/cmd_vel_teleop`, `/cmd_vel_nav` | `/cmd_vel_mux` | Active source, last message timestamps | Subscribes teleop/nav; Publishes `/cmd_vel_mux`, status | Timeout fallback (0.5s) to zero-twist | Implemented & Tested |
| **Terrain Monitor** | `ros2_ws/.../gracemo_nav2/terrain_monitor_node.py` | `TerrainMonitorNode` | `/imu/data`, `/cmd_vel_safe` | `/cmd_vel`, `/gracemo/coordinator/command` | Roll/pitch filter, emergency latch | Subscribes IMU/cmd_vel; Publishes `/cmd_vel`, E-stop | Auto-clamping velocity, emergency trigger | Implemented & Tested |
| **Sensor Bridge** | `ros2_ws/.../gracemo_bridge/sensor_bridge.py` | `SensorBridgeNode` | Gazebo GZ transport topics | ROS 2 standard topics (`/scan`, `/odom`, `/imu`) | GZ node handle | Subscribes GZ topics; Publishes ROS 2 sensor topics | Serialization exception catching | Implemented & Tested |
| **Cognitive Kernel** | `kernel/crates/gracemo-kernel` | Rust MNSE runtime daemon | REST requests (`http://127.0.0.1:7780`) | Context AST, plan ledger | Ledger state, active context graph | HTTP REST endpoints | Result/Option error bubbling | Partially Implemented (Daemon active, standalone) |

---

## 4. Deep Investigation of the Five Target Areas

### Area A: Cognitive Mission Arbitration
- **Mechanism:** User utterance is matched against regex patterns in `FastPathMatcher`. If matched, `PlaceResolver` extracts target coordinates. `SafetyGate` checks rate limits and forbidden phrases. An action event is emitted over `EventBus` to `InteractionBridgeNode`, which publishes a JSON string to `/gracemo/coordinator/command`. `CommandCoordinatorNode` parses the JSON and transitions its internal FSM.
- **Evaluation:** This is a classic hierarchical FSM architecture. While well-designed and cleanly decoupled, it does **not** employ a mathematically distinct arbitration algorithm (such as dynamic utility optimization or multi-objective Pareto arbitration). It represents standard industrial practice.

### Area B: Mission Continuity and Recovery
- **Mechanism:** On receipt of a `"pause"` command or non-fatal obstacle alert, `coordinator_node.py` calls `active_goal_handle.cancel_goal_async()`. The waypoint queue (`self.waypoints`) and current index (`self.current_waypoint_idx`) are preserved in memory. On receipt of `"resume"`, it queries the current robot map pose via TF, checks terrain safety, and issues a new Nav2 `NavigateToPose.Goal` for the preserved waypoint.
- **Evaluation:** This is standard Nav2 action client goal management. Nav2's native `waypoint_follower` and ROS action semantics are designed specifically for this workflow. It is not patentable.

### Area C: Physical-Progress-Aware Watchdog
- **Mechanism:** Remedied in November 2026 to resolve the critical "advancing RViz while stationary in Gazebo" defect. The watchdog calculates Euclidean displacement in the `map` frame:
  $$\Delta d = \sqrt{(x_{curr} - x_{last})^2 + (y_{curr} - y_{last})^2}$$
  If $\Delta d \ge 0.08\text{ m}$, `last_progress_pose` and `last_progress_time` are updated. If $\Delta d < 0.08\text{ m}$ for $t_{stagnant} > 14.0\text{ s}$ or total time $t_{total} > 180.0\text{ s}$, the active goal is aborted, costmaps are cleared, and a retry is scheduled.
- **Evaluation:** Decoupling progress verification from high-frequency feedback callbacks was an essential engineering correction. However, displacement-over-time stuck detection is well-known in robotics (e.g. Nav2 `SimpleProgressChecker`, iRobot patents US 2013/0310978 A1). To become patentable, it must be combined with multi-sensor kinematic slip verification.

### Area D: Cross-Layer Safety Arbitration
- **Mechanism:** A serialized 5-stage pipeline: Priority Mux ($t_{timeout}=0.5\text{ s}$) $\to$ Nav2 Velocity Smoother $\to$ Nav2 Collision Monitor (LiDAR polygonal footprint) $\to$ Terrain Monitor (IMU roll/pitch envelope: $12^\circ/15^\circ$ slowdown, $20^\circ/25^\circ$ E-stop) $\to$ MCU Transport (hardware safety lock).
- **Evaluation:** Highly effective for mobile robot safety, conforming to ISO 3691-4 principles. However, the pipeline stages themselves are standard ROS 2 nodes (`twist_mux`, `nav2_collision_monitor`, etc.). The combination represents solid systems engineering rather than an inventive algorithmic step.

### Area E: Localization-Aware Mission Recovery
- **Mechanism:** The coordinator monitors AMCL covariance diagonal elements: $\sigma_{xx} > 2.5$ or $\sigma_{yy} > 2.5$. When exceeded, it transitions to `STATE_LOCALIZATION_LOST`, cancels the goal, invokes `/request_nomotion_update` to force AMCL sensor updates while stationary, and verifies map bounding limits ($[-10, 10]\text{ m}$).
- **Evaluation:** Standard practice in probabilistic mobile robotics (Thrun et al., 2005).

---

## 5. Patentability & Section 3(k) Legal Risk Assessment

### 5.1 Jurisdictional Analysis
1. **United States (35 U.S.C. §§ 101, 102, 103):**
   - Under § 101 (Alice/Mayo framework), an algorithm controlling physical mobile robot actuation and sensor validation qualifies as patent-eligible subject matter because it improves an underlying physical process.
   - However, under § 103 (Obviousness), a patent examiner will cite standard Nav2 documentation, iRobot stasis detection, and Savioke task interruption, asserting that combining these is an obvious design choice for a skilled robotics engineer.
2. **India (Section 3(k), Patents Act, 1970):**
   - Section 3(k) strictly excludes "a mathematical or business method or a computer programme per se or algorithms".
   - The Indian Patent Office (IPO) Guidelines for Computer Related Inventions (CRIs) require that the software must have a **demonstrable technical effect** and be inextricably tied to **novel physical hardware interaction**.
   - If filed as an "algorithm for mission arbitration", it faces immediate statutory rejection. It can only survive if claimed as a *physical robotic control system with closed-loop multi-sensor hardware validation*.

---

## 6. Ranking of Potential Patent Candidates

| Rank | Candidate Title | Originality | Technical Effect | Non-Obviousness | Implementation Maturity | Section 3(k) Risk | Recommendation |
| :---: |---|:---: |:---: |:---: |:---: |:---: |---|
| **1** | **Grounded Multi-Modal Mission Supervision with Closed-Loop Invariant Verification** | Medium-High | High (Prevents physical motor/odometry divergence & tipping) | Moderate-High (If mathematical slip fusion is added) | Fully Implemented (Remediation validated) | Low (Tied to physical sensors/actuators) | **STRONG CANDIDATE (Proceed with algorithmic enhancement)** |
| **2** | **Multi-Tier Inertial Terrain Envelope & Active Waist Balancing System** | Medium | Medium (Dynamic tip-over prevention) | Low-Medium (Tilt speed reduction is widely known) | Fully Implemented | Low | **DEFENSIVE DISCLOSURE ONLY** |
| **3** | **Voice-Interrupted Waypoint Checkpointing and Recovery FSM** | Low | Low (Convenience feature) | Very Low (Standard Nav2/Savioke practice) | Fully Implemented | High (Considered pure software/FSM) | **DO NOT FILE (Will be rejected)** |
| **4** | **Rust-Based MNSE Cognitive Kernel AST Execution Daemon** | High (Architecture) | Low (In-silico computation) | Medium | Partially Implemented (Standalone daemon) | Extremely High (Pure software algorithm) | **PROTECT VIA TRADE SECRET / COPYRIGHT** |

---

## 7. Roadmap to Patent Filing

To transform Candidate 1 into an ironclad patent application:
1. **Develop and Implement Algorithm 1:** Multi-Sensor Kinematic Consistency Index ($\Gamma_{kinematic}$) cross-referencing IMU linear acceleration, LiDAR scan matching velocity, and wheel encoder odometry.
2. **Develop and Implement Algorithm 2:** Autonomous Mission Integrity Metric ($\mathcal{I}_{mission}$) synthesizing kinematic consistency, AMCL covariance, and terrain attitude.
3. **Execute Experimental Proof Plan:** Run the 10 controlled simulation test cases in `EXPERIMENTAL_PROOF_PLAN.md` to collect empirical comparative graphs proving failure prevention.
4. **Draft Patent Application:** Provide `PRELIMINARY_INVENTION_DISCLOSURE.md` to a registered patent agent specializing in robotics and cyber-physical systems.

