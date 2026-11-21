# GRACEMO VIRA — MASTER DEFECT REGISTER

> **Standard:** ISO/IEC/IEEE 29119 Software & Systems Engineering Testing  
> **Classification:** Engineering Audit Quality Record  
> **Project Root:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`  
> **Safety Mode:** Simulation-Only (`MOTOR_OUTPUT_ENABLED = 0`, `PROTOCOL_CONFIRMED = False`)  

---

## 1. DEFECT CLASSIFICATION SEVERITY MATRIX

| Severity | Definition | Operational Impact | Action Required |
| :--- | :--- | :--- | :--- |
| **P0 — Critical** | Safety violation, command bypass, or hardware hazard | Locomotion safety gate or emergency stop compromised | Immediate software halt and blocking fix |
| **P1 — High** | Major subsystem unavailable or core workflow broken | Autonomous navigation, voice engine, or UARCC non-functional | Must be resolved before system acceptance |
| **P2 — Medium** | Functionality partially working or integration incomplete | Fallback triggered, test harness incompatibility, or minor logic error | Resolved incrementally with regression verification |
| **P3 — Low** | Minor usability defect, environment edge case, or doc gap | Workaround available; minimal operational disruption | Documented in master runbook or queued for polish |
| **P4 — Informational**| Optimization opportunity or physical measurement prerequisite | System functions safely, but physical fidelity can be improved | Hardware measurement required prior to live deployment |

---

## 2. DEFECT LOG

### DEF-01: Motion Command Ownership Violation on `/cmd_vel`
- **Defect ID:** `DEF-01`
- **Subsystem:** ROS 2 Navigation & Bridge Architecture (`gracemo_bridge/kernel_bridge.py`)
- **Severity:** `P0 — Critical`
- **Description:** `gracemo_kernel_bridge` initialized a direct publisher on `/cmd_vel` (`self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)`), violating the mandatory architectural invariant that `gracemo_terrain_monitor` must be the sole authoritative publisher on `/cmd_vel`.
- **Reproduction Steps:**
  1. Launch full simulation stack: `ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization`.
  2. Inspect `/cmd_vel` publisher count: `ros2 topic info -v /cmd_vel`.
  3. Observe two concurrent publishers: `gracemo_kernel_bridge` and `gracemo_terrain_monitor`.
- **Expected Behaviour:** Exactly 1 publisher on `/cmd_vel` (`gracemo_terrain_monitor`). All teleoperation must route through `/cmd_vel_teleop` $\rightarrow$ `cmd_vel_mux` $\rightarrow$ `velocity_smoother` $\rightarrow$ `collision_monitor` $\rightarrow$ `terrain_monitor`.
- **Actual Behaviour:** Un-arbitrated velocity commands could bypass collision detection and terrain safety gates.
- **Root Cause:** Legacy development code in `kernel_bridge.py` published directly to `/cmd_vel` and subscribed redundantly to forward to Gazebo.
- **Evidence:** `tests/test_motion_command_ownership.py` failed with:
  `AssertionError: Expected exactly 1 publisher on /cmd_vel, found 2: ['gracemo_kernel_bridge', 'gracemo_terrain_monitor']`.
- **Correction Applied:**
  1. Changed `kernel_bridge.py` publisher target from `/cmd_vel` to `/cmd_vel_teleop`.
  2. Removed redundant Gazebo `/cmd_vel` forwarding subscriber in `kernel_bridge.py`.
- **Verification Status:** `VERIFIED RESOLVED` (Passed `tests/test_motion_command_ownership.py` with 1 publisher).

---

### DEF-02: Fast-Path Emergency Word Omission ("wait")
- **Defect ID:** `DEF-02`
- **Subsystem:** Interaction Engine (`vira_interaction/core/fast_path.py`)
- **Severity:** `P1 — High`
- **Description:** The phrase `"wait"` was omitted from `STOP_WORDS` in `vira_interaction/core/fast_path.py`, although it was present in `graceemo_interaction`. When a human uttered *"wait"*, the system failed to trigger the sub-15ms fast-path emergency stop and instead fell through to cloud LLM query.
- **Reproduction Steps:**
  1. Call `fast_path.match("wait")`.
  2. Return was `None` instead of `Intent: STOP`.
- **Expected Behaviour:** Utterance of *"wait"* must immediately yield `FastPathResult(action="EmergencyStop", state="EMERGENCY_STOP")` in $<15\text{ms}$.
- **Actual Behaviour:** Utterance was treated as an ungrounded general query, causing multi-second cloud API delay during an urgent human intervention.
- **Root Cause:** Incomplete synchronization between prototype `graceemo_interaction` and migration root `vira_interaction`.
- **Evidence:** `tests/unit/test_fast_path.py` failed with:
  `AssertionError: Failed for phrase: 'WAIT'`.
- **Correction Applied:** Added `"wait"` to `STOP_WORDS` tuple in `vira_interaction/core/fast_path.py`.
- **Verification Status:** `VERIFIED RESOLVED` (Passed `tests/unit/test_fast_path.py` with 100% stop words matched).

---

### DEF-03: Interaction Bridge Variable Unbound Error on Yaw Orientation
- **Defect ID:** `DEF-03`
- **Subsystem:** ROS 2 Interaction Bridge (`adapters/robot-bridge/gracemo_bridge/interaction_bridge.py`)
- **Severity:** `P2 — Medium`
- **Description:** In `_start_navigation(tx, ty, target_name, target_yaw, trace_id)`, `yaw_val` was referenced on line 362 when assembling `coord_cmd` before being defined on line 376.
- **Reproduction Steps:**
  1. Run `python3 -m unittest tests/ros2/test_interaction_bridge.py`.
  2. Send command `"navigate to kitchen"` without an explicit orientation angle.
- **Expected Behaviour:** Default yaw to $0.0\text{ rad}$ and dispatch goal cleanly.
- **Actual Behaviour:** Raised `UnboundLocalError: cannot access local variable 'yaw_val' where it is not associated with a value`.
- **Root Cause:** Code ordering error during coordinator payload refactoring.
- **Evidence:** `tests/ros2/test_interaction_bridge.py` failed with `UnboundLocalError`.
- **Correction Applied:** Moved `yaw_val = float(target_yaw) if target_yaw is not None else 0.0` to the top of `_start_navigation` before JSON coordinator payload generation in both `adapters/robot-bridge/gracemo_bridge/interaction_bridge.py` and `ros2_ws/src/gracemo_bridge/gracemo_bridge/interaction_bridge.py`.
- **Verification Status:** `VERIFIED RESOLVED` (All 23 tests in `tests/ros2/` passed).

---

### DEF-04: Interaction Bridge Legacy Topic Incompatibility
- **Defect ID:** `DEF-04`
- **Subsystem:** ROS 2 Integration & Test Harnesses (`gracemo_bridge/interaction_bridge.py`)
- **Severity:** `P2 — Medium`
- **Description:** Previous test suites and external components expected voice commands on `/interaction/user_input` and responses as JSON on `/interaction/spoken_feedback`. The newly migrated bridge exclusively listened to `/interaction/command` and published raw strings on `/interaction/speech`.
- **Reproduction Steps:** Run `test_whisper_voice_interaction.py` or `test_live_full_acceptance.py`.
- **Expected Behaviour:** Seamless compatibility with both modern and legacy voice communication topics.
- **Actual Behaviour:** Subscribers timed out awaiting messages on `/interaction/spoken_feedback`.
- **Root Cause:** Architectural rename of topics without maintaining dual pub/sub compatibility aliases.
- **Evidence:** `tests/test_whisper_voice_interaction.py` assertion error on subscriber timeout.
- **Correction Applied:**
  1. Added subscriber alias on `/interaction/user_input` pointing to `_on_command_msg`.
  2. Added publisher alias on `/interaction/spoken_feedback` publishing structured JSON metadata envelopes alongside raw `/interaction/speech`.
  3. Added attribute alias `self.pub_cmd_vel = self.pub_cmd_vel_teleop` for test harnesses mocking publisher handles.
- **Verification Status:** `VERIFIED RESOLVED` (Passed `tests/ros2/` and full acceptance suites).

---

### DEF-05: FastDDS Localhost Discovery Partitioning in Child Subshells
- **Defect ID:** `DEF-05`
- **Subsystem:** ROS 2 Middleware / DDS Communication
- **Severity:** `P3 — Low`
- **Description:** In Ubuntu 24.04 with FastDDS, ROS 2 nodes started across disparate subshells occasionally failed peer discovery unless explicit localhost discovery boundaries were declared.
- **Reproduction Steps:** Start daemon in terminal A without `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; run `ros2 node list` in terminal B.
- **Expected Behaviour:** Complete visibility of active graph nodes.
- **Actual Behaviour:** Nodes missing or intermittently visible.
- **Root Cause:** FastDDS dynamic multicast discovery defaults under Linux network namespaces.
- **Evidence:** Graph audit log discrepancies resolved once environment declared.
- **Correction Applied:** Mandated `export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` across all launch scripts and Section 1 of `ALL_RUN_COMMANDS.md`.
- **Verification Status:** `VERIFIED RESOLVED`.

---

### DEF-06: Physical Chassis Mass & Golden Inertia Divergence
- **Defect ID:** `DEF-06`
- **Subsystem:** Robot Description (`gracemo_description/urdf/canonical_gracemo_fixed.urdf`)
- **Severity:** `P4 — Informational / Physical Prerequisite`
- **Description:** The link inertial matrices in `canonical_gracemo_fixed.urdf` are calculated using Blender volumetric CAD approximations ($I_{xx}, I_{yy}, I_{zz}$) rather than physical bifilar pendulum oscillation measurements of the manufactured physical robot.
- **Reproduction Steps:** Compare URDF chassis mass ($38.0\text{ kg}$) with physical scale weight of the assembled robot frame.
- **Expected Behaviour:** Physical CAD mass matching actual hardware to within $\pm 2.0\%$.
- **Actual Behaviour:** Physical hardware access is required to weigh assembled chassis, battery pack, head display, and servo actuators with calipers and scale.
- **Root Cause:** Prototype stage physical hardware validation constraint.
- **Correction Applied:** Documented as an unresolved physical requirement in `ROBOT_MODEL_VALIDATION.md` and `SYSTEM_ACCEPTANCE_REPORT.md`.
- **Verification Status:** `OPEN — PENDING PHYSICAL HARDWARE ACCESS`.

---

### DEF-07: Local Qwen MLX Model Unsupported on Linux x86_64
- **Defect ID:** `DEF-07`
- **Subsystem:** Live Intelligence & LLM Gateway (`vira_interaction/core/gateway.py`)
- **Severity:** `P3 — Low`
- **Description:** `mlx_lm` is an Apple Silicon macOS-specific framework. When running on Ubuntu Linux x86_64, importing `mlx_lm` raises `ModuleNotFoundError`.
- **Reproduction Steps:** Boot `vira_interaction/model.py` on Linux.
- **Expected Behaviour:** Clean detection of platform, logging informative notice, and automatic fallback to Groq Cloud / Gemini APIs without crash.
- **Actual Behaviour:** Clean fallback confirmed:
  `ℹ Local Qwen MLX model unavailable on Linux (No module named 'mlx_lm')`
  `✓ Falling back to Groq Cloud API.`
- **Root Cause:** Hardware architecture constraint (Apple Silicon vs. Linux x86_64).
- **Correction Applied:** Graceful fallback verified; Groq / Gemini cloud paths handle LLM reasoning.
- **Verification Status:** `VERIFIED RESOLVED VIA ARCHITECTURAL FALLBACK`.

---
*End of Master Defect Register.*
