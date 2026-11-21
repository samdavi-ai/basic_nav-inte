# GRaCEmo ViRa — End-to-End Simulation Integration Test Results

> **Document Type:** End-to-End Integration Scenario Results  
> **Environment:** ROS 2 Jazzy | Gazebo Harmonic 8.9.0 | PySide6 UARCC | Ubuntu 24.04 LTS (x86_64)  
> **Simulation State:** Apartment Floor World (`gracemo_apartment.sdf`) with `gracemo_vira` Robot Model  
> **Date:** 2026-11-02  
> **Status:** ALL 8 SCENARIOS VERIFIED PASSED  

---

## Scenario 1 — Basic Robot Visualization & Sensor Synchronization

### Objective:
Verify that the canonical robot description (`gracemo_vira`), TF coordinate tree, static occupancy grid, and multi-modal sensors render accurately inside the embedded RViz2 viewport.

* **Preconditions:**
  - Gazebo Harmonic simulation active on `GZ_PARTITION=gracemo`.
  - Robot description published on `/robot_description`.
  - 360° LiDAR publishing on `/scan`, cameras publishing on `/camera/*`.
* **Action Performed:**
  - Launched UARCC and navigated to Navigation workspace.
  - Inspected `RobotModel`, `TF`, `Grid`, and `LaserScan` displays.
* **Observed Result:**
  - Robot model renders upright at $Z=0.000$m with correct joint orientations.
  - TF tree shows connected chain: `map -> odom -> base_footprint -> base_link -> lidar_link`.
  - Laser scan shows 360° planar beams in clean cyan squares without white blinding.
* **Verdict:** **PASS** (Test ID: `SCENARIO-01`)
* **Evidence:** `docs/validation/gui_functional_testing/04_navigation.png` & `18_corrected_visualization_no_whiteout.png`.

---

## Scenario 2 — Autonomous Waypoint Navigation Dispatch

### Objective:
Verify target destination coordinate resolution, Nav2 BT action goal dispatch, global and local path generation, and real-time coordinator state feedback.

* **Preconditions:**
  - AMCL localization active on `/amcl_pose`.
  - Navigation command coordinator running on `/gracemo_command_coordinator`.
* **Action Performed:**
  - Selected target destination `Kitchen (X=-5.0m, Y=-2.5m, Yaw=0.0 rad)` from destination combo.
  - Clicked `▶ Dispatch Goal`.
* **Observed Result:**
  - Target destination coordinates resolved from authoritative `destinations.json`.
  - Global planner generated Dijkstra/A* path in red.
  - Local DWB controller generated trajectory within costmap clearance corridors.
  - Live distance remaining and coordinator status updated in telemetry cards.
* **Verdict:** **PASS** (Test ID: `SCENARIO-02`)
* **Evidence:** `docs/validation/gui_functional_testing/02_autonomous_operations.png`.

---

## Scenario 3 — Obstacle Handling & Safety Reflex Pipeline

### Objective:
Verify that simulated obstacle proximity activates the safety velocity arbitration pipeline without compromising software safety invariants.

* **Preconditions:**
  - Robot navigating on simulated route.
  - Collision monitor running on `/collision_monitor`.
  - Terrain monitor running on `/gracemo_terrain_monitor`.
* **Action Performed:**
  - Monitored velocity topic chain during proximity to apartment corridor obstacle:
    `Nav2 BT -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smoothed -> collision_monitor -> /cmd_vel_safe -> terrain_monitor -> /cmd_vel`.
* **Observed Result:**
  - Collision monitor detected obstacle within stop buffer.
  - Output velocity scaled down to 0.0 m/s safely.
  - Terrain monitor preserved single-owner invariant to `/cmd_vel`. Zero physical motor actuation attempted (`MOTOR_OUTPUT_ENABLED = 0`).
* **Verdict:** **PASS** (Test ID: `SCENARIO-03`)
* **Evidence:** `docs/validation/gui_functional_testing/11_safety.png`.

---

## Scenario 4 — Voice-Triggered Navigation Interruption ("Hi GraceEMO")

### Objective:
Verify that speaking the wake phrase "Hi GraceEMO" during active navigation safely pauses the mission, halts robot motion, and transitions to conversational listening mode.

* **Preconditions:**
  - Robot executing an active waypoint dispatch.
  - Interaction bridge listening on `/gracemo_interaction_bridge`.
* **Action Performed:**
  - Injected spoken utterance: `"Hi GraceEMO"`.
* **Observed Result:**
  - FastPath matcher identified phrase as `WAKE_PHRASE`.
  - Dispatched semantic action `VoiceInterrupt`.
  - Mission coordinator halted active navigation goal and preserved waypoint queue state.
  - State machine transitioned to `LISTENING`.
* **Verdict:** **PASS** (Test ID: `SCENARIO-04`)
* **Evidence:** `tests/ros2/test_interaction_bridge.py` & `SCENARIO-04` execution log.

---

## Scenario 5 — Voice-Triggered Mission Resumption ("Bye bye GraceEMO")

### Objective:
Verify that speaking "Bye bye GraceEMO" triggers safety invariant checks, replans from the current localized pose, and resumes autonomous mission tracking.

* **Preconditions:**
  - Robot safely halted in conversation mode.
  - Safety gate clear (no E-stop latched).
* **Action Performed:**
  - Injected spoken utterance: `"Bye bye GraceEMO"`.
* **Observed Result:**
  - FastPath matcher identified phrase as `GOODBYE_PHRASE`.
  - Dispatched semantic action `VoiceResume`.
  - Safety gate evaluated prerequisites: confirmed `emergency_stop_latched == False`, verified localization confidence.
  - Mission coordinator dispatched resumed goal to Kitchen from current localized pose.
* **Verdict:** **PASS** (Test ID: `SCENARIO-05`)
* **Evidence:** FastPath match verification & coordinator state log.

---

## Scenario 6 — Middleware Connection Loss Handling & Recovery

### Objective:
Verify that loss of required ROS 2 middleware or Gazebo clock topics triggers appropriate warning indicators without freezing the UARCC GUI.

* **Preconditions:**
  - UARCC running with live telemetry feeds.
* **Action Performed:**
  - Tested watchdog timeout on `/clock` and `/rosout`.
* **Observed Result:**
  - Status indicators transitioned from `READY` to `NOT CONNECTED` upon timeout threshold.
  - GUI main thread remained 100% responsive without blocking or unhandled exceptions.
  - Telemetry badges resumed `READY` immediately upon message reception recovery.
* **Verdict:** **PASS** (Test ID: `SCENARIO-06`)
* **Evidence:** `gracemo_control_center/core/health_monitor.py` timeout test.

---

## Scenario 7 — RViz2 Process Restart & Re-Embedding Idempotency

### Objective:
Verify that restarting the embedded RViz2 process cleans up the existing window container and re-embeds the new window without creating duplicate processes or floating orphans.

* **Preconditions:**
  - ROS 2 navigation stack running continuously.
* **Action Performed:**
  - Triggered `▶ Restart RViz2` from the UARCC banner and executed 5 consecutive re-embed cycles.
* **Observed Result:**
  - Old window handle detached cleanly; process restarted.
  - New window ID detected via robust numerical geometry regex.
  - Window manager decorations stripped via `_MOTIF_WM_HINTS`.
  - Window reparented into `EmbeddedRVizWidget` container with zero desktop orphans.
* **Verdict:** **PASS** (Test ID: `SCENARIO-07`)
* **Evidence:** `docs/validation/gui_functional_testing/17_rviz2_embedding_after_restart.png`.

---

## Scenario 8 — UARCC Application Clean Restart & State Recovery

### Objective:
Verify that closing and restarting the UARCC application results in clean termination of background workers and seamless state recovery without orphaned threads or node conflicts.

* **Preconditions:**
  - Full workstation running.
* **Action Performed:**
  - Dispatched `window.close()`; verified clean shutdown of `SystemMonitorWorker` and `RosManager`.
  - Re-launched UARCC application.
* **Observed Result:**
  - App exit hook dispatched `sys_worker.stop()` and `ros_manager.stop()`.
  - Zero zombie ROS 2 nodes or orphan threads remained in process table.
  - On restart, application read settings, re-connected to ROS 2 graph, and restored full workspace layout.
* **Verdict:** **PASS** (Test ID: `SCENARIO-08`)
* **Evidence:** Clean process exit log (`Returncode: 0`).
