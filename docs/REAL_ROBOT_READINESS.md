# GRaCEmo ViRa — Real-Robot Shadow Mode & Deployment Readiness Document
**Document ID:** GRACEMO-DOC-PHASE12-READINESS  
**Target Platform:** GRaCEmo ViRa (Ubuntu 24.04 LTS, ROS 2 Jazzy, STM32 MCU, Livox/RPLiDAR S2)  
**Safety Classification:** Strictly Hardware-Locked Simulation & Shadow Testing  
**Status:** READY FOR HARDWARE SHADOWING (PHYSICAL ACTUATION INHIBITED)  

---

## 1. Executive Summary & Safety Invariant Locks

Physical deployment of autonomous navigation policies requires provable mathematical safety, multi-tiered watchdogs, and deterministic arbitration. As mandated by project safety constraints, all experimental verification conducted on `feature/rl-local-planner` adheres to hard invariants:

| Subsystem | Gate / Invariant | Status | Verification Mechanism |
| :--- | :--- | :--- | :--- |
| **Firmware Actuation Lock** | `MOTOR_OUTPUT_ENABLED = 0` | 🔒 LOCKED | Compile-time assertion in `firmware/config.h` & `firmware/motor_interface.h` |
| **Transport Protocol Gate** | `PROTOCOL_CONFIRMED = False` | 🔒 LOCKED | Runtime assertion in `ros2_ws/src/gracemo_hardware/gracemo_hardware/mcu_transport.py` |
| **Collision Shield** | `nav2_collision_monitor` | 🛡️ ACTIVE | Immutable polygons (`PolygonStop`, `PolygonSlow`); zero relaxation |
| **Default Trajectory Tracker** | DWB Local Planner | 🎯 DEFAULT | RL policy is opt-in via launch argument (`use_rl:=true`) |
| **Sim Ground Truth Isolation** | `sim_ground_truth:=false` | 🛡️ ENFORCED | Real robot uses TF `/map -> /base_footprint` arrival gates only |

---

## 2. Real-Robot Velocity Pipeline & Safety Architecture

The velocity pipeline enforces a layered hierarchy preventing unsanctioned or erratic velocity commands from reaching the motor controllers:

```
[ Nav2 Global Plan / Received Path ]
               │
               ▼
┌───────────────────────────────┐
│     Trajectory Generators     │
├───────────────┬───────────────┤
│ DWB Controller│  RL Policy    │
│ (/cmd_vel_nav)│ (/cmd_vel_rl) │
└───────┬───────┴───────┬───────┘
        │               │
        └───────┬───────┘
                ▼
┌───────────────────────────────┐
│       cmd_vel_mux Node        │◄── Priority 1: /cmd_vel_teleop (Recovery / E-Stop)
│ (Priority Arbitration Engine) │    Priority 2: /cmd_vel_rl (Opt-in)
└───────────────┬───────────────┘    Priority 3: /cmd_vel_nav (Default)
                ▼ /cmd_vel_mux
┌───────────────────────────────┐
│    nav2_velocity_smoother     │◄── Acceleration Limiting (a_v <= 1.50 m/s², a_w <= 3.20 rad/s²)
└───────────────┬───────────────┘
                ▼ /cmd_vel_smoothed
┌───────────────────────────────┐
│    nav2_collision_monitor     │◄── Direct Sensor Guard (LiDAR & Depth Cloud)
│ (PolygonSlow / PolygonStop)   │    Zero-velocity safety override
└───────────────┬───────────────┘
                ▼ /cmd_vel_safe
┌───────────────────────────────┐
│   gracemo_hardware (MCU)      │◄── MOTOR_OUTPUT_ENABLED = 0 (Actuation Locked)
└───────────────────────────────┘
```

---

## 3. Shadow Mode Operation Protocol

Shadow mode executes the full reinforcement learning local navigation pipeline on the live robot while the physical actuators are disconnected or commanded exclusively by the safety-certified baseline (DWB) or teleoperation:

### 3.1 Execution Steps
1. **Pre-flight Lock Verification:**
   Run `scripts/run_shadow_mode_comparator.py`. It confirms:
   - `MOTOR_OUTPUT_ENABLED == 0`
   - `PROTOCOL_CONFIRMED == False`
2. **Dual Trajectory Evaluation:**
   Both DWB and RL policies receive the identical 79-D observation vector derived from canonical `/scan`, `/odom`, and `/plan` topics.
3. **Discrepancy Logging:**
   The comparator logs at 10 Hz:
   $$\Delta v = |v_{\text{DWB}} - v_{\text{RL}}|, \quad \Delta \omega = |\omega_{\text{DWB}} - \omega_{\text{RL}}|$$
   along with sign agreement and distance to obstacles.
4. **Intervention Triggers:**
   If the RL policy commands a trajectory that breaches the `PolygonSlow` envelope while DWB remains outside, the event is flagged as a safety divergence.

---

## 4. Deterministic Rosbag Recording & Replay

To allow complete offline verification without occupying the physical robot or simulator:
- **Recording:**
  ```bash
  python3 scripts/rosbag_tool.py record --output rosbags --duration 60
  ```
  Captures: `/scan`, `/odom`, `/tf`, `/tf_static`, `/plan`, `/cmd_vel_nav`, `/cmd_vel_rl`, `/cmd_vel_mux`, `/cmd_vel_safe`, `/collision_monitor_state`, `/autonomy/state`.
- **Replay:**
  ```bash
  python3 scripts/rosbag_tool.py play rosbags/gracemo_bag_<TIMESTAMP> --rate 1.0
  ```
  Enables running new ONNX checkpoints against identical real-world telemetry frames.

---

## 5. Sim-to-Real Physical Deployment Gates

Prior to unlatching physical motor actuation (`MOTOR_OUTPUT_ENABLED = 1`), the following formal checklist MUST be passed:

- [x] **Consolidated Observation Parity:** Zero skew between inference and training (`test_parity_and_consolidation.py` 13/13 PASS).
- [x] **Safety Monitor Integrity:** `PolygonStop` and `PolygonSlow` active with verified deceleration profiles.
- [x] **Localization Health Monitor:** Active covariance tracking and jump detection (< 3.0s detection, safe in-place recovery verified).
- [x] **Obstacle Distance Speed Limiter:** Integrated in `rl_local_planner.py` with smooth deceleration scaling.
- [ ] **Physical Shadow Campaign:** $\ge 5$ hours of zero-intervention shadow mode on the physical robot with DWB driving.
- [ ] **Physical Sensor Validation:** Livox / RPLiDAR S2 beam noise and reflective ground clutter calibrated to $\sigma \le 0.02\text{m}$.
- [ ] **Formal Signoff:** Explicit human signoff required before updating `config.h` and `mcu_transport.py`.

