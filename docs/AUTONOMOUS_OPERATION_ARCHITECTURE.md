# GRaCEmo ViRa — Autonomous Operation Architecture

**Document Version:** 1.0.0  
**Status:** Bounded Gazebo autonomy verified; obstacle generalization and full sensor/fault qualification remain open
**Target Platform:** ROS 2 Jazzy Jalisco / Gazebo Harmonic / Ubuntu 24.04 LTS  

---

## 1. System Overview

GRaCEmo ViRa is a unified autonomous mobile robot (AMR) designed for indoor assistive and patrol missions. The autonomy stack bridges high-level cognitive interaction (GraceEMO avatar, semantic fast-path natural language parsing, and event bus telemetry) with deterministic robotics control (Nav2, AMCL, costmaps, velocity smoothing, collision monitoring, and terrain safety gating).

```mermaid
graph TD
    subgraph Cognitive & User Interface Layer
        UARCC[UARCC Control Center\nAutonomous Operations Page]
        Voice[GraceEMO Interaction\nVoice & Pygame Avatar]
        Adapters[Brain / Vision / Voice Adapters]
        Kernel[MNSE Rust Kernel\nEventBus SSE :7780]
    end

    subgraph Mission & Coordination Layer
        Supervisor[Autonomous Supervisor\ncoordinator_node.py\n11-State Authoritative FSM]
    end

    subgraph Nav2 Autonomy Core
        Nav2[Nav2 BT Navigator\nPlanner Server NavfnPlanner\nController Server DWB\nCostmaps & Smoother]
        AMCL[AMCL Localization\n& Map Server]
    end

    subgraph Safety & Command Arbitration Pipeline
        Mux[twist_mux / cmd_vel_mux\nTeleop: 100 | Nav2: 50]
        Smoother[Nav2 Velocity Smoother]
        CollMon[Nav2 Collision Monitor\nFootprint & Safety Stop]
        Terrain[Terrain Safety Monitor\nPitch/Roll/Drop Gate]
    end

    subgraph Simulation Platform
        Gazebo[Gazebo Harmonic\n4-Wheel Differential Drive\nLaser & IMU Sensors]
    end

    UARCC -->|/coordinator/command| Supervisor
    Voice -->|/coordinator/command| Supervisor
    Adapters -->|POST /emit\nActionRequested + trace_id| Kernel
    Kernel -->|SSE /events/live| Voice
    Voice -->|/coordinator/command\nmission_id + trace_id| Supervisor
    Supervisor -->|/autonomy/state, /autonomy/mission_status| UARCC
    Supervisor -->|/coordinator/feedback| Voice
    Supervisor -->|POST /emit\nAutonomyStateChanged + trace_id| Kernel
    Supervisor -->|/navigate_to_pose Action| Nav2
    AMCL -->|/map -> /odom TF| Nav2
    Nav2 -->|/cmd_vel_nav| Mux
    UARCC -->|/cmd_vel_teleop| Mux
    Mux -->|/cmd_vel_smoothed| Smoother
    Smoother --> CollMon
    CollMon -->|/cmd_vel_safe| Terrain
    Terrain -->|/cmd_vel| Gazebo
```

---

## 2. Core Autonomous Subsystems

### 2.1 Autonomous Supervisor (`gracemo_nav2/coordinator_node.py`)
- Acts as the single point of mission authority.
- Manages an authoritative 11-state Finite State Machine (FSM).
- Manages sequential multi-waypoint patrol queues with per-waypoint dwell times.
- Evaluates closed-loop arrival using map pose (0.30 m), Gazebo truth (0.35 m), and map/truth drift (0.15 m) guards.
- Implements a measured-progress watchdog (12 s stagnant-motion threshold).
- Implements bounded retries and costmap recovery (up to 3 attempts, then a reported failure).
- Publishes transient-local latchable telemetry for late-joining UARCC subscribers.

### 2.2 Navigation Core (`Nav2`)
- **Global Planner:** `nav2_navfn_planner::NavfnPlanner` computing Dijkstra/A* paths on global costmap ($20\text{ Hz}$).
- **Local Controller:** `dwb_core::DWBLocalPlanner` executing trajectory generation ($20\text{ Hz}$, max linear $0.4\text{ m/s}$, max angular $0.8\text{ rad/s}$).
- **Costmaps:** 2D occupancy grid global and local costmaps with obstacle, voxel, and inflation layers ($r_{\text{inflation}} = 0.55\text{ m}$, footprint radius $0.35\text{ m}$).
- **Recovery Behaviors:** `nav2_behaviors` providing `Spin`, `BackUp`, and `Wait`.

### 2.3 Localization & State Estimation
- **REP-105 Compliant TF Tree:** `map` $\rightarrow$ `odom` $\rightarrow$ `base_footprint` $\rightarrow$ `base_link` $\rightarrow$ `lidar_link`.
- **AMCL:** Adaptive Monte Carlo Localization with differential drive model, 500–2000 particles, laser beam model.
- **Odometry:** Closed-loop kinematic odometry from Gazebo 4-wheel differential drive plugin (`separation=0.570m`, `radius=0.115m`).

### 2.4 Command Arbitration & Safety Pipeline
A strictly gated 5-stage cascade ensures no unvalidated velocity reaches the drive system:
1. **Arbitration Mux (`twist_mux`):**
   - Teleoperation (`/cmd_vel_teleop`): Priority `100`, timeout `0.5s`
   - Autonomous Nav2 (`/cmd_vel_nav`): Priority `50`, timeout `0.5s`
2. **Velocity Smoother (`nav2_velocity_smoother`):** Bounds linear/angular acceleration ($a_{\text{lin}} \le 1.0\text{ m/s}^2$).
3. **Collision Monitor (`nav2_collision_monitor`):** Multi-zone polygon footprint evaluation directly halting robot if dynamic obstacles penetrate critical zone.
4. **Terrain Safety Gate (`gracemo_terrain_monitor`):** Monolithic final check on IMU pitch/roll thresholds ($|\theta_{\text{pitch}}| \le 0.25\text{ rad}$, $|\phi_{\text{roll}}| \le 0.25\text{ rad}$) and cliff/drop sensors. This node is the **sole authorized publisher** to the hardware/simulation `/cmd_vel`.
5. **Emergency Stop:** Immediate latching override via `/terrain/emergency_stop` forcing deterministic zero velocity.

---

## 3. Communication Topic & Service Matrix

| Topic / Service | Message Type | QoS | Direction / Role |
| :--- | :--- | :--- | :--- |
| `/coordinator/command` | `std_msgs/msg/String` | Reliable | Ingests mission commands (START, PAUSE, RESUME, CANCEL, STOP, RETURN_HOME) |
| `/coordinator/feedback` | `std_msgs/msg/String` | Reliable | Emits state transitions and avatar expressions to GraceEMO |
| `/autonomy/state` | `std_msgs/msg/String` | Transient Local (Durability: Latching) | Current FSM State string for UARCC GUI |
| `/autonomy/mission_status` | `std_msgs/msg/String` | Transient Local (Durability: Latching) | Detailed JSON telemetry (mission name, waypoint index, progress, battery, errors) |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Action | Nav2 goal execution action interface |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Reliable | AMCL pose seeding |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Transient Local | 2D environment map |
| `/odom` | `nav_msgs/msg/Odometry` | Default / 50 Hz | Wheel odometry ground truth |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Default / 20 Hz | Autonomous velocity output from Nav2 |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | Default | Manual teleop override |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Default | Authoritative motor velocity published by Terrain Monitor |
| `/terrain/emergency_stop` | `std_msgs/msg/Bool` | Reliable | Hardware & simulation emergency stop bus |

---

## 4. Hardware Isolation & Safety Invariants

In accordance with strict system governance:
1. `MOTOR_OUTPUT_ENABLED = 0` is hard-coded in `firmware/config.h`. Physical PWM actuation is permanently disengaged.
2. `SerialMCUTransport.PROTOCOL_CONFIRMED = False` in `adapters/robot-bridge/gracemo_bridge/serial_transport.py`. No serial packets can be sent to physical motor controllers.
3. All operations run strictly within the Gazebo Harmonic physics simulation container.

## 5. Adapter / Kernel / ROS Mission Contract

1. Adapter clients emit typed `ActionRequested` events to `POST /emit`. The Rust `Event` keeps the UUID `trace_id` when events are stored and sent over `/events/live`.
2. The ROS interaction bridge consumes the kernel SSE stream and translates navigation actions into `/coordinator/command` messages. It does not publish a competing Nav2 goal when the coordinator is present.
3. The coordinator is the only mission authority. It forwards goals to Nav2 and posts typed `AutonomyStateChanged` events back to `/emit` with the same trace ID.
4. Adapter navigation tools report success only after the coordinator records `MISSION_COMPLETED` for the same trace. A kernel HTTP acknowledgement alone is not a completed navigation.
5. The kernel bridge starts with the simulation and retries its configured EventBus URL, including when the kernel starts after ROS or uses a non-default URL.

For a real bridge-to-Gazebo patrol, build the kernel binary and ROS workspace, start `scripts/launch_full_simulation.sh`, then run `scripts/run_autonomous_mission.py` from a second ROS-sourced terminal. That runner uses actual configured room coordinates and coordinator mission history; it does not simulate travel or invent detections.
