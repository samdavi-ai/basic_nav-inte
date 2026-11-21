# GRaCEmo ViRa — Autonomous AI Robot System

> **Event-Driven Nervous System for Autonomous Robotics & Multimodal AI**

---

## 🧭 System Overview

GRaCEmo ViRa decouples high-level reasoning and physical body control using an **event-driven nervous system** pattern:

```
┌─────────────────────────────────────────────────────────────┐
│                       AI REASONING                          │
│               (LLMs / Task Planners / Memory)               │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                 GRaCEmo KERNEL (Rust)                       │
│     • Tokio EventBus (Broadcast & Priority Queues)          │
│     • Append-Only SQLite Ledger (Causal Event Store)        │
│     • Relational Knowledge Graph (Entity/State Tracking)    │
│     • High-Speed HTTP/SSE & Unix Socket API                 │
└───────▲──────────────────────▲───────────────────────┬──────┘
        │                      │                       │
        │ Event Stream         │ Event Stream          │ Action Dispatch
        │                      │                       │ (navigate, speak, look)
┌───────┴──────────────┐ ┌─────┴────────────────┐ ┌────▼──────────────┐
│   VISION ADAPTER     │ │   VOICE ADAPTER      │ │   ROBOT BRIDGE    │
│   (YOLO / YuNet)     │ │ (Whisper / Kokoro)   │ │  (ROS2 / Hardware)│
└──────────────────────┘ └──────────────────────┘ └───────────────────┘
```

---

## 📁 Repository Layout

- `kernel/` — Core Rust workspace: EventBus, types, ledger, graph, and MCP interfaces.
- `ros2_ws/src/gracemo_description/` — Robot URDF/Xacro, links, joints, and Gazebo plugins.
- `ros2_ws/src/gracemo_gazebo/` — Gazebo Harmonic worlds and ROS launch files.
- `ros2_ws/src/gracemo_nav2/` — Nav2, localization, costmaps, velocity arbitration, and mission coordinator.
- `ros2_ws/src/gracemo_bridge/` — Gazebo sensor and higher-level ROS bridges.
- `gracemo_control_center/` — Desktop control and monitoring application.
- `vira_interaction/` — Voice and interaction services.
- `config/` — Declarative TOML/YAML runtime configurations.
- `docs/` — Architecture documentation, hardware interfaces, and API references.

## 🤖 ROS 2 + Gazebo Navigation

Prerequisites: ROS 2 Jazzy, Gazebo Harmonic, and the ROS/Gazebo packages declared by the workspace.

Build the navigation stack from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --packages-up-to gracemo_gazebo gracemo_nav2 gracemo_rl gracemo_bridge
cd ..
python3 scripts/check_simulation_model_consistency.py
```

Start the simulator and Nav2 in terminal 1:

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=vira-navigation
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization headless:=true use_rviz:=false launch_interaction:=false use_rl:=false
```

Run a bounded navigation check from terminal 2 with the same `ROS_DOMAIN_ID` and `GZ_PARTITION`:

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=vira-navigation
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/run_gazebo_navigation_smoke.py --goal-x 0.9 --goal-y 0.0
```

The smoke check requires advancing `/clock`, fresh scan/odometry/IMU data, a ready Nav2 stack, and no active stop before it sends a goal. Stop the simulator with Ctrl+C in terminal 1. Gazebo's command configuration path is set by the launch file when the shell has not set one.

For a repeatable multi-room navigation run, replace the smoke command in terminal 2 with:

```bash
python3 scripts/run_gate_e_50_missions.py --missions 50 --seed 42 \
  --out docs/validation/gate_e_50_missions_2026-11-21_final_arrival_guard.json
```

This campaign checks readiness before moving and records each mission against Gazebo ground truth. It uses the apartment's fixed map and static obstacles; it does not test randomized or moving obstacles.

To validate the simulated emergency stop, mission rejection while stopped, and explicit reset:

```bash
python3 scripts/run_gazebo_estop_smoke.py \
  --out docs/validation/gazebo_estop_smoke_2026-11-21.json
```

Run this only with the Gazebo stack active. It keeps E-stop latched if any safety check fails.

For the full desktop launcher, use `./scripts/launch_full_simulation.sh`; it opens the Gazebo GUI and stops only process groups started by that launcher.

### Full adapter-to-Gazebo patrol

Build the kernel daemon if it is not already available:

```bash
cargo build --manifest-path kernel/Cargo.toml --bin gracemo-kernel
```

Start the standalone Kernel, Gazebo, Nav2, and ROS bridge stack in terminal 1:

```bash
./scripts/launch_full_simulation.sh
```

In terminal 2, source the same ROS domain and dispatch the real five-room mission through the Kernel EventBus:

```bash
export ROS_DOMAIN_ID=68
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/run_autonomous_mission.py --timeout 300 \
  --out docs/validation/adapter_kernel_gazebo_patrol.json
```

The mission runner waits for the coordinator's matching mission-history result. It exits nonzero on kernel rejection, coordinator failure, or timeout. Its success report reflects completed Gazebo navigation; it does not synthesize travel time or vision detections.

The DWB controller is the default. Keep `use_rl:=false` until the RL policy passes the same repeatable full-mission acceptance gate. Current measured status and remaining work are in [`docs/PROJECT_GAPS_AND_SIMULATION_STATUS.md`](docs/PROJECT_GAPS_AND_SIMULATION_STATUS.md).


