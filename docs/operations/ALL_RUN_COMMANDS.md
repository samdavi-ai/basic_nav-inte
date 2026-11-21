# GRACEMO VIRA — MASTER OPERATIONS MANUAL & ALL RUN COMMANDS

> **Classification:** Engineering Operational Reference Manual  
> **System:** GRaCEmo ViRa Autonomous Humanoid Mobile Robotic Platform  
> **Target Environment:** Ubuntu 24.04 LTS x86_64 / ROS 2 Jazzy Jalisco / Gazebo Harmonic (GZ Sim 8.9.0)  
> **Hardware Status:** Simulation & Hardware-in-the-Loop Safe Mode (`MOTOR_OUTPUT_ENABLED = 0`, `PROTOCOL_CONFIRMED = False`)  
> **Notice:** All routine operating, maintaining, launching, diagnosing, and testing commands must be referenced from this single document.

---

## TABLE OF CONTENTS
1. [SECTION 1 — Environment Setup](#section-1--environment-setup)
2. [SECTION 2 — Build Commands](#section-2--build-commands)
3. [SECTION 3 — Simulation Startup](#section-3--simulation-startup)
4. [SECTION 4 — GRaCEmo Interaction](#section-4--gracemo-interaction)
5. [SECTION 5 — UARCC](#section-5--uarcc)
6. [SECTION 6 — Autonomous Navigation](#section-6--autonomous-navigation)
7. [SECTION 7 — Sensor Diagnostics](#section-7--sensor-diagnostics)
8. [SECTION 8 — System Diagnostics](#section-8--system-diagnostics)
9. [SECTION 9 — Testing](#section-9--testing)
10. [SECTION 10 — Shutdown](#section-10--shutdown)
11. [SECTION 11 — Troubleshooting](#section-11--troubleshooting)

---

## SECTION 1 — Environment Setup

### 1.1 Source Core ROS 2 Jazzy & Gazebo Harmonic Environment
- **Command Purpose:** Initializes all standard ROS 2 Jazzy environment variables, DDS middleware paths, and Gazebo Harmonic transport bindings.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Linux Bash shell, Ubuntu 24.04 LTS
- **Terminal Identifier:** `TERM-ALL` (Prerequisite for all operational terminals)
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
- **Expected Result:** Clean exit without error; `ROS_DISTRO=jazzy` set in shell environment.
- **How to Verify Success:**
```bash
ros2 doctor --report | head -n 12
```
- **How to Stop:** Inactive environment command.

---

### 1.2 Source GRaCEmo Local Workspace Environment
- **Command Purpose:** Adds local ROS 2 colcon build artifacts and packages (`gracemo_description`, `gracemo_gazebo`, `gracemo_nav2`, `gracemo_bridge`) to standard ROS paths.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal initialized with 1.1.
- **Terminal Identifier:** `TERM-ALL`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export GZ_PARTITION=gracemo
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="/home/sabarishchandrasekaran/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_description/share:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_gazebo/share/gracemo_gazebo/models:/opt/ros/jazzy/share"
```
- **Expected Result:** `AMENT_PREFIX_PATH` contains `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install`.
- **How to Verify Success:**
```bash
ros2 pkg list | grep gracemo
```
- **Expected Output:**
```text
gracemo_bridge
gracemo_description
gracemo_gazebo
gracemo_nav2
```
- **How to Stop:** Inactive environment command.

---

### 1.3 Python Dedicated Virtual Environments
- **Command Purpose:** Activates dedicated virtual environments containing dependencies for voice recognition (`faster-whisper`), LLM gateways (`groq`, `google-genai`), and UI components (`PyQt6`).
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Ubuntu Python 3.12 with CUDA 12.x support for NVIDIA RTX 5060 Laptop GPU.
- **Terminal Identifier:** `TERM-INTERACTION` / `TERM-UARCC`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
# Voice & Interaction Virtualenv:
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/vira_interaction/.venv/bin/activate
```
- **Expected Result:** Virtualenv active; prompt prefixed with `(.venv)`.
- **How to Verify Success:**
```bash
python3 -c "import faster_whisper, groq, pygame; print('✓ Voice deps verified')"
```
- **How to Stop:** Execute `deactivate`.

---

### 1.4 Rust Toolchain Environment
- **Command Purpose:** Ensures Rust toolchain (cargo/rustc 1.80+) is active for the Multi-Modal Neural Semantic Engine (MNSE) cognitive kernel.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel`
- **Required Environment:** Standard user PATH containing `~/.cargo/bin`.
- **Terminal Identifier:** `TERM-KERNEL`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source "$HOME/.cargo/env"
```
- **Expected Result:** `cargo` and `rustc` accessible.
- **How to Verify Success:**
```bash
cargo --version && rustc --version
```
- **How to Stop:** Inactive environment command.

---

## SECTION 2 — Build Commands

### 2.1 Complete ROS 2 Workspace Incremental Build
- **Command Purpose:** Compiles all ROS 2 packages (`gracemo_description`, `gracemo_gazebo`, `gracemo_nav2`, `gracemo_bridge`) with symlink installation for rapid iteration.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws`
- **Required Environment:** Initialized with Section 1.1.
- **Terminal Identifier:** `TERM-BUILD`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
- **Expected Result:** Output ending with `Summary: 4 packages finished [X.XXs]`.
- **How to Verify Success:**
```bash
test -f install/setup.bash && echo "BUILD SUCCESS"
```
- **How to Stop:** Send `Ctrl+C` in terminal.

---

### 2.2 Individual Package Targeted Builds
- **Command Purpose:** Rebuilds only a single modified ROS 2 package to save compile time.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws`
- **Required Environment:** Initialized with Section 1.1.
- **Terminal Identifier:** `TERM-BUILD`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
# Rebuild Bridge only:
colcon build --symlink-install --packages-select gracemo_bridge

# Rebuild Nav2 configurations & coordinators only:
colcon build --symlink-install --packages-select gracemo_nav2

# Rebuild Robot URDF/Meshes description only:
colcon build --symlink-install --packages-select gracemo_description

# Rebuild Gazebo worlds and plugins only:
colcon build --symlink-install --packages-select gracemo_gazebo
```
- **Expected Result:** Targeted package compiles with `1 package finished`.
- **How to Verify Success:** Return code `0`.
- **How to Stop:** Send `Ctrl+C`.

---

### 2.3 Rust MNSE Cognitive Kernel Build
- **Command Purpose:** Compiles the high-performance Rust event bus and neural cognitive kernel.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel`
- **Required Environment:** Initialized with Section 1.4.
- **Terminal Identifier:** `TERM-BUILD`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel
# Debug binary:
cargo build

# Release optimized binary:
cargo build --release
```
- **Expected Result:** `target/debug/gracemo-kernel` or `target/release/gracemo-kernel` generated.
- **How to Verify Success:**
```bash
test -x target/debug/gracemo-kernel && echo "KERNEL COMPILED"
```
- **How to Stop:** Send `Ctrl+C`.

---

### 2.4 Clean Rebuild Procedure (DESTRUCTIVE CAUTION)
- **Command Purpose:** Removes all generated build caches, build objects, and install trees to force full recompilation from raw sources.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws`
- **Required Environment:** Initialized with Section 1.1.
- **Terminal Identifier:** `TERM-BUILD`
- **Notice & Warning:** ⚠️ **DESTRUCTIVE EFFECT:** Deletes `ros2_ws/build`, `ros2_ws/install`, `ros2_ws/log`. Requires 2-5 minutes of recompilation time.
- **Status:** `[INFERRED FROM SOURCE CONFIGURATION — REQUIRES APPROVAL BEFORE RUNNING]`
- **Command:**
```bash
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws
rm -rf build install log
colcon build --symlink-install
```
- **Expected Result:** Clean directory reconstituted with fresh artifacts.
- **How to Stop:** Inactive command once executed.

---

## SECTION 3 — Simulation Startup

> [!IMPORTANT]
> Always execute simulation components in the exact numbered order shown below across designated terminals to guarantee deterministic ROS-Gazebo bridge parameter attachment.

### 3.1 Start Rust Cognitive Kernel Daemon
- **Command Purpose:** Boots the local EventBus HTTP REST & SSE broker on port 7780 to receive telemetry and state events.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel`
- **Required Environment:** Standard shell.
- **Terminal Identifier:** `TERM 1: KERNEL`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel
./target/debug/gracemo-kernel
```
- **Expected Result:** Log output `[INFO] Server running on http://127.0.0.1:7780`.
- **How to Verify Success:**
```bash
curl -s http://127.0.0.1:7780/health
```
- **Expected Response:** `{"status":"healthy","uptime_seconds":...}`
- **How to Stop:** Press `Ctrl+C` in `TERM 1`.

---

### 3.2 Launch Complete Gazebo Harmonic Simulation & Nav2 Stack
- **Command Purpose:** Launches Gazebo Harmonic with canonical `smart_home.sdf` world, spawns the verified white-and-orange GRaCEmo ViRa robot model (`canonical_gracemo_fixed.urdf`), launches GZ-ROS bridge, robot state publisher, active neck controllers, Nav2 localization (AMCL) / costmaps / planners / controllers, and RViz2 navigation visualization.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Initialized with Sections 1.1 & 1.2.
- **Terminal Identifier:** `TERM 2: SIMULATION`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export DISPLAY=:0
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=gracemo
export GZ_IP=127.0.0.1
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="/home/sabarishchandrasekaran/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_description/share:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_gazebo/share/gracemo_gazebo/models:/opt/ros/jazzy/share"

ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization
```
- **Expected Result:**
  - Gazebo GUI opens showing smart home apartment and spawned GRaCEmo ViRa robot.
  - RViz2 window opens rendering occupancy grid map, laser scan point cloud, and robot model.
  - Nav2 lifecycle manager transitions to `active`.
- **How to Verify Success:**
```bash
ros2 topic hz /scan /odom /camera/camera_info /tf
```
- **How to Stop:** Send `Ctrl+C` in `TERM 2: SIMULATION`.

---

### 3.3 Launch Standalone Robot Visualization in RViz2
- **Command Purpose:** Renders the robot TF tree and visual geometry independently of Gazebo physics.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Initialized with Sections 1.1 & 1.2.
- **Terminal Identifier:** `TERM-RVIZ`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
ros2 launch gracemo_description display.launch.py
```
- **Expected Result:** RViz2 opens displaying white-and-orange GRaCEmo model with joint sliders.
- **How to Stop:** Send `Ctrl+C` in terminal.

---

## SECTION 4 — GRaCEmo Interaction

### 4.1 Launch GraceEMO Interactive Avatar & Voice Engine
- **Command Purpose:** Boots the full speech and visual interaction subsystem with Pygame avatar, local microphone capture, Adaptive Noise Threshold VAD, Faster-Whisper on NVIDIA RTX 5060 CUDA, Groq/Gemini cloud gateway with local fallback, and Edge-TTS voice synthesis.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/vira_interaction`
- **Required Environment:** Activated virtualenv with Sections 1.1 & 1.3.
- **Terminal Identifier:** `TERM 3: INTERACTION`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/vira_interaction/.venv/bin/activate
export DISPLAY=:0
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration:$PYTHONPATH

cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/vira_interaction
python3 model.py
```
- **Expected Result:**
  - Dedicated 800x600 Pygame window opens showing animated facial expressions (happy, neutral, blinking eyes, mouth movements).
  - Terminal logs `[INFO] Initialized faster-whisper (large-v3-turbo) on NVIDIA RTX 5060 CUDA`.
  - Background noise calibration completes in ~1.0s.
- **How to Verify Success:** Speak *"Hello GRaCEmo"*; verify immediate animated response and TTS voice playback.
- **How to Stop:** Press `Esc` or `q` in Pygame window, or send `Ctrl+C` in `TERM 3`.

---

### 4.2 Inspect Audio Hardware & Microphone Devices
- **Command Purpose:** Diagnoses and lists physical ALSA and PulseAudio capture channels to configure default audio inputs.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Linux Bash shell.
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
# List ALSA recording cards:
arecord -l

# Check default input device volume and status:
pactl get-default-source
pactl get-source-volume @DEFAULT_SOURCE@
```
- **Expected Result:** Hardware capture cards listed without error.
- **How to Stop:** Immediate command.

---

### 4.3 Verify Groq and Gemini API Gateway (Without Leaking Secrets)
- **Command Purpose:** Confirms that cloud LLM gateways are properly configured without printing confidential API tokens to logs.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Virtualenv activated.
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/vira_interaction/.venv/bin/python -c '
import os
groq = "CONFIGURED (" + str(len(os.environ.get("GROQ_API_KEY", ""))) + " chars)" if os.environ.get("GROQ_API_KEY") else "UNSET"
gemini = "CONFIGURED (" + str(len(os.environ.get("GEMINI_API_KEY", ""))) + " chars)" if os.environ.get("GEMINI_API_KEY") else "UNSET"
print(f"Groq API Gateway:   {groq}")
print(f"Gemini API Gateway: {gemini}")
'
```
- **Expected Result:** Displays status (`CONFIGURED` or `UNSET`) without displaying any key characters.
- **How to Stop:** Immediate command.

---

## SECTION 5 — UARCC (Unified Autonomous Robotics Control Center)

### 5.1 Launch UARCC Control Center with Integrated RViz2 Interface
- **Command Purpose:** Boots the professional UARCC desktop engineering workstation with the authentic **ROS 2 RViz2** visualization workspace embedded directly into the central viewport (via native X11 `QWindow.fromWinId()` / `QWidget.createWindowContainer()`), featuring the canonical GRaCEmo ViRa 3D humanoid robot model, live 360° LiDAR scan points, AMCL localization particle cloud, Nav2 global/local costmaps, route trajectory, and interactive 2D Nav Goal tools, alongside all 11 specialized engineering modules.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Sourced ROS 2 Jazzy + PySide6 (`QT_X11_NO_MITSHM=1`, `DISPLAY=:0`).
- **Terminal Identifier:** `TERM 4: UARCC`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH=.:$PYTHONPATH

python3 /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/main.py
```
- **Expected Result:**
  - UARCC launches in full engineering workstation layout (Top Menu & Toolbar, Left Displays Dock, Central Embedded RViz2 Workspace, Right Views & Properties Dock, Bottom Telemetry Bar).
  - Central workspace renders the authentic ROS 2 RViz2 window with canonical config `gracemo_nav2.rviz`.
  - Top banner displays `● RViz2 Embedded (WinID: 0x..., Node: rviz2_nav2)` with action buttons: `[⟳ Re-embed RViz]`, `[▶ Restart RViz2]`, `[Toggle Side Docks]`, `[Toggle Native 3D Viewport]`.
  - Switching module tabs gives instant access to all 11 operational modules (Overview, Autonomous Operations, Simulation, Navigation, Sensors, Localization, Voice, Cognitive, Safety, Diagnostics, Logs, Settings).
- **How to Verify Success:** Observe the 3D white/orange humanoid robot model with rotating cyan visor, LiDAR point cloud, and live Nav2 costmap rendering inside the central UARCC window.
- **How to Stop:** Close window via title bar, or send `Ctrl+C` in `TERM 4`.

---

### 5.2 RViz2 Architecture & Integration Technical Summary
- **Integration Approach:** Approach 3 — Separate RViz2 process with X11 window container reparenting (`QWindow.fromWinId()`).
- **Architectural Rationale:** Installed ROS 2 Jazzy `librviz_common.so` is compiled with **Qt 5.15** (`libQt5Widgets.so.5`), while UARCC uses **PySide6 / Qt 6.12**. In-process C++ linking of Qt5 into a Qt6 runtime causes dynamic symbol collision and fatal process abort (`Cannot mix incompatible Qt library (5.15) with (6.x)`). The containerized X11 reparenting architecture completely isolates address spaces, prevents symbol conflict, maintains hardware OpenGL/Ogre acceleration, and allows dynamic window resizing inside UARCC.
- **Detailed Reference:** See [`docs/architecture/RVIZ2_UARCC_INTEGRATION_ARCHITECTURE.md`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/architecture/RVIZ2_UARCC_INTEGRATION_ARCHITECTURE.md).

---

### 5.3 Run Comprehensive UARCC & RViz Test Suite
- **Command Purpose:** Executes the complete 42-point unit and integration test suite validating all mandatory criteria, RViz interface components, layer visibility toggles, orbit camera geometry, and event bus routing.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Sourced ROS 2 Jazzy environment.
- **Status:** `[VERIFIED ON CURRENT SYSTEM — 42/42 PASSED]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest gracemo_control_center/tests/ -v
```
- **Expected Result:** `42 passed in ~4s`.

---

## SECTION 6 — Autonomous Navigation

> [!CAUTION]
> All navigation commands in this section execute strictly against the Gazebo simulation environment. Never direct navigation outputs to physical motor controllers without physical safety approval.

### 6.1 Inspect Localization Quality & Odometry
- **Command Purpose:** Verifies that AMCL localization is converged and publishing `/amcl_pose` with acceptable covariance.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal with Section 1.1.
- **Terminal Identifier:** `TERM-NAV`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic echo /amcl_pose --once
```
- **Expected Result:** Returns valid `geometry_msgs/msg/PoseWithCovarianceStamped` message with position inside smart home map boundaries.
- **How to Stop:** Auto-terminates after 1 message.

---

### 6.2 Send Simulation-Only Navigation Goal to Semantic Destination
- **Command Purpose:** Commands the robot to autonomously navigate to a semantic destination (e.g. `kitchen`, `living_room`, `bedroom`, `lab`) via the verified `/goal_pose` interface.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal with Section 1.1.
- **Terminal Identifier:** `TERM-NAV`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Commands:**
```bash
# Option A: Via text interaction command (Natural Language grounding):
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "navigate to kitchen"}'

# Option B: Direct PoseStamped to Nav2 goal server (Kitchen: x=-5.0, y=-2.5):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: -5.0, y: -2.5, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```
- **Expected Result:**
  - Nav2 planner computes global path in RViz.
  - Robot wheels begin rotating in Gazebo.
  - Spoken feedback confirms: *"Navigating to Kitchen & Dining."*
- **How to Stop:** See 6.3 below.

---

### 6.3 Safe Navigation Cancellation & Simulation E-Stop
- **Command Purpose:** Instantly cancels active Nav2 navigation trajectories and halts robot motion deterministically via safety latching.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal with Section 1.1.
- **Terminal Identifier:** `TERM-NAV`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Commands:**
```bash
# Emergency Stop Broadcast:
ros2 topic pub --once /terrain/emergency_stop std_msgs/msg/Bool '{data: true}'

# Or voice/command stop:
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "stop"}'
```
- **Expected Result:** Velocity drops immediately to 0.0 m/s; Nav2 behavior tree cancels goal.
- **How to Stop:** Immediate command.

---

### 6.4 Voice-Triggered Navigation Interruption ("Hi GraceEMO") & Resumption ("Bye bye GraceEMO")
- **Command Purpose:** Validates the bidirectional voice interruption protocol during an active navigation mission:
  1. Spoken wake phrase *"Hi GraceEMO"* halts motion safely, preserves the current waypoint mission, and transitions the robot into conversational listening mode.
  2. Spoken goodbye phrase *"Bye bye GraceEMO"* verifies safety state and localization readiness, replans from the current localized pose, and resumes autonomous mission navigation.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal initialized with Section 1.1 + Section 1.2.
- **Terminal Identifier:** `TERM-NAV`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Commands:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 1. Start active waypoint mission to Kitchen:
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "take me to the kitchen"}'

# 2. Issue Voice Interruption Wake Phrase:
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "Hi GraceEMO"}'
# Verify: Robot halts, spoken feedback confirms pause ("Hello! I have safely paused navigation and am listening. How can I help you?"), and cognitive state transitions to LISTENING.

# 3. Issue Voice Resumption Goodbye Phrase:
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "Bye bye GraceEMO"}'
# Verify: Robot confirms resumption ("Goodbye! Resuming my mission now."), replans trajectory from localized pose, and resumes waypoint transit.
```
- **Safety Interlock Verification:**
```bash
# If Emergency Stop is active, voice resumption is blocked:
ros2 topic pub --once /terrain/emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /interaction/command std_msgs/msg/String '{data: "Bye bye GraceEMO"}'
# Verify: Resumption rejected with warning: "Safety fault active. Cannot resume navigation."
```
- **How to Stop:** Immediate command.

---

### 6.5 Reset Emergency Stop
- **Command Purpose:** Releases the emergency stop latch to allow subsequent autonomous actions.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Required Environment:** Terminal with Section 1.1.
- **Terminal Identifier:** `TERM-NAV`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Command:**
```bash
ros2 topic pub --once /terrain/emergency_stop std_msgs/msg/Bool '{data: false}'
```
- **Expected Result:** Robot returns to `IDLE` state ready for new commands.
- **How to Stop:** Immediate command.

---

## SECTION 7 — Sensor Diagnostics

### 7.1 Inspect 2D LiDAR Scan Stream
- **Command Purpose:** Verifies update frequency, range bounds, and point density from the 360-degree GPU ray lidar.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 topic hz /scan
```
- **Expected Result:** Average rate: ~40.0 Hz to 45.0 Hz.
- **How to Stop:** Press `Ctrl+C`.

---

### 7.2 Inspect Camera Image & Depth Telemetry
- **Command Purpose:** Verifies RGB camera and Depth perception channels streaming into ROS 2.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Commands:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Check Camera Info resolution (640x480):
ros2 topic echo /camera/camera_info --once

# Check Camera Frame Rate:
ros2 topic hz /camera/image_raw
```
- **Expected Result:** Resolution: 640x480; Rate: ~30 Hz.
- **How to Stop:** Press `Ctrl+C`.

---

### 7.3 Inspect IMU & Wheel Odometry
- **Command Purpose:** Verifies IMU angular velocity/linear acceleration and wheel odometry poses.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM — SIMULATION-ONLY]`
- **Commands:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 topic echo /imu/data --once
ros2 topic hz /odom
```
- **Expected Result:** IMU returns orientation quaternion; `/odom` rate: ~45 Hz.
- **How to Stop:** Press `Ctrl+C`.

---

## SECTION 8 — System Diagnostics

### 8.1 Complete ROS 2 Node & Topic Graph Audit
- **Command Purpose:** Inspects the active ROS 2 graph to verify no duplicate publishers or orphaned nodes exist.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

echo "=== ACTIVE NODES ==="
ros2 node list

echo "=== TOPIC PUBLISHER / SUBSCRIBER OWNERSHIP ==="
ros2 topic info -v /cmd_vel
ros2 topic info -v /scan
ros2 topic info -v /odom
```
- **Expected Result:**
  - Exactly 1 publisher on `/cmd_vel` (`gracemo_terrain_monitor`).
  - Exactly 1 subscriber on `/cmd_vel` (`gracemo_sensor_bridge`).
- **How to Stop:** Immediate command.

---

### 8.2 Monitor CPU, Memory & NVIDIA RTX GPU Usage
- **Command Purpose:** Monitors system resource consumption across all robotics processes.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-DIAG`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
# GPU Telemetry & VRAM consumption:
nvidia-smi

# Process CPU & Memory usage:
top -b -n 1 | grep -E "gz|ros2|python3|gracemo"
```
- **Expected Result:** GPU VRAM usage under 4 GB; CPU total load balanced.
- **How to Stop:** Immediate command.

---

## SECTION 9 — Testing

### 9.1 Fast Unit & Safety Invariant Test Suites
- **Command Purpose:** Runs the complete suite of local unit tests verifying MCU protocol parsers, state machines, kinematics, safety gates, and noise rejection algorithms.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-TEST`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
# 1. MCU Protocol Engine & Unit Tests (38 tests):
python3 -m unittest discover -s tests/unit -p "test_*.py"

# 2. Safety Gate & Invariant Tests (16 tests):
python3 -m unittest discover -s tests/safety -p "test_*.py"

# 3. Pre-STT Noise Rejection Unit Tests (16 tests):
vira_interaction/.venv/bin/python -m unittest vira_interaction/tests/test_pre_stt_noise_rejection.py

# 4. Robot Model 3D Widget Tests (6 tests):
PYTHONPATH=gracemo_control_center:$PYTHONPATH python3 gracemo_control_center/tests/test_robot_model_widget.py
```
- **Expected Result:** All tests return `OK`.
- **How to Stop:** Immediate test execution.

---

### 9.2 ROS 2 & Pipeline Integration Tests
- **Command Purpose:** Verifies ROS 2 interaction bridge, velocity pipeline mux, and simulation parity.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-TEST`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Commands:**
```bash
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 1. Voice Interruption & Interaction Bridge Test Suite (7 tests):
pytest -v tests/ros2/test_interaction_bridge.py

# 2. Velocity Pipeline & Command Velocity Mux Test Suite (10 tests):
pytest -v tests/ros2/test_velocity_pipeline_and_mux.py

# 3. Safety Gate & Fast-Path Invariant Test Suite (12 tests):
pytest -v tests/safety/test_safety_gate.py tests/unit/test_fast_path.py

# 4. Motion Command Ownership & Single-Publisher Invariant (3 checks):
python3 tests/test_motion_command_ownership.py

# 5. Simulation Parity & Canonical Robot Description (4 tests):
python3 tests/simulation/test_simulation_parity.py

# 6. Nav2 Autonomous Operations Hierarchy (11 tests):
python3 -m pytest -o pythonpath='/opt/ros/jazzy/lib/python3.12/site-packages ros2_ws/src/gracemo_bridge .' ros2_ws/src/gracemo_nav2/test/test_autonomous_operations.py
```
- **Expected Result:** 100% of tests pass cleanly.
- **How to Stop:** Immediate test execution.

---

### 9.3 UARCC Comprehensive 30-Point Acceptance Suite
- **Command Purpose:** Runs the end-to-end 30-point acceptance test suite covering all GUI pages, widgets, event buses, telemetry parsers, and reconnection handlers.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`
- **Terminal Identifier:** `TERM-TEST`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest -o pythonpath='/opt/ros/jazzy/lib/python3.12/site-packages /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration' gracemo_control_center/tests/test_uarcc_comprehensive.py
```
- **Expected Result:** `30 passed in 3.61s`.
- **How to Stop:** Immediate test execution.

---

### 9.4 Rust MNSE Cognitive Kernel Unit & Benchmark Tests
- **Command Purpose:** Executes native Rust tests for context compilers, semantic spatial graph queries, and token benchmarks.
- **Working Directory:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel`
- **Terminal Identifier:** `TERM-TEST`
- **Status:** `[VERIFIED ON CURRENT SYSTEM]`
- **Command:**
```bash
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/kernel
cargo test
```
- **Expected Result:** `16 passed; 0 failed; finished in 0.20s`.
- **How to Stop:** Immediate test execution.

---

## SECTION 10 — Shutdown

### 10.1 Orderly System Shutdown Sequence
Follow this exact descending shutdown sequence to cleanly disconnect ROS 2 nodes, flush state caches, and avoid zombie background processes:

1. **Step 1: Cancel Active Navigation & Safe Halt**
   Send zero velocity and halt any active behavior tree goals:
   ```bash
   ros2 topic pub --once /terrain/emergency_stop std_msgs/msg/Bool '{data: true}'
   ```
2. **Step 2: Terminate UARCC Control Center**
   Close the UARCC desktop window or send `SIGINT` (`Ctrl+C`) in `TERM 4: UARCC`.
3. **Step 3: Terminate GraceEMO Interaction Avatar**
   Press `Esc` in the Pygame window or send `SIGINT` (`Ctrl+C`) in `TERM 3: INTERACTION`.
4. **Step 4: Terminate Gazebo Simulation & Nav2 Stack**
   Send `SIGINT` (`Ctrl+C`) in `TERM 2: SIMULATION`. Allow up to 10 seconds for Nav2 lifecycle managers to transition down cleanly.
5. **Step 5: Terminate Rust Cognitive Kernel**
   Send `SIGINT` (`Ctrl+C`) in `TERM 1: KERNEL`.
6. **Step 6: Flush Dangling ROS 2 Daemons & Gazebo Servers**
   If any orphaned processes remain:
   ```bash
   pkill -f "ros2 launch gracemo_gazebo" || true
   pkill -f "gz sim" || true
   pkill -f "gracemo-kernel" || true
   ```

---

## SECTION 11 — Troubleshooting

### 11.1 Problem: `ModuleNotFoundError: No module named 'rclpy'`
- **Root Cause:** Sourcing `/opt/ros/jazzy/setup.bash` sets python paths for `/usr/bin/python3`, but running inside isolated virtualenvs or using bare `pytest` does not inherit `/opt/ros/jazzy/lib/python3.12/site-packages`.
- **Solution:** Always pass ROS python paths explicitly when executing pytest or virtualenv scripts:
```bash
python3 -m pytest -o pythonpath='/opt/ros/jazzy/lib/python3.12/site-packages .' <test_file>
```

### 11.2 Problem: ROS 2 Nodes Running in Background Not Discovered by Terminal
- **Root Cause:** By default, FastDDS discovery can span multiple subnets or fail across local child subshells if discovery range is not pinned.
- **Solution:** Export these variables in every terminal before running ROS 2 commands:
```bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

### 11.3 Problem: Gazebo Harmonic Fails to Find Meshes (`canonical_gracemo_fixed.urdf`)
- **Root Cause:** `GZ_SIM_RESOURCE_PATH` missing local workspace install paths.
- **Solution:** Export resource paths before launching simulation:
```bash
export GZ_SIM_RESOURCE_PATH="/home/sabarishchandrasekaran/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_description/share:/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_gazebo/share/gracemo_gazebo/models:/opt/ros/jazzy/share"
```

### 11.4 Problem: Multiple Publishers Detected on `/cmd_vel`
- **Root Cause:** Defect where a teleop bridge or coordinator node published directly to `/cmd_vel` instead of routing through `/cmd_vel_teleop`.
- **Solution:** Verify `test_motion_command_ownership.py`. Ensure only `gracemo_terrain_monitor` publishes to `/cmd_vel`.

### 11.5 Problem: Microphone VAD Triggering on Silent Ambient Noise
- **Root Cause:** High room ambient noise exceeding static RMS thresholds.
- **Solution:** Ensure `vira_interaction/model.py` runs with adaptive background noise calibration (calculates dynamic floor $1.35 \times \text{noise\_floor} + 250$). Verify with `tests/test_pre_stt_noise_rejection.py`.

---
*End of Master Operations Manual. Reference this document for all routine commands.*
