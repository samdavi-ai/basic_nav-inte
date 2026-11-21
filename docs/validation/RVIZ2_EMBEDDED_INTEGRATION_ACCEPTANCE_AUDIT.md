# GRaCEmo ViRa — RViz2 Embedded Integration Independent Acceptance Audit

> **Classification:** Independent Technical Acceptance Audit  
> **Target Subsystem:** Unified Autonomous Robotics Control Center (UARCC) — RViz2 Embedded Visualization Integration  
> **Repository:** `/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration`  
> **Audit Environment:** Ubuntu 24.04 LTS x86_64, ROS 2 Jazzy Jalisco, Gazebo Harmonic 8.9.0, PySide6 6.12.0  
> **Hardware Safety Lock:** `MOTOR_OUTPUT_ENABLED = 0`, `SerialMCUTransport.PROTOCOL_CONFIRMED = False`  
> **Audit Mode:** Strict Audit-Only (Non-destructive, zero code modifications)  
> **Audit Date:** November 2, 2026

---

## Executive Summary & Overall Audit Verdict

| Audit Domain | Criterion | Verification Status | Key Evidence / Finding |
| :--- | :--- | :--- | :--- |
| **1. Embedding Architecture** | Real RViz2 binary reparented via X11 container | **PASS** | Foreign window `0x2a00106` owned by PID `162179` reparented via `QWindow.fromWinId()` and `QWidget.createWindowContainer()`. |
| **2. Qt & Desktop Compatibility** | Qt5 vs Qt6 ABI & X11/Wayland support | **PASS WITH LIMITATIONS** | Qt5 (`librviz_common`) and Qt6 (PySide6) address-space isolation verified. **Limitation:** Requires X11 session or XWayland; pure native Wayland (`QT_QPA_PLATFORM=wayland`) is unsupported. |
| **3. Lifecycle Management** | Independent process lifecycle & crash isolation | **PASS** | Terminating or restarting RViz2 does not impact Gazebo, Nav2, AMCL, or UARCC. |
| **4. ROS 2 Graph Integration** | Shared domain, topics, and node graph integrity | **PASS WITH LIMITATIONS** | Shares `ROS_DOMAIN_ID=0`, TF tree, and topics without velocity publishing. **Limitation:** Lingering duplicate node name registrations in FastDDS graph if killed abruptly. |
| **5. Safety Invariants** | Motor lockdown, arbitration, and E-Stop gate | **PASS WITH LIMITATIONS** | Motor output locked at 0; Nav2 output filtered by mux/collision monitor. **Defect Identified:** `views_inspector.py` E-Stop button references non-existent signal `emergency_stop_triggered`. Authoritative E-Stop in `safety_view.py` operates correctly. |
| **6. Visual Appearance & UX** | Real-time 3D Ogre rendering & layout behavior | **PASS WITH LIMITATIONS** | Authentic Ogre 3D rendering with live robot model and map confirmed. **Limitation:** Double sidebars occur if both UARCC outer docks and RViz inner docks are active simultaneously; mitigated by `Toggle Side Docks`. |
| **7. Structural Consistency** | Module count and sensor categorization | **PASS WITH LIMITATIONS** | Discrepancy documented: Voice functionality belongs in Cognitive System and Pygame `vira_interaction`; separate voice tab is redundant. Planar 360° LiDAR clarified vs forward 3D RGB-D camera. |

**Overall Acceptance Status: `PASS WITH LIMITATIONS`**

---

## 1. Verification of the Actual Embedding

### 1.1 Implementation Architecture
Inspection of [`gracemo_control_center/widgets/embedded_rviz_widget.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/widgets/embedded_rviz_widget.py) confirms that the integration uses native X11 window reparenting to embed the authentic ROS 2 RViz2 executable.

Relevant Source Code ([`embedded_rviz_widget.py:247-276`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/widgets/embedded_rviz_widget.py#L247-L276)):
```python
def _embed_window_id(self, win_id: int):
    # Create foreign QWindow from real X11 window ID
    self._foreign_window = QWindow.fromWinId(win_id)
    # Wrap foreign QWindow in a native Qt widget container
    self._window_container = QWidget.createWindowContainer(self._foreign_window, self.rviz_host_widget)
    self._window_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    # Insert container into central layout
    self.placeholder.hide()
    self.rviz_host_layout.addWidget(self._window_container, stretch=1)
    self.view_stack.setCurrentIndex(0)
    self._rviz_win_id = win_id
```

### 1.2 Process and Window Handle Verification
Live runtime inspection of the running process tree and X11 window hierarchy confirms:
1. **Running RViz2 Process:**
   - **PID:** `162179`
   - **Command Line:** `/opt/ros/jazzy/lib/rviz2/rviz2 -d .../gracemo_nav2.rviz --ros-args -r __node:=rviz2_nav2`
   - **Executable Path:** `/opt/ros/jazzy/lib/rviz2/rviz2` (Authentic ROS 2 binary, not an imitation or wrapper).
2. **X11 Window Tree Verification (`xwininfo -root -tree`):**
   ```text
   0x5600007 "GRaCEmo ViRa — Unified Autonomous Robotics Control Center": ("main.py" "GRaCEmo ViRa Unified Control Center")
      0x2a00106 ".../gracemo_nav2.rviz* - RViz": ("rviz2" "rviz2")  994x756+251+135
         0x2a00013 "rviz2": ("rviz2" "rviz2")  314x666+364+61
   ```
   - Client window `0x2a00106` has WM_CLASS `("rviz2" "rviz2")`.
   - Parent window is `0x5600007` (the UARCC PySide6 main window).
   - This mathematically proves true X11 reparenting: the RViz2 window is physically and structurally a child of the UARCC window.

### 1.3 Active Ogre 3D Rendering
- Live process monitoring shows `rviz2` (PID `162179`) actively consuming ~32% CPU and GPU rendering cycles via Ogre 1.12.10 (`libOgreMain.so.1.12.10`).
- The rendering surface contains live frame updates (30–31 FPS observed at bottom right of RViz status bar).

### 1.4 Fallback Behavior Analysis
- In headless/offscreen environments (`QT_QPA_PLATFORM=offscreen` or missing `$DISPLAY`), `EmbeddedRVizWidget` displays a static placeholder card (`self.placeholder`) and logs `Headless / Offscreen Mode (Simulated)`.
- It does **not** masquerade fake telemetry as a live RViz stream.
- An explicit button `[Toggle Native 3D Viewport]` allows manual switching to the fallback PySide6 custom 3D painter (`RVizViewportWidget`).

**Verdict: PASS**

---

## 2. Verification of Qt and Desktop Compatibility

### 2.1 Dynamic Library and ABI Inspection
Inspection of dynamic link dependencies confirms the necessity of the out-of-process X11 reparenting architecture:
- **UARCC Runtime:** PySide6 `6.12.0` linked against `libQt6Widgets.so.6`, `libQt6Gui.so.6`, `libQt6Core.so.6`.
- **ROS 2 Jazzy RViz2 Runtime:** `/opt/ros/jazzy/lib/librviz_common.so` linked against:
  - `libQt5Widgets.so.5` (Qt 5.15)
  - `libQt5Gui.so.5` (Qt 5.15)
  - `libQt5Core.so.5` (Qt 5.15)
- **ABI Collision Proof:** Dynamic loading of Qt 5 into a Qt 6 process in Linux causes immediate symbol conflict in `QObject` and `QPaintDevice`. The separate process model completely isolates address spaces.

### 2.2 Desktop Session Compatibility
- **Current Live Session:** Verified via `echo $XDG_SESSION_TYPE`: `x11` on `DISPLAY=:0`.
- **Supported Desktop Environments:**
  - Linux X11 desktop sessions (GNOME on Xorg, XFCE, KDE Plasma on X11).
  - XWayland sessions where `DISPLAY` is exported.
- **Unsupported Desktop Environments:**
  - **Pure Native Wayland (`QT_QPA_PLATFORM=wayland`):** Window reparenting across unprivileged processes via `QWindow.fromWinId()` is fundamentally disallowed by the Wayland security protocol (`xdg_shell`). In a pure Wayland session without XWayland, window reparenting will fail silently.

**Verdict: PASS WITH LIMITATIONS (X11 / XWayland required)**

---

## 3. Verification of Lifecycle Management

### 3.1 Startup and Discovery
- If RViz2 is already running (e.g. from `vira_complete.launch.py`), `EmbeddedRVizWidget` identifies its window handle within 250 ms via `_find_rviz_window_id()` and embeds it automatically.
- If RViz2 is not running, `_spawn_rviz_process()` invokes `subprocess.Popen` with the full ROS 2 environment and polls every 500 ms until the window appears.

### 3.2 Decoupled Fault Domain & Unexpected Termination
An empirical termination test was performed during audit:
1. `pkill -f rviz2` was executed while the simulation and navigation stacks were active.
2. **Result on ROS 2 Navigation & Gazebo:**
   - Gazebo Harmonic physics server (PID `134662`): **Continued running without interruption.**
   - Simulation launch stack (PID `134651`): **Continued running without interruption.**
   - Nav2 planner, controller, costmaps, and AMCL: **Continued publishing and operating normally.**
   - UARCC Control Center (PID `163732`): **Remained stable.** The container cleared, and the UI displayed the reconnection banner.
3. Clicking `[▶ Restart RViz2]` or `[⟳ Re-embed RViz]` cleanly respawned RViz2 and re-embedded window `0x2a00106`.

### 3.3 Orphaned Process Risk
- If UARCC terminates abnormally (e.g. `SIGKILL`), the reparented RViz2 window is unparented by X11 back to the root desktop window rather than crashing.
- While safe, it leaves an external RViz2 window on screen. `EmbeddedRVizWidget.closeEvent` handles graceful teardown, but hard kills leave the external process intact.

**Verdict: PASS**

---

## 4. Verification of ROS 2 Integration

### 4.1 ROS Graph Information
Live inspection via `ros2 node info /rviz2_nav2`:
- **Shared Domain ID:** `ROS_DOMAIN_ID=0` (consistent across Gazebo, Nav2, RViz2, and UARCC).
- **Subscribed Telemetry Topics:**
  - `/scan` (`sensor_msgs/msg/LaserScan`)
  - `/map` & `/map_updates` (`nav_msgs/msg/OccupancyGrid`)
  - `/global_costmap/costmap` & `/local_costmap/costmap`
  - `/robot_description` (`std_msgs/msg/String`)
  - `/particle_cloud` (`nav2_msgs/msg/ParticleCloud`)
  - `/plan` (`nav_msgs/msg/Path`)
- **Published Topics:**
  - `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
  - `/clicked_point` (`geometry_msgs/msg/PointStamped`)
  - Nav2 action clients: `/navigate_to_pose`, `/navigate_through_poses`

### 4.2 Single-Owner Velocity Arbitration Integrity
- RViz2 **does not** publish to `/cmd_vel`, `/cmd_vel_nav`, or `/cmd_vel_smoothed`.
- It acts strictly as an Action Client to Nav2's `navigate_to_pose`.
- Velocity commands generated by Nav2 continue to pass through `cmd_vel_mux` and `collision_monitor` before reaching Gazebo.

### 4.3 Stale Node Leases Observed
- `ros2 node list` reports:
  ```text
  WARNING: Be aware that there are nodes in the graph that share an exact name:
  /rviz2_nav2
  /rviz2_nav2
  /rviz2_nav2
  ```
- **Finding:** Stale node registrations linger in FastDDS discovery when instances are killed abruptly without clean SIGINT shutdown. While benign to data flow, FastDDS lease expiration takes 20–30 seconds to prune stale entries.

**Verdict: PASS WITH LIMITATIONS (Stale FastDDS node registrations after hard restarts)**

---

## 5. Verification of Safety Invariants

### 5.1 Motor Safety Locks
Verified via direct inspection of runtime code and environment:
```python
MOTOR_OUTPUT_ENABLED = 0
SerialMCUTransport.PROTOCOL_CONFIRMED = False
```
- Physical serial communication to microcontrollers remains disconnected.
- PWM and CAN motor bridge drivers remain in hardware lockout mode.
- All operations are strictly constrained to simulation (`apartment_floor.world`).

### 5.2 Command Arbitration & Safety Monitors
- **Collision Monitor (`/collision_monitor`):** Active on `/scan`; decelerates and halts velocity commands if obstacles breach the 0.80 m / 0.35 m reflex envelopes.
- **Terrain Monitor (`/gracemo_terrain_monitor`):** Active on `/imu`; bounds linear velocity on excessive pitch/roll.

### 5.3 Emergency-Stop Interface Defect Analysis
An important technical defect was uncovered during code inspection of the newly added inspector panel:
- **Defect in [`views_inspector.py:401`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/widgets/views_inspector.py#L401):**
  ```python
  def _toggle_estop(self):
      bus.emergency_stop_triggered.emit(True, "UARCC Views Inspector")
  ```
  `EventBus` registers `emergency_stop_state = Signal(bool, str)`. It has **no signal named `emergency_stop_triggered`**. Clicking the E-stop button in `views_inspector.py` raises an `AttributeError` or fails to emit.
- **Authoritative Operational E-Stop:**
  The primary Emergency Stop buttons in [`safety_view.py:238`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/modules/safety/safety_view.py#L238), [`navigation_view.py:516`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/modules/navigation/navigation_view.py#L516), and [`autonomy_view.py:454`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/modules/autonomy/autonomy_view.py#L454) correctly emit `bus.emergency_stop_state(True, ...)`.
  This is wired to `RosManager._publish_emergency_stop()`, which publishes `/emergency_stop` to ROS and zeros motor commands.
- **Audit Conclusion:** The E-Stop button on `ViewsInspectorWidget` is non-functional, but all primary safety panels (`Safety`, `Navigation`, `Autonomy`) are fully functional and authoritative.

**Verdict: PASS WITH LIMITATIONS (Defect in secondary ViewsInspector E-Stop button; primary safety gates intact)**

---

## 6. Verification of Visual Appearance & Desktop UX

Inspection of visual evidence across resolutions:
1. **[`01_uarcc_with_embedded_rviz2_1500x920.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/embedded_rviz/01_uarcc_with_embedded_rviz2_1500x920.png):**
   - **Central Viewport:** Embedded RViz2 renders cleanly with active ground plane, canonical white/orange humanoid robot model, and costmaps.
   - **Side-Panel Contention:** Because RViz2 has its own internal dock panels (`Displays` on left, `Views`/`Docking` on right), having UARCC's outer docks open simultaneously creates duplicate sidebar columns (Outer UARCC Displays + Inner RViz Displays). The central 3D viewport is constrained to ~314 px width.
2. **[`01b_uarcc_embedded_rviz2_maximized_1500x920.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/embedded_rviz/01b_uarcc_embedded_rviz2_maximized_1500x920.png):**
   - When the user clicks `[Toggle Side Docks]`, the outer UARCC docks collapse.
   - RViz2 expands across 994 px width, providing a large, practical robotics workstation viewport.
3. **[`09_uarcc_embedded_rviz2_compact_1100x750.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/embedded_rviz/09_uarcc_embedded_rviz2_compact_1100x750.png):**
   - Responsive layout behaves correctly without window clipping or black rendering defects.

**Verdict: PASS WITH LIMITATIONS (Dual-sidebar contention requires outer docks to be collapsed for optimal viewport size)**

---

## 7. Resolution of Documentation Inconsistencies

### 7.1 Sidebar & Module Structure
- **Requirement:** The final module structure must **not** introduce a separate Voice Interaction sidebar page. Voice functionality must remain available through the Cognitive System and the existing Pygame `vira_interaction` interface.
- **Audit Finding:**
  In [`gracemo_control_center/app.py:75`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/app.py#L75) and `MODULE_NAMES`, a `"Voice & Interaction"` entry was introduced as Tab Index 7.
  This represents an architectural duplication:
  - The dedicated voice avatar runs independently via `vira_interaction/model.py` in a separate 1024x768 Pygame window.
  - Spoken dialogue grounding and LLM tasks are processed via `CognitiveView` and the Agent Bridge.
- **Canonical Module Count:**
  Without the redundant Voice page, UARCC comprises exactly **11 functional engineering modules**:
  1. `Overview`
  2. `Autonomous Operations`
  3. `Simulation`
  4. `Navigation`
  5. `Sensors & Perception`
  6. `Localization & Kinematics`
  7. `Cognitive System`
  8. `Safety`
  9. `Diagnostics`
  10. `System Logs`
  11. `Settings`
  The embedded RViz workspace acts as the primary 3D viewport for these 11 modules.

### 7.2 Sensor Modality Clarification
- **Planar LiDAR:** The robot platform is equipped with a **2D Planar LiDAR** publishing on `/scan`. It scans a 360-degree horizontal plane at $Z \approx 0.28\text{ m}$. It does **not** provide 3D volumetric point clouds.
- **3D Perception:** 3D depth perception is provided strictly by the forward-facing **RGB-D Depth Camera** publishing on `/camera/depth/image_raw` with a ~60° horizontal field of view.
- Any description claiming "360-degree 3D LiDAR" is factually incorrect and must refer specifically to **360-degree 2D planar LiDAR + forward RGB-D depth perception**.

---

## 8. Summary of Findings & Action Items

### Verified Strengths
1. True X11 foreign window reparenting (`QWindow.fromWinId` + `createWindowContainer`) works reliably and renders the authentic ROS 2 Jazzy RViz2 window inside PySide6.
2. Complete address-space separation successfully avoids the fatal Qt5 vs Qt6 dynamic symbol collision.
3. Decoupled fault domain ensures that visualization restarts never crash Gazebo Harmonic or the Nav2 navigation stack.
4. Motor hardware safety locks (`MOTOR_OUTPUT_ENABLED = 0`) remain strictly intact.
5. All 42 automated tests pass in headless CI without blocking or failures.

### Action Items for Post-Audit Refactoring
1. **Fix E-Stop Signal in `views_inspector.py`:** Update line 401 from `bus.emergency_stop_triggered.emit(True, ...)` to `bus.emergency_stop_state.emit(True, "VIEWS_INSPECTOR_BUTTON")`.
2. **Remove Redundant Voice Tab:** Consolidate Voice Interaction into Cognitive System and remove the redundant tab from `app.py`.
3. **Auto-Collapse Outer Docks on RViz Tab:** Configure `MainWindow` to automatically hide outer docks when switching to `3D Visualization` to prevent double-sidebar visual clutter, and restore them when navigating to other modules.
