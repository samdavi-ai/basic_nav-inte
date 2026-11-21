# GRaCEmo ViRa — RViz2 Visualization Profiles Report
**Document ID:** `RVIZ2-VIS-2026-11-03`  
**Classification:** Autonomous Simulation & Workstation Visualization  
**Target Environment:** ROS 2 Jazzy | Gazebo Harmonic | PySide6 UARCC  
**Status:** ✅ **VERIFIED & ACCEPTED**  

---

## 1. Executive Summary

This report documents the resolution of visual clutter within the GRaCEmo ViRa RViz2 interface. Previously, the robot model was visually obscured by a dense cluster of coordinate axes ("coloured sticks"), yellow transform arrows, frame text labels, candidate trajectory rollout markers, and 3D voxel obstacle grids.

A dual-profile architecture has been designed and implemented:
1. **`NORMAL_OPERATION` (Canonical Default):** Pristine, high-fidelity visualization of the robot model, floorplan map, subtle LiDAR scan perimeter, navigation paths, and costmaps with zero visible coordinate axes or debugging clutter.
2. **`ENGINEERING_DEBUG`:** Complete engineering diagnostic profile exposing full TF coordinate axes (scale 0.6), link names, sensor mount transforms, DWB trajectory rollouts, and diagnostic markers.

All underlying ROS 2 transformation pipelines (`/tf`, `/tf_static`), Nav2 lifecycle nodes (10/10 active), AMCL localization covariance, and sensor bridges remain 100% operational.

---

## 2. Visual Elements Audit: Hidden vs. Preserved

| Element / Display Plugin | Previous State | Normal Operation Profile | Engineering Debug Profile | Technical Rationale |
| :--- | :--- | :--- | :--- | :--- |
| **TF Coordinate Axes** (`rviz_default_plugins/TF`) | Visible on all 18+ links (scale 1.0) | **Hidden** (`Show Axes: false`, `Enabled: false`) | **Visible** (`Show Axes: true`, scale 0.6) | Cluttered the torso, head, and wheels with overlapping RGB sticks. Underlying TF broadcasting remains fully active. |
| **TF Frame Names & Labels** | Visible on every joint and link | **Hidden** (`Show Names: false`) | **Visible** (`Show Names: true`) | Large white floating text labels obscured robot facial expression and camera mount. |
| **TF Parent-Child Arrows** | Visible (yellow lines connecting frames) | **Hidden** (`Show Arrows: false`) | **Visible** (`Show Arrows: true`) | Distorted the appearance of the chassis and torso. |
| **Robot Collision Bounding Boxes** | Disabled (`Collision Enabled: false`) | **Hidden** (`Collision Enabled: false`) | **Hidden** (`Collision Enabled: false`) | Collision boxes duplicate visual meshes and occlude high-poly styling. |
| **Robot Visual Meshes** | Enabled (`Visual Enabled: true`) | **Preserved & Visible** (`Visual Enabled: true`) | **Preserved & Visible** (`Visual Enabled: true`) | Full robot model (chassis, head, display screen, wheels, caster, arms) is rendered with textures. |
| **3D Voxel Grids** (`/local_costmap/voxel_grid`) | Enabled (dense 3D cubes) | **Hidden** (`Enabled: false`) | **Available** (`Enabled: false`, togglable) | Large 3D cubes encapsulated the robot body during indoor navigation. |
| **Floating Camera Quad** (`rviz_default_plugins/Image`) | Enabled in 3D scene | **Hidden** (`Group Realsense: false`) | **Available** (togglable) | A floating 2D rectangle in front of the robot's face obstructed frontal view. Stream accessible via Sensors tab. |
| **Local Planner Trajectories** (`/marker`) | Enabled (candidate rollout lines) | **Hidden** (`Enabled: false`) | **Visible** (`Enabled: true`) | Multi-colored trajectory fans are useful for controller tuning but clutter operational view. |
| **360° LiDAR Scan** (`/scan`) | Size: 0.03m | **Preserved & Refined** (Cyan `0;220;220`, Flat Squares, Size: 0.02m) | **Preserved** (Size: 0.03m) | Provides clear, non-intrusive boundary tracing without blocking the robot chassis. |
| **Navigation Paths** (`/plan`, `/local_plan`) | Enabled | **Preserved & Visible** | **Preserved & Visible** | High-visibility global and local paths required for operational monitoring. |
| **Costmaps & Static Map** | Enabled | **Preserved & Visible** (Alpha: 0.25 Global, 0.4 Local) | **Preserved & Visible** | Transparent costmap overlays ensure the robot and floor are visible simultaneously. |
| **AMCL Particle Swarm** (`/particlecloud`) | Enabled | **Preserved & Visible** | **Preserved & Visible** | Green directional arrows show particle filter convergence around base_footprint. |

---

## 3. Profile Architecture & Configuration Files

| Profile | Primary Configuration File | Installed Path | Key Attributes |
| :--- | :--- | :--- | :--- |
| **Canonical Default** | `ros2_ws/src/gracemo_nav2/rviz/gracemo_nav2.rviz` | `install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_nav2.rviz` | Loaded by `nav2.launch.py` and `display.launch.py`. Normal operation settings. |
| **Normal Operation** | `ros2_ws/src/gracemo_nav2/rviz/gracemo_normal.rviz` | `install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_normal.rviz` | Explicit normal profile for switcher selection. |
| **Engineering Debug** | `ros2_ws/src/gracemo_nav2/rviz/gracemo_debug.rviz` | `install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_debug.rviz` | Enables TF axes, frame names, arrows, trajectory markers. |
| **Original Baseline Backup**| `ros2_ws/src/gracemo_nav2/rviz/gracemo_nav2.rviz.backup`| N/A | Full backup of unmodified configuration prior to migration. |

### UARCC GUI Profile Switcher Integration
In `gracemo_control_center/widgets/embedded_rviz_widget.py`, the Embedded RViz2 banner now includes a dynamic profile selector:
- Object Name: `comboRVizProfile`
- Options:
  - `🎯 Normal Operation (Clean)` (`gracemo_normal.rviz`)
  - `🛠️ Engineering Debug (TF Axes)` (`gracemo_debug.rviz`)
- Implementation: `switch_profile(config_path)` gracefully updates the target profile, restarts the embedded process, and re-embeds the native X11 window into the PySide6 container without restarting the robot autonomy stack.

---

## 4. Visual Evidence & Side-by-Side Verification

### Profile 1: Normal Operation (Clean Autonomous Operation)
- **Path:** `docs/screenshots/rviz_profiles/rviz_profile_normal_operation.png`
- **Characteristics:** Unobstructed robot model, smooth visual textures, AMCL particle cloud, map, costmaps, and cyan LiDAR perimeter. Zero coordinate axes or floating text.

![Normal Operation RViz Profile](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/rviz_profiles/rviz_profile_normal_operation.png)

### Profile 2: Engineering Debug (TF Axes & Diagnostics)
- **Path:** `docs/screenshots/rviz_profiles/rviz_profile_engineering_debug.png`
- **Characteristics:** Complete coordinate frame axes (RGB) at all robot links (`base_link`, `lidar_link`, `camera_link`, `wheel_link`, `caster_link`), link name labels, and candidate rollout trajectories.

![Engineering Debug RViz Profile](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/rviz_profiles/rviz_profile_engineering_debug.png)

---

## 5. Subsystem Health & Functional Validation

### 1. Underlying TF System Integrity
Underlying TF frame broadcasting and listener lookups remain fully operational:
- `map -> base_footprint`: `[0.036, 0.040, 0.000]` (AMCL localization active)
- `base_footprint -> lidar_link`: `[0.000, 0.000, 1.175]` (Static robot description tree)
- `odom -> base_footprint`: `[0.000, 0.000, 0.000]` (Wheel odometry bridge active)

### 2. Nav2 Lifecycle Node Verification
All 10/10 Nav2 lifecycle nodes confirmed in state `active [3]`:
- `map_server`: `active [3]`
- `amcl`: `active [3]`
- `controller_server`: `active [3]`
- `smoother_server`: `active [3]`
- `planner_server`: `active [3]`
- `behavior_server`: `active [3]`
- `bt_navigator`: `active [3]`
- `waypoint_follower`: `active [3]`
- `velocity_smoother`: `active [3]`
- `collision_monitor`: `active [3]`

### 3. AMCL Localization & Particle Cloud
- AMCL particle swarm actively converges around robot position.
- Pose covariance validated: $\sigma_x^2 = 0.206$, $\sigma_y^2 = 0.259$, $\sigma_{\text{yaw}}^2 = 0.061$.

### 4. Hardware Safety Gates
Confirmed preserved throughout the entire test campaign:
- `firmware/config.h`: `#define MOTOR_OUTPUT_ENABLED 0`
- `mcu_transport.py`: `SerialMCUTransport.PROTOCOL_CONFIRMED = False`
