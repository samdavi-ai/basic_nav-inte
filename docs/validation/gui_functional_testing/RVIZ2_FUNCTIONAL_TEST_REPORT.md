# GRaCEmo ViRa — RViz2 Functional Test & Workspace Integration Report

> **Document Type:** Dedicated ROS 2 RViz2 Functional Test & Integration Report  
> **Subsystem:** Native RViz2 Ogre 3D Visualization Workspace & Nav2 Plugin Integration  
> **Host Environment:** ROS 2 Jazzy | Gazebo Harmonic 8.9.0 | PySide6 Qt 6.4.2 | Ubuntu 24.04 LTS (x86_64)  
> **Date:** 2026-11-02  
> **Status:** VERIFIED FULLY FUNCTIONAL  

---

## 1. Primary Objectives & Scope

This report validates the actual ROS 2 Jazzy RViz2 application running as an embedded process inside the UARCC central workspace. It confirms that the visualization is authentic, the window reparenting is robust and undecorated, the canonical displays tree is complete, the 3D camera controls are responsive, and the Nav2 interactive tools operate without conflict.

---

## 2. Embedding & Window Lifecycle Management

### 2.1 Window Containment & Window Manager Stripping
- **Host Container:** `EmbeddedRVizWidget` (`QWidget` with `Qt.WA_NativeWindow`).
- **Embedding Mechanism:** `QWindow.fromWinId(win_id)` wrapped via `QWidget.createWindowContainer()`.
- **Decoration Removal:** Explicit X11 `_MOTIF_WM_HINTS` property set to `(flags=2, decorations=0)`.
- **Parent-Child Relationship:** Verified via `xwininfo -id 0x2a00106 -children`. The parent window is the UARCC container (`0x560000a`), confirming zero desktop window leakage.
- **Process Identity:** Window `0x2a00106` belongs to PID 284108 (`/opt/ros/jazzy/lib/rviz2/rviz2`), confirming that the authentic ROS 2 RViz2 executable is embedded.

### 2.2 Window Lifecycle & Stress Matrix

| Lifecycle Event | Action Executed | Observed Result | Verdict |
|---|---|---|---|
| **Initial Launch** | Start UARCC with simulation active | RViz2 detected and embedded within 250ms | **PASS** |
| **Window Maximize** | Trigger desktop window maximize | RViz2 expands to fill full central workspace | **PASS** |
| **Window Restore** | Restore normal 1500x920 size | RViz2 contracts without rendering distortion | **PASS** |
| **Sidebar Collapse** | Collapse sidebar to 54px | Central RViz2 workspace expands horizontally | **PASS** |
| **Sidebar Restore** | Restore sidebar to 230px | Central RViz2 workspace contracts smoothly | **PASS** |
| **Page Navigation Away** | Switch to Sensors, Safety, or Overview | Foreign RViz2 window hidden cleanly | **PASS** |
| **Page Navigation Return** | Return to Navigation workspace | RViz2 window restored without re-launching process | **PASS** |
| **5x Consecutive Re-embed** | Trigger re-embed cycle 5 times | Stable embedding maintained; 0 orphans | **PASS** |

Evidence: `docs/validation/gui_functional_testing/05_embedded_rviz2.png` and `17_rviz2_embedding_after_restart.png`.

---

## 3. Displays Panel Verification

All 12 canonical RViz2 visualization layers were audited and validated against `ros2_ws/src/gracemo_nav2/rviz/gracemo_nav2.rviz`:

| Display Layer | Class | Topic / Binding | Frame / Parameters | Status |
|---|---|---|---|---|
| **Global Options** | `rviz_common/Displays` | N/A | Fixed Frame: `map`, Background: `#1E1E1E`, Rate: 30 FPS | **VERIFIED** |
| **Grid** | `rviz_default_plugins/Grid` | N/A | Plane: `XY`, Cell Size: 1.0 m, Alpha: 0.5, Color: `160; 160; 164` | **VERIFIED** |
| **RobotModel** | `rviz_default_plugins/RobotModel` | `/robot_description` | Visual: `true`, Collision: `false`, Durability: `Transient Local` | **VERIFIED** |
| **TF** | `rviz_default_plugins/TF` | `/tf`, `/tf_static` | Marker Scale: 1.0, Show Arrows: `true`, Show Axes: `true` | **VERIFIED** |
| **LaserScan** | `rviz_default_plugins/LaserScan` | `/scan` | Color: `0; 220; 220` (Cyan), Style: `Flat Squares`, Size: 0.03m | **VERIFIED** |
| **Bumper Hit** | `rviz_default_plugins/PointCloud2` | `/mobile_base/sensors/bumper_pointcloud` | Color: `255; 140; 0` (Orange), Enabled: `false` (default) | **VERIFIED** |
| **Map** | `rviz_default_plugins/Map` | `/map` | Color Scheme: `map`, Draw Behind: `true`, Alpha: 1.0 | **VERIFIED** |
| **AMCL Particle Cloud** | `rviz_default_plugins/PointCloud` | `/particle_cloud` | Style: `Points`, Color: `0; 255; 0` | **VERIFIED** |
| **Global Costmap** | `rviz_default_plugins/Map` | `/global_costmap/costmap` | Color Scheme: `costmap`, Topic: `/global_costmap/costmap` | **VERIFIED** |
| **Local Costmap** | `rviz_default_plugins/Map` | `/local_costmap/costmap` | Color Scheme: `costmap`, Topic: `/local_costmap/costmap` | **VERIFIED** |
| **Global Plan** | `rviz_default_plugins/Path` | `/plan` | Color: `255; 0; 0` (Red), Line Width: 0.03m | **VERIFIED** |
| **Local Plan** | `rviz_default_plugins/Path` | `/local_plan` | Color: `0; 255; 0` (Green), Line Width: 0.02m | **VERIFIED** |

---

## 4. Elimination of Excessive White Rendering

In previous test runs, the robot model was obscured by an overexposed white point cloud. Two distinct root causes were identified and corrected:

1. **LaserScan Intensity Normalization Bug:**
   - *Previous:* `Color Transformer: Intensity` with `Min Intensity: 0` and `Max Intensity: 0`. This caused all 360 planar scan points to be rendered in pure white (`255, 255, 255`).
   - *Correction:* Updated to `Color Transformer: FlatColor`, `Color: 0; 220; 220` (Bright Cyan), `Style: Flat Squares`, `Size (m): 0.03`.
2. **Untextured Realsense Depth Cloud Overlap:**
   - *Previous:* `/intel_realsense_r200_depth/points` was set to `Enabled: true` in pure white `255; 255; 255`.
   - *Correction:* Set to `Enabled: false` by default, with `Color Transformer: AxisColor` along Z so that elevation tinting is used when explicitly enabled.

*Outcome:* The robot model chassis, black caster wheels, silver torso, and pan-tilt head are now sharp, clearly defined, and distinct from the surrounding environment.  
Evidence: `docs/validation/gui_functional_testing/18_corrected_visualization_no_whiteout.png`.

---

## 5. 3D Camera & View Controls Verification

Tested in `ViewsInspectorWidget` and toolbar camera presets:
- **Orbit Camera Controls:**
  - Distance: 0.5m to 50.0m (Tested at 2.3m, 5.0m, 7.5m).
  - Pitch: -89.0° to +89.0° (Tested at 35.0° and 89.0°).
  - Yaw: -180.0° to +180.0° (Tested at -45.0° and 0.0°).
- **Camera Presets:**
  - `Isometric (3D)`: Pitch = 35.0°, Distance = 7.5m — PASS.
  - `Top-Down (2D)`: Pitch = 89.0°, Yaw = 0.0° — PASS.
  - `Front View`: Pitch = 0.0°, Yaw = 0.0° — PASS.
  - `Side View`: Pitch = 0.0°, Yaw = -90.0° — PASS.
  - `Reset View`: Restores engineering default orbit — PASS.

Evidence: `docs/validation/gui_functional_testing/07_rviz2_views_panel.png`.

---

## 6. Interactive Nav2 Tools Testing

1. **Interact Tool (`Q`):** Allows selecting interactive markers and interacting with Nav2 waypoints. Tested and set as default.
2. **Move Camera Tool (`M`):** Enables orbiting, panning, and zooming without placing goals.
3. **Select Tool (`S`):** Enables selecting items in the 3D scene to inspect bounding boxes and TF frames.
4. **2D Pose Estimate (`P`):** Publishes initial pose to `/initialpose` for AMCL localization seeding.
5. **2D Nav Goal (`G`):** Publishes goal pose to `/goal_pose` for autonomous Nav2 path planning.

All tool buttons synchronize their check states smoothly with `MainWindow._set_active_tool()`, and notifications are displayed in the bottom status bar without exceptions.

---

## 7. Configuration Integrity & Persistence

- Canonical source file: `ros2_ws/src/gracemo_nav2/rviz/gracemo_nav2.rviz`.
- Installed configuration: `ros2_ws/install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_nav2.rviz`.
- Verified backup: `gracemo_nav2.rviz.backup` created prior to modifications.
- Persistence verified: Application restarts read the updated configuration cleanly without configuration file corruption.

---

## 8. Conclusion

Native RViz2 integration inside UARCC is fully functional. The window embedding regression has been permanently solved, window manager title bars and borders are completely eliminated, and the visualization is clear, professional, and free of overexposure.
