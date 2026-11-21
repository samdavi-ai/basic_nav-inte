# GRaCEmo ViRa — GUI Defect Register

> **Document Type:** Canonical Defect Classification, Root Cause & Correction Register  
> **System:** GRaCEmo ViRa Unified Autonomous Robotics Control Center (UARCC) & Embedded RViz2  
> **Date:** 2026-11-02  
> **Status:** ALL CRITICAL & HIGH DEFECTS VERIFIED FIXED (0 OPEN DEFECTS)  

---

## 1. Defect Summary Matrix

| Defect ID | Module | Affected Control | Severity | Root Cause Summary | Status |
|---|---|---|---|---|---|
| **`DEF-RVIZ-01`** | Navigation / Embedded RViz2 | `_find_rviz_window_id & _embed_window_id` | **CRITICAL** | Substring check `"3x3"` discarded window `353x394`; missing `_MOTIF_WM_HINTS` left native titlebar. | **VERIFIED FIXED** |
| **`DEF-RVIZ-02`** | RViz2 Visualization | `LaserScan & PointCloud2 in gracemo_nav2.rviz` | **HIGH** | Intensity color transformer with zero bounds and untextured depth points caused blinding whiteout. | **VERIFIED FIXED** |
| **`DEF-GUI-01`** | Top Menu / 3D Navigation Tools | `Tools Menu & MainWindow._set_active_tool` | **HIGH** | `MainWindow` lacked compatibility attributes `btn_tool_move`, `btn_tool_select`, etc., causing `AttributeError`. | **VERIFIED FIXED** |
| **`DEF-GUI-02`** | Overview Dashboard | `OverviewView._on_coordinator_feedback` | **HIGH** | `COLOR_INFO` was used in status color ternary expression but missing from token imports. | **VERIFIED FIXED** |

---

## 2. Itemized Defect Records

### Defect `DEF-RVIZ-01`: RViz2 Window Detection Geometry Substring Collision & Decoration Bleed

* **Defect ID:** `DEF-RVIZ-01`
* **Module:** Navigation / `EmbeddedRVizWidget`
* **Control or Widget:** `_find_rviz_window_id()` and `_embed_window_id()`
* **Severity:** **CRITICAL**
* **Reproduction Steps:**
  1. Launch simulation with `vira_complete.launch.py mode:=localization`.
  2. Launch UARCC when RViz2 is active at dimensions where width ends in 3 and height begins in 3 (e.g. `353x394+0+0`).
  3. Observe that RViz2 appears as an independent floating desktop window with its native Mutter/GNOME title bar and borders.
* **Expected Result:**
  RViz2 client window should be detected, stripped of decorations, and seamlessly embedded into the UARCC central workspace.
* **Actual Result:**
  `_find_rviz_window_id()` returned `None` because `"3x3" not in line` evaluated to `False` on `353x394`. RViz remained a floating desktop window overlapping UARCC.
* **Root Cause:**
  1. Naive substring exclusion `if "3x3" not in line` inadvertently matched real window dimensions like `353x394`.
  2. Conflicting raw X11 call `x11.XReparentWindow(disp, win_id, parent_wid, 0, 0)` broke Qt's window container hierarchy.
  3. Window manager decorations were not stripped via `_MOTIF_WM_HINTS`.
* **Proposed Correction:**
  1. Parse window geometry using strict numerical regex (`\s+(\d+)x(\d+)\+`) and require `width >= 200 and height >= 200`.
  2. Apply `_MOTIF_WM_HINTS` property with `flags=2, decorations=0` via X11 ctypes to strip native window decorations.
  3. Allow `QWidget.createWindowContainer()` to manage reparenting without conflicting X11 calls.
* **Verification Status:** **VERIFIED FIXED**. Tested across 5 consecutive re-embed cycles; parent window verified as UARCC container `0x560000a`. Evidence: `docs/validation/gui_functional_testing/17_rviz2_embedding_after_restart.png`.

---

### Defect `DEF-RVIZ-02`: Blinding White LaserScan & Point Cloud Rendering Obscuring Robot Model

* **Defect ID:** `DEF-RVIZ-02`
* **Module:** RViz2 3D Viewport
* **Control or Widget:** `LaserScan`, `PointCloud2`, and `Bumper Hit` in `gracemo_nav2.rviz`
* **Severity:** **HIGH**
* **Reproduction Steps:**
  1. Open UARCC Navigation page with embedded RViz2.
  2. Inspect the 3D viewport containing the robot model and planar scan.
  3. Observe that a dense white cloud completely masks the robot's silver torso and black chassis.
* **Expected Result:**
  Planar LiDAR scan and depth points should be clearly rendered in distinct colors, leaving the robot model clearly visible.
* **Actual Result:**
  Planar scan was rendered in pure white `255, 255, 255` with `Size: 3` points, and `/intel_realsense_r200_depth/points` flooded the viewport in white.
* **Root Cause:**
  1. `LaserScan` used `Color Transformer: Intensity` with `Min Intensity: 0` and `Max Intensity: 0`. Uniform simulation scan intensities caused RViz2 to map all points to maximum intensity (pure white).
  2. `RealsenseDepthImage` was enabled by default with pure white color mapping.
* **Proposed Correction:**
  1. Set `LaserScan` to `Color Transformer: FlatColor`, `Color: 0; 220; 220` (Clean Engineering Cyan), `Style: Flat Squares`, `Size (m): 0.03`.
  2. Set `RealsenseDepthImage` to `Enabled: false` by default, with `Color Transformer: AxisColor` along Z when toggled on.
  3. Set `Bumper Hit` to `Enabled: false`, `Color: 255; 140; 0` (Orange).
* **Verification Status:** **VERIFIED FIXED**. The robot model is now sharp, distinct, and unobstructed by blinding white points. Evidence: `docs/validation/gui_functional_testing/18_corrected_visualization_no_whiteout.png`.

---

### Defect `DEF-GUI-01`: Unhandled AttributeError on Tools Menu Activation

* **Defect ID:** `DEF-GUI-01`
* **Module:** Top Menu Bar / 3D Navigation Tools
* **Control or Widget:** `Tools` Menu (`Move Camera`, `Select`, `2D Pose Estimate`, `2D Nav Goal`) and `MainWindow._set_active_tool`
* **Severity:** **HIGH**
* **Reproduction Steps:**
  1. Open top menu bar.
  2. Click `Tools -> Move Camera` (or `Select`, `2D Pose Estimate`, `2D Nav Goal`).
  3. Observe application crash with `AttributeError: 'MainWindow' object has no attribute 'btn_tool_move'`.
* **Expected Result:**
  Tool mode switches cleanly to the selected action and updates the bottom status notification.
* **Actual Result:**
  `_set_active_tool()` attempted to check `.isChecked()` on uninitialized toolbar button attributes.
* **Root Cause:**
  `MainWindow` defined only `btn_tool_interact` for backward compatibility, omitting `btn_tool_move`, `btn_tool_select`, `btn_tool_pose`, and `btn_tool_goal`.
* **Proposed Correction:**
  1. Initialize `btn_tool_move`, `btn_tool_select`, `btn_tool_pose`, and `btn_tool_goal` attributes on `MainWindow`.
  2. Use safe dictionary mapping with `getattr()` to synchronize tool button check states and update status notification.
* **Verification Status:** **VERIFIED FIXED**. All 5 tool menu actions tested and passed in `MENU-02` through `MENU-05`.

---

### Defect `DEF-GUI-02`: Missing `COLOR_INFO` Import in Overview Dashboard

* **Defect ID:** `DEF-GUI-02`
* **Module:** Overview Dashboard
* **Control or Widget:** `OverviewView._on_coordinator_feedback()`
* **Severity:** **HIGH**
* **Reproduction Steps:**
  1. Open UARCC Overview page.
  2. Dispatch a navigation mission or change coordinator state.
  3. Observe exception in log: `NameError: name 'COLOR_INFO' is not defined`.
* **Expected Result:**
  `metric_robot_state` updates its value and status color smoothly without unhandled exceptions.
* **Actual Result:**
  Ternary expression evaluated `COLOR_INFO` when state was not `IDLE` or `SUCCEEDED`, throwing `NameError`.
* **Root Cause:**
  `COLOR_INFO` was omitted from the `tokens.py` import block in `overview_view.py`.
* **Proposed Correction:**
  Add `COLOR_INFO` to the import list in `gracemo_control_center/modules/overview/overview_view.py`.
* **Verification Status:** **VERIFIED FIXED**. Verified during mission dispatch in `MOD-AUTONOMY-02` through `MOD-AUTONOMY-05`. Zero exceptions raised.

---

## 3. Defect Trend & Quality Assessment

- **Total Defects Identified:** 4
- **Critical Severity:** 1 (Resolved)
- **High Severity:** 3 (Resolved)
- **Medium / Low Severity:** 0
- **Open Defects Remaining:** 0
- **Defect Resolution Rate:** **100%**
