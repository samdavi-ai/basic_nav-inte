# GRaCEmo ViRa — UARCC Professional Engineering Design System

**Document Version:** 1.0.0  
**Target Subsystem:** Unified Autonomous Robotics Control Center (UARCC)  
**Classification:** Engineering Specification & UI/UX Design System  
**Last Updated:** November 2026  
**Status:** IMPLEMENTED, QUALIFIED & VERIFIED  

---

## 1. Executive Summary & Design Philosophy

The **Unified Autonomous Robotics Control Center (UARCC)** serves as the authoritative desktop telemetry, mission coordination, and reflex safety monitoring interface for the **GRaCEmo ViRa** autonomous mobile service robot. 

### Core Design Principles

1. **Industrial Credibility over Decorative Gimmicks:**
   - Robotics operators during live deployment demand high legibility, strict contrast, and predictable visual hierarchies.
   - Elimination of dark navy gradients, glowing borders, AI-dashboard decorative effects, neon highlights, and arbitrary glassmorphism.
   - Emulates top-tier industrial robotics command software (e.g., Foxglove Studio, Autoware, Boston Dynamics Scout, ROS Webviz).

2. **Calm, High-Contrast Engineering Canvas:**
   - Background canvas: Neutral slate grey (`#F4F6F8`).
   - Workspace surfaces: Pure white (`#FFFFFF`) cards with crisp hairline slate borders (`#DCE2E8`).
   - Persistent command sidebar: Deep slate navy (`#172435`) with subtle secondary grouping (`#203247`).

3. **Data Integrity & Safety Transparency:**
   - Real-world telemetry is never fabricated or masked with fake "READY" badges.
   - Stale data (>2.0s without tick) automatically turns amber (`#B7791F`).
   - Disconnected or uninitialized hardware explicitly registers as `UNAVAILABLE` or `OFFLINE` (`#596779` / `#C83D43`).
   - Emergency stop is permanently visible, highly distinguished (`#C83D43`), and requires human operator confirmation to clear.
   - Strict hardware safety locks (`MOTOR_OUTPUT_ENABLED = 0`, `SerialMCUTransport.PROTOCOL_CONFIRMED = False`) are persistently displayed across all viewports.

---

## 2. Design Tokens & Color System

The design system is codified in `gracemo_control_center/resources/styles/tokens.py` and implemented across the master stylesheet in `gracemo_control_center/resources/styles/dark_theme.py`.

### 2.1 Color Palette

| Token Identifier | Hex Value | Semantic Role |
| :--- | :--- | :--- |
| `COLOR_BG_MAIN` | `#F4F6F8` | Main application canvas background |
| `COLOR_WORKSPACE` | `#FFFFFF` | Primary card, table, and telemetry panel surface |
| `COLOR_SUBTLE_SURFACE` | `#EEF2F5` | Secondary wells, headers, input field backgrounds |
| `COLOR_SIDEBAR_BG` | `#172435` | Command navigation sidebar surface |
| `COLOR_SIDEBAR_SURFACE`| `#203247` | Active navigation pill, hardware lock footer |
| `COLOR_TEXT_PRIMARY` | `#182433` | Deep legible charcoal for titles, headings, values |
| `COLOR_TEXT_SECONDARY`| `#596779` | Slate grey for labels, descriptions, and metadata |
| `COLOR_TEXT_MUTED` | `#8190A0` | Muted slate for units, coordinate frames, timestamps |
| `COLOR_ACCENT_PRIMARY` | `#3478B8` | Steel industrial blue for primary actions & heading arrows |
| `COLOR_ACCENT_SECONDARY`| `#268F91`| Precision teal for planned paths and secondary telemetry |
| `COLOR_SUCCESS` | `#25845B` | Forest green for verified healthy states, arrived goals |
| `COLOR_WARNING` | `#B7791F` | Amber for stale telemetry, high covariance, paused states |
| `COLOR_ERROR` | `#C83D43` | Urgent industrial crimson for active E-stops, errors |
| `COLOR_OFFLINE` | `#596779` | Slate grey for disconnected or uninitialized subsystems |
| `COLOR_BORDER` | `#DCE2E8` | 1px hairline boundary separating cards and viewports |

### 2.2 Status Indicator Tint Tints

| Subsystem State | Background Pill | Hairline Border | Text / LED Dot |
| :--- | :--- | :--- | :--- |
| **READY / ACTIVE** | `#EBF5F0` | `#C3E6D5` | `#25845B` |
| **INITIALIZING / STALE** | `#FDF6E2` | `#F5DEB3` | `#B7791F` |
| **ERROR / E-STOP** | `#FBEBEB` | `#F7C5C5` | `#C83D43` |
| **NOT CONNECTED** | `#F1F5F9` | `#DCE2E8` | `#596779` |
| **NAVIGATING** | `#EDF6F8` | `#BCE2E4` | `#268F91` |

---

## 3. Typography Scale & Hierarchy

All typefaces follow modern, readable industrial typography standards:
- **Primary Font Family:** Inter, IBM Plex Sans, Source Sans 3, or Segoe UI / Roboto fallback.
- **Monospace Font Family:** JetBrains Mono, Fira Code, Consolas, or Liberation Mono.

| Typography Element | Size | Weight | Line Height | Color |
| :--- | :--- | :--- | :--- | :--- |
| **Window Brand Title** | 15px | 700 (Bold) | 1.2 | `#FFFFFF` |
| **Page Header Title** | 16px | 700 (Bold) | 1.2 | `#182433` |
| **Page Header Subtitle**| 12px | 400 (Regular) | 1.4 | `#596779` |
| **Section Header Title**| 11px | 700 (Bold) | 1.2 | `#3478B8` (Caps) |
| **Metric Display Value**| 18px | 700 (Bold) | 1.1 | Semantic / `#182433` |
| **Metric Display Label**| 10px | 600 (SemiBold)| 1.2 | `#596779` |
| **Body & Label Text** | 12px | 400 (Regular) | 1.4 | `#182433` |
| **Monospace Telemetry** | 11px | 600 (SemiBold)| 1.3 | `#182433` (Tabular) |
| **Console Output** | 11px | 400 (Regular) | 1.4 | `#E2E8F0` on `#111827` |

---

## 4. Application Shell & Layout Architecture

The application layout uses an engineering content grid partitioned into four structural zones:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│ TOP STATUS BAR: Active Page Breadcrumb | ROS 2 | Gazebo | Robot FSM | E-Stop | Sim Clock│
├───────────────┬────────────────────────────────────────────────────────────────────────┤
│ SIDEBAR       │ STACKED APPLICATION WORKSPACE CANVAS (1440x900 or 1024x720)            │
│  WORKSPACE    │                                                                        │
│   Overview    │  ┌──────────────────────────────────────────────────────────────────┐  │
│   Autonomy    │  │ PageHeader: Title, Subtitle, Context Action Badge               │  │
│   Simulation  │  ├──────────────────────────────────────────────────────────────────┤  │
│   Navigation  │  │ Key Telemetry Metric Row (MetricDisplay widgets)                 │  │
│   Sensors     │  ├──────────────────────────────────────────────────────────────────┤  │
│  ROBOT SYSTEMS│  │ Multi-column Card Layout / Tabbed Viewports                      │  │
│   Localization│  │                                                                  │  │
│   Voice       │  │                                                                  │  │
│   Cognitive   │  │                                                                  │  │
│  ENGINEERING  │  └──────────────────────────────────────────────────────────────────┘  │
│   Safety      │                                                                        │
│   Diagnostics │                                                                        │
│   Logs        │                                                                        │
│   Settings    │                                                                        │
│ ───────────── │                                                                        │
│ 🔒 HW LOCKED  │                                                                        │
├───────────────┴────────────────────────────────────────────────────────────────────────┤
│ BOTTOM BAR: System Status Notification Stream | ROS Discovery | Gazebo Partition       │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Reusable Component Library

The control center leverages shared, tested components from `gracemo_control_center/widgets/common_components.py` and `gracemo_control_center/widgets/`:

1. **`PageHeader`:**
   - Standardized page identification containing a steel blue accent bar, title, subtitle, and live subsystem status badge.
2. **`SectionHeader`:**
   - Uppercase technical heading with accent color, providing clear visual anchoring for cards.
3. **`MetricDisplay`:**
   - Compact numerical/state display card featuring a small metadata label and prominent tabular numerical value.
4. **`StatusIndicator`:**
   - Interactive pill containing an anti-aliased LED dot, label, and state badge with clickable routing to relevant diagnostic views.
5. **`MapViewWidget`:**
   - High-performance 2D occupancy grid renderer with interactive pan/zoom, vector robot heading cone, dashed teal global plan, and clear destination targets.
6. **`CameraViewWidget` & `DepthViewWidget`:**
   - Video viewports with aspect ratio preservation, dark matte viewfinder (`#0F172A`), and live FPS/resolution overlay.
7. **`LidarViewWidget`:**
   - 360-degree polar/Cartesian radar view with distance rings, safety stop boundary (0.35m), proximity warning envelope (0.80m), and live scan points.
8. **`RobotPoseWidget` & `CompassWidget`:**
   - Authoritative pose monitor with circular compass needle pointing to true yaw, tabular X/Y/Yaw coordinates, and AMCL covariance variance bounds.

---

## 6. Page-by-Page Transformation Details

### Page 1 — Overview
- **Before:** Grey-on-black cards, unaligned status dots, dark camera viewport with low contrast.
- **After:** Pure white card surfaces on `#F4F6F8` canvas. Subsystem health matrix organized in an intuitive 3-row grid with clickable navigation shortcuts. Integrated robot pose and odometry widget with live compass needle.

### Page 2 — Simulation
- **Before:** Dominated by an oversized raw terminal dump and lack of clear separation.
- **After:** Structured into 4 distinct sections:
  1. Simulation Key Metrics (Sim Clock, Real-Time Factor, Bridge State, Active World).
  2. Physics Engine & Bridge Telemetry grid (step size, target frequency, SDF world name, GZ partition).
  3. External Gazebo 3D Viewer attachment controls with process duplication guards.
  4. Monospace Diagnostic Console with severity color tags (`[INFO]`, `[WARN]`, `[SUCCESS]`, `[ERROR]`), clipboard copy, and clear functionality.

### Page 3 — Navigation
- **Before:** Dark green/black map with barely visible paths, subdued emergency stop button.
- **After:** Clear technical occupancy grid map:
  - Unknown space: Neutral hairline boundary (`#DCE2E8`).
  - Free space: Crisp white workspace (`#FFFFFF`).
  - Occupied obstacles: Dark solid charcoal (`#182433`).
  - Robot body: Steel blue (`#3478B8`) with heading cone.
  - Planned route: Dashed precision teal (`#268F91`).
  - Emergency Stop Button: Prominent industrial crimson (`#C83D43`, 40px height) with immediate visibility.

### Page 4 — Sensors
- **Before:** Weak visual hierarchy with empty spaces and raw text labels.
- **After:** Professional multi-modal instrumentation center:
  - Top: Perception Subsystem Health Matrix table displaying Topic, Frame ID, Status (`ACTIVE`, `STALE`, `UNAVAILABLE`), Frequency (Hz), and Data Age.
  - Tab 1: Live Vision & LiDAR Streams: Forward RGB feed and Depth sensor preview on the left, 360° Planar LiDAR polar sweep on the right.
  - Tab 2: Kinematics & IMU Telemetry: Full pose widget side-by-side with 6-DOF IMU attitude, linear accelerations, and gyro rates.
  - Tab 3: LiDAR Engineering Specifications and safety stop envelopes.

### Page 5 — Localization
- **Before:** Unbounded text fields and no explicit lifecycle distinction.
- **After:** AMCL particle filter confidence metrics, TF coordinate hierarchy chain (`map → odom → base_footprint → sensors`) with independent link health pills, and covariance variance monitoring (`σx²`, `σy²`, `σθ²`).

### Page 6 — Voice & Interaction
- **Before:** Ambiguous text fields with no pipeline state.
- **After:** Dedicated pipeline stage indicators (`Whisper STT`, `Intent Parser`, `Place Grounding`, `Spoken TTS`), dialogue history stream differentiating User utterances from Robot responses, and safe text command dispatch.

### Page 7 — Cognitive System
- **Before:** Raw JSON text dumps with risk of exposing environment variables.
- **After:** Dedicated MNSE daemon and Agent Bridge MCP communication monitoring, approval gate authorization locks, and structured task queue event stream.

### Page 8 — Safety
- **Before:** Basic toggle switches with unclear velocity pipeline arbitration.
- **After:** High-integrity safety command center:
  - Authoritative E-Stop gate with explicit modal confirmation for human operator reset.
  - Command velocity arbitration pipeline flow (`Nav2 BT → /cmd_vel_nav → velocity_smoother → collision_monitor → terrain_monitor → /cmd_vel`).
  - Terrain grade and tilt watchdog thresholds.
  - Hardware Safety Invariants (`MOTOR_OUTPUT_ENABLED = 0`, `SerialMCUTransport.PROTOCOL_CONFIRMED = False`) clearly presented in green monospace.

### Page 9 — Diagnostics
- **Before:** Flat unstyled tables.
- **After:** Host workstation resource telemetry (CPU %, RAM GB, NVIDIA RTX 5060 GPU Util, VRAM MiB) with progress bars, and live ROS 2 Jazzy topic telemetry table.

---

## 7. Accessibility, Contrast & Window Resizing Verification

1. **WCAG AA Compliance:**
   - Text contrast ratio between primary text (`#182433`) and workspace surface (`#FFFFFF`) is **14.2:1** (exceeds WCAG AAA requirement of 7.0:1).
   - Secondary text (`#596779`) on white surface is **5.4:1** (exceeds WCAG AA requirement of 4.5:1).
   - White button text on Crimson Emergency Stop (`#C83D43`) is **5.1:1** (exceeds WCAG AA).
2. **Multi-Coded Safety States:**
   - Critical indicators never rely solely on color. Every indicator includes an explicit text badge (e.g., `EMERGENCY STOP: ACTIVE (SAFETY_PANEL_BUTTON) 🚨`, `ACTIVE`, `STALE`, `UNAVAILABLE`).
3. **Window Resizing & Responsive Adaptation:**
   - Verified across desktop resolutions:
     - **1440x900 (Native Desktop Standard):** Zero clipping, comfortable margins, cards span harmoniously.
     - **1024x720 (Compact Minimum Bounds):** Content automatically wraps in smooth scrollable viewports with zero horizontal overflow or truncated labels.

---

## 8. Visual Evidence Artifacts

Screenshots captured directly from the live running application:

- **Overview (1440x900):** `docs/screenshots/after_professional/overview_1440x900.png`
- **Navigation (1440x900):** `docs/screenshots/after_professional/navigation_1440x900.png`
- **Sensors (1440x900):** `docs/screenshots/after_professional/sensors_1440x900.png`
- **Simulation (1440x900):** `docs/screenshots/after_professional/simulation_1440x900.png`
- **Safety (1440x900):** `docs/screenshots/after_professional/safety_1440x900.png`
- **Compact View (1024x720):** `docs/screenshots/after_professional/overview_1024x720.png`

---

## 9. Test Suite Verification & Quality Gates

All test suites were executed to verify zero regression across the autonomy pipeline, control center, and voice interaction:

```bash
# 1. UARCC Comprehensive Integration Test Suite
pytest gracemo_control_center/tests/test_uarcc_comprehensive.py -v
# Result: 30 PASSED in 4.41s

# 2. Autonomous Operations & FSM Test Suite
python3 -m pytest ros2_ws/src/gracemo_nav2/test/test_autonomous_operations.py -v
# Result: 11 PASSED in 1.96s

# 3. Voice Interaction, Dynamic VAD & Pre-STT Noise Rejection
python3 -m pytest vira_interaction/tests/ -v
# Result: 69 PASSED in 2.25s
```

**Total Tests Verified:** **110 / 110 Passed (100% Success Rate)**.  
**Hardware Invariant:** `MOTOR_OUTPUT_ENABLED = 0` strictly enforced.  
**Serial Invariant:** `SerialMCUTransport.PROTOCOL_CONFIRMED = False` strictly enforced.
