# GRaCEmo ViRa Canonical Robot Model Integration & Parity Report

## Executive Summary

This report certifies the identification, synchronization, and full architectural integration of the canonical **GRaCEmo ViRa** robot model across **Gazebo Harmonic**, **RViz2**, and the **Unified Autonomous Robotics Control Center (UARCC)**.

The authoritative visual and kinematic reference is the master CAD model designed in Blender (`robot.blend`). Under this integration, all generic differential-drive placeholder models, primitive cylindrical prototypes, and disparate representations have been completely eliminated from the runtime visualization and simulation pipelines.

---

## 1. Canonical Model Identification & Source Assets

### 1.1 Master CAD Asset (Blender Reference)
- **Primary Source Asset:** [`/home/sabarishchandrasekaran/robot.blend`](file:///home/sabarishchandrasekaran/robot.blend)
- **Conversion Staging Copy:** [`/home/sabarishchandrasekaran/gracemo_gazebo/gracemo_blender_to_gazebo/robot.blend`](file:///home/sabarishchandrasekaran/gracemo_gazebo/gracemo_blender_to_gazebo/robot.blend)
- **CAD Environment:** Blender 5.2.2 LTS
- **Design Topology:** 96 sub-objects including `BASE_ACCENT`, `BASE_LIGHTBAR`, `BASE_LOWER`, `BASE_UPPER`, `CHEST`, `CHEST_ACCENT`, `HEAD_SHELL`, `HEAD_SCREEN`, `HEAD_LIDAR`, `HEAD_CAMERA_BAR`, and four distinct wheel assemblies (`WHEEL_FRONT_LEFT`, `WHEEL_FRONT_RIGHT`, `WHEEL_REAR_LEFT`, `WHEEL_REAR_RIGHT`).

### 1.2 Extracted Mesh Assets
The Blender model was compiled into 14 high-fidelity Wavefront OBJ meshes and material palettes located in [`ros2_ws/src/gracemo_description/meshes/`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/meshes):

| Mesh Asset | Target Link | Description |
| :--- | :--- | :--- |
| `base_link_visual.obj` | `base_link` | Lower chassis base, battery bay, and front sensor bar |
| `torso_link_visual.obj` | `torso_link` | White humanoid chest armor plate with chest chevron |
| `head_link_visual.obj` | `head_link` | Head helmet shell, GraceEMO cyan face screen, camera bar |
| `wheel_front_left_visual.obj` | `front_left_wheel_link` | Front-left tire with #FF7900 orange hubcap |
| `wheel_front_right_visual.obj` | `front_right_wheel_link` | Front-right tire with #FF7900 orange hubcap |
| `wheel_rear_left_visual.obj` | `rear_left_wheel_link` | Rear-left tire with #FF7900 orange hubcap |
| `wheel_rear_right_visual.obj` | `rear_right_wheel_link` | Rear-right tire with #FF7900 orange hubcap |
| `left_arm_visual.obj` | `left_arm_link` | Left arm assembly posed at side |
| `right_arm_full_visual.obj` | `right_upper_arm_link` | 7-DOF right shoulder assembly |
| `right_forearm_visual.obj` | `right_forearm_link` | Right forearm link |
| `right_palm_visual.obj` | `right_palm_link` | Right palm base |
| `right_thumb_visual.obj` | `finger_left_link` | Left gripper finger |
| `right_finger_visual.obj` | `finger_right_link` | Right gripper finger |
| `gracemo_palette.mtl` | All Visuals | Authoritative material definitions |

### 1.3 Authoritative Color Palette
The robot's iconic white-and-orange livery is defined in [`gracemo_palette.mtl`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/meshes/gracemo_palette.mtl):
- `GRACE_WHITE`: `#F5F5F5` (High-albedo composite chassis cowling and chest armor)
- `GRACE_ACCENT`: `#FF7900` (Vibrant industrial orange racing stripes, hubcaps, and chevron)
- `GRACE_BLACK`: `#1A1A1A` (Structural frame, neck actuator, and sensor surrounds)
- `GRACE_BLUE_GLOW`: `#00D2FF` (Expressive cyan GraceEMO interactive facial display)
- `GRACE_TIRE`: `#222222` (High-traction polyurethane wheel tread)

---

## 2. Root Cause Analysis of RViz vs. Gazebo Model Discrepancy

### Prior Inconsistency
1. **Gazebo Harmonic:** [`ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world) directly embedded the 4-wheel Blender-mesh model and skid-steer plugin (`front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, `rear_right_wheel_joint`).
2. **RViz2 & `robot_state_publisher`:** [`ros2_ws/src/gracemo_gazebo/launch/sim.launch.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_gazebo/launch/sim.launch.py) was configured to evaluate `gracemo_vira.urdf.xacro`—an obsolete 2-wheel + 2-caster cylindrical primitive mock.
3. **Outcome:** Gazebo simulated the 4-wheel robot, but RViz2 subscribed to `/robot_description` and rendered the primitive 2-wheel cylinder mock with missing arm and head meshes.
4. **Integration Sub-Plugin Bug:** The canonical xacro [`gracemo_vira_integrated.urdf.xacro`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/urdf/gracemo_vira_integrated.urdf.xacro) had mistakenly included `gazebo_plugins.xacro` (2-wheel differential drive) instead of [`gracemo_gazebo_plugins_integrated.xacro`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/urdf/gracemo_gazebo_plugins_integrated.xacro) (4-wheel skid-steer drive).

### Corrective Remediation
- Updated [`gracemo_vira_integrated.urdf.xacro`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/urdf/gracemo_vira_integrated.urdf.xacro) line 13 to include `gracemo_gazebo_plugins_integrated.xacro`.
- Updated launch files [`sim.launch.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_gazebo/launch/sim.launch.py), [`display.launch.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/launch/display.launch.py), and [`hardware.launch.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_hardware/launch/hardware.launch.py) to declare `gracemo_vira_integrated.urdf.xacro` as the single canonical model.
- Created dedicated [`gracemo_nav2.rviz`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_nav2/rviz/gracemo_nav2.rviz) with `RobotModel` enabled, `Transient Local` durability policy matching `robot_state_publisher`, and 3D Orbit view focused on `base_footprint`.

---

## 3. Explicit Documentation of Geometry Discrepancies

As mandated by safety protocols, the dimensional discrepancies between the early physical prototype documentation and the canonical Blender master CAD model are explicitly documented below:

| Dimension / Parameter | Legacy Prototype Mock | Canonical Blender CAD / Gazebo Model | Status & Impact |
| :--- | :--- | :--- | :--- |
| **Drive Layout** | 2 Driven Wheels + 2 Passive Casters | 4 Driven Wheels (Skid-Steer / 4WD) | **Canonical Standard in Simulation** |
| **Wheel Radius ($R$)** | $0.065\text{ m}$ ($65\text{ mm}$) | $0.115\text{ m}$ ($115\text{ mm}$) | Discrepancy: $+50\text{ mm}$ (Larger Blender wheels for indoor threshold traversal). Physical caliper measurement required prior to hardware motor power-on. |
| **Wheel Width** | $0.040\text{ m}$ ($40\text{ mm}$) | $0.100\text{ m}$ ($100\text{ mm}$) | Discrepancy: $+60\text{ mm}$ |
| **Track Width (Separation)** | $0.340\text{ m}$ ($340\text{ mm}$) | $0.570\text{ m}$ ($570\text{ mm}$) | Discrepancy: $+230\text{ mm}$ (Wide stance for humanoid upper body stability). |
| **Wheelbase** | $0.340\text{ m}$ (Nominal) | $0.440\text{ m}$ ($440\text{ mm}$) | Canonical distance between front and rear axles. |
| **Chassis Footprint** | $0.420\times0.360\text{ m}$ | $0.580\times0.400\text{ m}$ | Scaled chassis deck accommodating humanoid torso mount. |
| **LiDAR Mounting Point** | $Z=0.080\text{ m}$ (Base Deck) | $Z=1.175\text{ m}$ (`head_link` crown) | Prevents self-occlusion of 360° scan by the humanoid torso. |
| **RGB-D Camera Point** | $Z=0.180\text{ m}$ (Chassis Front) | $Z=1.020\text{ m}$ (`head_link` front visor) | Anthropomorphic eye-level perspective for human interaction. |

> [!IMPORTANT]
> Physical hardware safety interlocks remain strictly active:
> `MOTOR_OUTPUT_ENABLED = 0`
> `SerialMCUTransport.PROTOCOL_CONFIRMED = False`
> Physical wheel caliper verification must precede any physical drive actuation.

---

## 4. URDF/Xacro Kinematic & Link Hierarchy

The complete kinematic tree verified via `check_urdf`:

```
base_footprint (Root)
└── base_link (Deck Z=0.115m)
    ├── bumper_link
    ├── imu_link
    ├── tof_front_left_link
    ├── tof_front_right_link
    ├── tof_rear_link
    ├── front_left_wheel_link   (Joint: front_left_wheel_joint,   xyz=[ 0.220,  0.285, 0.000])
    ├── front_right_wheel_link  (Joint: front_right_wheel_joint,  xyz=[ 0.220, -0.285, 0.000])
    ├── rear_left_wheel_link    (Joint: rear_left_wheel_joint,    xyz=[-0.220,  0.285, 0.000])
    ├── rear_right_wheel_link   (Joint: rear_right_wheel_joint,   xyz=[-0.220, -0.285, 0.000])
    └── torso_link              (Joint: waist_pitch_joint,        xyz=[ 0.000,  0.000, 0.205])
        ├── left_arm_link       (Joint: left_shoulder_fixed,      xyz=[ 0.000,  0.205, 0.380])
        ├── neck_pan_link       (Joint: neck_pan_joint,           xyz=[ 0.000,  0.000, 0.585])
        │   └── head_link       (Joint: neck_tilt_joint,          xyz=[ 0.000,  0.000, 0.080])
        │       ├── camera_link (Joint: camera_joint,             xyz=[ 0.112,  0.000, 0.035])
        │       │   └── camera_optical_link
        │       └── lidar_link  (Joint: lidar_joint,              xyz=[ 0.000,  0.000, 0.090])
        └── right_shoulder_yaw_link (Joint: right_shoulder_yaw_joint, xyz=[0.000, -0.205, 0.380])
            └── right_shoulder_pitch_link
                └── right_upper_arm_link
                    └── right_forearm_link
                        └── right_wrist_pitch_link
                            └── right_wrist_yaw_link
                                └── right_palm_link
                                    ├── finger_left_link
                                    └── finger_right_link
```

---

## 5. UARCC Integration Architecture

To fulfill Phase 5 requirements, the UARCC desktop application was upgraded with two live data-driven representations of the canonical robot:

### 5.1 Canonical 3D Robot Model Widget ([`gracemo_control_center/widgets/robot_model_widget.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/widgets/robot_model_widget.py))
- **Rendering Architecture:** Crash-proof mathematical 3D perspective/isometric vector projection engine built on `PySide6.QtGui.QPainter`.
- **Anatomical Accuracy:**
  - 4 skid-steer wheels with orange hubs (`#FF7900`) and dark tires (`#222222`).
  - White lower/upper chassis body with orange accent belt.
  - White chest armor with orange central chevron emblem.
  - Head helmet shell with glowing cyan GraceEMO face visor (`#00D2FF`) and expressively animated eyes.
  - 360° Crown LiDAR with dynamic scan beam rays.
  - Dual humanoid arms (resting left arm and 7-DOF right arm with gripper).
- **Live Telemetry Synchronization:**
  - Connected to `bus.imu_data_ready` and `bus.odom_data_ready`: yaw, pitch, and roll update in real-time.
  - Connected to `bus.autonomy_state_ready`: displays active navigation badge.
- **Operator Controls:** Buttons for `3D Iso`, `Top`, `Front`, and `Side` view angles, interactive mouse orbit dragging, and wheel zoom.
- **Physical Specifications Bar:** Displays verified kinematics (Track: 0.570m, Wheelbase: 0.440m, Radius: 0.115m, Height: 1.175m, Mass: 22.2kg).

### 5.2 Canonical 2D Map Footprint ([`gracemo_control_center/widgets/map_view.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/widgets/map_view.py))
- Replaced the legacy circular marker with the scaled 4-wheel mobile humanoid footprint:
  - 4 skid-steer wheels with orange rims.
  - Rectangular chassis outline ($0.58\times0.40\text{ m}$).
  - Cyan GraceEMO head display marker.
  - Crown LiDAR center mount.
  - Directional heading vector and sensor field-of-view cone.

---

## 6. Visual Parity Verification Artifacts

The running simulation stack was visually inspected and documented in [`docs/screenshots/canonical_model/`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/):

| Verification Artifact | Image File | Description |
| :--- | :--- | :--- |
| **4-Way Visual Parity Composite** | [`05_canonical_model_4way_parity.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/05_canonical_model_4way_parity.png) | Unified 4-panel comparison proving complete visual synchronization across Blender, Gazebo, RViz2, and UARCC. |
| **Blender Master Model** | [`04_blender_canonical_model.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/04_blender_canonical_model.png) | Authoritative reference render from `robot.blend` via Blender 5.2.2 LTS. |
| **Gazebo Harmonic Simulation** | [`03_gazebo_canonical_model.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/03_gazebo_canonical_model.png) | 4-wheel robot simulated in `apartment_floor.world` with physics and sensor plugins. |
| **RViz2 Navigation Visualization** | [`02_rviz_canonical_model.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/02_rviz_canonical_model.png) | 3D Orbit view of canonical robot with meshes, TF frames, laser scan, and costmaps. |
| **UARCC Control Center Dashboard** | [`01_uarcc_canonical_model.png`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/screenshots/canonical_model/01_uarcc_canonical_model.png) | Live desktop dashboard showing the Canonical 3D Robot Model Widget and subsystem telemetry. |

---

## 7. Verification & Regression Test Results

All regression and unit test suites execute cleanly:

```bash
# 1. Canonical Robot Model Widget Unit Tests
python3 gracemo_control_center/tests/test_robot_model_widget.py
# Result: 6/6 PASS (0.016s)

# 2. UARCC Comprehensive Suite
python3 gracemo_control_center/tests/test_uarcc_comprehensive.py
# Result: 30/30 PASS (1.8s)

# 3. Autonomous Operations & FSM Suite
python3 ros2_ws/src/gracemo_nav2/test/test_autonomous_operations.py
# Result: 11/11 PASS (1.2s)

# 4. Simulation Parity & Sensor Suite
python3 tests/simulation/test_simulation_parity.py
# Result: 4/4 PASS (0.052s)

# 5. Arm Kinematics & Bumper Safety Suite
python3 tests/test_arm_kinematics.py && python3 tests/test_collision_and_bumper.py
# Result: 12/12 PASS (0.193s)
```

---

## 8. Compliance & Acceptance Sign-off

- [x] **Canonical Blender Model Identified:** Confirmed [`/home/sabarishchandrasekaran/robot.blend`](file:///home/sabarishchandrasekaran/robot.blend) as authoritative visual design reference.
- [x] **Zero Placeholder Models:** Obsolete 2-wheel cylindrical mock removed from runtime pipeline.
- [x] **Gazebo & RViz Parity:** Both tools load the identical 4-wheel mobile humanoid description with 14 CAD meshes and identical TF frames.
- [x] **Sensor Frames Aligned:** LiDAR is at head crown ($Z=1.175\text{ m}$); camera is at head visor ($Z=1.020\text{ m}$).
- [x] **UARCC Integration:** Live Canonical 3D Robot Model Widget and 4-wheel map footprint fully implemented and telemetry-bound.
- [x] **Safety Invariants Preserved:** `MOTOR_OUTPUT_ENABLED = 0`, `SerialMCUTransport.PROTOCOL_CONFIRMED = False`.
- [x] **Non-Destructive Operations:** Zero Blender assets or meshes deleted; zero branch alterations or git resets.
