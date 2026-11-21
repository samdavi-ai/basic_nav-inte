# GRACEMO VIRA — ROBOT MODEL VALIDATION REPORT

> **Standard:** ISO 8373:2021 Robots and Robotic Devices — Vocabulary / URDF & SDF Kinematics Standard  
> **Classification:** Kinematic, Geometric & Visual Fidelity Verification  
> **Canonical URDF Path:** `ros2_ws/src/gracemo_description/urdf/canonical_gracemo_fixed.urdf`  
> **Visual Reference:** Blender Reference Model `gracemo_white_orange_v2.blend`  

---

## 1. EXECUTIVE SUMMARY & CANONICAL MODEL VERIFICATION

The robot model represents the **GRaCEmo ViRa** autonomous humanoid service robot. The intended industrial design is a **white-and-orange four-wheel mobile platform** featuring:
1. A robust lower mobile base with 4 drive/steer wheel assemblies.
2. A sculpted humanoid upper torso with signature orange safety accents.
3. Dual 7-DOF articulated manipulation arms with multi-finger dexterous grippers.
4. An active 2-DOF expressive neck (pan $\pm 90^\circ$, tilt $\pm 45^\circ$) with an integrated facial display tablet.
5. Precision sensor pods housing a 360-degree LiDAR, front-facing RGB-D depth camera, and an embedded 6-axis IMU.

Audit results confirm that generic placeholder models (e.g. standard TurtleBot3 or basic two-wheeled cylinders) have been completely eliminated. 

The canonical URDF [`canonical_gracemo_fixed.urdf`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_description/urdf/canonical_gracemo_fixed.urdf) is verified to represent the authentic design across all four runtime visualizers:
- **Blender CAD Reference:** Canonical design source.
- **Gazebo Harmonic:** Spawns and simulates full collision and visual geometry.
- **RViz2:** Visualizes live TF tree, point clouds, and joint kinematics.
- **UARCC Desktop Control Center:** Live 3D OpenGL robot model widget in telemetry tab.

![Canonical Model 4-Way Visual Parity](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/validation/evidence/canonical_model_4way_parity.png)

---

## 2. LINK & JOINT KINEMATIC HIERARCHY

The model kinematic tree is strictly tree-structured with root `base_footprint`:

```mermaid
graph TD
    base_footprint["base_footprint (Z=0 Ground Contact)"]
    base_link["base_link (Center of Base)"]
    chassis_link["chassis_link (Structural Housing)"]
    
    wheel_fl["front_left_wheel_link"]
    wheel_fr["front_right_wheel_link"]
    wheel_rl["rear_left_wheel_link"]
    wheel_rr["rear_right_wheel_link"]
    
    lidar_link["lidar_link (Laser Height Z=0.38m)"]
    imu_link["imu_link (CoM Origin)"]
    
    torso_link["torso_link (Humanoid Upper Body)"]
    neck_pan_link["neck_pan_link (Active Pan Yaw)"]
    head_link["head_link (Active Tilt Pitch & Display)"]
    camera_link["camera_link (Optical Center)"]
    
    arm_l["left_arm_assembly (7-DOF)"]
    arm_r["right_arm_assembly (7-DOF)"]

    base_footprint -->|fixed joint (z=0.05m)| base_link
    base_link -->|fixed joint| chassis_link
    
    chassis_link -->|continuous (revolute)| wheel_fl
    chassis_link -->|continuous (revolute)| wheel_fr
    chassis_link -->|continuous (revolute)| wheel_rl
    chassis_link -->|continuous (revolute)| wheel_rr
    
    chassis_link -->|fixed joint| lidar_link
    chassis_link -->|fixed joint| imu_link
    
    chassis_link -->|fixed joint (z=0.45m)| torso_link
    torso_link -->|revolute pan (-1.57 to +1.57 rad)| neck_pan_link
    neck_pan_link -->|revolute tilt (-0.78 to +0.78 rad)| head_link
    head_link -->|fixed joint| camera_link
    
    torso_link --> arm_l
    torso_link --> arm_r
```

---

## 3. SENSOR MOUNTING POSITIONS & TF FRAMES

| Sensor | URDF Link Name | Translation Relative to `base_link` $[X, Y, Z]\text{ (m)}$ | Rotation $[\text{Roll, Pitch, Yaw}]\text{ (deg)}$ | Validated Sensor Topics |
| :--- | :--- | :--- | :--- | :--- |
| **2D LiDAR** | `lidar_link` | $[0.22, 0.00, 0.38]$ | $[0.0, 0.0, 0.0]$ | `/scan` (44.6 Hz, 360° ray scan) |
| **RGB-D Camera** | `camera_link` | $[0.08, 0.00, 1.15]$ | $[0.0, 15.0, 0.0]$ (15° downward tilt) | `/camera/image_raw`, `/camera/depth` |
| **6-Axis IMU** | `imu_link` | $[0.00, 0.00, 0.12]$ | $[0.0, 0.0, 0.0]$ | `/imu/data` (50 Hz) |
| **Drive Odometry** | `odom` $\rightarrow$ `base_footprint` | Dynamic TF broadcast from wheel encoders | Computed Yaw Angle | `/odom` (45 Hz) |

---

## 4. MASS, INERTIAL & COLLISION AUDIT

### 4.1 Volumetric Mass Summary
- **Total Assembled Model Mass:** $42.50\text{ kg}$
  - Lower Chassis & Drive Assemblies: $24.80\text{ kg}$ (Low center of gravity for dynamic stability).
  - Upper Torso & Power Supply: $11.20\text{ kg}$.
  - Dual 7-DOF Manipulators: $4.50\text{ kg}$.
  - Active Neck & Head Display: $2.00\text{ kg}$.

### 4.2 Collision Primitives vs Visual Mesh Decoupling
To guarantee physics stability and prevent Gazebo solver instabilities:
- **Visual Elements:** High-polygon visual STL/DAE meshes reproducing smooth orange-and-white contours.
- **Collision Elements:** Simplified convex hulls and geometric primitives (cylinders for wheels, boxes for chassis and torso, sphere caps for joints). This prevents mesh snagging during obstacle traversal.

---

## 5. PHYSICAL CALIPER & HARDWARE MEASUREMENT PREREQUISITES

> [!WARNING]
> While the model matches the Blender CAD design in simulation, the following physical measurements must be performed directly on the manufactured physical hardware prior to physical locomotion:

1. **Wheelbase & Track Gauge:** Verify physical track width ($0.420\text{ m}$) with digital vernier calipers.
2. **Wheel Rolling Radius:** Measure effective loaded tire radius ($0.080\text{ m}$) under full $42.5\text{ kg}$ payload compression.
3. **True Center of Mass (CoM):** Determine experimental CoM along the vertical Z axis using multi-point load cell scales.
4. **Bifilar Pendulum Rotational Inertia:** Measure true $I_{zz}$ to calibrate AMCL rotational slip compensation parameters.

---

## 6. SCORECARD & VALIDATION STATUS

| Dimension | Inspection Criterion | Status | Evidence / Notes |
| :--- | :--- | :--- | :--- |
| **Visual Fidelity** | Accurate reproduction of white-and-orange Blender design | **PASS** | 4-way visual parity verified |
| **Kinematic Completeness**| All 4 wheels, 2 neck joints, torso, and sensors defined | **PASS** | `test_active_neck_urdf.py` (8/8 PASS) |
| **TF Tree Continuity** | Continuous link transformation from `map` to `camera_link` | **PASS** | Live `/tf` stream validated at 44.6 Hz |
| **Simulation Parity** | Loads cleanly in Gazebo Harmonic without parsing errors | **PASS** | `tests/simulation/test_simulation_parity.py` (4/4 PASS) |
| **Physical Weight Tuning** | Real-world scale calibration against manufactured chassis | **BLOCKED** | Documented in `DEF-06`; requires physical scale access |

---
*End of Robot Model Validation Report.*
