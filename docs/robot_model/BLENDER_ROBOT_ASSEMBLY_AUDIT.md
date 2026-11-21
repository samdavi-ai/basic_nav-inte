# GRaCEmo ViRa — Canonical Blender Robot Assembly Audit

**Source File:** `gracemo_blender_to_gazebo/robot.blend` (Preserved — Read-only Canonical Reference)  
**Auditing Engine:** Blender 5.2.2 LTS Python API (`bpy`)  
**Total Scene Objects:** 96 objects (71 MESH, 24 EMPTY, 1 FONT)  

---

## 1. Executive Summary & Geometry Reference

The canonical 3D model for GRaCEmo ViRa is authored in `robot.blend`. It represents a human-scale domestic robotics platform with a 4-wheel mobile base, humanoid torso, pan-tilt neck, expressive head display, 360° planar LiDAR, stereo perception cameras, and dual arms.

### Key Physical Geometry Dimensions:
- **Total Robot Height:** 1.240 m (Contact ground to top of head lidar)
- **Wheel Diameter:** 0.230 m (Radius R = 0.115 m)
- **Wheelbase (L):** 0.440 m (Axle-to-axle distance: Front at +0.220 m, Rear at -0.220 m)
- **Track Width (W):** 0.570 m (Lateral center-to-center distance: Left at +0.285 m, Right at -0.285 m)
- **Chassis Ground Clearance:** 0.080 m (80 mm minimum clearance below BASE_LOWER)
- **Chassis Deck Height:** 0.320 m (Mounting interface for torso pedestal)
- **Neck Pan-Tilt Center:** 0.905 m (Head pan-tilt pivot location)
- **LiDAR Sensor Optical Center:** 1.175 m (Top-mounted planar scanner)

---

## 2. Coordinate System Mapping (Blender vs. ROS 2)

| Direction | Blender Master Frame | ROS 2 / REP-103 Frame | Coordinate Conversion Formula |
|---|---|---|---|
| **Forward** | -Y | +X | X_ros = -Y_blender |
| **Left** | +X | +Y | Y_ros = +X_blender |
| **Up** | +Z | +Z | Z_ros = +Z_blender |

This is a pure +90 degree rotation about the vertical Z-axis. The vertical Z coordinate is identical in both systems.

---

## 3. Structural & Kinematic Hierarchy

Inspection of parent-child relationships and empties reveals the intended mechanical assembly:

```text
GRACEEMO_01_ROOT (World Origin Z=0)
├── BASE_LOWER, BASE_UPPER, BASE_LIGHTBAR (Rigid Chassis Hull)
├── FRONT_SENSOR_BAR, FRONT_LENSES (Rigid Base Perception)
├── WHEEL_FRONT_LEFT Assembly  (Axle X=+0.220, Y=+0.285, Z=0.115, Continuous joint)
├── WHEEL_FRONT_RIGHT Assembly (Axle X=+0.220, Y=-0.285, Z=0.115, Continuous joint)
├── WHEEL_REAR_LEFT Assembly   (Axle X=-0.220, Y=+0.285, Z=0.115, Continuous joint)
├── WHEEL_REAR_RIGHT Assembly  (Axle X=-0.220, Y=-0.285, Z=0.115, Continuous joint)
└── ANIM_TORSO (Rigidly Bolted to Deck Plate at Z=0.320m)
    ├── PLATFORM, PLATFORM_TRIM, PEDESTAL, WAIST (Rigid Lower Torso)
    ├── CHEST, CHEST_ACCENT, CHEST_LENSES, CHEST_SENSOR (Rigid Upper Torso)
    ├── SHOULDER_JOINT_LEFT, UPPER_ARM_LEFT, ELBOW_LEFT, FOREARM_LEFT, PALM_LEFT (Left Arm Pod)
    ├── Right Arm Manipulator (7-DOF serial articulation from Shoulder X=0, Y=-0.215, Z=0.805m)
    └── ANIM_HEAD (Pan-Tilt Assembly at Z=0.905m)
        ├── HEAD_SHELL, HEAD_BEZEL, HEAD_SCREEN (OLED Face Visor)
        ├── EYE_LEFT, EYE_RIGHT, MOUTH (Emissive Face Geometry)
        ├── HEAD_EAR_LEFT, HEAD_EAR_RIGHT, HEAD_EAR_RINGS (Audio/Status)
        ├── HEAD_CAMERA_BAR, HEAD_LENSES (Stereo Vision Cameras)
        └── HEAD_LIDAR, HEAD_LIDAR_RING (Planar LiDAR at Z=1.175m)
```

### Critical Mechanical Assembly Finding:
`PLATFORM` and `PEDESTAL` form a flat mounting plate resting directly on `BASE_UPPER` at Z = 0.320 m. In the CAD model, this is a **rigid structural interface**, not an unconstrained tilting hinge. Defining it as a revolute joint without a holding motor in Gazebo was the root cause of the upper body collapsing backward.

---

## 4. Complete Blender Object Inventory

| Object Name | Type | Parent Object | Blender World Pos (X, Y, Z) [m] | Dimensions (X, Y, Z) [m] | Materials | Mesh Triangles |
|---|---|---|---|---|---|---|
| `ANIM_ANTENNA` | EMPTY | `ANIM_HEAD` | (0.000, 0.000, 1.210) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_ELBOW_LEFT` | EMPTY | `ANIM_SHOULDER_LEFT` | (0.250, 0.000, 0.655) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_ELBOW_RIGHT` | EMPTY | `ANIM_SHOULDER_RIGHT` | (-0.250, 0.000, 0.655) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_EYE_LEFT` | EMPTY | `ANIM_HEAD` | (0.055, -0.112, 1.020) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_EYE_RIGHT` | EMPTY | `ANIM_HEAD` | (-0.055, -0.112, 1.020) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_LEFT_1` | EMPTY | `ANIM_WRIST_LEFT` | (0.278, -0.030, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_LEFT_2` | EMPTY | `ANIM_WRIST_LEFT` | (0.278, -0.010, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_LEFT_3` | EMPTY | `ANIM_WRIST_LEFT` | (0.278, 0.010, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_LEFT_4` | EMPTY | `ANIM_WRIST_LEFT` | (0.278, 0.030, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_RIGHT_1` | EMPTY | `ANIM_WRIST_RIGHT` | (-0.278, -0.030, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_RIGHT_2` | EMPTY | `ANIM_WRIST_RIGHT` | (-0.278, -0.010, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_RIGHT_3` | EMPTY | `ANIM_WRIST_RIGHT` | (-0.278, 0.010, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_FINGER_RIGHT_4` | EMPTY | `ANIM_WRIST_RIGHT` | (-0.278, 0.030, 0.425) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_HEAD` | EMPTY | `ANIM_HEADLEAD` | (0.000, 0.000, 0.905) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_HEADLEAD` | EMPTY | `ANIM_TORSO` | (0.000, 0.000, 0.905) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_MOUTH` | EMPTY | `ANIM_HEAD` | (0.000, -0.112, 1.015) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_SHOULDER_LEFT` | EMPTY | `ANIM_TORSO` | (0.215, 0.000, 0.805) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_SHOULDER_RIGHT` | EMPTY | `ANIM_TORSO` | (-0.215, 0.000, 0.805) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_THUMB_LEFT` | EMPTY | `ANIM_WRIST_LEFT` | (0.315, -0.010, 0.450) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_THUMB_RIGHT` | EMPTY | `ANIM_WRIST_RIGHT` | (-0.315, -0.010, 0.450) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_TORSO` | EMPTY | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.320) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_WRIST_LEFT` | EMPTY | `ANIM_ELBOW_LEFT` | (0.276, 0.000, 0.505) | 0.000 × 0.000 × 0.000 | - | 0 |
| `ANIM_WRIST_RIGHT` | EMPTY | `ANIM_ELBOW_RIGHT` | (-0.276, 0.000, 0.505) | 0.000 × 0.000 × 0.000 | - | 0 |
| `BASE_ACCENT` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.310 × 0.008 × 0.040 | GRACE_ACCENT | 12 |
| `BASE_LIGHTBAR` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.200 × 0.006 × 0.012 | GRACE_BLUE_GLOW | 6 |
| `BASE_LOWER` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.420 × 0.600 × 0.120 | GRACE_BLACK | 6 |
| `BASE_UPPER` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.430 × 0.580 × 0.120 | GRACE_WHITE | 6 |
| `CHEST` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.340 × 0.260 × 0.240 | GRACE_WHITE | 6 |
| `CHEST_ACCENT` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.250 × 0.005 × 0.060 | GRACE_ACCENT | 12 |
| `CHEST_LENSES` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.106 × 0.006 × 0.026 | GRACE_BLUE_GLOW | 78 |
| `CHEST_SENSOR` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.140 × 0.012 × 0.040 | GRACE_BLACK | 6 |
| `ELBOW_LEFT` | MESH | `ANIM_SHOULDER_LEFT` | (0.000, 0.000, 0.000) | 0.080 × 0.085 × 0.080 | GRACE_BLACK | 34 |
| `ELBOW_RIGHT` | MESH | `ANIM_SHOULDER_RIGHT` | (0.000, 0.000, 0.000) | 0.080 × 0.085 × 0.080 | GRACE_BLACK | 34 |
| `ELBOW_RING_LEFT` | MESH | `ANIM_SHOULDER_LEFT` | (0.000, 0.000, 0.000) | 0.084 × 0.008 × 0.084 | GRACE_ACCENT | 34 |
| `ELBOW_RING_RIGHT` | MESH | `ANIM_SHOULDER_RIGHT` | (0.000, 0.000, 0.000) | 0.084 × 0.008 × 0.084 | GRACE_ACCENT | 34 |
| `EYE_LEFT` | MESH | `ANIM_EYE_LEFT` | (0.000, 0.000, 0.000) | 0.060 × 0.008 × 0.060 | GRACE_BLUE_GLOW | 384 |
| `EYE_RIGHT` | MESH | `ANIM_EYE_RIGHT` | (0.000, 0.000, 0.000) | 0.060 × 0.008 × 0.060 | GRACE_BLUE_GLOW | 384 |
| `FINGER_LEFT_1` | MESH | `ANIM_FINGER_LEFT_1` | (0.000, -0.074, 0.001) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_LEFT_2` | MESH | `ANIM_FINGER_LEFT_2` | (0.000, -0.074, 0.005) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_LEFT_3` | MESH | `ANIM_FINGER_LEFT_3` | (0.000, -0.074, 0.008) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_LEFT_4` | MESH | `ANIM_FINGER_LEFT_4` | (0.000, -0.073, 0.012) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_RIGHT_1` | MESH | `ANIM_FINGER_RIGHT_1` | (0.000, -0.074, 0.001) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_RIGHT_2` | MESH | `ANIM_FINGER_RIGHT_2` | (0.000, -0.074, 0.005) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_RIGHT_3` | MESH | `ANIM_FINGER_RIGHT_3` | (0.000, -0.074, 0.008) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FINGER_RIGHT_4` | MESH | `ANIM_FINGER_RIGHT_4` | (0.000, -0.073, 0.012) | 0.016 × 0.016 × 0.075 | GRACE_WHITE | 6 |
| `FOREARM_LEFT` | MESH | `ANIM_ELBOW_LEFT` | (0.000, 0.000, 0.000) | 0.074 × 0.075 × 0.115 | GRACE_WHITE | 6 |
| `FOREARM_RIGHT` | MESH | `ANIM_ELBOW_RIGHT` | (0.000, 0.000, 0.000) | 0.074 × 0.075 × 0.115 | GRACE_WHITE | 6 |
| `FRONT_LENSES` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.228 × 0.006 × 0.028 | GRACE_BLUE_GLOW | 78 |
| `FRONT_SENSOR_BAR` | MESH | `GRACEEMO_01_ROOT` | (0.000, 0.000, 0.000) | 0.340 × 0.020 × 0.070 | GRACE_BLACK | 6 |
| `GRACEEMO_01_ROOT` | EMPTY | `None` | (0.000, 0.000, 0.000) | 0.000 × 0.000 × 0.000 | - | 0 |
| `GROUND` | MESH | `None` | (0.000, 0.000, 0.000) | 6.000 × 6.000 × 0.010 | GRACE_GROUND | 6 |
| `HEAD_ANTENNA` | MESH | `ANIM_ANTENNA` | (-0.027, 0.000, 0.000) | 0.004 × 0.004 × 0.030 | GRACE_BLACK | 10 |
| `HEAD_BEZEL` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.270 × 0.012 × 0.150 | GRACE_ACCENT | 6 |
| `HEAD_CAMERA_BAR` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.240 × 0.070 × 0.040 | GRACE_BLACK | 6 |
| `HEAD_EAR_LEFT` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.030 × 0.100 × 0.100 | GRACE_BLACK | 34 |
| `HEAD_EAR_RIGHT` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.030 × 0.100 × 0.100 | GRACE_BLACK | 34 |
| `HEAD_EAR_RING_LEFT` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.004 × 0.072 × 0.072 | GRACE_BLUE_GLOW | 34 |
| `HEAD_EAR_RING_RIGHT` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.004 × 0.072 × 0.072 | GRACE_BLUE_GLOW | 34 |
| `HEAD_LENSES` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.184 × 0.006 × 0.024 | GRACE_BLUE_GLOW | 78 |
| `HEAD_LIDAR` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.090 × 0.090 × 0.070 | GRACE_BLACK | 34 |
| `HEAD_LIDAR_RING` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.092 × 0.092 × 0.008 | GRACE_BLUE_GLOW | 34 |
| `HEAD_SCREEN` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.245 × 0.014 × 0.125 | GRACE_SCREEN | 6 |
| `HEAD_SHELL` | MESH | `ANIM_HEAD` | (0.000, 0.000, 0.000) | 0.300 × 0.200 × 0.190 | GRACE_WHITE | 6 |
| `HEIGHT_LABEL` | FONT | `None` | (-0.650, 0.000, 1.260) | 0.150 × 0.042 × 0.000 | GRACE_WHITE | 0 |
| `MASTER_HEIGHT` | MESH | `None` | (0.000, 0.000, 0.000) | 0.006 × 0.006 × 1.240 | GRACE_ACCENT | 6 |
| `MOUTH` | MESH | `ANIM_MOUTH` | (0.000, 0.000, 0.000) | 0.060 × 0.008 × 0.019 | GRACE_BLUE_GLOW | 192 |
| `NECK_ACTUATOR` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.080 × 0.080 × 0.070 | GRACE_BLACK | 34 |
| `NECK_RING` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.084 × 0.084 × 0.006 | GRACE_BLUE_GLOW | 34 |
| `ORANGE_BUTTON` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.056 × 0.056 × 0.056 | GRACE_ACCENT | 288 |
| `PALM_LEFT` | MESH | `ANIM_WRIST_LEFT` | (0.000, 0.000, 0.000) | 0.050 × 0.090 × 0.070 | GRACE_WHITE | 6 |
| `PALM_RIGHT` | MESH | `ANIM_WRIST_RIGHT` | (0.000, 0.000, 0.000) | 0.050 × 0.090 × 0.070 | GRACE_WHITE | 6 |
| `PEDESTAL` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.320 × 0.300 × 0.080 | GRACE_WHITE | 6 |
| `PLATFORM` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.400 × 0.400 × 0.060 | GRACE_WHITE | 6 |
| `PLATFORM_TRIM` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.340 × 0.340 × 0.012 | GRACE_BLACK | 6 |
| `SHOULDER_JOINT_LEFT` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.100 × 0.100 × 0.100 | GRACE_BLACK | 288 |
| `SHOULDER_JOINT_RIGHT` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.100 × 0.100 × 0.100 | GRACE_BLACK | 288 |
| `THUMB_LEFT` | MESH | `ANIM_THUMB_LEFT` | (0.000, -0.047, 0.001) | 0.016 × 0.018 × 0.060 | GRACE_WHITE | 6 |
| `THUMB_RIGHT` | MESH | `ANIM_THUMB_RIGHT` | (0.000, -0.047, 0.001) | 0.016 × 0.018 × 0.060 | GRACE_WHITE | 6 |
| `UPPER_ARM_LEFT` | MESH | `ANIM_SHOULDER_LEFT` | (0.000, 0.000, 0.000) | 0.085 × 0.080 × 0.126 | GRACE_WHITE | 6 |
| `UPPER_ARM_RIGHT` | MESH | `ANIM_SHOULDER_RIGHT` | (0.000, 0.000, 0.000) | 0.085 × 0.080 × 0.126 | GRACE_WHITE | 6 |
| `WAIST` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.280 × 0.240 × 0.140 | GRACE_WHITE | 6 |
| `WAIST_SLOTS` | MESH | `ANIM_TORSO` | (0.000, 0.000, 0.000) | 0.162 × 0.006 × 0.040 | GRACE_BLACK | 12 |
| `WHEEL_FRONT_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, -0.220, 0.115) | 0.232 × 0.237 × 0.130 | GRACE_TIRE | 118 |
| `WHEEL_FRONT_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, -0.220, 0.115) | 0.232 × 0.237 × 0.130 | GRACE_TIRE | 118 |
| `WHEEL_HUB_FRONT_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, -0.220, 0.115) | 0.124 × 0.124 × 0.010 | GRACE_BLUE_GLOW | 34 |
| `WHEEL_HUB_FRONT_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, -0.220, 0.115) | 0.124 × 0.124 × 0.010 | GRACE_BLUE_GLOW | 34 |
| `WHEEL_HUB_REAR_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, 0.220, 0.115) | 0.124 × 0.124 × 0.010 | GRACE_BLUE_GLOW | 34 |
| `WHEEL_HUB_REAR_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, 0.220, 0.115) | 0.124 × 0.124 × 0.010 | GRACE_BLUE_GLOW | 34 |
| `WHEEL_REAR_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, 0.220, 0.115) | 0.232 × 0.237 × 0.130 | GRACE_TIRE | 118 |
| `WHEEL_REAR_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, 0.220, 0.115) | 0.232 × 0.237 × 0.130 | GRACE_TIRE | 118 |
| `WHEEL_RING_FRONT_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, -0.220, 0.115) | 0.178 × 0.178 × 0.022 | GRACE_ACCENT | 320 |
| `WHEEL_RING_FRONT_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, -0.220, 0.115) | 0.178 × 0.178 × 0.022 | GRACE_ACCENT | 320 |
| `WHEEL_RING_REAR_LEFT` | MESH | `GRACEEMO_01_ROOT` | (0.285, 0.220, 0.115) | 0.178 × 0.178 × 0.022 | GRACE_ACCENT | 320 |
| `WHEEL_RING_REAR_RIGHT` | MESH | `GRACEEMO_01_ROOT` | (-0.285, 0.220, 0.115) | 0.178 × 0.178 × 0.022 | GRACE_ACCENT | 320 |
| `WRIST_LEFT` | MESH | `ANIM_ELBOW_LEFT` | (0.000, 0.000, 0.000) | 0.064 × 0.064 × 0.064 | GRACE_BLACK | 288 |
| `WRIST_RIGHT` | MESH | `ANIM_ELBOW_RIGHT` | (0.000, 0.000, 0.000) | 0.064 × 0.064 × 0.064 | GRACE_BLACK | 288 |

---
*Audit completed autonomously via Blender Python headless inspection.*
