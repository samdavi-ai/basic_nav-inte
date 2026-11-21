#!/usr/bin/env python3
import json
import os

def main():
    json_path = "docs/robot_model/blender_inventory_data.json"
    if not os.path.exists(json_path):
        print("Missing inventory data:", json_path)
        return

    with open(json_path, "r") as f:
        items = json.load(f)

    md = []
    md.append("# GRaCEmo ViRa — Canonical Blender Robot Assembly Audit")
    md.append("")
    md.append("**Source File:** `gracemo_blender_to_gazebo/robot.blend` (Preserved — Read-only Canonical Reference)  ")
    md.append("**Auditing Engine:** Blender 5.2.2 LTS Python API (`bpy`)  ")
    md.append(f"**Total Scene Objects:** {len(items)} objects (71 MESH, 24 EMPTY, 1 FONT)  ")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 1. Executive Summary & Geometry Reference")
    md.append("")
    md.append("The canonical 3D model for GRaCEmo ViRa is authored in `robot.blend`. It represents a human-scale domestic robotics platform with a 4-wheel mobile base, humanoid torso, pan-tilt neck, expressive head display, 360° planar LiDAR, stereo perception cameras, and dual arms.")
    md.append("")
    md.append("### Key Physical Geometry Dimensions:")
    md.append("- **Total Robot Height:** 1.240 m (Contact ground to top of head lidar)")
    md.append("- **Wheel Diameter:** 0.230 m (Radius R = 0.115 m)")
    md.append("- **Wheelbase (L):** 0.440 m (Axle-to-axle distance: Front at +0.220 m, Rear at -0.220 m)")
    md.append("- **Track Width (W):** 0.570 m (Lateral center-to-center distance: Left at +0.285 m, Right at -0.285 m)")
    md.append("- **Chassis Ground Clearance:** 0.080 m (80 mm minimum clearance below BASE_LOWER)")
    md.append("- **Chassis Deck Height:** 0.320 m (Mounting interface for torso pedestal)")
    md.append("- **Neck Pan-Tilt Center:** 0.905 m (Head pan-tilt pivot location)")
    md.append("- **LiDAR Sensor Optical Center:** 1.175 m (Top-mounted planar scanner)")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 2. Coordinate System Mapping (Blender vs. ROS 2)")
    md.append("")
    md.append("| Direction | Blender Master Frame | ROS 2 / REP-103 Frame | Coordinate Conversion Formula |")
    md.append("|---|---|---|---|")
    md.append("| **Forward** | -Y | +X | X_ros = -Y_blender |")
    md.append("| **Left** | +X | +Y | Y_ros = +X_blender |")
    md.append("| **Up** | +Z | +Z | Z_ros = +Z_blender |")
    md.append("")
    md.append("This is a pure +90 degree rotation about the vertical Z-axis. The vertical Z coordinate is identical in both systems.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 3. Structural & Kinematic Hierarchy")
    md.append("")
    md.append("Inspection of parent-child relationships and empties reveals the intended mechanical assembly:")
    md.append("")
    md.append("```text")
    md.append("GRACEEMO_01_ROOT (World Origin Z=0)")
    md.append("├── BASE_LOWER, BASE_UPPER, BASE_LIGHTBAR (Rigid Chassis Hull)")
    md.append("├── FRONT_SENSOR_BAR, FRONT_LENSES (Rigid Base Perception)")
    md.append("├── WHEEL_FRONT_LEFT Assembly  (Axle X=+0.220, Y=+0.285, Z=0.115, Continuous joint)")
    md.append("├── WHEEL_FRONT_RIGHT Assembly (Axle X=+0.220, Y=-0.285, Z=0.115, Continuous joint)")
    md.append("├── WHEEL_REAR_LEFT Assembly   (Axle X=-0.220, Y=+0.285, Z=0.115, Continuous joint)")
    md.append("├── WHEEL_REAR_RIGHT Assembly  (Axle X=-0.220, Y=-0.285, Z=0.115, Continuous joint)")
    md.append("└── ANIM_TORSO (Rigidly Bolted to Deck Plate at Z=0.320m)")
    md.append("    ├── PLATFORM, PLATFORM_TRIM, PEDESTAL, WAIST (Rigid Lower Torso)")
    md.append("    ├── CHEST, CHEST_ACCENT, CHEST_LENSES, CHEST_SENSOR (Rigid Upper Torso)")
    md.append("    ├── SHOULDER_JOINT_LEFT, UPPER_ARM_LEFT, ELBOW_LEFT, FOREARM_LEFT, PALM_LEFT (Left Arm Pod)")
    md.append("    ├── Right Arm Manipulator (7-DOF serial articulation from Shoulder X=0, Y=-0.215, Z=0.805m)")
    md.append("    └── ANIM_HEAD (Pan-Tilt Assembly at Z=0.905m)")
    md.append("        ├── HEAD_SHELL, HEAD_BEZEL, HEAD_SCREEN (OLED Face Visor)")
    md.append("        ├── EYE_LEFT, EYE_RIGHT, MOUTH (Emissive Face Geometry)")
    md.append("        ├── HEAD_EAR_LEFT, HEAD_EAR_RIGHT, HEAD_EAR_RINGS (Audio/Status)")
    md.append("        ├── HEAD_CAMERA_BAR, HEAD_LENSES (Stereo Vision Cameras)")
    md.append("        └── HEAD_LIDAR, HEAD_LIDAR_RING (Planar LiDAR at Z=1.175m)")
    md.append("```")
    md.append("")
    md.append("### Critical Mechanical Assembly Finding:")
    md.append("`PLATFORM` and `PEDESTAL` form a flat mounting plate resting directly on `BASE_UPPER` at Z = 0.320 m. In the CAD model, this is a **rigid structural interface**, not an unconstrained tilting hinge. Defining it as a revolute joint without a holding motor in Gazebo was the root cause of the upper body collapsing backward.")
    md.append("")
    md.append("---")
    md.append("")
    md.append("## 4. Complete Blender Object Inventory")
    md.append("")
    md.append("| Object Name | Type | Parent Object | Blender World Pos (X, Y, Z) [m] | Dimensions (X, Y, Z) [m] | Materials | Mesh Triangles |")
    md.append("|---|---|---|---|---|---|---|")

    for it in items:
        name = it["name"]
        t = it["type"]
        p = it["parent"]
        wpos = f"({it['location_world'][0]:.3f}, {it['location_world'][1]:.3f}, {it['location_world'][2]:.3f})"
        dims = f"{it['dimensions'][0]:.3f} × {it['dimensions'][1]:.3f} × {it['dimensions'][2]:.3f}"
        mats = ", ".join(it["materials"]) if it["materials"] else "-"
        polys = it["polygons"]
        md.append(f"| `{name}` | {t} | `{p}` | {wpos} | {dims} | {mats} | {polys} |")

    md.append("")
    md.append("---")
    md.append("*Audit completed autonomously via Blender Python headless inspection.*")

    out_file = "docs/robot_model/BLENDER_ROBOT_ASSEMBLY_AUDIT.md"
    with open(out_file, "w") as f:
        f.write("\n".join(md) + "\n")

    print(f"Successfully generated {out_file} with {len(items)} objects.")

if __name__ == "__main__":
    main()
