#!/usr/bin/env python3
"""
Run inside Blender:
blender robot.blend --background --python convert_blend.py -- --output generated_robot --config config.json
Exports mesh objects to STL and generates an initial URDF. Wheel recognition is intentionally conservative.
"""
import bpy
import os, sys, json, math, struct, argparse
from mathutils import Matrix, Vector, Quaternion

def args_after_double_dash():
    argv = sys.argv
    argv = argv[argv.index("--") + 1:] if "--" in argv else []
    p = argparse.ArgumentParser()
    p.add_argument("--output", default="generated_robot")
    p.add_argument("--config", default="")
    return p.parse_args(argv)

def safe_name(s):
    out = "".join(c.lower() if c.isalnum() else "_" for c in s)
    while "__" in out:
        out = out.replace("__", "_")
    return out.strip("_") or "object"

def vec3(v):
    return [float(v.x), float(v.y), float(v.z)]

def rpy_from_matrix(m):
    # URDF fixed-axis roll-pitch-yaw representation.
    e = m.to_euler("XYZ")
    return [float(e.x), float(e.y), float(e.z)]

def write_binary_stl(filepath, triangles, name=b"GRACEMO"):
    header = (name[:80]).ljust(80, b" ")
    with open(filepath, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(triangles)))
        for a, b, c in triangles:
            n = (b - a).cross(c - a)
            if n.length > 1e-12:
                n.normalize()
            else:
                n = Vector((0, 0, 0))
            f.write(struct.pack("<12fH", n.x, n.y, n.z,
                                a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, 0))

def main():
    args = args_after_double_dash()
    outdir = os.path.abspath(args.output)
    meshdir = os.path.join(outdir, "meshes")
    os.makedirs(meshdir, exist_ok=True)

    cfg = {}
    if args.config and os.path.isfile(args.config):
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = json.load(f)

    yaw = math.radians(float(cfg.get("blender_to_ros_yaw_degrees", 90.0)))
    C = Matrix.Rotation(yaw, 3, "Z")
    C4 = C.to_4x4()
    wheel_names_cfg = set(cfg.get("wheel_object_names", []))
    tokens = [t.lower() for t in cfg.get("wheel_name_tokens", ["wheel", "tyre", "tire", "roller"])]
    expected = int(cfg.get("expected_wheel_count", 4))
    base_name = cfg.get("base_link_name", "base_link")
    robot_name = cfg.get("robot_name", "gracemo_vira")

    scene_objects = [o for o in bpy.context.scene.objects if o.type == "MESH" and not o.hide_render]
    inventory = []
    exported = []
    wheel_objects = []

    for obj in scene_objects:
        # Apply evaluated modifiers; transform local vertices by object scale only.
        depsgraph = bpy.context.evaluated_depsgraph_get()
        eval_obj = obj.evaluated_get(depsgraph)
        mesh = eval_obj.to_mesh()
        try:
            mesh.calc_loop_triangles()
            scale = Matrix.Diagonal((obj.scale.x, obj.scale.y, obj.scale.z, 1.0))
            tris = []
            for tri in mesh.loop_triangles:
                pts = []
                for vi in tri.vertices:
                    v = mesh.vertices[vi].co
                    # Scale in object-local coordinates, then convert Blender basis to ROS basis.
                    vs = Vector((v.x * obj.scale.x, v.y * obj.scale.y, v.z * obj.scale.z))
                    pts.append(C @ vs)
                tris.append(tuple(pts))
            if not tris:
                continue
            name = safe_name(obj.name)
            filename = name + ".stl"
            path = os.path.join(meshdir, filename)
            write_binary_stl(path, tris, name.encode("ascii", "ignore") or b"GRACEMO")

            # Convert object transform into ROS coordinates.
            world_loc = C @ obj.matrix_world.translation
            world_rot_bl = obj.matrix_world.to_3x3().normalized()
            world_rot_ros = C @ world_rot_bl @ C.inverted()
            rpy = rpy_from_matrix(world_rot_ros)
            dims = [float(d) for d in obj.dimensions]
            item = {
                "object_name": obj.name,
                "safe_name": name,
                "mesh": "meshes/" + filename,
                "dimensions_blender": dims,
                "location_ros": vec3(world_loc),
                "rpy_ros": rpy,
                "triangles": len(tris),
                "vertex_count": len(mesh.vertices)
            }
            inventory.append(item)
            exported.append(item)

            if obj.name in wheel_names_cfg or any(tok in obj.name.lower() for tok in tokens):
                wheel_objects.append(item)
        finally:
            eval_obj.to_mesh_clear()

    with open(os.path.join(outdir, "scene_inventory.json"), "w", encoding="utf-8") as f:
        json.dump({
            "blender_version": bpy.app.version_string,
            "blend_file": bpy.data.filepath,
            "mesh_object_count": len(scene_objects),
            "exported_mesh_count": len(exported),
            "objects": inventory,
            "wheel_candidates": [x["object_name"] for x in wheel_objects]
        }, f, indent=2)

    if len(wheel_objects) != expected:
        note = f"""# Conversion stopped before URDF generation

Exported {len(exported)} mesh objects to STL and wrote `scene_inventory.json`.

Expected wheel count: {expected}
Detected wheel candidates: {len(wheel_objects)}
Candidates: {[x['object_name'] for x in wheel_objects]}

The script deliberately does not generate a driveable URDF because wheel identification is ambiguous.
Review `scene_inventory.json`, then set `wheel_object_names` in the conversion config to the exact Blender object names and rerun.
"""
        with open(os.path.join(outdir, "CONVERSION_NOTES.md"), "w", encoding="utf-8") as f:
            f.write(note)
        print(note)
        raise SystemExit(2)

    wheel_set = {x["object_name"] for x in wheel_objects}
    base_visuals = [x for x in exported if x["object_name"] not in wheel_set]
    # Provisional inertial model. Fixed visual components attach to base; wheel links are dynamic.
    total_mass = float(cfg.get("provisional_total_mass_kg", 38.5))
    wheel_mass = float(cfg.get("provisional_wheel_mass_kg_each", 1.5))
    base_mass = max(1.0, total_mass - wheel_mass * len(wheel_objects))
    axis = cfg.get("wheel_joint_axis_ros", [0.0, 1.0, 0.0])

    def origin(xyz, rpy):
        return f'<origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}"/>'

    lines = [
        '<?xml version="1.0"?>',
        f'<robot name="{robot_name}">',
        f'  <link name="{base_name}">',
        f'    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="{base_mass:.4f}"/><inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/></inertial>'
    ]
    for item in base_visuals:
        lines.append(f'    <visual>{origin(item["location_ros"], item["rpy_ros"])}<geometry><mesh filename="{item["mesh"]}"/></geometry></visual>')
        lines.append(f'    <collision>{origin(item["location_ros"], item["rpy_ros"])}<geometry><mesh filename="{item["mesh"]}"/></geometry></collision>')
    lines.append('  </link>')

    for idx, item in enumerate(wheel_objects):
        link = "wheel_" + str(idx + 1) + "_link"
        joint = "wheel_" + str(idx + 1) + "_joint"
        # Wheel mesh local coordinates are centered on object origin only if Blender origin is correctly placed.
        # This is reported as a condition to verify visually.
        lines.append(f'  <link name="{link}">')
        lines.append(f'    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="{wheel_mass:.4f}"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>')
        lines.append(f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{item["mesh"]}"/></geometry></visual>')
        lines.append(f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="{item["mesh"]}"/></geometry></collision>')
        lines.append('  </link>')
        lines.append(f'  <joint name="{joint}" type="continuous">')
        lines.append(f'    <parent link="{base_name}"/><child link="{link}"/>')
        lines.append(f'    {origin(item["location_ros"], item["rpy_ros"])}')
        lines.append(f'    <axis xyz="{axis[0]} {axis[1]} {axis[2]}"/>')
        lines.append('    <dynamics damping="0.1" friction="0.0"/>')
        lines.append('  </joint>')

    lines.append('</robot>')
    urdf_path = os.path.join(outdir, "robot.urdf")
    with open(urdf_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    with open(os.path.join(outdir, "CONVERSION_NOTES.md"), "w", encoding="utf-8") as f:
        f.write(f"""# Conversion notes

- Blender version: {bpy.app.version_string}
- Mesh objects exported: {len(exported)}
- Wheel candidates: {len(wheel_objects)}
- URDF generated: `{urdf_path}`
- ROS coordinate conversion: +{math.degrees(yaw):.1f} degrees about Z basis conversion.
- Provisional total mass: {total_mass:.2f} kg.
- Provisional wheel mass: {wheel_mass:.2f} kg each.
- Base mass assigned as remainder: {base_mass:.2f} kg.
- Wheel inertia values are placeholders and MUST be replaced or validated.
- Base inertia is a placeholder and MUST be replaced or validated.
- All exported meshes are used as collision geometry in this initial URDF. Simplify collision meshes before serious physics testing.
- Review wheel object origins, joint axes, wheel locations, and rotation directions in Gazebo before using the model.
- Sensor definitions are not generated from housing appearance alone.
""")
    print(f"Exported {len(exported)} mesh objects; wrote {urdf_path}")

if __name__ == "__main__":
    main()
