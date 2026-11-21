# GRaCEmo ViRa — Blender to ROS 2 / Gazebo Harmonic conversion kit

This package is a **reproducible conversion toolkit** for the supplied `robot.blend`. It includes a Blender-side exporter and a ROS 2 description package generator. The uploaded Blender file has not been executed in this environment because a Blender executable / `bpy` runtime is unavailable here. Therefore, this archive is the conversion toolchain, not a claim that the meshes have already been exported or that Gazebo tests have passed.

## Target

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `ros_gz`
- URDF/Xacro-compatible robot description

## What it does

The Blender script:
1. Inspects mesh objects and writes an inventory report.
2. Exports each mesh object to an STL mesh with scale and Blender-to-ROS axis conversion baked into vertices.
3. Converts object positions and orientations into ROS coordinates.
4. Identifies likely wheels using object names. It deliberately stops before generating a driveable URDF if it cannot identify four wheel objects confidently.
5. Generates a URDF with a mobile base, fixed visual components, and wheel links/joints if four wheel objects are identified.
6. Writes a JSON manifest and an initial parameter report.

## Important limitations

- Object names are used to identify wheels. If the wheel names do not contain a term such as `wheel`, `tire`, `tyre`, or `roller`, edit `config/conversion_config.json` after reviewing the inventory report.
- The wheel axis and physical properties are provisional and must be checked against the actual Blender geometry.
- Masses and inertia are provisional. They are not verified hardware specifications.
- Sensor housings are not proof of sensor type. Gazebo sensor plugins should only be enabled after the intended sensor type and mounting frame are confirmed.
- The script does not claim a tested Gazebo launch. You must run the validation commands below on a machine with Blender, ROS 2 Jazzy, and Gazebo Harmonic installed.

## 1. Place the source file

Copy `robot.blend` into this directory, or pass its absolute path to the script.

## 2. Run the Blender exporter

Run Blender's Python script from the command line:

```bash
blender /path/to/robot.blend --background --python scripts/convert_blend.py -- \
  --output ./generated_robot \
  --config config/conversion_config.json
```

The script writes:
- `generated_robot/meshes/*.stl`
- `generated_robot/robot.urdf`
- `generated_robot/scene_inventory.json`
- `generated_robot/CONVERSION_NOTES.md`

If it cannot confidently identify four wheels, it writes the inventory and exits with a clear message. Inspect `scene_inventory.json`, then update the wheel object names in `config/conversion_config.json` and rerun.

## 3. Validate the generated description

```bash
source /opt/ros/jazzy/setup.bash
check_urdf generated_robot/robot.urdf
```

If Xacro is used in a later iteration, also run `xacro` and validate its expanded URDF.

## 4. Build a ROS 2 description package

Copy the generated `meshes/` and `robot.urdf` into a ROS 2 package's `urdf/` and `meshes/` directories. This toolkit intentionally does not fabricate a controller configuration before the wheel joint names, wheel axes, and physical properties have been reviewed.

## 5. Required Gazebo tests

After reviewing the generated URDF and configuring the actual controller integration:

- Spawn the robot in Gazebo Harmonic.
- Confirm the robot settles without falling or exploding.
- Confirm wheel joints and wheel-ground contacts.
- Send low-speed forward, reverse, rotate, and stop commands.
- Verify odometry and TF.
- Enable each sensor independently and inspect actual message data.
- Test RViz mesh display and frame alignment.
- Test clean shutdown and restart.

A topic existing is not proof that the sensor data is valid. An active controller is not proof that the robot moves correctly.

## Coordinate convention

ROS: X forward, Y left, Z up. The converter uses a +90-degree Z-axis basis conversion from the Blender scene convention, consistent with the project's stated Blender forward direction of −Y. Confirm the result visually before using the robot for navigation.

## Provisional values

The converter uses placeholder mass/inertia values for initial structural export. Replace them with CAD-derived mass properties or measured/mechanically justified values before using simulation results for engineering decisions.
