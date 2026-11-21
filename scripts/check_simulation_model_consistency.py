#!/usr/bin/env python3
"""Fail when the active Gazebo world drifts from the ROS robot Xacro."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
XACRO = ROOT / "ros2_ws/src/gracemo_description/urdf/gracemo_gazebo_plugins_integrated.xacro"
WORLD = ROOT / "ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world"


def value(element, path):
    child = element.find(path)
    return child.text.strip() if child is not None and child.text else None


def compare_text(errors, label, xacro_value, world_value):
    if xacro_value != world_value:
        errors.append(f"{label}: Xacro={xacro_value!r}, world={world_value!r}")


def compare_number(errors, label, xacro_value, world_value, tolerance=1e-9):
    try:
        equal = math.isclose(float(xacro_value), float(world_value), abs_tol=tolerance)
    except (TypeError, ValueError):
        equal = False
    if not equal:
        errors.append(f"{label}: Xacro={xacro_value!r}, world={world_value!r}")


def main():
    xacro_root = ET.parse(XACRO).getroot()
    world_root = ET.parse(WORLD).getroot()
    model = world_root.find(".//model[@name='gracemo_vira']")
    if model is None:
        print("FAIL: apartment_floor.world has no gracemo_vira model", file=sys.stderr)
        return 1

    xacro_sensors = {sensor.get("name"): sensor for sensor in xacro_root.findall(".//sensor")}
    world_sensors = {sensor.get("name"): sensor for sensor in model.findall(".//sensor")}
    errors = []

    for name in ("lidar", "camera", "depth_camera", "imu"):
        xs = xacro_sensors.get(name)
        ws = world_sensors.get(name)
        if xs is None or ws is None:
            errors.append(f"{name}: missing from Xacro or active world")
            continue
        compare_text(errors, f"{name}.topic", value(xs, "topic"), value(ws, "topic"))
        compare_number(errors, f"{name}.update_rate", value(xs, "update_rate"), value(ws, "update_rate"))
        if name in ("camera", "depth_camera"):
            for path in ("camera/horizontal_fov", "camera/image/width", "camera/image/height"):
                compare_number(errors, f"{name}.{path}", value(xs, path), value(ws, path))
        if name == "lidar":
            for path in ("ray/scan/horizontal/samples", "ray/range/min", "ray/range/max"):
                compare_number(errors, f"{name}.{path}", value(xs, path), value(ws, path))

    xacro_drive = xacro_root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    world_drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    if xacro_drive is None or world_drive is None:
        errors.append("DiffDrive: plugin missing from Xacro or active world")
    else:
        for path in (
            "wheel_separation", "wheel_radius", "min_linear_acceleration",
            "max_linear_acceleration", "min_angular_acceleration",
            "max_angular_acceleration", "min_linear_velocity", "max_linear_velocity",
            "min_angular_velocity", "max_angular_velocity", "odom_publish_frequency",
        ):
            compare_number(errors, f"DiffDrive.{path}", value(xacro_drive, path), value(world_drive, path))
        for path in ("topic", "odom_topic", "frame_id", "child_frame_id"):
            compare_text(errors, f"DiffDrive.{path}", value(xacro_drive, path), value(world_drive, path))
        for side in ("left_joint", "right_joint"):
            xacro_joints = sorted(node.text.strip() for node in xacro_drive.findall(side) if node.text)
            world_joints = sorted(node.text.strip() for node in world_drive.findall(side) if node.text)
            if xacro_joints != world_joints:
                errors.append(f"DiffDrive.{side}: Xacro={xacro_joints!r}, world={world_joints!r}")

    if errors:
        print("FAIL: active simulation model differs from the Xacro:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print("PASS: active Gazebo sensors and differential-drive limits match the ROS Xacro.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
