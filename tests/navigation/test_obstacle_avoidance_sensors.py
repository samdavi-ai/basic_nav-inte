#!/usr/bin/env python3
"""
Unit and Integration Tests for GRaCEmo ViRa Multi-Sensor Obstacle Avoidance & Collision Prevention.
Validates:
1. Physical footprint model matches actual URDF kinematic envelope (wheels + chassis).
2. Local and Global costmaps subscribe to both 2D LiDAR (/scan) and 3D Depth PointCloud (/camera/depth/points).
3. Costmap inflation creates a repulsive potential field centered in corridors (inflation_radius >= 0.60m).
4. Collision Monitor defines enclosing PolygonStop and PolygonSlow zones powered by multi-sensor inputs.
5. DWB Local Planner weights obstacle avoidance critic (BaseObstacle >= 15.0) to dynamically steer around obstacles.
6. Sensor Bridge self-exclusion filter prevents false self-body LiDAR reflections.
7. Hardware safety invariants remain strictly locked.
"""

import os
import re
import yaml
import pytest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent


def test_hardware_safety_invariants_preserved():
    """Verify strictly: MOTOR_OUTPUT_ENABLED = 0 and PROTOCOL_CONFIRMED = False."""
    config_h = ROOT / "firmware" / "config.h"
    assert config_h.exists(), "firmware/config.h not found"
    content = config_h.read_text()
    assert "#define MOTOR_OUTPUT_ENABLED 0" in content, "MOTOR_OUTPUT_ENABLED is not 0!"

    mcu_transport = ROOT / "ros2_ws" / "src" / "gracemo_hardware" / "gracemo_hardware" / "mcu_transport.py"
    if mcu_transport.exists():
        t_content = mcu_transport.read_text()
        assert re.search(r"PROTOCOL_CONFIRMED\s*(:\s*bool)?\s*=\s*False", t_content), "PROTOCOL_CONFIRMED is not False!"


def test_nav2_costmap_footprint_encloses_physical_envelope():
    """Verify that nav2 costmap footprint properly encloses the robot physical envelope."""
    params_path = ROOT / "ros2_ws" / "src" / "gracemo_nav2" / "config" / "nav2_params.yaml"
    assert params_path.exists()
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    local_fp = params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    global_fp = params["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]

    # Footprint string representation check
    assert "0.35" in local_fp and "0.34" in local_fp
    assert "0.35" in global_fp and "0.34" in global_fp

    # Parse points
    import ast
    pts_local = ast.literal_eval(local_fp)
    pts_global = ast.literal_eval(global_fp)

    for pts in (pts_local, pts_global):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        assert max(xs) >= 0.34, f"Forward footprint {max(xs)} must be >= 0.34m to clear front bumper"
        assert min(xs) <= -0.34, f"Rear footprint {min(xs)} must be <= -0.34m to clear rear wheels"
        assert max(ys) >= 0.33, f"Left footprint {max(ys)} must be >= 0.33m to clear outer wheel edge"
        assert min(ys) <= -0.33, f"Right footprint {min(ys)} must be <= -0.33m to clear outer wheel edge"


def test_costmap_multi_sensor_observation_sources():
    """Verify local and global costmaps incorporate both 2D LiDAR and 3D depth point cloud."""
    params_path = ROOT / "ros2_ws" / "src" / "gracemo_nav2" / "config" / "nav2_params.yaml"
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    # Local costmap
    local_obs = params["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]["observation_sources"]
    assert "scan" in local_obs and "pointcloud" in local_obs

    local_pc = params["local_costmap"]["local_costmap"]["ros__parameters"]["obstacle_layer"]["pointcloud"]
    assert local_pc["topic"] == "/camera/depth/points"
    assert local_pc["data_type"] == "PointCloud2"
    assert local_pc["marking"] is True and local_pc["clearing"] is True

    # Global costmap
    global_obs = params["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]["observation_sources"]
    assert "scan" in global_obs and "pointcloud" in global_obs

    global_pc = params["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]["pointcloud"]
    assert global_pc["topic"] == "/camera/depth/points"
    assert global_pc["data_type"] == "PointCloud2"


def test_costmap_inflation_repulsive_gradient():
    """Verify inflation radius and decay rate push navigation away from doorframes and walls."""
    params_path = ROOT / "ros2_ws" / "src" / "gracemo_nav2" / "config" / "nav2_params.yaml"
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    local_inf = params["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    global_inf = params["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]

    assert local_inf["inflation_radius"] >= 0.60, "Inflation radius must be >= 0.60m for corridor centering"
    assert local_inf["cost_scaling_factor"] <= 3.5, "Cost scaling factor must be <= 3.5 for smooth repulsive field"
    assert global_inf["inflation_radius"] >= 0.60
    assert global_inf["cost_scaling_factor"] <= 3.5


def test_dwb_obstacle_critic_weighting():
    """Verify DWB local planner prioritizes dynamic obstacle avoidance over strict path adherence."""
    params_path = ROOT / "ros2_ws" / "src" / "gracemo_nav2" / "config" / "nav2_params.yaml"
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    controller = params["controller_server"]["ros__parameters"]["FollowPath"]
    assert "BaseObstacle" in controller["critics"]
    assert controller["BaseObstacle.scale"] >= 15.0, "BaseObstacle weight must be >= 15.0 to steer clear of walls"
    assert controller["PathAlign.scale"] <= controller["BaseObstacle.scale"], "PathAlign must not dominate BaseObstacle"
    assert controller["PathDist.scale"] <= controller["BaseObstacle.scale"], "PathDist must not dominate BaseObstacle"


def test_collision_monitor_polygons_and_sensors():
    """Verify collision monitor has stop/slow zones enclosing the robot with multi-sensor sources."""
    params_path = ROOT / "ros2_ws" / "src" / "gracemo_nav2" / "config" / "nav2_params.yaml"
    with open(params_path, "r") as f:
        params = yaml.safe_load(f)

    cm = params["collision_monitor"]["ros__parameters"]
    assert "PolygonStop" in cm["polygons"]
    assert "PolygonSlow" in cm["polygons"]

    import ast
    pts_stop = ast.literal_eval(cm["PolygonStop"]["points"])
    xs_stop = [p[0] for p in pts_stop]
    ys_stop = [p[1] for p in pts_stop]

    # PolygonStop must strictly surround the robot outer envelope (0.35m x 0.34m)
    assert max(xs_stop) >= 0.36
    assert min(xs_stop) <= -0.36
    assert max(ys_stop) >= 0.35
    assert min(ys_stop) <= -0.35

    # PolygonSlow must extend further out
    pts_slow = ast.literal_eval(cm["PolygonSlow"]["points"])
    xs_slow = [p[0] for p in pts_slow]
    ys_slow = [p[1] for p in pts_slow]
    assert max(xs_slow) >= 0.60
    assert max(ys_slow) >= 0.45

    # Multi-sensor observation sources
    obs = cm["observation_sources"]
    assert "scan" in obs
    assert "pointcloud" in obs
    assert cm["pointcloud"]["topic"] == "/camera/depth/points"


def test_sensor_bridge_self_exclusion_covers_wheels():
    """Verify sensor_bridge self-exclusion filter covers the full wheel track width."""
    bridge_path = ROOT / "ros2_ws" / "src" / "gracemo_bridge" / "gracemo_bridge" / "sensor_bridge.py"
    assert bridge_path.exists()
    content = bridge_path.read_text()
    assert "-0.35 <= x_bf <= 0.35" in content
    assert "-0.35 <= y_bf <= 0.35" in content
