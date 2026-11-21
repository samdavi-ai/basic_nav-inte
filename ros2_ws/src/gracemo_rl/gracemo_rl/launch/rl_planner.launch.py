#!/usr/bin/env python3
"""
Launch file for RL Local Planner Node.
Runs gracemo_rl/rl_local_planner with configurable ONNX model path and topics.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from gracemo_rl.common.limits import (
    RL_MAX_LINEAR_VEL,
    RL_MAX_ANGULAR_VEL,
    RL_MIN_REVERSE_VEL
)


def generate_launch_description():
    pkg_share = get_package_share_directory("gracemo_rl")
    default_model = os.path.join(pkg_share, "models", "gracemo_sac_policy.onnx")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true"
    )

    declare_model_path = DeclareLaunchArgument(
        "model_path",
        default_value=default_model,
        description="Path to ONNX policy checkpoint"
    )

    declare_cmd_vel_topic = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel_rl",
        description="Output velocity topic for RL local planner"
    )

    declare_lookahead_dist = DeclareLaunchArgument(
        "lookahead_dist",
        default_value="0.80",
        description="Lookahead waypoint tracking distance in meters"
    )

    rl_planner_node = Node(
        package="gracemo_rl",
        executable="rl_local_planner",
        name="rl_local_planner",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "model_path": LaunchConfiguration("model_path"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "lookahead_dist": LaunchConfiguration("lookahead_dist"),
            "max_linear_vel": RL_MAX_LINEAR_VEL,
            "max_angular_vel": RL_MAX_ANGULAR_VEL,
            "allow_reverse": True,
            "min_reverse_vel": RL_MIN_REVERSE_VEL,
            "watchdog_timeout_sec": 0.50,
            "control_rate_hz": 10.0,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_model_path,
        declare_cmd_vel_topic,
        declare_lookahead_dist,
        rl_planner_node
    ])

