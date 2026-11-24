import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gracemo_gazebo = get_package_share_directory("gracemo_gazebo")
    pkg_gracemo_nav2 = get_package_share_directory("gracemo_nav2")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    headless = LaunchConfiguration("headless", default="false")
    use_rl = LaunchConfiguration("use_rl", default="false")
    use_static_localization = LaunchConfiguration("use_static_localization", default="false")
    launch_nav2 = LaunchConfiguration("launch_nav2", default="true")
    launch_interaction = LaunchConfiguration("launch_interaction", default="true")
    kernel_url = LaunchConfiguration("kernel_url", default="http://127.0.0.1:7780")
    navigation_stagnant_timeout_sec = LaunchConfiguration("navigation_stagnant_timeout_sec", default="12.0")

    mode = LaunchConfiguration("mode", default="localization")
    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(pkg_gracemo_nav2, "maps", "apartment_map.yaml"),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2 visualizer"),
        DeclareLaunchArgument("headless", default_value="false", description="Disable RViz2 and keep the Gazebo server without a GUI"),
        DeclareLaunchArgument("use_rl", default_value="false", description="Enable Reinforcement Learning local navigation policy"),
        DeclareLaunchArgument("use_static_localization", default_value="false", description="Use static identity localization map->odom instead of AMCL"),
        DeclareLaunchArgument("launch_nav2", default_value="true", description="Launch Nav2 and SLAM Toolbox"),
        DeclareLaunchArgument("launch_interaction", default_value="true", description="Launch Interaction & Navigation Bridge"),
        DeclareLaunchArgument("kernel_url", default_value="http://127.0.0.1:7780", description="VIRA Kernel EventBus URL"),
        DeclareLaunchArgument("navigation_stagnant_timeout_sec", default_value="12.0", description="Seconds without measured progress before coordinator cancels and retries a Nav2 goal"),
        DeclareLaunchArgument("mode", default_value="localization", description="Autonomy mode: 'localization' or 'mapping'"),
        DeclareLaunchArgument("map", default_value=os.path.join(pkg_gracemo_nav2, "maps", "apartment_map.yaml"), description="Path to map yaml file"),

        # 1. Base Simulation: Gazebo Harmonic + URDF Robot Description + Physical Sensor Bridge + Kernel Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gracemo_gazebo, "launch", "sim.launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "kernel_url": kernel_url,
                "navigation_stagnant_timeout_sec": navigation_stagnant_timeout_sec,
            }.items()
        ),

        # 2. Autonomous Navigation Stack: Nav2 Planner/Controller + AMCL/SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gracemo_nav2, "launch", "nav2.launch.py")
            ),
            condition=IfCondition(launch_nav2),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": use_rviz,
                "headless": headless,
                "use_rl": use_rl,
                "use_static_localization": use_static_localization,
                "mode": mode,
                "map": map_yaml_file,
            }.items()
        ),

        # 3. Interaction & Voice/Text Navigation Bridge
        Node(
            package="gracemo_bridge",
            executable="interaction_bridge",
            name="gracemo_interaction_bridge",
            output="screen",
            condition=IfCondition(launch_interaction),
            parameters=[{
                "require_coordinator": True,
                "kernel_url": kernel_url,
                "use_sim_time": use_sim_time,
            }]
        ),

        # 4. Command Coordinator & Terrain Safety Monitor
        Node(
            package="gracemo_nav2",
            executable="coordinator_node.py",
            name="gracemo_command_coordinator",
            output="screen",
            condition=IfCondition(launch_nav2),
            parameters=[{
                "use_sim_time": use_sim_time,
                "use_static_localization": use_static_localization,
                "use_room_doorway_waypoints": True,
                "kernel_url": kernel_url,
            }],
        ),
        Node(
            package="gracemo_nav2",
            executable="terrain_monitor_node.py",
            name="gracemo_terrain_monitor",
            output="screen",
            condition=IfCondition(launch_nav2),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
