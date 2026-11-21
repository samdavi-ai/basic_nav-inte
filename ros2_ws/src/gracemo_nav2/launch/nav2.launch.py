import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_gracemo_nav2 = get_package_share_directory("gracemo_nav2")
    pkg_gracemo_rl = get_package_share_directory("gracemo_rl")

    nav2_params_file = os.path.join(pkg_gracemo_nav2, "config", "nav2_params.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    headless = LaunchConfiguration("headless", default="false")
    show_rviz = PythonExpression(["'", use_rviz, "' == 'true' and '", headless, "' != 'true'"])
    use_rl = LaunchConfiguration("use_rl", default="false")

    mode = LaunchConfiguration("mode", default="localization")
    use_static_localization = LaunchConfiguration("use_static_localization", default="false")
    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(pkg_gracemo_nav2, "maps", "apartment_map.yaml"),
    )
    is_mapping = PythonExpression(["'", mode, "' == 'mapping'"])
    is_amcl_localization = PythonExpression(["'", mode, "' == 'localization' and '", use_static_localization, "' != 'true'"])
    is_static_localization = PythonExpression(["'", mode, "' == 'localization' and '", use_static_localization, "' == 'true'"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2 visualizer"),
        DeclareLaunchArgument("headless", default_value="false", description="Disable RViz2 for unattended simulation runs"),
        DeclareLaunchArgument("use_rl", default_value="false", description="Enable Reinforcement Learning local navigation policy"),
        DeclareLaunchArgument("use_static_localization", default_value="false", description="Use static identity localization map->odom instead of AMCL"),
        DeclareLaunchArgument("mode", default_value="localization", description="Operation mode: 'localization' (AMCL + Map Server) or 'mapping' (SLAM Toolbox)"),
        DeclareLaunchArgument("map", default_value=os.path.join(pkg_gracemo_nav2, "maps", "apartment_map.yaml"), description="Full path to map yaml file for localization"),

        # 1A. MODE A: SLAM Toolbox (Live 2D Mapping from 360 LiDAR)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
            ),
            condition=IfCondition(is_mapping),
            launch_arguments={
                "use_sim_time": use_sim_time
            }.items()
        ),

        # 1B. MODE B: Localization (Map Server + AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, "launch", "localization_launch.py")
            ),
            condition=IfCondition(is_amcl_localization),
            launch_arguments={
                "map": map_yaml_file,
                "use_sim_time": use_sim_time,
                "params_file": nav2_params_file,
                "autostart": "true",
            }.items()
        ),

        # 1C. MODE C: Static Localization (Map Server + Identity map->odom TF)
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "yaml_filename": map_yaml_file}],
            condition=IfCondition(is_static_localization),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "autostart": True, "node_names": ["map_server"]}],
            condition=IfCondition(is_static_localization),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_map_to_odom",
            output="screen",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id", "map", "--child-frame-id", "odom"],
            condition=IfCondition(is_static_localization),
        ),

        # 2. Nav2 Navigation Stack (Path Planning & DWB Obstacle Avoidance)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gracemo_nav2, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params_file,
                "autostart": "true"
            }.items()
        ),

        # 3. Command Velocity Multiplexer / Arbiter (Priority: Teleop > RL > Nav2)
        Node(
            package="gracemo_nav2",
            executable="cmd_vel_mux_node.py",
            name="cmd_vel_mux",
            output="screen",
            parameters=[nav2_params_file, {"use_sim_time": use_sim_time}],
        ),

        # 3B. Reinforcement Learning Local Planner (Enabled when use_rl:=true)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gracemo_rl, "launch", "rl_planner.launch.py")
            ),
            condition=IfCondition(use_rl),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "cmd_vel_topic": "/cmd_vel_rl",
            }.items()
        ),

        # 4. RViz2 Visualizer with Nav2 & 2D Goal Pose Tool
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_nav2",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-d", os.path.join(pkg_gracemo_nav2, "rviz", "gracemo_nav2.rviz")],
            condition=IfCondition(show_rviz),
        )
    ])
