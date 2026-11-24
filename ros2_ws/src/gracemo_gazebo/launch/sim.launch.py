import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_ros_gz_sim          = get_package_share_directory("ros_gz_sim")
    pkg_gracemo_description = get_package_share_directory("gracemo_description")
    pkg_gracemo_gazebo      = get_package_share_directory("gracemo_gazebo")

    world_file  = os.path.join(pkg_gracemo_gazebo, "worlds", "apartment_floor.world")
    camera_trial = (os.environ.get("VIRA_TEST_DEPTH_10HZ") == "1"
                    and os.environ.get("GZ_PARTITION", "").startswith("vira-restart-"))
    rgb_rate, depth_rate = 10., 5.
    if camera_trial:
        rgb_rate = depth_rate = 20. if os.getenv("VIRA_TEST_RGBD_20HZ") == "1" else 10.
        world_file = os.path.join(os.environ["GRACEMO_STATE_DIR"], "camera_rate.world")
    xacro_file  = os.path.join(pkg_gracemo_description, "urdf", "gracemo_vira_integrated.urdf.xacro")
    models_path    = os.path.join(pkg_gracemo_gazebo, "models")
    fuel_cache_gz  = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models")
    fuel_cache_ign = os.path.expanduser("~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models")
    cb_tex_path    = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/materials/textures")
    cb_mesh_path   = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/meshes")
    ros_share      = "/opt/ros/jazzy/share"
    resource_paths = f"{models_path}:{fuel_cache_gz}:{fuel_cache_ign}:{cb_tex_path}:{cb_mesh_path}:{os.path.dirname(pkg_gracemo_description)}:{pkg_gracemo_description}:{ros_share}"
    gz_config_path = os.environ.get("GZ_CONFIG_PATH")
    if not gz_config_path and os.path.isdir("/usr/share/gz"):
        # The ROS vendor `gz` wrapper may not include distro configuration files
        # in its own share directory. Use Gazebo's installed command registry.
        gz_config_path = "/usr/share/gz"

    os.environ["GZ_SIM_RESOURCE_PATH"] = resource_paths

    # Route EGL/GLX to NVIDIA vendor driver if available to eliminate Mesa dri2 warning
    if os.path.exists("/usr/share/glvnd/egl_vendor.d/10_nvidia.json") and os.system("which nvidia-smi > /dev/null 2>&1") == 0:
        os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
        os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"

    robot_description = ParameterValue(Command(["xacro ", xacro_file]), value_type=str)
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    kernel_url = LaunchConfiguration("kernel_url", default="http://127.0.0.1:7780")

    nodes = [
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument("kernel_url", default_value="http://127.0.0.1:7780", description="GRaCEmo kernel health and event API URL"),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_paths),
        *([SetEnvironmentVariable("GZ_CONFIG_PATH", gz_config_path)] if gz_config_path else []),
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"),

        # 1. Launch Gazebo Harmonic (gz sim 8) with world (contains embedded robot)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": f"-s -r {world_file}",
                "gz_version": "8"
            }.items()
        ),

        # 2. Robot State Publisher (publishes TF tree from robot_description)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time
            }]
        ),

        # 3. GRaCEmo Physical Sensor Bridge (Gazebo Harmonic <-> ROS 2 topics)
        Node(
            package="gracemo_bridge",
            executable="sensor_bridge",
            name="gracemo_sensor_bridge",
            output="screen",
            parameters=[{
                "rgb_publish_hz": rgb_rate,
                "depth_image_publish_hz": depth_rate,
                "use_sim_time": use_sim_time
            }]
        ),

        # The bridge reconnects to the configured EventBus URL while it is
        # offline, so launch order and non-default kernel URLs do not decide
        # whether the adapter bridge exists for the lifetime of the simulation.
        Node(
            package="gracemo_bridge",
            executable="kernel_bridge",
            name="gracemo_kernel_bridge",
            output="screen",
            parameters=[{
                "kernel_url": kernel_url,
                "use_sim_time": use_sim_time,
            }]
        ),
    ]

    return LaunchDescription(nodes)
