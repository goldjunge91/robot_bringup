#!/usr/bin/env python3

# Simple bringup orchestration for my_steel robot

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configuration = LaunchConfiguration("configuration")
    include_nerf_launcher = LaunchConfiguration("include_nerf_launcher")
    mecanum = LaunchConfiguration("mecanum")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    use_sim = LaunchConfiguration("use_sim")
    microros = LaunchConfiguration("microros")
    camera = LaunchConfiguration("camera")

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description="Robot configuration preset (basic/telepresence/autonomy/manipulation)",
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    # Always use mecanum drive for robot_xl
    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description="Use mecanum drive (always True for robot_xl)",
        choices=["True"],
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value="robot_xl"),
        description="Specify robot model (only robot_xl supported)",
        choices=["robot_xl"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all launched nodes.",
    )

    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Use simulation (Gazebo) with gz_ros2_control",
        choices=["True", "False"],
    )

    declare_microros_arg = DeclareLaunchArgument(
        "microros",
        default_value="False",
        description="Start micro-ROS agent (only meaningful when use_sim=False)",
        choices=["True", "False"],
    )

    declare_include_nerf_arg = DeclareLaunchArgument(
        "include_nerf_launcher",
        default_value="False",
        description="Include Nerf launcher in URDF and controllers",
        choices=["True", "False"],
    )

    declare_camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="True",
        description="Start USB camera node",
        choices=["True", "False"],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={
            "configuration": configuration,
            "mecanum": mecanum,
            "namespace": namespace,
            "robot_model": robot_model,
            "use_sim": use_sim,
            "include_nerf_launcher": include_nerf_launcher,
        }.items(),
    )

    microros_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "microros_agent.launch.py"]
            )
        ),
        condition=IfCondition(microros),
    )

    # USB Camera Node
    camera_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="camera",
        parameters=[{
            "video_device": EnvironmentVariable("CAMERA_DEVICE", default_value="/dev/video0"),
            "image_width": 1920,
            "image_height": 1080,
            "framerate": 30.0,
            "pixel_format": "yuyv",
            "camera_frame_id": "camera_link",
            "io_method": "mmap",
        }],
        condition=IfCondition(camera),
    )

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_robot_model_arg,
            declare_namespace_arg,
            declare_mecanum_arg,
            declare_use_sim_arg,
            declare_microros_arg,
            declare_include_nerf_arg,
            declare_camera_arg,
            microros_agent_launch,
            controller_launch,
            camera_node,
        ]
    )
