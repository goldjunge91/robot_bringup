#!/usr/bin/env python3

"""
Complete hardware bringup for robot on Raspberry Pi
Starts all necessary nodes: micro-ROS agent, camera, controllers, foxglove
"""

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
    enable_camera = LaunchConfiguration("enable_camera")
    enable_foxglove = LaunchConfiguration("enable_foxglove")
    camera_device = LaunchConfiguration("camera_device")

    declare_configuration_arg = DeclareLaunchArgument(
        "configuration",
        default_value="basic",
        description="Robot configuration preset",
        choices=["basic", "telepresence", "autonomy", "manipulation", "manipulation_pro"],
    )

    declare_mecanum_arg = DeclareLaunchArgument(
        "mecanum",
        default_value="True",
        description="Use mecanum drive (True) or diff drive (False)",
        choices=["True", "False"],
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable("ROBOT_MODEL_NAME", default_value="robot_xl"),
        description="Specify robot model",
        choices=["robot", "robot_xl"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for all launched nodes",
    )

    declare_include_nerf_arg = DeclareLaunchArgument(
        "include_nerf_launcher",
        default_value="False",
        description="Include Nerf launcher",
        choices=["True", "False"],
    )

    declare_enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value="True",
        description="Enable camera node",
        choices=["True", "False"],
    )

    declare_enable_foxglove_arg = DeclareLaunchArgument(
        "enable_foxglove",
        default_value="True",
        description="Enable Foxglove Bridge",
        choices=["True", "False"],
    )

    declare_camera_device_arg = DeclareLaunchArgument(
        "camera_device",
        default_value="/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0",
        description="Camera device path",
    )

    # Controller launch (includes micro-ROS agent, controllers, robot_state_publisher)
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
            "use_sim": "False",  # Hardware mode!
            "include_nerf_launcher": include_nerf_launcher,
        }.items(),
    )

    # Micro-ROS Agent
    microros_agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "microros_agent.launch.py"]
            )
        ),
    )

    # Camera node
    camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[
            {
                "image_size": [640, 480],
                "camera_frame_id": "camera_link_optical",
                "video_device": camera_device,
                "camera_name": "camera",
                "pixel_format": "YUYV",
                "camera_info_url": "file:///home/pi/.ros/camera_info/camera.yaml",
            }
        ],
        condition=IfCondition(enable_camera),
    )

    # Foxglove Bridge
    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "foxglove.launch.py"]
            )
        ),
        condition=IfCondition(enable_foxglove),
    )

    return LaunchDescription(
        [
            declare_configuration_arg,
            declare_mecanum_arg,
            declare_robot_model_arg,
            declare_namespace_arg,
            declare_include_nerf_arg,
            declare_enable_camera_arg,
            declare_enable_foxglove_arg,
            declare_camera_device_arg,
            microros_agent_launch,
            controller_launch,
            camera_node,
            foxglove_launch,
        ]
    )
