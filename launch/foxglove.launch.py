#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    max_qos_depth = LaunchConfiguration("max_qos_depth")

    declare_port_arg = DeclareLaunchArgument(
        "port",
        default_value="8765",
        description="WebSocket port for Foxglove Bridge",
    )

    declare_address_arg = DeclareLaunchArgument(
        "address",
        default_value="0.0.0.0",
        description="WebSocket address (0.0.0.0 = all interfaces)",
    )

    declare_max_qos_depth_arg = DeclareLaunchArgument(
        "max_qos_depth",
        default_value="10",
        description="Maximum QoS depth for subscriptions",
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": port,
                "address": address,
                "max_qos_depth": max_qos_depth,
                "send_buffer_limit": 10000000,
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_port_arg,
            declare_address_arg,
            declare_max_qos_depth_arg,
            foxglove_bridge,
        ]
    )
