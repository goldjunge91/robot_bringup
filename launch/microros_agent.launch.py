"""Launch file to start the micro-ROS Agent either via local ros2 executable or Docker.

Usage examples:
  ros2 launch robot_bringup microros_agent.launch.py use_docker:=false transport:=serial \
    serial_port:=/dev/ttyACM0 baudrate:=115200

  ros2 launch robot_bringup microros_agent.launch.py use_docker:=true transport:=serial \
    serial_port:=/dev/ttyACM0 baudrate:=115200

Arguments:
  use_docker: (true/false) run agent in Docker container (default: false)
  transport: serial|udp4|tcp4|canfd (default: serial)
  serial_port: device path for serial transport (default: /dev/ttyACM0)
  baudrate: serial baudrate (default: 115200)
  udp_port: UDP/TCP port (default: 8888)

This file intentionally keeps the logic small and shells out to either the
`ros2 run micro_ros_agent micro_ros_agent ...` command or the official
`microros/micro-ros-agent` Docker image.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import ExecuteProcess, LogInfo


def _generate_agent_process(context, *args, **kwargs):
    use_docker = LaunchConfiguration('use_docker').perform(context)
    transport = LaunchConfiguration('transport').perform(context)
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baudrate = LaunchConfiguration('baudrate').perform(context)
    udp_port = LaunchConfiguration('udp_port').perform(context)

    actions = []

    if use_docker.lower() in ('1', 'true', 'yes'):
        # Run official Docker image. Use --network host to simplify network transports
        # and --privileged to allow access to serial devices if necessary.
        cmd = [
            'docker', 'run', '--rm', '--name', 'microros_agent_launch',
            '--privileged', '--network', 'host',
            'microros/micro-ros-agent:humble'
        ]
        if transport == 'serial':
            cmd += ['serial', '--dev', serial_port, '-b', baudrate, '-v6']
        elif transport in ('udp4', 'tcp4'):
            cmd += [transport, '--port', udp_port, '-v6']
        elif transport == 'canfd':
            # user must set CAN device in serial_port variable when using canfd
            cmd += ['canfd', '--dev', serial_port, '-v6']
        else:
            cmd += [transport, '--port', udp_port, '-v6']

        actions.append(LogInfo(msg=['Starting micro-ROS Agent (docker): ', str(cmd)]))
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))
    else:
        # Use system-installed micro-ROS Agent via ros2 run
        cmd = ['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent']
        if transport == 'serial':
            cmd += ['serial', '--dev', serial_port, '-b', baudrate, '-v6']
        elif transport in ('udp4', 'tcp4'):
            cmd += [transport, '--port', udp_port, '-v6']
        elif transport == 'canfd':
            cmd += ['canfd', '--dev', serial_port, '-v6']
        else:
            cmd += [transport, '--port', udp_port, '-v6']

        actions.append(LogInfo(msg=['Starting micro-ROS Agent (ros2): ', str(cmd)]))
        actions.append(ExecuteProcess(cmd=cmd, output='screen'))

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('use_docker', default_value='false', description='Run micro-ROS Agent inside Docker'))
    ld.add_action(DeclareLaunchArgument('transport', default_value='serial', description='Transport: serial|udp4|tcp4|canfd'))
    ld.add_action(DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0', description='Serial device (if using serial)'))
    ld.add_action(DeclareLaunchArgument('baudrate', default_value='115200', description='Serial baudrate'))
    ld.add_action(DeclareLaunchArgument('udp_port', default_value='8888', description='UDP/TCP port for network transports'))

    # Build appropriate ExecuteProcess at runtime with evaluated launch configurations
    ld.add_action(OpaqueFunction(function=_generate_agent_process))

    return ld
