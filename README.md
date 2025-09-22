# robot_bringup

Bringup / Launch‑Orchestrierung für das Robot System (controller_manager, spawners, micro-ROS agent).

Wichtige Dateien
- launch/bringup.launch.py — Hauptbringup
- launch/microros_agent.launch.py — (optional) micro-ROS Agent
- config/ros2_control_params.yaml — ros2_control hardware & joint config
- scripts/spawn_controllers.sh — Hilfsskript

Startbeispiel
- Mit URDF und ros2_control params:
  ros2 launch robot_bringup bringup.launch.py robot_model:=my_steel drive_type:=mecanum serial_port:=/dev/ttyACM0

Parameter
- robot_model (my_steel / robot_xl)
- drive_type (mecanum / diff)
- microros (true/false)
- serial_port, baudrate

Fehlerbehebung
- controller_manager logs prüfen
- ros2 param get /controller_manager ros__parameters
- ros2 service call /controller_manager/list_controllers