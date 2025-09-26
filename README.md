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

Micro-ROS Agent
----------------

Optional kann der micro-ROS Agent gestartet werden, wenn die Firmware auf dem Pico micro-ROS verwendet.

Beispiele:

  - Launch (Docker, empfohlen für Tests):
    ros2 launch robot_bringup microros_agent.launch.py use_docker:=true transport:=serial \
      serial_port:=/dev/ttyACM0 baudrate:=115200

  - Launch (system ros2 run):
    ros2 launch robot_bringup microros_agent.launch.py use_docker:=false transport:=serial \
      serial_port:=/dev/ttyACM0 baudrate:=115200

  - Hilfsskript (lokal):
    src/robot_bringup/scripts/run_microros_agent.sh docker serial /dev/ttyACM0 115200

Operations: SBC (Ubuntu) and Remote PC (Ubuntu)
---------------------------------------------

Dieses Kapitel beschreibt präzise, welche Services auf einer SBC (z. B. Raspberry Pi) und auf einem Remote-Entwicklungs-PC (Ubuntu) zu starten sind — mit und ohne echte Hardware. Alle Befehle sind für Ubuntu (Bash) ausgelegt.

Hinweis zu Terminals: Ich verwende `tmux`-Session/Window-Namen zur eindeutigen Identifikation der Terminals. Wenn ihr `tmux` nicht nutzen wollt, sind alternative `gnome-terminal`-Befehle angegeben.

Voraussetzungen (beide Systeme)
- Ubuntu 22.04
- ROS2 Humble installiert und `source /opt/ros/humble/setup.bash` ausführbar
- Workspace gebaut (falls nötig): `colcon build --symlink-install` und `source install/setup.bash`
- `tmux` empfohlen: `sudo apt-get install -y tmux`

1) SBC (mit Hardware attached — empfohlen)

Ziel: Auf einer SBC laufen folgende Dienste:
- micro-ROS Agent (systemd service)
- ROS2 bringup (controller_manager, ros2_control node, spawners, ggf. teleop)

Schritt A — micro-ROS Agent (system-wide via systemd)
- Terminal name: tmux session = `sbc_microros`, window = `agent`

tmux (einmalig):
```bash
# Terminal: "local_sbc_setup"
tmux new-session -d -s sbc_microros -n agent
tmux send-keys -t sbc_microros:agent 'sudo systemctl status microros-agent.service || true' C-m
tmux attach -t sbc_microros
```

Wenn ihr kein tmux möchtet, alternativ (einmalig) prüfen:
```bash
# Terminal: "local_sbc_setup"
sudo systemctl status microros-agent.service
```

Schritt B — Bringup: ros2 bringup (controller_manager usw.)
- Terminal name: tmux session = `sbc_bringup`, window = `bringup`

Im neuen tmux session/window:
```bash
# Terminal: "local_sbc_bringup"
tmux new-session -d -s sbc_bringup -n bringup
tmux send-keys -t sbc_bringup:bringup 'source /opt/ros/humble/setup.bash; source ~/my_steel-robot_ws/install/setup.bash; ros2 launch robot_bringup bringup.launch.py robot_model:=my_steel drive_type:=mecanum microros:=true serial_port:=/dev/ttyACM0' C-m
tmux attach -t sbc_bringup
```

Erläuterung: `microros:=true` sorgt dafür, dass der bringup erwartet, dass ein Agent läuft (systemd-Service oben). `serial_port` anpassen falls anders.

Schritt C — Überwachung / Logs
- Terminal name: tmux session = `sbc_logs`, window = `roslogs`

```bash
# Terminal: "local_sbc_logs"
tmux new-session -d -s sbc_logs -n roslogs
tmux send-keys -t sbc_logs:roslogs 'source /opt/ros/humble/setup.bash; journalctl -u microros-agent.service -f' C-m
tmux split-window -h -t sbc_logs:roslogs
tmux send-keys -t sbc_logs:roslogs.1 'source /opt/ros/humble/setup.bash; ros2 topic list' C-m
tmux attach -t sbc_logs
```

2) SBC (ohne Hardware attached — Simulation / Development)

Wenn keine physische Pico/Firmware angeschlossen ist, verwendet ihr statt `microros-agent` das Docker-Image oder startet den agent nicht. Bringup kann dennoch gestartet (mit microros:=false) und Controller im Simulation-/Testmodus laufen.

Start Bringup ohne micro-ROS:
```bash
# Terminal: "local_sbc_bringup_nohw"
tmux new-session -d -s sbc_bringup_nohw -n bringup
tmux send-keys -t sbc_bringup_nohw:bringup 'source /opt/ros/humble/setup.bash; source ~/my_steel-robot_ws/install/setup.bash; ros2 launch robot_bringup bringup.launch.py robot_model:=my_steel drive_type:=mecanum microros:=false' C-m
tmux attach -t sbc_bringup_nohw
```

3) Remote PC (Ubuntu) — Entwicklerrechner, mit oder ohne Hardware

Ziel: Auf dem Remote-PC betreiben wir Simulation, Rviz/Gazebo, und optional den micro-ROS Agent (Docker oder ros2 run) wenn ihr eine physische Verbindung zum SBC/Board herstellen wollt.

A) Remote PC: Start Simulation (Gazebo) und bringup
- Terminal name: tmux session = `pc_sim`, window = `gazebo`

```bash
# Terminal: "remote_pc_sim"
tmux new-session -d -s pc_sim -n gazebo
tmux send-keys -t pc_sim:gazebo 'source /opt/ros/humble/setup.bash; source ~/my_steel-robot_ws/install/setup.bash; ros2 launch robot_gazebo start_sim.launch.py' C-m
tmux attach -t pc_sim
```

B) Remote PC: Start micro-ROS Agent (Docker) — wenn du das Board per serial/tcp erreichst
- Terminal name: tmux session = `pc_agent`, window = `agent`

Docker Variante (empfohlen wenn ihr mit USB/Serial arbeitet und Docker hat Zugriff auf das Device):
```bash
# Terminal: "remote_pc_agent"
tmux new-session -d -s pc_agent -n agent
tmux send-keys -t pc_agent:agent 'docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200 -v6' C-m
tmux attach -t pc_agent
```

C) Remote PC: Start micro-ROS Agent via ros2 run (system install)
- Terminal name: tmux session = `pc_agent_ros2`, window = `agent`

```bash
# Terminal: "remote_pc_agent_ros2"
tmux new-session -d -s pc_agent_ros2 -n agent
tmux send-keys -t pc_agent_ros2:agent 'source /opt/ros/humble/setup.bash; ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6' C-m
tmux attach -t pc_agent_ros2
```

4) Kurze Validierung (auf jedem System)

- Prüfe Topics vom micro-ROS Client:
```bash
# Terminal: "check_topics"
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /some_topic --once
```

- Prüfe Controller Manager (bei Bringup):
```bash
# Terminal: "check_controllers"
source /opt/ros/humble/setup.bash
ros2 service call /controller_manager/list_controllers ros2_control_msgs/srv/ListControllers
```

5) Hinweise und Troubleshooting
- Wenn `microros-agent.service` fehlschlägt, `sudo journalctl -u microros-agent.service -b --no-pager` anschauen.
- Serielle Devices: Stelle sicher, dass der Service-User in `dialout` ist: `sudo usermod -aG dialout $USER` und neu einloggen.
- Wenn Docker verwendet wird und `/dev` nicht verfügbar ist, starte Docker mit `--device /dev/ttyACM0` oder gib passende Rechte.
- Wenn `ros2 run micro_ros_agent ...` nicht gefunden wird, installiere `ros-humble-micro-ros-agent` oder verwende Docker.

Wenn du willst, kann ich jetzt die Bringup-Launch-Datei so erweitern, dass sie `microros_agent.launch.py` automatisch inkludiert, wenn `microros:=true` gesetzt ist. Sollen wir das jetzt hinzufügen?



Parameter
- robot_model (my_steel / robot_xl)
- drive_type (mecanum / diff)
- microros (true/false)
- serial_port, baudrate

Fehlerbehebung
- controller_manager logs prüfen
- ros2 param get /controller_manager ros__parameters
- ros2 service call /controller_manager/list_controllers