#!/usr/bin/env bash
set -e

# Simple helper to start micro-ROS Agent for common transports.
# Usage: ./run_microros_agent.sh [docker|ros2] transport serial_port baudrate udp_port

MODE=${1:-docker}    # docker or ros2
TRANSPORT=${2:-serial}
SERIAL_PORT=${3:-/dev/ttyACM0}
BAUD=${4:-115200}
PORT=${5:-8888}

if [ "$MODE" = "docker" ]; then
  echo "Starting micro-ROS Agent in Docker (transport=$TRANSPORT)"
    CMD=(docker run --rm --name microros_agent --privileged --network host microros/micro-ros-agent:humble)
    if [ "$TRANSPORT" = "serial" ]; then
      CMD+=(serial --dev "$SERIAL_PORT" -b "$BAUD" -v6)
    else
      CMD+=("$TRANSPORT" "--port" "$PORT" "-v6")
    fi
  echo "Running: ${CMD[*]}"
  exec "${CMD[@]}"
else
  echo "Starting micro-ROS Agent via system 'ros2' (transport=$TRANSPORT)"
  CMD=(ros2 run micro_ros_agent micro_ros_agent)
  if [ "$TRANSPORT" = "serial" ]; then
    CMD+=(serial --dev "$SERIAL_PORT" -b "$BAUD" -v6)
  else
    CMD+=("$TRANSPORT" "--port" "$PORT" "-v6")
  fi
  echo "Running: ${CMD[*]}"
  exec "${CMD[@]}"
fi
