#!/usr/bin/env bash
set -euo pipefail

TEMPLATE="$(dirname "$0")/../packaging/systemd/microros-agent.service.template"
DEST_UNIT="/etc/systemd/system/microros-agent.service"

USER_NAME=${MICROROS_USER:-$(whoami)}
SERIAL_PORT=${MICROROS_SERIAL_PORT:-/dev/ttyACM0}
BAUD=${MICROROS_BAUD:-115200}

echo "This script will install micro-ROS Agent (apt) and create a systemd service."
read -r -p "Continue? [y/N] " yn
if [[ "$yn" != "y" && "$yn" != "Y" ]]; then
  echo "Aborted."
  exit 1
fi

echo "Installing micro-ROS Agent package (requires sudo)..."
sudo apt-get update
sudo apt-get install -y ros-humble-micro-ros-agent || {
  echo "Failed to install package 'ros-humble-micro-ros-agent'. You can try 'pip install micro-ros-agent' or use the Docker image instead." >&2
}

if [ ! -f "$TEMPLATE" ]; then
  echo "Template not found: $TEMPLATE" >&2
  exit 1
fi

echo "Generating systemd unit at $DEST_UNIT"
sudo sed \
  -e "s|%USER%|$USER_NAME|g" \
  -e "s|%SERIAL_PORT%|$SERIAL_PORT|g" \
  -e "s|%BAUD%|$BAUD|g" \
  "$TEMPLATE" | sudo tee "$DEST_UNIT" > /dev/null

echo "Reloading systemd and enabling service"
sudo systemctl daemon-reload
sudo systemctl enable --now microros-agent.service

echo "Service status:"
sudo systemctl status --no-pager microros-agent.service || true

echo "Installation complete. Adjust the unit file at $DEST_UNIT if you need a different transport or options."
