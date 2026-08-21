#!/bin/bash

# Ensure script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root (e.g., sudo ./install_daemon.sh)"
  exit 1
fi

echo "=========================================="
echo " Road Painter Daemon Installer"
echo "=========================================="

SERVICE_FILE="robot_exec.service"
SERVICE_DEST="/etc/systemd/system/${SERVICE_FILE}"

if [ ! -f "$SERVICE_FILE" ]; then
    echo "Error: $SERVICE_FILE not found in the current directory."
    echo "Please run this script from inside the daemon/ directory."
    exit 1
fi

echo "1. Copying $SERVICE_FILE to systemd directory..."
cp $SERVICE_FILE $SERVICE_DEST
chmod 644 $SERVICE_DEST

echo "2. Reloading systemd daemon..."
systemctl daemon-reload

echo "3. Enabling robot_exec to start automatically on boot..."
systemctl enable robot_exec.service

echo "4. Starting the robot_exec service now..."
systemctl restart robot_exec.service

echo ""
echo "Installation and Startup complete! "
echo "------------------------------------------"
echo "Useful Commands:"
echo " Check Status : sudo systemctl status robot_exec"
echo " View Logs    : sudo journalctl -u robot_exec -f"
echo " Stop Service : sudo systemctl stop robot_exec"
echo " Start Service: sudo systemctl start robot_exec"
echo "=========================================="
