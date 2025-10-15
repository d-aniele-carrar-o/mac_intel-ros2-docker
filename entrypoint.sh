#!/bin/bash
set -e

# Setup runtime directory
mkdir -p $XDG_RUNTIME_DIR
chmod 0700 $XDG_RUNTIME_DIR

# Start virtual display
echo "Starting virtual display..."
Xvfb :1 -screen 0 1024x768x24 > /dev/null 2>&1 &
sleep 2

# Start window manager
echo "Starting window manager..."
fluxbox > /dev/null 2>&1 &

# Start VNC server
echo "Starting VNC server on port 5900..."
x11vnc -display :1 -nopw -listen 0.0.0.0 -shared -forever -quiet > /dev/null 2>&1 &
sleep 2

# Source ROS environment
source /opt/ros/humble/setup.bash

# If arguments provided, run them; otherwise start RViz2
if [ $# -eq 0 ]; then
    echo "Starting RViz2..."
    exec ros2 run rviz2 rviz2
else
    exec "$@"
fi