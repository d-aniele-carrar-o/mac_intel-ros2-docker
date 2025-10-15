#!/bin/bash
mkdir -p $XDG_RUNTIME_DIR
chmod 0700 $XDG_RUNTIME_DIR
Xvfb :1 -screen 0 1024x768x24 > /dev/null 2>&1 &
sleep 2
fluxbox > /dev/null 2>&1 &
x11vnc -display :1 -nopw -listen 0.0.0.0 -shared -forever -quiet > /dev/null 2>&1 &
sleep 2
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2