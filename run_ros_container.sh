#!/bin/bash

# --- Configuration ---
# Set the name of the Docker image you want to use
DOCKER_IMAGE="my-ros-rviz"

# Set the path to your local ROS 2 workspace that you want to mount
# The script will automatically create this directory if it doesn't exist.
# Example: ~/ros2_ws will be mounted inside the container at /home/user/ros2_ws
ROS_WS_HOST="$HOME/Documents/docker_ros"

# --- Script Start ---
echo "Starting ROS 2 Docker container..."

# 1. Create the workspace directory on your Mac if it's not there
mkdir -p "$ROS_WS_HOST"
echo "Mounting host directory: $ROS_WS_HOST"

# 2. Run container with VNC
docker run -it --rm \
    --name ros2_dev \
    -p 5900:5900 \
    -v "$ROS_WS_HOST:/home/user/ros2_ws" \
    "$DOCKER_IMAGE"

echo "RViz2 is running! Connect with VNC client to: localhost:5900"
echo "Try: brew install --cask tigervnc-viewer"

# --- End of Script ---
