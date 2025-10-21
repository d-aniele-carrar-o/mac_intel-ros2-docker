#!/bin/bash
set -e

echo "Building ROS 2 Docker image with RealSense support..."
docker build -t my-ros-rviz .
echo "Build complete!"