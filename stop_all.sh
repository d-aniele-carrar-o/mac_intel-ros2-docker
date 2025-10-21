#!/bin/bash

echo "Stopping all ROS2 Docker containers..."

# Stop all containers with our names
docker stop ros2_rviz ros2_realsense ros2_dev >/dev/null 2>&1 || true

# Remove stopped containers
docker rm ros2_rviz ros2_realsense ros2_dev >/dev/null 2>&1 || true

echo "All containers stopped and removed."