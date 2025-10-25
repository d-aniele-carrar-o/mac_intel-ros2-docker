#!/bin/bash
set -e

echo "Starting RealSense camera container..."

# Stop existing container if running
docker stop ros2_realsense 2>/dev/null || true
docker rm ros2_realsense 2>/dev/null || true

# Run with full USB device access
docker run -d \
    --name ros2_realsense \
    --privileged \
    --device-cgroup-rule='c *:* rmw' \
    -v /dev:/dev \
    -v "$(pwd)/src:/workspace/src" \
    -v "$(pwd)/launch:/workspace/launch" \
    -v "$(pwd)/config:/workspace/config" \
    my-ros-rviz \
    bash -c "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py"

echo "RealSense camera started!"
echo "Container name: ros2_realsense"