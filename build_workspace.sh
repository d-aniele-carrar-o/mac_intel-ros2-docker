#!/bin/bash
set -e

echo "Building ROS 2 workspace..."

# Connect to running container and build
docker exec -it ros2_dev bash -c "
    cd /workspace && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    echo 'Workspace built successfully!'
"