#!/bin/bash
set -e

echo "Starting ROS 2 development environment..."

# Stop existing container if running
docker stop ros2_dev 2>/dev/null || true
docker rm ros2_dev 2>/dev/null || true

# Start the development container
docker run -d \
    --name ros2_dev \
    -p 5900:5900 \
    -v "$(pwd)/src:/workspace/src" \
    -v "$(pwd)/launch:/workspace/launch" \
    -v "$(pwd)/config:/workspace/config" \
    my-ros-rviz bash -c "while true; do sleep 30; done"

echo ""
echo "Development container started!"
echo "- Container name: ros2_dev"
echo "- VNC access: localhost:5900"
echo "- Workspace mounted: ./src -> /workspace/src"
echo ""
echo "To connect to the container:"
echo "  ./connect.sh"
echo ""
echo "To stop the container:"
echo "  docker stop ros2_dev && docker rm ros2_dev"