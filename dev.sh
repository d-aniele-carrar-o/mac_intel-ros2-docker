#!/bin/bash
set -e

echo "Starting ROS 2 development environment..."

# Stop existing container if running
docker stop ros2_dev 2>/dev/null || true
docker rm ros2_dev 2>/dev/null || true

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux: Use X11 forwarding with USB devices
    xhost +local:docker >/dev/null 2>&1
    docker run -d \
        --name ros2_dev \
        --privileged \
        --device-cgroup-rule='c *:* rmw' \
        -v /dev:/dev \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        my-ros-rviz bash -c "while true; do sleep 30; done"
else
    # macOS: Use VNC with USB devices
    docker run -d \
        --name ros2_dev \
        --privileged \
        --device-cgroup-rule='c *:* rmw' \
        -v /dev:/dev \
        -p 5900:5900 \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        my-ros-rviz bash -c "while true; do sleep 30; done"
fi

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