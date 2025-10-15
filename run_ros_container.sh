#!/bin/bash
set -e

# Configuration
DOCKER_IMAGE="my-ros-rviz"
WORKSPACE_HOST="$HOME/Documents/docker_ros"
CONTAINER_NAME="ros2_dev"

# Create workspace directory
mkdir -p "$WORKSPACE_HOST"

echo "Starting ROS 2 container with RViz2..."
echo "Workspace: $WORKSPACE_HOST -> /workspace"
echo "VNC Access: localhost:5900"

# Run container
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    -p 5900:5900 \
    -v "$WORKSPACE_HOST:/workspace" \
    "$DOCKER_IMAGE" "$@"

echo ""
echo "Container stopped. To reconnect to a running container:"
echo "  docker exec -it $CONTAINER_NAME bash"
