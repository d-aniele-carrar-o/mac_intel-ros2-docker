#!/bin/bash

echo "Testing Camera Viewer with Rectangle Detection and Dimension Extraction"
echo "======================================================================="

# Start development container
echo "Starting development container..."
./dev.sh

# Wait for container to be ready
sleep 3

# Build the camera viewer package
echo "Building camera viewer package..."
docker exec ros2_dev bash -c "cd /workspace && colcon build --packages-select camera_viewer"

# Launch RealSense camera + camera viewer together
echo "Launching RealSense camera with rectangle detection..."
echo "Note: OpenCV window will be available via VNC at localhost:5900"
echo "Press Ctrl+C to stop"
docker exec ros2_dev bash -c "export DISPLAY=:1 && source /opt/ros/humble/setup.bash && cd /workspace && source install/setup.bash && ros2 launch /workspace/launch/realsense.launch.py"

# Cleanup
echo "Cleaning up..."
docker stop ros2_dev 2>/dev/null
docker rm ros2_dev 2>/dev/null