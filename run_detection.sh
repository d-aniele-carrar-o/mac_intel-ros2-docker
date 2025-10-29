#!/bin/bash

echo "Rectangle Detection with RealSense Camera"
echo "============================================="

# Check if container is already running
if docker ps | grep -q ros2_dev; then
    echo "Using existing development container..."
else
    echo "Starting development container..."
    ./dev.sh
    sleep 3
fi

# Build the camera viewer package
echo "Building camera viewer package..."
docker exec ros2_dev bash -c "cd /workspace && colcon build --packages-select camera_viewer"

# Launch RealSense camera + rectangle detection
echo "Launching RealSense camera with rectangle detection..."
echo "OpenCV window available via VNC at localhost:5900"
echo "Press Ctrl+C to stop"
echo ""

docker exec -it ros2_dev bash -c "
    export DISPLAY=:1 && 
    source /opt/ros/humble/setup.bash && 
    cd /workspace && 
    source install/setup.bash && 
    ros2 launch /workspace/launch/realsense.launch.py
"

# Cleanup on exit
echo ""
echo "Cleaning up..."
docker stop ros2_dev 2>/dev/null
docker rm ros2_dev 2>/dev/null
echo "Done"