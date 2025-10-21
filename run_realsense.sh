#!/bin/bash
set -e

echo "Starting RealSense camera..."

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    # Linux: Use X11 forwarding
    xhost +local:docker >/dev/null 2>&1
    docker run -it --rm \
        --name ros2_realsense \
        --privileged \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        -v /dev:/dev \
        my-ros-rviz \
        ros2 launch launch/realsense.launch.py
else
    # macOS: Use VNC
    docker run -it --rm \
        --name ros2_realsense \
        --privileged \
        -p 5900:5900 \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        -v /dev:/dev \
        my-ros-rviz \
        ros2 launch launch/realsense.launch.py
fi