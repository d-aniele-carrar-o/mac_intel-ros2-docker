#!/bin/bash
set -e

echo "Starting RViz2..."

if [[ "$OSTYPE" == "linux-gnu"* ]] && [ -n "$DISPLAY" ]; then
    echo "Using X11 forwarding with DISPLAY=$DISPLAY"
    xhost +local:docker >/dev/null 2>&1 || echo "Warning: xhost failed"
    docker run -it --rm \
        --name ros2_rviz \
        --net=host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        my-ros-rviz ros2 run rviz2 rviz2
else
    echo "Using VNC (connect to localhost:5900)"
    docker run -it --rm \
        --name ros2_rviz \
        -p 5900:5900 \
        -v "$(pwd)/src:/workspace/src" \
        -v "$(pwd)/launch:/workspace/launch" \
        -v "$(pwd)/config:/workspace/config" \
        my-ros-rviz ros2 run rviz2 rviz2
fi