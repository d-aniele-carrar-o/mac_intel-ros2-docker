#!/bin/bash
set -e

echo "Starting RViz2 with VNC (connect to localhost:5900)..."

docker run -it --rm \
    --name ros2_rviz_vnc \
    -p 5900:5900 \
    -v "$(pwd)/src:/workspace/src" \
    -v "$(pwd)/launch:/workspace/launch" \
    -v "$(pwd)/config:/workspace/config" \
    my-ros-rviz bash -c "
        # Setup VNC
        mkdir -p /tmp/runtime-root
        chmod 0700 /tmp/runtime-root
        export XDG_RUNTIME_DIR=/tmp/runtime-root
        export DISPLAY=:1
        
        # Start services
        Xvfb :1 -screen 0 1024x768x24 > /dev/null 2>&1 &
        sleep 2
        fluxbox > /dev/null 2>&1 &
        x11vnc -display :1 -nopw -listen 0.0.0.0 -shared -forever -quiet > /dev/null 2>&1 &
        sleep 2
        
        # Start RViz
        ros2 run rviz2 rviz2
    "