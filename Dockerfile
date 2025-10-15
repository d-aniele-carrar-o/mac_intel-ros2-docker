FROM ros:humble-ros-base

# Environment variables for GUI and OpenGL
ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root

# Install ROS 2 GUI packages, VNC server, and development tools
RUN echo "nameserver 8.8.8.8" > /etc/resolv.conf && \
    apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rviz2 \
        ros-humble-rqt-* \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        git \
        vim \
        nano \
        tree \
        xvfb \
        x11vnc \
        fluxbox \
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        x11-utils \
    && rm -rf /var/lib/apt/lists/*

# Configure fluxbox window manager
RUN mkdir -p /root/.fluxbox && \
    echo "session.screen0.workspaces: 1" > /root/.fluxbox/init && \
    echo "session.screen0.toolbar.visible: false" >> /root/.fluxbox/init

# Setup ROS environment for all sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Copy and setup startup script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Create workspace directory
RUN mkdir -p /workspace
WORKDIR /workspace

EXPOSE 5900
ENTRYPOINT ["/entrypoint.sh"]
