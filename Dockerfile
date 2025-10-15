FROM ros:humble-ros-base

ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN echo "nameserver 8.8.8.8" > /etc/resolv.conf && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rviz2 \
    xvfb \
    x11vnc \
    fluxbox \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    x11-utils \
    && mkdir -p /root/.fluxbox \
    && echo "session.screen0.workspaces: 1" > /root/.fluxbox/init \
    && echo "session.screen0.toolbar.visible: false" >> /root/.fluxbox/init \
    && rm -rf /var/lib/apt/lists/*

COPY start_vnc.sh /start_vnc.sh
RUN chmod +x /start_vnc.sh

EXPOSE 5901
CMD ["/start_vnc.sh"]
