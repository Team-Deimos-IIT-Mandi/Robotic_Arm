FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/ros_ws

# Install basic tools and rosdep
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create catkin workspace
WORKDIR ${CATKIN_WS}
COPY ./src ./src

# Source ROS and install dependencies via rosdep
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=${ROS_DISTRO}

# Build workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin config --extend /opt/ros/${ROS_DISTRO} && \
    catkin build

# Entrypoint script to source setup files
RUN cat << 'EOF' > /usr/local/bin/docker-entrypoint.sh && \
chmod +x /usr/local/bin/docker-entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${CATKIN_WS}/devel/setup.bash
exec "$@"
EOF

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]
