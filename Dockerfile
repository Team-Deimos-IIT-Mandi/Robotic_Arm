# Minimal working Dockerfile for ROS Noetic with MoveIt
FROM osrf/ros:noetic-desktop-full

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Install essential packages and ROS controllers
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace
WORKDIR ${WORKSPACE_DIR}
RUN mkdir -p src

# Copy source code
COPY ./src ./src/

# Install dependencies and build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true && \
    catkin build || catkin_make

# Create entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/noetic/setup.bash\n\
[ -f "/root/ros_ws/devel/setup.bash" ] && source /root/ros_ws/devel/setup.bash\n\
echo "🤖 ROS Noetic Environment Ready!"\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]