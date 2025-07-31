# Final, Robust Dockerfile for ROS Noetic Project
FROM osrf/ros:noetic-desktop-full

# Set the shell to bash for all subsequent RUN commands
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/ros_ws

# Install essential build tools
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create, and set as, the working directory
WORKDIR ${CATKIN_WS}

# Copy your source code into the container's workspace
COPY ./src ./src

# Install all ROS dependencies from your package.xml files
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the ROS workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build

# Set up the entrypoint script to source the environment
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${CATKIN_WS}/devel/setup.bash\n\
exec "$@"' > /usr/local/bin/docker-entrypoint.sh && chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]