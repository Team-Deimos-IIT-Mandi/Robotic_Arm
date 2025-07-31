# Final, Robust Dockerfile for ROS Noetic Project
FROM osrf/ros:noetic-desktop-full

# Set the shell to bash for all subsequent RUN commands, fixing "source: not found" errors
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/ros_ws

# Update and install all required dependencies in a single layer for efficiency.
# This directly installs the packages that rosdep was failing to find.
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-moveit-simple-controller-manager \
    ros-noetic-trajectory-controllers \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep for any other dependencies
RUN rosdep init || true
RUN rosdep update

# Create and set the working directory
WORKDIR ${CATKIN_WS}

# Copy your source code into the container's workspace
COPY ./src ./src

# Install any remaining dependencies from your package.xml files
# The previously failing packages are already installed, so this will now succeed.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO}

# Build the ROS workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build

# Set up the entrypoint script to source the environment correctly
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/${ROS_DISTRO}/setup.bash\n\
source ${CATKIN_WS}/devel/setup.bash\n\
exec "$@"' > /usr/local/bin/docker-entrypoint.sh && chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]