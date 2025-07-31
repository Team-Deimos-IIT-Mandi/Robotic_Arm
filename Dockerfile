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

# Initialize rosdep (handle if already exists)
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Create, and set as, the working directory
WORKDIR ${CATKIN_WS}

# Copy your source code into the container's workspace
COPY ./src ./src

# Install all ROS dependencies from your package.xml files
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} || echo "Some dependencies could not be installed"

# Build the ROS workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build

# Create entrypoint script with proper formatting
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nsource ${CATKIN_WS}/devel/setup.bash\nexec "$@"\n' > /usr/local/bin/docker-entrypoint.sh

# Make entrypoint script executable
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

# Verify the script was created
RUN ls -la /usr/local/bin/docker-entrypoint.sh && cat /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]