# Minimal working Dockerfile for ROS Noetic
FROM osrf/ros:noetic-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Install essential build tools. MoveIt and controllers are in the base image.
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace and set as working directory
WORKDIR ${WORKSPACE_DIR}
COPY ./src ./src

# Install dependencies from your source packages. Let it fail if deps are missing.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace. Let it fail if the build is broken.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build

# Create entrypoint. This now runs AFTER a successful build.
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
source ${WORKSPACE_DIR}/devel/setup.bash\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]