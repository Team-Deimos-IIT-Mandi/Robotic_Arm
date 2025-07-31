# Simple and robust Dockerfile for ROS Noetic
FROM osrf/ros:noetic-desktop-full

# --- FIX: Set shell to bash for all subsequent RUN commands ---
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Install essential packages. Most are already in the base image, but this ensures they are present.
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create and set the working directory
WORKDIR ${WORKSPACE_DIR}
COPY ./src ./src

# Build the workspace only if the src directory is not empty
# IMPORTANT: This will now correctly fail if rosdep or catkin build has an error.
RUN if [ -n "$(ls -A src)" ]; then \
        echo "--- Found packages in src. Installing dependencies and building. ---" && \
        source /opt/ros/${ROS_DISTRO}/setup.bash && \
        rosdep install --from-paths src --ignore-src -r -y && \
        catkin build; \
    else \
        echo "--- src directory is empty. Skipping build. ---"; \
    fi

# Create a reliable entrypoint script that sources the environment
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
# Source the workspace setup file only if it exists\n\
if [ -f "${WORKSPACE_DIR}/devel/setup.bash" ]; then\n\
    source ${WORKSPACE_DIR}/devel/setup.bash\n\
fi\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command is an interactive bash shell
CMD ["bash"]