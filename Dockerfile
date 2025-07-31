# DEBUG DOCKERFILE
# Purpose: Find the exact error during the build process.
FROM osrf/ros:noetic-desktop-full
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/ros_ws

# Install build tools
RUN apt-get update && apt-get install -y python3-catkin-tools python3-rosdep && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Copy code and attempt to build
WORKDIR ${CATKIN_WS}
COPY ./src ./src

# This section will now fail with a clear error message if something is wrong.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} && \
    catkin build