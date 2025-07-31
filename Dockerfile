# Robotic Arm Docker Container
# Based on ROS Noetic, which already includes MoveIt, Gazebo, etc.
FROM osrf/ros:noetic-desktop-full

# Metadata
LABEL maintainer="anishkumar59085@gmail.com"
LABEL description="Complete Robotic Arm Simulation with MoveIt and Gazebo"
LABEL version="1.1"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Install essential build tools and combine update/install into one layer.
# NOTE: MoveIt, Gazebo, Rviz, etc., are already in the base image.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep. This is best practice for any custom dependencies.
RUN rosdep init || true
RUN rosdep update

# Create and set the working directory
WORKDIR ${WORKSPACE_DIR}

# Copy only your ROS source code (honoring .dockerignore)
COPY ./src ./src

# Use rosdep to install any dependencies listed in your package.xml files
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace using catkin build (more modern than catkin_make)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build

# Set up a reliable entrypoint script instead of modifying .bashrc
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
source ${WORKSPACE_DIR}/devel/setup.bash\n\
echo "🤖 Welcome to the Robotic Arm Simulation Environment! 🤖"\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

# Set the entrypoint to our custom script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# The default command to run is an interactive bash shell
CMD ["bash"]