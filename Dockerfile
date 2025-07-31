# Robotic Arm Docker Container
# Based on ROS Noetic with all required packages pre-installed
FROM osrf/ros:noetic-desktop-full

# Metadata
LABEL maintainer="anishkumar59085@gmail.com"
LABEL description="Complete Robotic Arm Simulation with MoveIt and Gazebo"
LABEL version="1.0"

# Set shell to bash for all RUN commands
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Update package lists first
RUN apt-get update

# Install essential packages in smaller chunks to avoid timeout
RUN apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    htop \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    software-properties-common

# Install MoveIt packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-commander \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-planners-ompl \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-moveit-fake-controller-manager

# Install controller packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-trajectory-controllers \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-joint-trajectory-controller

# Install Gazebo and additional packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-rqt-robot-steering

# Clean up apt cache
RUN rm -rf /var/lib/apt/lists/* && apt-get clean

# Initialize rosdep
RUN rosdep init || true && rosdep update

# Create workspace and set working directory
WORKDIR ${WORKSPACE_DIR}
RUN mkdir -p ${WORKSPACE_DIR}/src

# Copy only the src directory
COPY ./src ${WORKSPACE_DIR}/src/

# Set proper ownership
RUN chown -R root:root ${WORKSPACE_DIR}

# Install dependencies for your packages (if any)
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin build || catkin_make

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
echo "🤖 Welcome to the Robotic Arm Simulation Environment! 🤖"\n\
echo "=========================================="\n\
echo "ROS Workspace: /root/ros_ws"\n\
echo "Available packages:"\n\
source /opt/ros/noetic/setup.bash\n\
if [ -f "/root/ros_ws/devel/setup.bash" ]; then\n\
    source /root/ros_ws/devel/setup.bash\n\
    echo "Custom packages loaded successfully!"\n\
    rospack list | grep -v "^ros-" | head -10\n\
else\n\
    echo "No custom packages found - using base ROS packages"\n\
fi\n\
echo "=========================================="\n\
export ROS_PACKAGE_PATH=/root/ros_ws/src:/opt/ros/noetic/share\n\
export GAZEBO_MODEL_PATH=/root/ros_ws/src:${GAZEBO_MODEL_PATH}\n\
cd /root/ros_ws\n\
exec "$@"' > /usr/local/bin/entrypoint.sh && chmod +x /usr/local/bin/entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Default command
CMD ["bash"]

# Expose common ROS ports
EXPOSE 11311 11345