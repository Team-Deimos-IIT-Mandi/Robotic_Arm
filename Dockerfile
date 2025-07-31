# Robotic Arm Docker Container
# Based on ROS Noetic with all required packages pre-installed
FROM osrf/ros:noetic-desktop-full

# Metadata
LABEL maintainer="anishkumar59085@gmail.com"
LABEL description="Complete Robotic Arm Simulation with MoveIt and Gazebo"
LABEL version="1.0"

# Set shell to bash for all RUN commands (fixes source command issue)
SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Update and install all packages in one layer for efficiency
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    # Essential system packages
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
    software-properties-common \
    # MoveIt packages
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
    ros-${ROS_DISTRO}-moveit-fake-controller-manager \
    # Controller packages
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-joint-state-controller \
    ros-${ROS_DISTRO}-trajectory-controllers \
    ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-effort-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    # Gazebo packages
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-plugins \
    # Additional ROS packages
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    # Rviz and visualization
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-rqt-robot-steering \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Initialize rosdep in one step
RUN rosdep init || true && rosdep update

# Create workspace and set working directory
WORKDIR ${WORKSPACE_DIR}
RUN mkdir -p ${WORKSPACE_DIR}/src

# Copy only the src directory (more efficient and cleaner)
COPY ./src ${WORKSPACE_DIR}/src/

# Set proper ownership
RUN chown -R root:root ${WORKSPACE_DIR}

# Install dependencies and build workspace in one step
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin build -DCMAKE_BUILD_TYPE=Release

# Create entrypoint script (better than modifying .bashrc)
RUN echo '#!/bin/bash\n\
set -e\n\
echo "🤖 Welcome to the Robotic Arm Simulation Environment! 🤖"\n\
echo "=========================================="\n\
echo "Available commands:"\n\
echo "  • Launch full simulation: roslaunch your_package main.launch"\n\
echo "  • Launch MoveIt only: roslaunch arm_moveit_config demo.launch"\n\
echo "  • Launch Gazebo only: roslaunch your_package gazebo.launch"\n\
echo "  • List packages: rospack list"\n\
echo "=========================================="\n\
source /opt/ros/noetic/setup.bash\n\
source /root/ros_ws/devel/setup.bash\n\
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