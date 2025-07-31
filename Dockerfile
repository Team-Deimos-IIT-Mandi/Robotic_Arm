# Robotic Arm Docker Container
# Based on ROS Noetic with all required packages pre-installed
FROM osrf/ros:noetic-desktop-full

# Metadata
LABEL maintainer="anishkumar59085@gmail.com"
LABEL description="Complete Robotic Arm Simulation with MoveIt and Gazebo"
LABEL version="1.0"

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE_DIR=/root/ros_ws

# Update system packages
RUN apt-get update && apt-get upgrade -y

# Install essential system packages
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
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install MoveIt and all required ROS packages
RUN apt-get update && apt-get install -y \
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
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create workspace directory
WORKDIR ${WORKSPACE_DIR}
RUN mkdir -p ${WORKSPACE_DIR}/src

# Copy your project files into the container
# This will copy everything from your host project directory
COPY . ${WORKSPACE_DIR}/src/

# Set proper ownership
RUN chown -R root:root ${WORKSPACE_DIR}

# Build the workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${WORKSPACE_DIR} && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${WORKSPACE_DIR}/devel/setup.bash" >> ~/.bashrc
RUN echo "export ROS_PACKAGE_PATH=${WORKSPACE_DIR}/src:/opt/ros/${ROS_DISTRO}/share" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=${WORKSPACE_DIR}/src:${GAZEBO_MODEL_PATH}" >> ~/.bashrc

# Create a startup script
RUN echo '#!/bin/bash\n\
echo "🤖 Welcome to the Robotic Arm Simulation Environment! 🤖"\n\
echo "=========================================="\n\
echo "Available commands:"\n\
echo "  • Launch full simulation: roslaunch your_package main.launch"\n\
echo "  • Launch MoveIt only: roslaunch arm_moveit_config demo.launch"\n\
echo "  • Launch Gazebo only: roslaunch your_package gazebo.launch"\n\
echo "=========================================="\n\
source /opt/ros/noetic/setup.bash\n\
source /root/ros_ws/devel/setup.bash\n\
export ROS_PACKAGE_PATH=/root/ros_ws/src:/opt/ros/noetic/share\n\
cd /root/ros_ws\n\
exec "$@"' > /root/startup.sh && chmod +x /root/startup.sh

# Set the default command
CMD ["/root/startup.sh", "/bin/bash"]

# Expose common ROS ports
EXPOSE 11311 11345