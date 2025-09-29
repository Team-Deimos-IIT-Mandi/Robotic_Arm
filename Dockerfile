
FROM osrf/ros:noetic-desktop-full

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/ros_ws

# Install essential tools and ROS dependencies
RUN apt-get update && apt-get install -y curl sudo

RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl nano \
    python3-catkin-tools \
    python3-rosdep \
    ros-noetic-ruckig \
    ros-noetic-moveit \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Create workspace
WORKDIR ${CATKIN_WS}
RUN mkdir -p src

# Clone the GitHub repository into src
RUN git clone https://github.com/Team-Deimos-IIT-Mandi/Robotic_Arm.git src

# Install ROS dependencies from package.xml
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} || echo "Some dependencies could not be installed"

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build

# Entrypoint script
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\nsource ${CATKIN_WS}/devel/setup.bash\nexec "$@"\n' > /usr/local/bin/docker-entrypoint.sh
RUN chmod +x /usr/local/bin/docker-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["bash"]
