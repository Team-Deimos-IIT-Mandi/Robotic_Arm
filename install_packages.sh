#!/bin/bash

# Update package lists
apt-get update

# Install MoveIt packages
apt-get install -y ros-noetic-moveit
apt-get install -y ros-noetic-moveit-core
apt-get install -y ros-noetic-moveit-commander
apt-get install -y ros-noetic-moveit-planners
apt-get install -y ros-noetic-moveit-planners-ompl
apt-get install -y ros-noetic-moveit-ros-move-group
apt-get install -y ros-noetic-moveit-ros-planning
apt-get install -y ros-noetic-moveit-ros-planning-interface
apt-get install -y ros-noetic-moveit-ros-visualization
apt-get install -y ros-noetic-moveit-setup-assistant
apt-get install -y ros-noetic-moveit-simple-controller-manager
apt-get install -y ros-noetic-moveit-fake-controller-manager

# Install Controller packages
apt-get install -y ros-noetic-controller-manager
apt-get install -y ros-noetic-joint-state-controller
apt-get install -y ros-noetic-trajectory-controllers
apt-get install -y ros-noetic-position-controllers
apt-get install -y ros-noetic-effort-controllers
apt-get install -y ros-noetic-velocity-controllers

# Install Gazebo control packages
apt-get install -y ros-noetic-gazebo-ros-control
apt-get install -y ros-noetic-gazebo-ros-pkgs
apt-get install -y ros-noetic-gazebo-plugins

# Install RViz plugins
apt-get install -y ros-noetic-rviz
apt-get install -y ros-noetic-moveit-ros-visualization

# Install additional dependencies
apt-get install -y ros-noetic-joint-trajectory-controller
apt-get install -y ros-noetic-robot-state-publisher
apt-get install -y ros-noetic-joint-state-publisher
apt-get install -y ros-noetic-joint-state-publisher-gui

# Clean up
apt-get clean
rm -rf /var/lib/apt/lists/*

echo "All packages installed successfully!"
echo "Please rebuild your workspace with: catkin_make"