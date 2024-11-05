# Robotic Arm Teleoperation in Gazebo
This repository provides simulation and teleoperation tools for controlling a robotic arm in Gazebo using keyboard input and ROS controllers.

## Features
- **Keyboard Control**: Manual control of end-effector velocities.
- **Velocity Mapping**: Key-based directional velocity control.
- **Inverse Kinematics**: Joint velocities calculated using the Jacobian inverse.

## Folder Structure
- **arm_urdf**: Contains the robot URDF files.
- **moveit_pkg**: MoveIt configuration for planning and control.
- **teleop_arm**: Teleoperation scripts.

## How to use this repo
### Prerequisites :
- #### Install Gazebo
  We will be launching our world in Gazebo so make sure to install it by using the command 
  ```
  curl -sSL http://get.gazebosim.org | sh
  ```
- #### Install ROS dependencies
  Used in integrating Gazebo to ROS
  ```
  sudo apt-get install ros-noetic-gazebo-ros-pkgs
  ```
  Used for describing the properties and parts of the model
  ```
  sudo apt-get install ros-noetic-urdf ros-noetic-xacro
  ```
  Provides the necessary controller manager.
  ```
  sudo apt-get install ros-noetic-ros-control ros-<distro>-ros-controllers
  ```
  For motion planning
  ```
  sudo apt-get install ros-noetic-moveit
  ```
  
> [!NOTE]
> All the installation commands are for rosdep noetic change noetic with <your_distro_name>

- #### Install ROS packages
  Make a workspace and create a directory 'src' where all the packages will be stored, clone this repo to get the packages and then build the catkin workspace.
  ```
  cd ~/robo_arm/src/
  git clone https://github.com/Team-Deimos-IIT-Mandi/Robotic_Arm.git
  cd ~/robo_arm && catkin_make
  ```
  Source your workspace in .bashrc file by running the following command so that you don't have to source it in every terminal
  ```
  echo "source ~/robo_arm/devel/setup.bash" >> ~/.bashrc
  ```

  ## Usage

  ### Launch Gazebo
  To start the Gazebo simulation:
  
  ```
  roslaunch moveit_pkg final.launch
  ```
  ### Run Teleoperation
  To control the robotic arm via keyboard:
  
  ```
  rosrun teleop_arm jaco_pt2.py
  ```
  ### Keyboard Controls
  - `w` - Move +X
  - `x` - Move -X
  - `a` - Move +Y
  - `d` - Move -Y
  - `e` - Move +Z
  - `z` - Move -Z
  
  Press `Ctrl+C` to exit.

## Controllers
This project uses the following controllers:

- **Plan Controller**: Uses `JointGroupPositionController` for `joint_0` to `joint_3`, enabling position control.
- **Joint State Controller**: Publishes joint states at 100 Hz.
