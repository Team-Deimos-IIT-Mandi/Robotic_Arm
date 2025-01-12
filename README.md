# Robotic Arm Teleoperation in Gazebo
This repository provides simulation and teleoperation tools for controlling a robotic arm in Gazebo using keyboard input and ROS controllers.


## How to use this repo
### Prerequisites :
- #### Install Gazebo
  We will be launching our world in Gazebo so make sure to install it by using the command 
  ```
  curl -sSL http://get.gazebosim.org | sh
  ```
- #### Install ROS dependencies

  ```
  sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-urdf ros-noetic-xacro ros-noetic-ros-control ros-noetic-ros-controllers
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
  cd ~/robo_arm && catkin build
  ```
  Source your workspace in .bashrc file by running the following command so that you don't have to source it in every terminal
  ```
  echo "source ~/robo_arm/devel/setup.bash" >> ~/.bashrc
  ```

  ## Control new arm

  ### To start the Gazebo simulation with moveit:
  
  ```
  roslaunch arm_moveit_config final_arm_launch.launch 
  ```

> [!NOTE]
> Rviz may take 2-3 minutes to load but still if planning library is not loaded, then press "reset" at bottom-left of rviz window.

  ### To control the robotic arm via keyboard:
  
  ```
  rosrun teleop_arm moveit_teleop_niwesh.py
  ```

  ### For finding x,y,z using stereo vision (work under progress):
  
  ```
  rosrun stereo_vision detect_cylinder_xyz.py 
  ```
