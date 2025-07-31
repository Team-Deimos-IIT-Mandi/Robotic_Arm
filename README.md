# 🤖 Robotic Arm Simulation Project

A complete robotic arm simulation environment using ROS Noetic, MoveIt, and Gazebo, packaged in Docker for easy deployment.

## 📋 Features

- **Complete ROS Noetic Environment**: Pre-configured with all necessary packages
- **MoveIt Motion Planning**: Advanced path planning and execution
- **Gazebo Simulation**: Realistic physics simulation
- **Docker Containerized**: Zero-setup deployment for team collaboration
- **GUI Support**: Full X11 forwarding for RViz, Gazebo, and other GUI applications
- **Persistent Container**: Maintains workspace state between sessions

## 🚀 Quick Start

### Prerequisites

- Docker installed on your system
- X11 forwarding support (Linux/macOS with XQuartz)
- NVIDIA GPU drivers (optional, for GPU acceleration)

### Option 1: Using the Launch Script (Recommended)

1. **Clone the repository:**
   ```bash
   git clone <your-repo-url>
   cd robotic-arm-project
   ```

2. **Make the launch script executable:**
   ```bash
   chmod +x launch_container.sh
   ```

3. **Launch the container:**
   ```bash
   ./launch_container.sh
   ```

### Option 2: Using Docker Compose

1. **Setup X11 forwarding:**
   ```bash
   export XAUTH=/tmp/.docker.xauth
   touch $XAUTH
   xauth nxlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nxmerge -
   xhost +local:docker
   ```

2. **Launch with Docker Compose:**
   ```bash
   docker-compose up -d
   docker-compose exec robotic_arm /bin/bash
   ```

### Option 3: Manual Docker Commands

1. **Build the image:**
   ```bash
   docker build -t robotic_arm:latest .
   ```

2. **Run the container:**
   ```bash
   docker run -it \
     --gpus all \
     --name robotic_arm_container \
     --env="DISPLAY=$DISPLAY" \
     --env="QT_X11_NO_MITSHM=1" \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
     --net=host \
     --privileged \
     robotic_arm:latest
   ```

## 🎮 Usage

Once inside the container, you can run various commands:

### Launch Full Simulation
```bash
# Launch complete environment (Gazebo + MoveIt + RViz)
roslaunch your_package_name main.launch
```

### Launch Individual Components
```bash
# Launch only MoveIt with RViz
roslaunch arm_moveit_config demo.launch

# Launch only Gazebo simulation
roslaunch your_package_name gazebo.launch

# Launch MoveIt move_group node
roslaunch arm_moveit_config move_group.launch
```

### Debug Commands
```bash
# Check available controller types
rosservice call controller_manager/list_controller_types

# Monitor joint states
rostopic echo /joint_states

# List active controllers
rosservice call controller_manager/list_controllers
```

## 📁 Project Structure

```
robotic-arm-project/
├── Dockerfile                 # Main container definition
├── docker-compose.yml         # Docker Compose configuration
├── launch_container.sh        # Enhanced launch script
├── README.md                  # This file
├── src/                       # ROS packages
│   ├── Arm_Urdf/              # Robot URDF and configurations
│   ├── arm_moveit_config/     # MoveIt configuration
│   └── teleop_arm/            # Teleoperation package
└── .dockerignore              # Docker ignore patterns
```

## 🔧 Configuration Files

### Key Configuration Files:
- `src/Arm_Urdf/config/joint_trajectory_control.yaml` - Controller configuration
- `src/arm_moveit_config/config/controllers.yaml` - MoveIt controller settings
- `src/arm_moveit_config/launch/move_group.launch` - MoveIt launch file

### Controller Configuration:
The project uses `position_controllers/JointTrajectoryController` for both:
- **Body Controller**: Controls main arm joints (Joint_1 to Joint_6)
- **End Effector Controller**: Controls gripper fingers (Finger_1, Finger_2)

## 🐛 Troubleshooting

### Common Issues:

1. **GUI Applications Not Displaying:**
   ```bash
   # Ensure X11 forwarding is set up
   xhost +local:docker
   export DISPLAY=:0
   ```

2. **Controller Loading Failures:**
   ```bash
   # Check available controller types
   rosservice call controller_manager/list_controller_types
   
   # Verify controller configuration
   rosparam list | grep controller
   ```

3. **Trajectory Execution Failures:**
   - Check joint state synchronization
   - Verify controller tolerances in YAML files
   - Ensure proper timing parameters

4. **Container Build Failures:**
   ```bash
   # Clean build with no cache
   docker build --no-cache -t robotic_arm:latest .
   ```

### Launch Script Options:

```bash
./launch_container.sh --help     # Show help
./launch_container.sh --rebuild  # Rebuild image and container
./launch_container.sh --fresh    # Create fresh container
```

## 🔄 Development Workflow

1. **Make changes** to your code on the host system
2. **Rebuild** the container: `./launch_container.sh --rebuild`
3. **Test** your changes inside the container
4. **Commit** and **push** to your repository

## 📦 What's Included

### Pre-installed Packages:
- ROS Noetic Desktop Full
- MoveIt Motion Planning Framework  
- Gazebo Robot Simulator
- All trajectory and position controllers
- RViz visualization tools
- Joint state publishers
- TF2 transformation library
- Xacro XML processing tools

### Ready-to-Use Features:
- ✅ Robotic arm URDF model
- ✅ MoveIt configuration
- ✅ Gazebo simulation world
- ✅ Controller configurations
- ✅ Launch files for all scenarios
- ✅ GUI applications support

## 👥 Team Collaboration

Your teammates can simply:

1. **Clone** the repository
2. **Run** `./launch_container.sh`
3. **Start working** immediately!

No need to:
- Install ROS
- Configure MoveIt
- Set up Gazebo
- Install dependencies
- Debug package conflicts

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test in the Docker environment
5. Submit a pull request

## 📞 Support

For issues and questions:
- Open an issue in the repository
- Check the troubleshooting section
- Review ROS/MoveIt documentation

---

**Happy Robotics! 🤖✨**