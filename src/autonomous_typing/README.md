# Autonomous Typing Package

## Overview

The `autonomous_typing` package is a ROS-based system that enables a robotic arm to autonomously type on a physical keyboard. The system uses computer vision (YOLO object detection) to identify keyboard keys and MoveIt! motion planning to control the robotic arm for precise typing actions.

## Package Structure

```
autonomous_typing/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package dependencies and metadata
├── config/                 # Configuration files
│   └── Default.perspective # RViz perspective configuration
├── launch/                 # Launch files
│   └── test.launch        # Main launch file for testing
├── src/                   # Source code (Python scripts)
│   ├── autonomous.py      # Main autonomous typing controller
│   ├── detection_node.py  # YOLO-based key detection node
│   ├── typing_node.py     # MoveIt! typing execution node
│   ├── keyboard_layout.json # Keyboard key positions mapping
│   ├── img/              # Image assets
│   ├── redundant_code/   # Legacy/experimental code
│   └── tools_to_hardcode_keyboard/ # Development tools
└── urdf/                 # URDF/SDF models
    └── Razer_BlackWidow/ # Keyboard model for Gazebo simulation
```

## Core Dependencies

### ROS Dependencies
- `rospy` - Python ROS interface
- `trajectory_msgs` - Trajectory message types
- `sensor_msgs` - Sensor data message types
- `std_msgs` - Standard ROS message types
- `cv_bridge` - OpenCV-ROS bridge

### Python Dependencies
- `ultralytics` - YOLO object detection
- `opencv-python-headless` - Computer vision processing
- `numpy-quaternion` - Quaternion mathematics
- `pinocchio` - Robotics algorithms
- `python-quadprog` - Quadratic programming solver
- `pynput` - Input device control
- `pyqt5` - GUI framework

### Motion Planning
- `moveit_commander` - MoveIt! motion planning interface

## Installation & Environment Setup

### ROS Dependencies Installation

Install the required ROS packages:
```bash
# Install core ROS packages
apt update
apt install -y ros-noetic-pinocchio ros-noetic-moveit

# Source ROS environment
source /opt/ros/noetic/setup.bash
```

### Python Environment Configuration

**Important**: Due to how ROS Noetic packages Pinocchio on Ubuntu 20.04, you need to configure the Python path correctly:

```bash
# Add the correct Python path for ROS Noetic's Pinocchio installation
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.8/site-packages

# Make this permanent by adding to your bashrc
echo 'export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.8/site-packages' >> ~/.bashrc
source ~/.bashrc
```

**Verification**: Test that Pinocchio imports correctly:
```bash
python3 -c "import pinocchio; print('SUCCESS: Pinocchio version', pinocchio.__version__)"
```

### Python Dependencies Installation

Install Python packages via pip:
```bash
pip3 install ultralytics opencv-python-headless numpy-quaternion pynput PyQt5
```

**Note**: Use `ros-noetic-pinocchio` (apt) instead of `pip3 install pin` to avoid dependency conflicts.

### Workspace Setup

```bash
# Build the workspace
cd /root/ros_ws
catkin_make

# Source the workspace
source devel/setup.bash

# Make Python scripts executable
find /root/ros_ws/src/Robotic_Arm/src/autonomous_typing/src -name "*.py" -exec chmod +x {} \;
```

## System Architecture

The autonomous typing system follows a modular ROS-based architecture that separates concerns for better maintainability and scalability:

### Modular Design (Recommended)
```
┌─────────────────┐    /keyboard_key_poses    ┌──────────────────┐
│ detection_node  │ ──────────────────────→ │  typing_node     │
│                 │       (PoseArray)        │                  │
│ - YOLO detection│                          │ - MoveIt! motion │
│ - 3D positioning│                          │ - Key pressing   │
│ - TF transforms │                          │ - Sequence exec  │
└─────────────────┘                          └──────────────────┘
        ↑                                             ↑
    Camera Topics                                MoveIt! Planning
```

### Monolithic Design (Legacy)
```
┌─────────────────────────────────────────────────────────────────┐
│                    autonomous.py                                │
│                                                                 │
│  ┌─────────────────┐              ┌──────────────────────────┐  │
│  │ Detection Logic │              │   Motion Planning Logic  │  │
│  │ - YOLO model    │              │   - MoveIt! commands     │  │
│  │ - Stereo vision │              │   - Trajectory execution │  │
│  │ - Key detection │              │   - Safety monitoring    │  │
│  └─────────────────┘              └──────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

**Benefits of Modular Approach**:
- **Separation of Concerns**: Detection and motion planning can be developed independently
- **Fault Isolation**: Failure in one component doesn't crash the entire system
- **Reusability**: Detection node can be used with different motion planners
- **Scalability**: Easy to add new features or replace components
- **Testing**: Each component can be unit tested independently
- **Debugging**: Easier to isolate and fix issues in specific subsystems

## Main Components

### 1. Detection Node (`detection_node.py`)

**Purpose**: Detects keyboard keys using YOLO object detection model.

**Key Features**:
- Uses trained YOLOv8 model (`trained_yolov8n.pt`) for key detection
- Processes synchronized color and depth camera images
- Publishes detected key poses as `PoseArray` messages
- Supports 60+ keyboard key classes including letters, numbers, and special keys

**Topics**:
- **Subscribes**: 
  - `/camera/color/image_raw` (Image)
  - `/camera/depth/image_raw` (Image)
- **Publishes**: 
  - `/keyboard_key_poses` (PoseArray)

### 2. Typing Node (`typing_node.py`)

**Purpose**: Executes typing motions using MoveIt! motion planning.

**Key Features**:
- Receives detected key positions from detection node
- Plans and executes arm movements to press specific keys
- Implements hover-and-press motion strategy
- Supports typing sequences with configurable commands

**Topics**:
- **Subscribes**: 
  - `/keyboard_key_poses` (PoseArray)

### 3. Autonomous Controller (`autonomous.py`)

**Purpose**: Comprehensive autonomous typing system (monolithic controller).

**Key Features**:
- Integrates detection and motion planning in a single node
- Supports stereo camera input (left/right cameras)
- Uses hardcoded keyboard layout from `keyboard_layout.json`
- Implements complete typing pipeline from vision to motion
- Supports debug mode for development

**Parameters**:
- `~input_topic_left`: Left camera image topic
- `~input_topic_right`: Right camera image topic
- `~camera_frame_left`: Left camera frame ID
- `~camera_frame_right`: Right camera frame ID
- `~base_frame`: Robot base frame ID

> **Note**: The monolithic `autonomous.py` is being refactored into separate `detection_node.py` and `typing_node.py` for better modularity, maintainability, and system scalability. This modular approach allows independent development, testing, and deployment of detection and motion planning components.

## Key Detection Classes

The system can detect 60+ keyboard key classes:
```
'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 
'tab', 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 
'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', '"', '\\', 
'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 
'shift-right', 'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
```

## Usage

### Quick Start

1. **Set up environment**:
```bash
# Source ROS and workspace
source /opt/ros/noetic/setup.bash
cd /root/ros_ws
source devel/setup.bash

# Ensure Python path is set for Pinocchio
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.8/site-packages
```

2. **Launch simulation**:
```bash
roslaunch autonomous_typing test.launch
```

### 1. Simulation Mode

Launch the complete system with Gazebo simulation:

```bash
# Ensure environment is properly configured
source /opt/ros/noetic/setup.bash
cd /root/ros_ws
source devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.8/site-packages

# Launch the full system
roslaunch autonomous_typing test.launch
```

This will:
- Start the robotic arm in Gazebo
- Spawn a Razer BlackWidow keyboard model
- Initialize MoveIt! motion planning
- Launch the IK solver

### 2. Running Individual Nodes

**Modular Approach (Recommended)**:

Run detection and typing nodes separately:
```bash
# Terminal 1: Start detection node
rosrun autonomous_typing detection_node.py

# Terminal 2: Start typing execution node  
rosrun autonomous_typing typing_node.py
```

**Legacy Monolithic Approach**:

Run the complete integrated system:
```bash
rosrun autonomous_typing autonomous.py
```

**Detection only** (for testing/debugging):
```bash
rosrun autonomous_typing detection_node.py
```

**Typing execution only** (when key poses are available):
```bash
rosrun autonomous_typing typing_node.py
```

## Configuration

### Keyboard Layout

The `keyboard_layout.json` file contains precise 2D coordinates for each key on the keyboard. Each key is mapped to its physical position:

```json
{
    "A": [x_coordinate, y_coordinate],
    "B": [x_coordinate, y_coordinate],
    ...
}
```

### Camera Calibration

Default camera intrinsics matrix (can be updated based on your camera):
```python
K = np.array([
    [360.01333, 0.0, 243.87228],
    [0.0, 360.013366699, 137.9218444],
    [0.0, 0.0, 1.0]
])
```

## Hardware Requirements

### Robot Configuration
- Compatible with MoveIt!-enabled robotic arms
- Requires gripper-mounted cameras (stereo setup preferred)
- Motion group named "body" for arm control

### Camera Setup
- RGB camera for key detection
- Depth camera for 3D positioning
- Synchronized image capture capability

### Keyboard
- Physical keyboard compatible with the trained YOLO model
- Razer BlackWidow keyboard model included for simulation

## Development Notes

### Model Training
- Uses custom-trained YOLOv8 model (`trained_yolov8n.pt`)
- Model trained specifically for keyboard key detection
- Supports oriented bounding boxes (OBB) for improved accuracy

### Motion Planning
- Implements hover-then-press strategy for key actuation
- Uses MoveIt! for collision-aware path planning
- Configurable hover height (default: 2cm above key)

### Frame Transformations
- Supports TF2 transformations between camera and robot frames
- Handles coordinate system conversions for accurate positioning

## Troubleshooting

### Common Issues

1. **YOLO Model Loading**: Ensure `trained_yolov8n.pt` is in the workspace root

2. **Pinocchio Import Error**: 
   ```bash
   # Solution: Set the correct Python path
   export PYTHONPATH=$PYTHONPATH:/opt/ros/noetic/lib/python3.8/site-packages
   
   # Verify Pinocchio can be imported:
   python3 -c "import pinocchio; print('SUCCESS: Pinocchio version', pinocchio.__version__)"
   ```

3. **Camera Topics**: Verify camera topic names match your hardware setup

4. **MoveIt! Configuration**: Ensure the arm's MoveIt! config is properly loaded

5. **TF Frames**: Check that all required TF frames are being published

6. **Environment Setup**: Always source both ROS and workspace:
   ```bash
   source /opt/ros/noetic/setup.bash
   source /root/ros_ws/devel/setup.bash
   ```

### Debug Mode

Enable debug logging:
```python
controller = Controller(debug=True)
```

### Environment Verification

Run these commands to verify your setup:
```bash
# Check ROS environment
echo $ROS_DISTRO

# Check Python path
python3 -c "import sys; print('\n'.join(sys.path))"

# Test Pinocchio import
python3 -c "import pinocchio; print('Pinocchio OK')"

# Check workspace
ls /root/ros_ws/devel/setup.bash
```
To Make All Scripts Executable
```bash
find ~/ros_ws/src/Robotic_Arm -type f -name "*.py" -exec chmod +x {} \;
```
## Future Enhancements

- [ ] Error correction and retry mechanisms
- [ ] Real-time keyboard detection and registration
- [ ] Visual Servoing for fine actuator control near the point of contact
- [ ] Automatic camera calibration
- [ ] Support for different keyboard layouts
- [ ] Docker container with pre-configured environment