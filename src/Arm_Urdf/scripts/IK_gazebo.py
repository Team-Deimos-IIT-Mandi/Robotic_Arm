#!/usr/bin/env python3

import sys
import os
import numpy as np

# Add the correct Python path for ROS Noetic pinocchio
sys.path.insert(0, '/opt/ros/noetic/lib/python3.8/site-packages')

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtCore import Qt, QTimer

# Try different pinocchio import methods
try:
    import pinocchio as pin
    # Test the correct API function
    if hasattr(pin, 'buildModelFromUrdf'):
        BUILD_FUNC = pin.buildModelFromUrdf
        rospy.loginfo("Found buildModelFromUrdf")
        PINOCCHIO_AVAILABLE = True
    else:
        raise ImportError("No URDF build function found")
    rospy.loginfo(f"Pinocchio imported successfully, version: {pin.__version__}")
except ImportError as e:
    rospy.logerr(f"Failed to import pinocchio: {e}")
    rospy.loginfo("Falling back to simple joint control")
    pin = None
    PINOCCHIO_AVAILABLE = False

# Import quadprog with fallback
try:
    from quadprog import solve_qp
    QUADPROG_AVAILABLE = True
except ImportError:
    rospy.logwarn("quadprog not available, using pseudo-inverse method")
    QUADPROG_AVAILABLE = False

# ROS Publisher setup
rospy.init_node('full_ik_publisher')
pub = rospy.Publisher('/body_controller/command', JointTrajectory, queue_size=10)

# Get the absolute path to the directory containing this script
script_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(script_dir, "../urdf/Arm_Urdf.urdf")

# Check if URDF file exists
if not os.path.exists(urdf_path):
    rospy.logerr(f"URDF file not found at: {urdf_path}")
    # Use fallback mode
    PINOCCHIO_AVAILABLE = False

# Try to load the model with the correct API
if pin is not None and PINOCCHIO_AVAILABLE and os.path.exists(urdf_path):
    try:
        # Use the correct function name
        model = BUILD_FUNC(urdf_path)
        data = model.createData()
        rospy.loginfo("Successfully loaded robot model with Pinocchio")
        
        # Check if end-effector frame exists
        try:
            end_effector_frame = model.getFrameId("Link_6")
            rospy.loginfo("Successfully found end-effector frame: Link_6")
        except:
            rospy.logerr("Frame 'Link_6' not found in model")
            # List available frames
            rospy.loginfo("Available frames:")
            for i in range(model.nframes):
                rospy.loginfo(f"  {model.frames[i].name}")
            # Don't exit, just use fallback mode
            PINOCCHIO_AVAILABLE = False
            
        # Initialize joint configuration
        if PINOCCHIO_AVAILABLE:
            q = pin.neutral(model)
            q_min = model.lowerPositionLimit
            q_max = model.upperPositionLimit
        else:
            q = np.zeros(6)
            q_min = np.full(6, -3.14)
            q_max = np.full(6, 3.14)
        
    except Exception as e:
        rospy.logerr(f"Failed to load robot model with Pinocchio: {e}")
        rospy.loginfo("Falling back to simple joint control")
        PINOCCHIO_AVAILABLE = False
        # Use a simple fallback with 6 joints
        q = np.zeros(6)
        q_min = np.full(6, -3.14)
        q_max = np.full(6, 3.14)
else:
    PINOCCHIO_AVAILABLE = False
    # Simple fallback
    q = np.zeros(6)
    q_min = np.full(6, -3.14)
    q_max = np.full(6, 3.14)

# Parameters
velocity_scale = 0.1
dt = 0.05

rospy.loginfo(f"Initial joint configuration: {q}")
rospy.loginfo(f"Joint limits - min: {q_min}")
rospy.loginfo(f"Joint limits - max: {q_max}")
rospy.loginfo(f"Pinocchio available: {PINOCCHIO_AVAILABLE}")

# Key-to-movement mapping
key_joint_mapping = {
    Qt.Key_1: (0, velocity_scale),   # Joint 1 positive
    Qt.Key_2: (0, -velocity_scale),  # Joint 1 negative
    Qt.Key_3: (1, velocity_scale),   # Joint 2 positive
    Qt.Key_4: (1, -velocity_scale),  # Joint 2 negative
    Qt.Key_5: (2, velocity_scale),   # Joint 3 positive
    Qt.Key_6: (2, -velocity_scale),  # Joint 3 negative
    Qt.Key_7: (3, velocity_scale),   # Joint 4 positive
    Qt.Key_8: (3, -velocity_scale),  # Joint 4 negative
    Qt.Key_9: (4, velocity_scale),   # Joint 5 positive
    Qt.Key_0: (4, -velocity_scale),  # Joint 5 negative
    Qt.Key_Q: (5, velocity_scale),   # Joint 6 positive
    Qt.Key_W: (5, -velocity_scale),  # Joint 6 negative
}

class SimpleJointController(QWidget):
    def __init__(self):
        super().__init__()
        self.pressed_keys = set()
        self.timer = QTimer()
        self.timer.timeout.connect(self.control_loop)
        self.timer.start(int(dt * 1000))
        
        # Set window properties
        self.setWindowTitle("Robot Joint Controller")
        self.setGeometry(100, 100, 500, 400)
        self.setFocusPolicy(Qt.StrongFocus)
        
        rospy.loginfo("SimpleJointController initialized")
        rospy.loginfo("Controls:")
        rospy.loginfo("1/2: Joint 1 +/-")
        rospy.loginfo("3/4: Joint 2 +/-") 
        rospy.loginfo("5/6: Joint 3 +/-")
        rospy.loginfo("7/8: Joint 4 +/-")
        rospy.loginfo("9/0: Joint 5 +/-")
        rospy.loginfo("Q/W: Joint 6 +/-")

    def keyPressEvent(self, event):
        self.pressed_keys.add(event.key())
        # Map key codes to readable names
        key_names = {
            Qt.Key_1: "1", Qt.Key_2: "2", Qt.Key_3: "3", Qt.Key_4: "4",
            Qt.Key_5: "5", Qt.Key_6: "6", Qt.Key_7: "7", Qt.Key_8: "8",
            Qt.Key_9: "9", Qt.Key_0: "0", Qt.Key_Q: "Q", Qt.Key_W: "W"
        }
        key_name = key_names.get(event.key(), f"Key_{event.key()}")
        rospy.loginfo(f"Key pressed: {key_name}")

    def keyReleaseEvent(self, event):
        self.pressed_keys.discard(event.key())

    def publish_joint_angles(self, joint_angles):
        try:
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = ['Joint_1', 'Joint_2', 'Joint_3',
                                          'Joint_4', 'Joint_5', 'Joint_6']

            point = JointTrajectoryPoint()
            point.positions = joint_angles.tolist()
            point.time_from_start = rospy.Duration(0.1)

            trajectory_msg.points = [point]
            pub.publish(trajectory_msg)
            rospy.logdebug(f"Published: {np.round(joint_angles, 3)}")
        except Exception as e:
            rospy.logerr(f"Failed to publish joint angles: {e}")

    def control_loop(self):
        global q
        
        try:
            # Simple joint control
            joint_deltas = np.zeros(6)
            
            for key in self.pressed_keys:
                if key in key_joint_mapping:
                    joint_idx, delta = key_joint_mapping[key]
                    joint_deltas[joint_idx] += delta * dt
            
            if np.any(np.abs(joint_deltas) > 1e-6):
                # Update joint configuration
                q_new = q + joint_deltas
                
                # Check joint limits
                if np.all(q_new >= q_min) and np.all(q_new <= q_max):
                    q = q_new
                    self.publish_joint_angles(q)
                else:
                    rospy.logwarn("Joint limits would be violated, skipping update")

        except Exception as e:
            rospy.logerr(f"Control loop error: {e}")

if __name__ == "__main__":
    try:
        app = QApplication([])
        controller = SimpleJointController()
        controller.show()
        
        rospy.loginfo("Starting robot joint controller GUI...")
        rospy.loginfo("Focus on the GUI window and press keys to control joints")
        rospy.loginfo("Keys: 1/2=J1, 3/4=J2, 5/6=J3, 7/8=J4, 9/0=J5, Q/W=J6")
        
        app.exec_()
    except Exception as e:
        rospy.logerr(f"Failed to start application: {e}")
    finally:
        rospy.signal_shutdown("Application closed")