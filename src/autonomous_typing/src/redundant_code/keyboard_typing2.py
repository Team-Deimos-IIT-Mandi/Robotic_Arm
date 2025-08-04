#!/usr/bin/env python3

import pinocchio as pin
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
from PyQt5.QtCore import Qt, QTimer
from quadprog import solve_qp
import os
from pinocchio.visualize import MeshcatVisualizer
from typing import Dict, List, Tuple
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
from ultralytics import YOLO
import json

TOTAL_LENGTH_MM = 354.076  # Length in mm
TOTAL_WIDTH_MM = 123.444   # Width in mm

class KeyboardTyperWithIK(QWidget):
    def __init__(self, camera_matrix=None, dist_coeffs=None):
        super().__init__()
        
        # Initialize ROS node
        rospy.init_node('keyboard_typer_ik')
        self.pub = rospy.Publisher('/body_controller/command', JointTrajectory, queue_size=10)
        
        self.setup_robot()
        self.setup_ui()
        self.setup_control()
        
        # Keyboard typer specific attributes
        self.keboard_dimensions = (TOTAL_LENGTH_MM, TOTAL_WIDTH_MM)
        self.key_positions: Dict[str, np.ndarray] = {}
        self.home_position: np.ndarray = np.array([0.2, 0, 0.3])  # Default home position
        self.hover_distance = 0.01  # 1cm hover distance
        
        self.all_keys = [
            "ESC", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12",
            "PRTSC", "SCRLK", "PAUSE", "`", "1", "2", "3", "4", "5", "6", "7", "8", "9", "0",
            "-", "=", "BACKSPACE", "INS", "HOME", "PAGEUP", "TAB", "Q", "W", "E", "R", "T", "Y",
            "U", "I", "O", "P", "[", "]", "\\", "DEL", "END", "PAGEDOWN", "CAPSLOCK", "A", "S",
            "D", "F", "G", "H", "J", "K", "L", ";", "'", "ENTER", "SHIFT", "Z", "X", "C", "V",
            "B", "N", "M", ",", ".", "/", "UP", "CTRL", "WIN", "ALT", "SPACE", "FN", "MENU",
            "LEFT", "DOWN", "RIGHT"
        ]
        
        # Camera parameters (optional)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        
        self.model_path = rospy.get_param('~model_path', 'yolov8n-seg.pt')
        self.keyboard_layout_path = rospy.get_param('~keyboard_layout_path', 'keyboard_layout.json')
        self.yolo_segmenter = None
        image_topic = rospy.get_param('~image_topic', '/camera_gripper/image_raw')

    def scan(self, image_msg):
        """Simulate an external function call to process the image."""
        self.key_positions = self.yolo_segmenter.process(image_msg)
        
    def setup_robot(self):
        """
        Initialize robot model and visualization
        """
        # Get the path to the URDF file
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "../urdf/Arm_Urdf.urdf")
        
        # Build the robot model
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # Load visualization models
        self.visual_model = pin.buildGeomFromUrdf(self.model, urdf_path, pin.GeometryType.VISUAL)
        self.collision_model = pin.buildGeomFromUrdf(self.model, urdf_path, pin.GeometryType.COLLISION)
        
        # Setup visualizer
        self.viz = MeshcatVisualizer(self.model, self.collision_model, self.visual_model)
        self.viz.initViewer(loadModel=True)
        
        # Get end effector frame
        self.end_effector_frame = self.model.getFrameId("Link_6")  # Adjust frame name if needed
        
        # Get joint limits
        self.q_min = self.model.lowerPositionLimit
        self.q_max = self.model.upperPositionLimit
        
    def setup_ui(self):
        """
        Setup the user interface
        """
        # Create main layout
        layout = QVBoxLayout()
        
        # Add start typing button
        self.start_btn = QPushButton('Start Typing')
        self.start_btn.clicked.connect(self.start_typing)
        layout.addWidget(self.start_btn)
        
        # Add home position button
        self.home_btn = QPushButton('Go Home')
        self.home_btn.clicked.connect(self.move_to_home)
        layout.addWidget(self.home_btn)
        
        # Set the layout
        self.setLayout(layout)
        self.setWindowTitle('Keyboard Typer IK Controller')
        self.resize(300, 200)
        
    def setup_control(self):
        """
        Initialize control parameters and timers
        """
        # Control parameters
        self.velocity_scale = 0.1  # Velocity scaling factor
        self.dt = 0.05            # Time step for integration
        self.damping = 1e-6       # Regularization factor
        
        # Joint velocity limits
        self.theta_dot_max = 1.0 * np.ones(self.model.nv)
        self.theta_dot_min = -1.0 * np.ones(self.model.nv)
        
        # Initialize joint configuration to neutral position
        self.q = pin.neutral(self.model)
        self.viz.display(self.q)
        
        # Setup control timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.control_loop)
        self.timer.start(int(self.dt * 1000))
        
    def publish_joint_angles(self, joint_angles):
        """
        Publish joint angles to ROS
        """
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = ['Joint_1', 'Joint_2', 'Joint_3',
                                    'Joint_4', 'Joint_5', 'Joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(0.1)
        
        trajectory_msg.points = [point]
        self.pub.publish(trajectory_msg)
    
    def move_to_key(self, target_position: np.ndarray):
        """
        Move to a specific key position
        """
        if self.check_workspace_limits(target_position):
            self.set_target(target_position)
            
            # Wait for movement to complete
            while hasattr(self, 'current_target'):
                rospy.sleep(0.01)
        else:
            print("Target position out of workspace bounds")

    def move_to_home(self):
        """
        Move the robot to its home position
        """
        self.move_to_key(self.home_position)
        
    def start_typing(self):
        """
        Start the typing process
        TODO: Implement typing logic
        """
        print("Typing functionality not yet implemented")

    def control_loop(self):
        """
        Main control loop for robot movement
        Checks key positions and executes movements based on current state
        """
        try:
            # Get current robot state
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            current_position = self.data.oMf[self.end_effector_frame].translation
            
            # Safety check - ensure we're not too close to workspace limits
            if not self.check_workspace_limits(current_position):
                print("Warning: Approaching workspace limits")
                self.emergency_stop()
                return
                
            # Check if we're currently executing a movement
            if hasattr(self, 'current_target'):
                # Get distance to target
                error = self.current_target - current_position
                
                if np.linalg.norm(error) < 1e-3:  # Within tolerance
                    # Movement complete
                    delattr(self, 'current_target')
                else:
                    # Continue movement to target
                    # Compute Jacobian
                    J = pin.computeFrameJacobian(self.model, self.data, self.q, 
                                            self.end_effector_frame,
                                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :]
                    
                    # QP Setup for smooth motion
                    H = J.T @ J + self.damping * np.eye(self.model.nv)
                    g = -J.T @ error
                    
                    # Joint limits constraints
                    q_upper_violation = (self.q_max - self.q) / self.dt
                    q_lower_violation = (self.q_min - self.q) / self.dt
                    
                    C = np.vstack([np.eye(self.model.nv), -np.eye(self.model.nv),
                                np.eye(self.model.nv), -np.eye(self.model.nv)])
                    b = np.hstack([self.theta_dot_min, -self.theta_dot_max,
                                q_lower_violation, -q_upper_violation])
                    
                    # Solve QP for joint velocities
                    try:
                        dq = solve_qp(H, g, C.T, b)[0]
                        
                        # Scale velocities if needed
                        vel_scale = min(1.0, self.velocity_scale / np.max(np.abs(dq)))
                        dq *= vel_scale
                        
                        # Update configuration
                        self.q = pin.integrate(self.model, self.q, dq * self.dt)
                        
                        # Update visualization and send to robot
                        self.viz.display(self.q)
                        self.publish_joint_angles(self.q)
                        
                    except Exception as e:
                        print(f"QP solver error: {e}")
                        self.emergency_stop()
                        
        except Exception as e:
            print(f"Control loop error: {e}")
            self.emergency_stop()

    def set_target(self, target_position):
        """
        Set a new target position for the control loop
        """
        if self.check_workspace_limits(target_position):
            self.current_target = target_position
        else:
            print("Target position outside workspace limits")
    
    def emergency_stop(self):
        """
        Emergency stop function
        """
        self.timer.stop()
        # Stop all movement
        self.publish_joint_angles(self.q)
        print("Emergency stop activated!")
        
    def check_workspace_limits(self, position):
        """
        Check if target position is within robot workspace
        """
        # Define workspace limits
        workspace_limits = {
            'x': (-0.5, 0.5),  # meters
            'y': (-0.5, 0.5),
            'z': (0.0, 0.6)
        }
        
        return (workspace_limits['x'][0] <= position[0] <= workspace_limits['x'][1] and
                workspace_limits['y'][0] <= position[1] <= workspace_limits['y'][1] and
                workspace_limits['z'][0] <= position[2] <= workspace_limits['z'][1])

# Static utility method for converting string to key clicks
def string_to_keyboard_clicks(input_string):
    keyboard_clicks = []
    caps_active = False  # Track CAPS state

    for char in input_string:
        if char.isupper() and not caps_active:
            # Activate CAPS if the character is uppercase and CAPS is not active
            keyboard_clicks.append("CAPSLOCK")
            caps_active = True
        elif char.islower() and caps_active:
            # Deactivate CAPS if the character is lowercase and CAPS is active
            keyboard_clicks.append("CAPSLOCK")
            caps_active = False
        
        if char.isalnum() or char in {'-', '_'}:  # Letters, numbers, and some symbols
            keyboard_clicks.append(char.upper() if not caps_active else char)
        elif char.isspace():
            keyboard_clicks.append("SPACE")
        else:
            # Add any non-alphanumeric, non-space character as is
            keyboard_clicks.append(char)
    
    # End with ENTER
    keyboard_clicks.append("ENTER")
    
    return keyboard_clicks
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point

import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
from ultralytics import YOLO
import json

class YoloPixelSegmentationNode:
    
    def __init__(self):
        rospy.init_node('yolo_pixel_segmentation_node', anonymous=True)
        
        # Parameters
        self.input_topic = rospy.get_param('~input_topic', '/camera_gripper/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera_gripper/camera_info')
        self.output_topic = rospy.get_param('~output_topic', '/camera_gripper/processed_image')
        self.class_to_detect = rospy.get_param('~class_to_detect', 66)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link2')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Real-world keyboard dimensions (mm)
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        
        # Initialize YOLO model
        self.model = YOLO('yolov8n-seg.pt')
        
        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Setup ROS communication
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=10)
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Load keyboard layout
        with open('keyboard_layout.json', 'r') as f:
            self.keyboard_points_dict = json.load(f)
            
        self.key_points = []
        
        rospy.loginfo("YoloPixelSegmentationNode initialized.")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        


    # def get_transform(self):
    #     try:
    #         # Get transform from camera frame to base frame
    #         transform = self.tf_buffer.lookup_transform("world", self.camera_frame, rospy.Time(0), rospy.Duration(1.0))
            
    #         # Convert to 4x4 transformation matrix
    #         translation = transform.transform.translation
    #         rotation = transform.transform.rotation
            
    #         # # Create transformation matrix
    #         # T_base_camera = np.eye(4)
    #         # T_base_camera[:3, 3] = [translation.x, translation.y, translation.z]
            
    #         # # Convert quaternion to rotation matrix
    #         # quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    #         # T_base_camera[:3, :3] = tf2_ros.transformations.quaternion_matrix(quat)[:3, :3]
            
    #         # return T_base_camera
    #     except Exception as e:
    #         rospy.logerr(f"Failed to get transform: {e}")
    #         return None
    
    def get_transform(self, camera_point , camera_frame="camera_link2"):
        """Transform a point from the camera frame to the world frame."""
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            # Wait for the transform to be available
            transform = tf_buffer.lookup_transform("world", camera_frame, rospy.Time(0), rospy.Duration(1.0))
            # Transform the point
            world_point = do_transform_point(camera_point, transform)
            return world_point
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
        
    def process_image(self, cv_image, results):
        vis_image = cv_image.copy()
        
        if len(results) == 0:
            return vis_image

        # Get predictions from the first result
        result = results[0]
        
        if len(result.boxes) == 0:
            return vis_image

        # Get the camera-to-base transformation matrix
        T_base_camera = self.get_transform()
        if T_base_camera is None:
            return vis_image

        for idx in range(len(result.boxes)):
            class_id = int(result.boxes.cls[idx].item())
            if class_id == self.class_to_detect:
                box = result.boxes.xyxy[idx].cpu().numpy()
                x1, y1, x2, y2 = map(int, box)
                keyboard_labels = self.keyboard_points_dict.keys()
                keyboard_points = np.array(list(self.keyboard_points_dict.values()))
                box_length = x2 - x1
                box_width = y2 - y1
                scaled_points = (keyboard_points / np.array([self.KEYBOARD_LENGTH, self.KEYBOARD_WIDTH])) * \
                              np.array([box_length, box_width]) + np.array([x1, y1])

                if self.camera_matrix is not None:
                    try:
                        model_points = np.zeros((len(keyboard_points), 3))
                        model_points[:, 0] = keyboard_points[:, 0]
                        model_points[:, 1] = keyboard_points[:, 1]
                        
                        success, rvec, tvec = cv2.solvePnP(model_points, scaled_points, 
                                                           self.camera_matrix, self.dist_coeffs,
                                                           flags=cv2.SOLVEPNP_ITERATIVE)
                        if success:
                            self.draw_axes(vis_image, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                            projected_points, _ = cv2.projectPoints(model_points, rvec, tvec, 
                                                                    self.camera_matrix, self.dist_coeffs)
                            projected_points = projected_points.reshape(-1, 2) 

                            # Transform to base frame
                            # model_points_h = np.hstack((model_points, np.ones((model_points.shape[0], 1))))  # Homogeneous
                            # points_in_camera_frame = (np.hstack((np.eye(3), tvec)).dot(model_points_h.T)).T
                            
                            # points_in_camera_frame_h = np.hstack((points_in_camera_frame, np.ones((points_in_camera_frame.shape[0], 1))))
                            
                            # points_in_base_frame = (T_base_camera.dot(points_in_camera_frame_h.T)).T[:, :3]
                            
                            points_in_base_frame = self.get_transform(projected_points)
                            self.key_points = zip(keyboard_labels,keyboard_points)

                            # Save points
                            np.save('projected_points.npy', points_in_base_frame)
                            
                            for point in points_in_base_frame:
                                rospy.loginfo(f"Point in base frame: {point}")
                            
                    except Exception as e:
                        rospy.logwarn(f"PnP estimation failed: {e}")
        
        return vis_image

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 640))
            results = self.model(cv_image)
            processed_image = self.process_image(cv_image, results)
            ros_image = self.bridge.cv2_to_imgmsg(processed_image, encoding="bgr8")
            self.publisher.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")


