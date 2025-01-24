#!/usr/bin/env python3

import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
import json
import os
import pinocchio as pin
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Constants for keyboard dimensions
TOTAL_LENGTH_MM = 354.076  # Length in mm
TOTAL_WIDTH_MM = 123.444   # Width in mm

class KeyboardTyper:
    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            rospy.init_node("autonomous_typing", log_level=rospy.DEBUG)
            logging.basicConfig(level=logging.DEBUG)
        else:
            rospy.init_node("autonomous_typing", log_level=rospy.INFO)
            logging.basicConfig(level=logging.INFO)
        
        rospy.loginfo("Initializing KeyboardTyper")
        
        # Initialize variables
        self.KEYBOARD_LENGTH = TOTAL_LENGTH_MM
        self.KEYBOARD_WIDTH = TOTAL_WIDTH_MM
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            rospy.loginfo(f"Loading keyboard points from {json_path}")
            self.keyboard_points = json.load(f)
        
        self.class_to_detect = 66  # YOLO class ID for keyboard
        
        # Camera parameters
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        # Initialize YOLO model
        rospy.logdebug("Loading YOLO model")
        self.model = YOLO('yolov8n-seg.pt')
        rospy.sleep(1)
        os.environ['YOLO_LOG_LEVEL'] = 'error'
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        
        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        
        # Publishers
        self.annotated_image_pub_l = rospy.Publisher("/annotated_image_l", Image, queue_size=10)
        self.annotated_image_pub_r = rospy.Publisher("/annotated_image_r", Image, queue_size=10)
        
        # Initialize variables
        self.left_img = None
        self.right_img = None
        self.camera_info_left = None
        self.camera_info_right = None
        self.keypoints_left = None
        self.keypoints_right = None
        self.keyboard_points_3d = None
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.baseline = 0.1  # Baseline distance between cameras
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize Pinocchio robot model
        self.setup_pinocchio()
        
        rospy.loginfo("KeyboardTyper initialized")

    def setup_pinocchio(self):
        """
        Initialize Pinocchio robot model for kinematics and dynamics.
        """
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(script_dir, "../urdf/Arm_Urdf.urdf")
        
        # Build the robot model
        self.model_pin = pin.buildModelFromUrdf(urdf_path)
        self.data_pin = self.model_pin.createData()
        
        # Get end effector frame
        self.end_effector_frame = self.model_pin.getFrameId("Link_6")  # Adjust frame name if needed
        
        # Initialize joint configuration to neutral position
        self.q = pin.neutral(self.model_pin)
        
        # Joint velocity limits
        self.theta_dot_max = 1.0 * np.ones(self.model_pin.nv)
        self.theta_dot_min = -1.0 * np.ones(self.model_pin.nv)
        
        # Control parameters
        self.velocity_scale = 0.1  # Velocity scaling factor
        self.dt = 0.05            # Time step for integration
        self.damping = 1e-6       # Regularization factor
        
        # Publisher for joint trajectories
        self.pub = rospy.Publisher('/body_controller/command', JointTrajectory, queue_size=10)
    
    def image_callback_left(self, msg):
        """
        Callback for left camera image.
        """
        rospy.logdebug("Left image callback triggered")
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_left, corners = self.processing_image(self.left_img)
            self.display_output_l(self.left_img, corners)
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        """
        Callback for right camera image.
        """
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_right, corners = self.processing_image(self.right_img)
            self.display_output_r(self.right_img, corners)
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")
    
    def camera_info_callback_left(self, msg):
        """
        Callback for left camera info.
        """
        rospy.logdebug("Left camera info callback triggered")
        self.camera_info_left = msg

    def camera_info_callback_right(self, msg):
        """
        Callback for right camera info.
        """
        rospy.logdebug("Right camera info callback triggered")
        self.camera_info_right = msg
    
    def processing_image(self, img):
        """
        Process the image to detect keyboard and keypoints using YOLO.
        """
        rospy.logdebug("Processing image")
        scaled_points = {}
        try:
            result = self.model(img, stream=True)
            result = next(result)
            for idx in range(len(result.boxes)):
                class_id = int(result.boxes.cls[idx].item())
                rospy.logdebug(f"Detected class_id: {class_id}")
                if class_id == self.class_to_detect:
                    box = result.boxes.xyxy[idx].cpu().numpy()
                    x1, y1, x2, y2 = map(int, box)
                    box_length = x2 - x1
                    box_width = y2 - y1
                    rospy.logdebug(f"Box dimensions - Length: {box_length}, Width: {box_width}")
                    corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                    for key, value in self.keyboard_points.items():
                        try:
                            scaled_x = (float(value[0]) / self.KEYBOARD_LENGTH) * box_length + x1
                            scaled_y = (float(value[1]) / self.KEYBOARD_WIDTH) * box_width + y1
                            scaled_points[key] = [int(scaled_x), int(scaled_y)]
                        except ValueError:
                            rospy.logerr(f"Non-numeric value for key '{key}': {value}")
            rospy.logdebug(f"Scaled points: {scaled_points}")
            return scaled_points, corners
        except StopIteration:
            rospy.logwarn("No YOLO results")
            return scaled_points, []
        except Exception as e:
            rospy.logerr(f"Error in processing_image: {e}")
            return scaled_points, []
    
    def calc_3d_pos(self, keypoints_left, keypoints_right, camera_info_left, camera_info_right):
        """
        Calculate the 3D position of keypoints using stereo vision.
        """
        if keypoints_left is None or keypoints_right is None:
            rospy.logwarn("Keypoints from one or both cameras are missing.")
            return {}

        if camera_info_left is None or camera_info_right is None:
            rospy.logwarn("Camera info for one or both cameras is missing.")
            return {}

        # Extract camera intrinsic parameters
        fx_left = camera_info_left.K[0]  # Focal length in x-direction (left camera)
        fy_left = camera_info_left.K[4]  # Focal length in y-direction (left camera)
        cx_left = camera_info_left.K[2]  # Principal point x-coordinate (left camera)
        cy_left = camera_info_left.K[5]  # Principal point y-coordinate (left camera)

        fx_right = camera_info_right.K[0]  # Focal length in x-direction (right camera)
        fy_right = camera_info_right.K[4]  # Focal length in y-direction (right camera)
        cx_right = camera_info_right.K[2]  # Principal point x-coordinate (right camera)
        cy_right = camera_info_right.K[5]  # Principal point y-coordinate (right camera)

        keypoints_3d = {}

        for key in keypoints_left:
            if key not in keypoints_right:
                rospy.logwarn(f"Keypoint {key} not found in right camera image.")
                continue

            # Get pixel coordinates from left and right images
            x_left, y_left = keypoints_left[key]
            x_right, y_right = keypoints_right[key]

            # Calculate disparity (difference in x-coordinates)
            disparity = x_left - x_right

            if disparity == 0:
                rospy.logwarn(f"Disparity for keypoint {key} is zero.")
                continue

            # Calculate depth (Z) using the disparity
            Z = (fx_left * self.baseline) / disparity

            # Calculate X and Y in 3D space
            X = (x_left - cx_left) * Z / fx_left
            Y = (y_left - cy_left) * Z / fy_left

            # Store the 3D position
            keypoints_3d[key] = [X, Y, Z]

        return keypoints_3d
    
    def get_transform(self, point):
        """
        Transform a point from the camera frame to the world frame.
        """
        try:
            camera_point = PointStamped()
            camera_point.header.frame_id = self.camera_frame_left
            camera_point.point.x = point[2]
            camera_point.point.y = -point[0]
            camera_point.point.z = -point[1]
            transform = self.tf_buffer.lookup_transform("world", self.camera_frame_left, rospy.Time(0), rospy.Duration(1.0))
            world_point = do_transform_point(camera_point, transform)
            return [world_point.point.x, world_point.point.y, world_point.point.z]
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
    
    def move_arm_to_position(self, position_3d):
        """
        Move the robotic arm to the calculated 3D position using Pinocchio.
        """
        # Set target position
        self.set_target(position_3d)
        
        # Wait for movement to complete
        while hasattr(self, 'current_target'):
            rospy.sleep(0.01)
        
        return True
    
    def set_target(self, target_position):
        """
        Set a new target position for the control loop.
        """
        if self.check_workspace_limits(target_position):
            self.current_target = target_position
        else:
            rospy.logwarn("Target position outside workspace limits")
    
    def check_workspace_limits(self, position):
        """
        Check if target position is within robot workspace.
        """
        workspace_limits = {
            'x': (-1.0, 1.0),  # meters
            'y': (-1.0, 1.0),
            'z': (0.0, 1.0)
        }
        
        return (workspace_limits['x'][0] <= position[0] <= workspace_limits['x'][1] and
                workspace_limits['y'][0] <= position[1] <= workspace_limits['y'][1] and
                workspace_limits['z'][0] <= position[2] <= workspace_limits['z'][1])
    
    def control_flow(self):
        """
        Execute keyboard clicks by moving the robotic arm to each key's position.
        """
        input_string = input("Enter the string to type: ")
        keyboard_clicks = self.string_to_keyboard_clicks(input_string)
        print(f"Keyboard clicks: {keyboard_clicks}")
        
        # Calculate 3D positions of keypoints
        ans = self.calc_3d_pos(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
        for key in ans:
            ans[key] = np.array(self.get_transform(ans[key]))
        self.keyboard_points_3d = ans
        
        try:
            # Move to home position before starting
            # self.move_arm_to_position(self.home_position)
            
            # Iterate through keyboard clicks
            for click in keyboard_clicks:
                if click not in self.keyboard_points:
                    rospy.logwarn(f"Key {click} not found in keyboard points. Skipping.")
                    continue
                
                # Get the 3D position for this key
                key_position = self.keyboard_points_3d[click]
                hover_position = key_position + np.array([0, -0.1, 0])
                
                # Move arm to the key position
                success = self.move_arm_to_position(hover_position)
                if not success:
                    rospy.logerr(f"Failed to move to key {click}")
                    return False
                
                rospy.sleep(0.5)  # Brief pause to simulate key press
            
            # Return to home position after typing
            # self.move_arm_to_position(self.home_position)
            
            return True
        
        except Exception as e:
            rospy.logerr(f"Error in control flow: {e}")
            return False
    
    def string_to_keyboard_clicks(self, input_string):
        """
        Convert input string to a sequence of keyboard clicks.
        """
        keyboard_clicks = []
        caps_active = False  # Track CAPS state

        for char in input_string:
            if char.isupper() and not caps_active:
                keyboard_clicks.append("CAPSLOCK")
                caps_active = True
            elif char.islower() and caps_active:
                keyboard_clicks.append("CAPSLOCK")
                caps_active = False
            
            if char.isalnum() or char in {'-', '_'}:  # Letters, numbers, and some symbols
                keyboard_clicks.append(char.upper() if not caps_active else char)
            elif char.isspace():
                keyboard_clicks.append("SPACE")
            else:
                keyboard_clicks.append(char)
        
        keyboard_clicks.append("ENTER")
        
        return keyboard_clicks

def main():
    controller = KeyboardTyper(debug=False)
    rospy.sleep(15)  # Wait for initialization
    controller.control_flow()
    rospy.spin()

if __name__ == "__main__":
    main()