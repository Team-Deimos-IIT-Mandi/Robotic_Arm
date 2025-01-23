import rospy
import rospy
from PyQt5.QtWidgets import QApplication
from threading import Thread
from PyQt5.QtCore import QTimer
from gui import TypingGUI
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import json
import os

class Controller:
    def __init__(self):
        # Parameters
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        self.class_to_detect = 66
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_base_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_base_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_base_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_base_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_base_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_base_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        # YOLO model
        self.model = YOLO('yolov8n-seg.pt')

        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)

        # Images from cameras
        self.left_img = None
        self.right_img = None
        self.camera_info_left = None
        self.camera_info_right = None
        
        # Results from YOLO model
        self.result_left = None
        self.result_right = None


        # GUI
        self.app = QApplication([])
        self.gui = TypingGUI()
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(100)  # Update every 100ms
        
        # Keypoints
        self.keypoints_left = None
        self.keypoints_right = None
        
        # Keyboard points
        self.keyboard_points = None #dictionary
        
        self.keyboard_points_dict = None
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            self.keyboard_points = json.load(f)
            
        # Camera parameters
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.baseline = 0.1
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # MoveIt setup
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
    
    def update_gui(self):
        """Update the GUI with the latest data."""
        self.gui.update_images(self.left_img, self.right_img)
        # Example 2D and 3D data
        if self.left_img is not None and self.right_img is not None:
            self.result_left = self.processing_image(self.left_img)
            self.result_right = self.processing_image(self.right_img)
            if self.result_left is not None and self.result_right is not None:
                key_2d_left = self.result_left
                key_2d_right = self.result_right
            key_3d = self.calculate_3d_position(key_2d_left, key_2d_right, self.camera_info_left, self.camera_info_right)
            self.gui.update_coordinates(key_2d_left, key_2d_right, key_3d)


    def image_callback_left(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_left = self.processing_image(self.left_img)
            if self.scan == 1:
                self.keyboard_points = self.calculate_3d_position(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
            # self.keyboard_corners,depth = self.calculate_depth()
            # self.keyboard_points = ke
            
            
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_right = self.processing_image(self.right_img)
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")
            
    def camera_info_callback_left(self, msg):
        self.camera_info_left = msg
    def camera_info_callback_right(self, msg):
        self.camera_info_right = msg
        
    def processing_image(self, img):
        result = self.model(img, stream=True)
        result = result[0]
        for idx in range(len(result.boxes)):
            class_id = int(result.boxes.cls[idx].item())
            if class_id == self.class_to_detect:
                box = result.boxes.xyxy[idx].cpu().numpy()
                x1, y1, x2, y2 = map(int, box)
                box_length = x2 - x1
                box_width = y2 - y1
                # corners = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
                scaled_points = {}
                for key, value in self.keyboard_points_dict.items():
                    scaled_x = (value[0] / self.KEYBOARD_LENGTH) * box_length + x1
                    scaled_y = (value[1] / self.KEYBOARD_WIDTH) * box_width + y1
                    scaled_points[key] = [int(scaled_x), int(scaled_y)]
        return scaled_points

    def calculate_3d_position(self, key_2d_left, key_2d_right, camera_info_left, camera_info_right):
        """Calculate the 3D position of a key using stereo vision."""
        
        k2vl = key_2d_left.values()
        k2vr = key_2d_right.values()
        
        fx_left = camera_info_left.K[0]
        fy_left = camera_info_left.K[4]
        cx_left = camera_info_left.K[2]
        cy_left = camera_info_left.K[5]

        fx_right = camera_info_right.K[0]
        fy_right = camera_info_right.K[4]
        cx_right = camera_info_right.K[2]
        cy_right = camera_info_right.K[5]

        disparity = k2vl[0] - k2vr[0]
        if disparity == 0:
            rospy.logerr("Disparity is zero, cannot calculate depth")
            return None

        z = (fx_left * self.baseline) / disparity
        x = (k2vl[0] - cx_left) * z / fx_left
        y = (k2vl[1] - cy_left) * z / fy_left
        final = np.array([x,y,z])
        
        # Applied the transformation matrix to world frame
        final = self.get_transform(final)
        
        return dict(zip(key_2d_left.keys(),final))
    
    def display_output(self,img):
        # Display the output
        cv2.imshow("Image", img)
        cv2.waitKey(1)
        
    def scanning(self):
        self.scan = 1
        rospy.sleep(1)
        self.scan = 0

    def get_transform(self, camera_point):
        """Transform a point from the camera frame to the world frame."""
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            # Wait for the transform to be available
            transform = tf_buffer.lookup_transform("world", self.camera_frame_left, rospy.Time(0), rospy.Duration(1.0))
            # Transform the point
            world_point = do_transform_point(camera_point, transform)
            return world_point
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
    
    def move_arm_to_position(self, position_3d):
        """Move the robotic arm to the calculated 3D position."""
        pose_target = PoseStamped()
        pose_target.header.frame_id = self.base_frame
        pose_target.pose.position.x = position_3d[0]
        pose_target.pose.position.y = position_3d[1]
        pose_target.pose.position.z = position_3d[2]
        pose_target.pose.orientation.w = 1.0

        self.arm_group.set_pose_target(pose_target)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def check_key_pressed(self, position_3d):
        """Verify if the key is pressed correctly."""
        # Check if the key's 3D position has changed
        rospy.sleep(1.0)  # Wait for potential movement
        key_new_position = self.get_key_position_3d()
        if key_new_position is None:
            return False

        movement_threshold = 0.01  # Threshold for detecting key press
        displacement = np.linalg.norm(position_3d - key_new_position)
        return displacement > movement_threshold

    def get_key_position_3d(self):
        """Placeholder for obtaining the new 3D position of the key."""
        # This can be implemented with the YOLO model and stereovision again.
        return np.array([0.0, 0.0, 0.0])  # Replace with actual calculation

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
    
    def control_flow(self):
        """
        Execute keyboard clicks by moving the robotic arm to each key's position
        
        Args:
            input_string (str): The string to be typed
        
        Returns:
            bool: Success of the entire typing operation
        """
        # Prompt user for input string
        input_string = input("Enter the string to type: ")

        while True:
            print(f"You entered: {input_string}")
            confirmation = input("Do you want to proceed with this string? (y/n): ").lower()
            if confirmation == 'y':
                break
            else:
                input_string = input("Please enter the new string to type: ")
        
        # Convert input string to keyboard clicks
        keyboard_clicks = self.string_to_keyboard_clicks(input_string)
        
        # Check if keyboard points are detected
        if not self.keyboard_points:
            rospy.logerr("No keyboard points detected. Cannot proceed with typing.")
            return False
        
        try:
            # Move to home position before starting
            home_pose = self.arm_group.get_current_pose().pose
            
            self.arm_group.set_pose_target(home_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            # Iterate through keyboard clicks
            for click in keyboard_clicks:
                # Check if the click exists in keyboard points
                if click not in self.keyboard_points:
                    rospy.logwarn(f"Key {click} not found in keyboard points. Skipping.")
                    continue
                
                # Get the 3D position for this key
                key_position = self.keyboard_points[click]
                
                # Move arm to the key position
                success = self.move_arm_to_position(key_position)
                if not success:
                    rospy.logerr(f"Failed to move to key {click}")
                    return False
                
                # Simulate key press (you might want to add actual key press mechanism)
                rospy.sleep(0.5)  # Brief pause to simulate key press
                
                # Optional: Verify key press
                if not self.check_key_pressed(key_position):
                    rospy.logwarn(f"Key press verification failed for {click}")
            
            # Return to home position after typing
            self.arm_group.set_pose_target(home_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            return True
        
        except Exception as e:
            rospy.logerr(f"Error in control flow: {e}")
            return False
    
def main():
    # Initialize the ROS node
    rospy.init_node('keyboard_typing_robot', anonymous=True)

    try:
        # Instantiate the controller
        keyboard_controller = Controller()
        print("Press Enter to start scanning the keyboard...")
        input()
        while keyboard_controller.keyboard_points is None:
            keyboard_controller.scanning()
            rospy.sleep(1)
        
        # Execute the control flow for typing the input string
        success = keyboard_controller.control_flow()

        if success:
            rospy.loginfo("Successfully typed the string on the keyboard.")
        else:
            rospy.logerr("Failed to type the string on the keyboard.")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Node interrupted.")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    

if __name__ == "__main__":
    main()
