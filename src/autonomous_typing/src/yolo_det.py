import rospy
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
from geometry_msgs.msg import Pose
import logging
# Suppress YOLOv8 logs
os.environ['YOLO_LOG_LEVEL'] = 'error'  # Set the YOLO logging level
logging.getLogger("ultralytics").setLevel(logging.ERROR)


class Controller:
    def __init__(self):
        # Initialize the keyboard_points_dict to an empty dictionary
        # self.keyboard_points_dict = {}
        # Initialize scan to a default value (e.g., 0 or False)
        self.scan = 0
        # Parameters
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            rospy.loginfo(f"Loading keyboard points from {json_path}")
            self.keyboard_points = json.load(f)
            
            
        self.class_to_detect = 66
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
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
        
        # Publishers
        self.annotated_image_pub_l = rospy.Publisher("/annotated_image_l", Image, queue_size=10)
        self.annotated_image_pub_r = rospy.Publisher("/annotated_image_r", Image, queue_size=10)
        
        # Images from cameras
        self.left_img = None
        self.right_img = None
        self.camera_info_left = None
        self.camera_info_right = None
        
        # Results from YOLO model
        self.result_left = None
        self.result_right = None
        
        # Keypoints
        self.keypoints_left = None
        self.keypoints_right = None
        
        # Keyboard points
        self.keyboard_points_3d = None

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
        self.arm_group = MoveGroupCommander("body")

    def image_callback_left(self, msg):
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_left,corners = self.processing_image(self.left_img)
            
            # if self.keyboard_points_3d is None:
            #     self.keyboard_points_3d = self.calculate_3d_position(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
            #     print(self.keyboard_points_3d)
            # self.keyboard_corners,depth = self.calculate_depth()
            # self.keyboard_points = ke
            self.display_output_l(self.left_img, corners)
            
            
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_right,corners = self.processing_image(self.right_img)
            self.display_output_r(self.right_img, corners)
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")
            
    def camera_info_callback_left(self, msg):
        self.camera_info_left = msg
    def camera_info_callback_right(self, msg):
        self.camera_info_right = msg
        
    def get_keyboard_points(self):
        print("Getting keyboard points")
        print("left",self.keypoints_left)
        print("right",self.keypoints_right)
        input(">>")
        self.keyboard_points_3d = self.calculate_3d_position(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
        
        
    def processing_image(self, img):
        # Ensure `self.keyboard_points_dict` is initialized
        # if not self.keyboard_points_dict:
        #     rospy.logerr("keyboard_points_dict is not initialized!")
        #     return None

        # Initialize `scaled_points` to avoid uninitialized reference
        scaled_points = {}

        result = self.model(img, stream=True)
        result = next(result)  # Get the first result from the stream
        for idx in range(len(result.boxes)):
            class_id = int(result.boxes.cls[idx].item())
            if class_id == self.class_to_detect:
                box = result.boxes.xyxy[idx].cpu().numpy()
                x1, y1, x2, y2 = map(int, box)
                box_length = x2 - x1
                box_width = y2 - y1
                corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
                # Compute scaled points only for detected class
                for key, value in self.keyboard_points.items():
                    scaled_x = (value[0] / self.KEYBOARD_LENGTH) * box_length + x1
                    scaled_y = (value[1] / self.KEYBOARD_WIDTH) * box_width + y1
                    scaled_points[key] = [int(scaled_x), int(scaled_y)]

        # Return `scaled_points`, even if empty
        return scaled_points,corners


    def calculate_3d_position(self, key_2d_left, key_2d_right, camera_info_left, camera_info_right):
        """Calculate the 3D position of a key using stereo vision."""
        
        k2vl = np.array(list(key_2d_left.values()))
        k2vr = np.array(list(key_2d_right.values()))
        
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
            rospy.logerr("Dis`parity is zero, cannot calculate depth")
            return None

        z = (fx_left * self.baseline) / disparity
        x = (k2vl[0] - cx_left) * z / fx_left
        y = (k2vl[1] - cy_left) * z / fy_left
        final = np.array([x,y,z])
        
        # Applied the transformation matrix to world frame
        final = self.get_transform(final)
        
        return dict(zip(key_2d_left.keys(),final))
    
    def display_output_l(self, img, corners):
        # Create a box
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
        # Convert to ROS Image message and publish
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.annotated_image_pub_l.publish(annotated_image_msg)
        # rospy.loginfo("Published annotated image.")
    def display_output_r(self, img, corners):
        # Create a box
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
        # Convert to ROS Image message and publish
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.annotated_image_pub_r.publish(annotated_image_msg)
        # rospy.loginfo("Published annotated image.")
        
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

    def string_to_keyboard_clicks(self,input_string):
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

        # while True:
        #     print(f"You entered: {input_string}")
        #     confirmation = input("Do you want to proceed with this string? (y/n): ").lower()
        #     if confirmation == 'y':
        #         break
        #     else:
        #         input_string = input("Please enter the new string to type: ")
        
        # Convert input string to keyboard clicks
        keyboard_clicks = self.string_to_keyboard_clicks(input_string)
        print(f"Keyboard clicks: {keyboard_clicks}")
        
        # Check if keyboard points are detected
        # if not self.keyboard_points_3d:
        #     print("Keyboard points not detected. Please ensure the keyboard is in view.")
        self.get_key_position_3d()
        print(self.keyboard_points_3d)
        
        try:
            # Move to home position before starting
            home_pose = self.arm_group.get_current_pose().pose
            
            self.arm_group.set_pose_target(home_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            
            # dummy_pose = Pose()
            # dummy_pose.position.x = 0.0
            # dummy_pose.position.y = -0.65
            # dummy_pose.position.z = 0.3
            # dummy_pose.orientation.w = 1.0
            # self.arm_group.set_pose_target(dummy_pose)
            # success = self.arm_group.go(wait=True)
            # input("Press Enter to continue...")
            
            # Iterate through keyboard clicks
            # for k in self.keyboard_points:
            #     print(k, self.keyboard_points[k])
            for click in keyboard_clicks:
                # Check if the click exists in keyboard points
                if click not in self.keyboard_points:
                    rospy.logwarn(f"Key {click} not found in keyboard points. Skipping.")
                    continue
                
                # Get the 3D position for this key
                key_position = self.keyboard_points_3d[click]
                
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
        # print("Press Enter to start the keyboard...")
        # input()
        rospy.sleep(7)
        # while keyboard_controller.keyboard_points is None:
        #     keyboard_controller.scanning()
        #     rospy.sleep(1)
        
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
