import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import json
import os
from geometry_msgs.msg import PointStamped

class Controller:
    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            rospy.init_node("autonomous_typing", log_level=rospy.DEBUG)
            logging.basicConfig(level=logging.DEBUG)
        else:
            rospy.init_node("autonomous_typing", log_level=rospy.INFO)
            logging.basicConfig(level=logging.INFO)
        
        rospy.loginfo("Initializing self")
        # Initialize variables
        self.scan = 0
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'keyboard_layout.json')
        with open(json_path, 'r') as f:
            rospy.loginfo(f"Loading keyboard points from {json_path}")
            self.keyboard_points = json.load(f)
            # print(self.keyboard_points['E'])
        
        self.class_to_detect = 66
        # Parameters
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

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
        self.result_left = None
        self.result_right = None
        self.keypoints_left = None
        self.keypoints_right = None
        self.keyboard_points_3d = None
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.baseline = 0.1
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("body")
        rospy.loginfo("self initialized")

    def image_callback_left(self, msg):
        rospy.logdebug("Left image callback triggered")
        try:
            self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_left, corners = self.processing_image(self.left_img)
            self.display_output_l(self.left_img, corners)
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}")

    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.keypoints_right, corners = self.processing_image(self.right_img)
            self.display_output_r(self.right_img, corners)
        except Exception as e:
            rospy.logerr(f"Error in right image callback: {e}")
            
    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        self.camera_info_left = msg

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        self.camera_info_right = msg
    
    def display_output_l(self, img, corners):
        rospy.logdebug("Displaying left output")
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        # draw the keypoints
        for keypoint in self.keypoints_left:
            cv2.circle(img, (int(self.keypoints_left[keypoint][0]), int(self.keypoints_left[keypoint][1])), 5, (0, 0, 255), -1)
            
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        
        self.annotated_image_pub_l.publish(annotated_image_msg)
        rospy.logdebug("Published annotated left image")

    def display_output_r(self, img, corners):
        rospy.logdebug("Displaying right output")
        for i in range(4):
            cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        for keypoint in self.keypoints_right:
            cv2.circle(img, (int(self.keypoints_right[keypoint][0]), int(self.keypoints_right[keypoint][1])), 5, (0, 0, 255), -1)
            
        annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.annotated_image_pub_r.publish(annotated_image_msg)
        rospy.logdebug("Published annotated right image")
            
    def processing_image(self, img):
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
                            # print(f"Scaled point for {key}: {scaled_points[key]}")
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

        :param keypoints_left: Dictionary of keypoints from the left camera image (labels: positions).
        :param keypoints_right: Dictionary of keypoints from the right camera image (labels: positions).
        :param camera_info_left: CameraInfo message for the left camera.
        :param camera_info_right: CameraInfo message for the right camera.
        :param baseline: The baseline distance between the left and right cameras (in meters).
        :return: A dictionary of keypoints with their 3D positions (labels: [X, Y, Z]).
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
        """Transform a point from the camera frame to the world frame."""
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        try:
            camera_point = PointStamped()
            camera_point.header.frame_id = self.camera_frame_left
            camera_point.point.x = point[2]
            camera_point.point.y = -point[0]
            camera_point.point.z = -point[1]
            # Wait for the transform to be available
            transform = tf_buffer.lookup_transform("world", self.camera_frame_left, rospy.Time(0), rospy.Duration(1.0))
            # Transform the point
            world_point = do_transform_point(camera_point, transform)
            return [world_point.point.x,world_point.point.y,world_point.point.z]
        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup failed: {e}")
            return None
        
    
    # Modify the control flow to use the new method
    # def get_keyboard_points(self):
    #     """Get keyboard points using PnP on both cameras"""
    #     if self.keypoints_left and self.camera_info_left:
    #         left_points = self.calculate_3d_position(
    #             self.keypoints_left, 
    #             self.camera_info_left
    #         )
        
    #     if self.keypoints_right and self.camera_info_right:
    #         right_points = self.calculate_3d_position(
    #             self.keypoints_right, 
    #             self.camera_info_right
    #         )
        
    #     # Merge or validate points from both cameras if needed
    #     self.keyboard_points_3d = left_points or right_points
    
    # def display_output_l(self, img, corners):
    #     # Create a box
    #     for i in range(4):
    #         cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
    #     # Convert to ROS Image message and publish
    #     annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
    #     self.annotated_image_pub_l.publish(annotated_image_msg)
    #     # rospy.loginfo("Published annotated image.")
    # def display_output_r(self, img, corners):
    #     # Create a box
    #     for i in range(4):
    #         cv2.line(img, corners[i], corners[(i+1) % 4], (0, 255, 0), 2)
        
    #     # Convert to ROS Image message and publish
    #     annotated_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
    #     self.annotated_image_pub_r.publish(annotated_image_msg)
    #     # rospy.loginfo("Published annotated image.")
        

    
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
        # self.get_key_position_3d()
        # print(self.keyboard_points_3d)
        ans = self.calc_3d_pos(self.keypoints_left, self.keypoints_right, self.camera_info_left, self.camera_info_right)
        # ans = zip(ans.keys(),self.get_transform(ans.values()))
        for key in ans:
            ans[key] = np.array(self.get_transform(ans[key]))
            # print(f"{key} : {ans[key]}")
        self.keyboard_points_3d = ans
        
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
                hover_position = key_position + np.array([0, -0.1, 0])
                
                # Move arm to the key position
                print(hover_position)
                success = self.move_arm_to_position(hover_position)
                print(success)
                
                # success = self.move_arm_to_position(key_position)
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
    controller = Controller(debug=False)
    rospy.sleep(15)
    
    # self.display_keyboard()
    # while not rospy.is_shutdown():
    # ans = controller.calc_3d_pos(controller.keypoints_left, controller.keypoints_right, controller.camera_info_left, controller.camera_info_right)
    # # ans = zip(ans.keys(),controller.get_transform(ans.values()))
    # for key in ans:
    #     print(f"3D position of {key}: {controller.get_transform(ans[key])}")
    controller.control_flow()
    
    
    rospy.spin()

if __name__ == "__main__":
    main()
