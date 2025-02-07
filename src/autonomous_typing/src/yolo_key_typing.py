import rospy
import logging
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
from cv_bridge import CvBridge
from tf2_geometry_msgs import do_transform_point
import tf2_geometry_msgs
import tf2_ros
import cv2
import numpy as np
from ultralytics import YOLO
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import json
import os
from geometry_msgs.msg import PointStamped, WrenchStamped

class AutonomousTyping:
    def __init__(self, debug=False):
        self.debug = debug
        if self.debug:
            rospy.init_node("autonomous_typing", log_level=rospy.DEBUG)
            logging.basicConfig(level=logging.DEBUG)
        else:
            rospy.init_node("autonomous_typing", log_level=rospy.INFO)
            logging.basicConfig(level=logging.INFO)

        rospy.loginfo("Initializing AutonomousTyping")

        # Initialize variables
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
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        rospy.logdebug("Loading YOLO model")
        self.model = YOLO('trained_yolov8n.pt')
        rospy.sleep(1)
        os.environ['YOLO_LOG_LEVEL'] = 'error'
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]

        
        # Subscribers
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        self.force_sub = rospy.Subscriber('/force_torque_sensor', WrenchStamped, self.force_callback)
        
        # Publishers
        self.image_pub_l = rospy.Publisher("/output_image_l", Image, queue_size=10)
        self.image_pub_r = rospy.Publisher("/output_image_r", Image, queue_size=10)
        
        # Initialize variables
        self.left_img = None
        self.right_img = None
        self.left_scan = None
        self.right_scan = None
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
        self.stereo_matcher = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        self.force_threshold = 0.3  # Adjust based on keyboard key actuation force
        self.force_detected = False
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
            # self.display_output_l()
            self.result_left = self.detection(self.left_img)
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")


    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # self.display_output_r()
            self.result_right = self.detection(self.right_img)
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        try:
            self.camera_info_left = msg
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_left = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Error in left camera info callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        try:
            self.camera_info_right = msg
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_right = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Error in right camera info callback: {e}. Message type: {type(msg)}")
    
    def force_callback(self, msg):
        try:
            force_z = abs(msg.wrench.force.z)  # Read force in Z-direction
            if force_z > self.force_threshold:
                self.force_detected = True  # Stop movement when force exceeds threshold
        except:
            rospy.logdebug("Error in force callback : Koi baat nhi")
            
    def detection(self,img):
        results = self.model(img,iou=0.7, conf=0.5)
        results = self.corrections(results[0])
        return results
    
    def corrections(self,results):
        # Regression and Graph neural network will be applied here
        return results
            
    def move_arm_to_position(self, position_3d, tolerance=0.0005, max_step=0.005, min_step=0.0001):
        """
        Move the robotic arm to the calculated 3D position with incremental steps.
        
        Args:
            position_3d (np.array): Target 3D position 
            tolerance (float): Acceptable distance to target position
        
        Returns:
            bool: Success of movement
        """
        try:
            
            rate = rospy.Rate(1000)  # High-frequency control at 1kHz
            
            # Get current pose
            current_pose = self.arm_group.get_current_pose().pose
            
            # Compute the distance to the goal
            dx = position_3d[0] - current_pose.position.x
            dy = position_3d[1] - current_pose.position.y
            dz = position_3d[2] - current_pose.position.z
            distance = np.linalg.norm([dx, dy, dz])

            # Move in small steps until within tolerance
            # Dynamically scale step size for ultra-precise movements
            step_size = max(min_step, min(max_step, distance * 0.3))
            # step_size = 0.01
            
            while distance > tolerance and not rospy.is_shutdown():
                # Calculate step sizes proportional to direction
                step_dx = dx * min(step_size / distance, 1.0)
                step_dy = dy * min(step_size / distance, 1.0)
                step_dz = dz * min(step_size / distance, 1.0)

                # Create target pose
                target_pose = Pose()
                target_pose.position.x = current_pose.position.x + step_dx
                target_pose.position.y = current_pose.position.y + step_dy
                target_pose.position.z = current_pose.position.z + step_dz
                target_pose.orientation = current_pose.orientation

                # Set and execute target pose
                self.arm_group.set_pose_target(target_pose)
                success = self.arm_group.go(wait=True)
                self.arm_group.stop()
                
                if not success:
                    rospy.logerr("Failed to move incrementally")
                    return False

                # Update current pose and recalculate distance
                current_pose = self.arm_group.get_current_pose().pose
                dx = position_3d[0] - current_pose.position.x
                dy = position_3d[1] - current_pose.position.y
                dz = position_3d[2] - current_pose.position.z
                distance = np.linalg.norm([dx, dy, dz])

            self.arm_group.clear_pose_targets()
            return True

        except Exception as e:
            rospy.logerr(f"Error in move_arm_to_position: {e}")
            return False
    
    def get_3d_pos(self,pos_left,pos_right,camera_matrix,dist_coeffs): 
        
        disparity = self.stereo_matcher.compute(pos_left, pos_right)
        # Get the camera matrix and distortion coefficients
    
    def center_align(self):
        pass