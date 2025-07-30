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
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")


    def image_callback_right(self, msg):
        rospy.logdebug("Right image callback triggered")
        try:
            self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Error in left image callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_left(self, msg):
        rospy.logdebug("Left camera info callback triggered")
        try:
            self.camera_info_left = msg
        except Exception as e:
            rospy.logerr(f"Error in left camera info callback: {e}. Message type: {type(msg)}")

    def camera_info_callback_right(self, msg):
        rospy.logdebug("Right camera info callback triggered")
        try:
            self.camera_info_right = msg
        except Exception as e:
            rospy.logerr(f"Error in right camera info callback: {e}. Message type: {type(msg)}")
            
        
    def processing_img(self,img):
        rospy.logdebug("Scanning for keyboard")
        try:
            results = self.model(img)
            results = results.xyxy[results.xyxy[:,5] == self.class_to_detect]
            if len(result) > 0:
                result = results[results[:, 4].argmax()]
                rospy.logdebug(f"Keyboard detected at {result[0:4]}")
                return result[0:4]
            else:
                rospy.logdebug("No keyboard detected.")
                return None
            
        except Exception as e:
            rospy.logerr(f"Error scanning for keyboard: {e}. Message type: {type(e)}")
            
    def display_output_l(self):
        rospy.logdebug("Displaying left output")
        img = self.left_img.copy()
        if self.result_left is not None:
            x1, y1, x2, y2 = self.result_left
            cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(img, "Keyboard", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        self.image_pub_l.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        
        
        
    def display_output_r(self):
        rospy.logdebug("Displaying right output")
        
    def scanning_for_keyboard(self):
        while self.result_left is None or self.result_right is None:
            rospy.sleep(1)
            rospy.loginfo("Keyboard detected")
            rospy.loginfo("Starting scanning for keyboard")
            self.result_left = self.processing_img(self.left_img)
            self.result_right = self.processing_img(self.right_img)
            rospy.loginfo("Keyboard scanned")
        
    
        
            
def main():
    at = AutonomousTyping()
    rospy.loginfo("Autonomous Typing node started")
    rospy.loginfo("Waiting for camera info...")
    while not rospy.is_shutdown() and (at.camera_info_left is None or at.camera_info_right is None):
        rospy.sleep(1)
    rospy.loginfo("Camera info received. Starting main loop.")
    input(">>>")
    while not rospy.is_shutdown():
        if at.left_img is not None and at.right_img is not None:
            at.scanning_for_keyboard()
            
        rospy.sleep(0.1)
    rospy.spin()