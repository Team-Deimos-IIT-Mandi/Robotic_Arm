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


class Controller:
    
    def __init__(self):
        
        # Params
        self.KEYBOARD_LENGTH = 354.076
        self.KEYBOARD_WIDTH = 123.444
        self.class_to_detect = 66
        self.camera_input_left = rospy.get_param('~input_topic', '/camera_base_left/image_raw')
        self.camera_input_right = rospy.get_param('~input_topic', '/camera_base_right/image_raw')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link_base_right')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link_base_left')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        
        self.model = YOLO('yolov8n-seg.pt')
        
        self.bridge = CvBridge()
        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_right)
        
        self.left_img = None
        self.right_img = None
        
    def image_callback_left(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 640))
            self.left_img = cv_image
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")
            
    def image_callback_right(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 640))
            self.right_img = cv_image
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")