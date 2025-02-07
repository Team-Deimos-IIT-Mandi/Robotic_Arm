
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import os
from tf2_geometry_msgs import do_transform_point
import tf2_geometry_msgs
import tf2_ros


class Detector:
    def __init__(self):
        self.model = YOLO('trained_yolov8n.pt')
        self.camera_input_left = rospy.get_param('~input_topic_left', '/camera_gripper_left/image_raw')
        self.camera_info_left = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        self.camera_input_right = rospy.get_param('~input_topic_right', '/camera_gripper_right/image_raw')
        self.camera_info_right = rospy.get_param('~camera_info_topic', '/camera_gripper_right/camera_info')
        
        self.camera_frame_left = rospy.get_param('~camera_frame_left', 'camera_link_gripper_left')
        self.camera_frame_right = rospy.get_param('~camera_frame_right', 'camera_link_gripper_right')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')

        self.subscriber_left = rospy.Subscriber(self.camera_input_left, Image, self.image_callback_left)
        self.subscriber_right = rospy.Subscriber(self.camera_input_right, Image, self.image_callback_right)
        self.camera_info_sub_left = rospy.Subscriber(self.camera_info_left, CameraInfo, self.camera_info_callback_left)
        self.camera_info_sub_right = rospy.Subscriber(self.camera_info_right, CameraInfo, self.camera_info_callback_right)
        
        # self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.bridge = CvBridge()
        # self.subscriber = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        # self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        self.publisher = rospy.Publisher('yolo_output', Image, queue_size=10)
        self.publisher_depth = rospy.Publisher('stereo', Image, queue_size=10)
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.img_left = None
        self.img_right = None
        self.dist_coeffs_right = None
        self.dist_coeffs_left = None
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        
    def camera_info_callback_left(self, msg):
        try:
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_left = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Dum as fk cam_info_left : {e}")
        
    def camera_info_callback_right(self, msg):
        try:
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_right = np.array(msg.D)
        except Exception as e:
            rospy.logerr(f"Dum as fk cam_info_right : {e}")
        
    def image_callback_left(self,msg):
        try:
            self.img_left = self.image_callback(msg)
        except Exception as e:
            rospy.logerr(f"Dum as fk img_left : {e}")
        
    def image_callback_right(self,msg):
        try:
            self.img_right = self.image_callback(msg)
        except Exception as e:
            rospy.logerr(f"Dum as fk img_right : {e}")
        
    def get_tf(self):
        transform = self.tf_buffer.lookup_transform(self.camera_frame_right, self.camera_frame_left, rospy.Time(0), rospy.Duration(3.0))
        if transform is None:
            return None, None
        # Extract translation and rotation from the transform
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        T = np.array([translation.x, translation.y, translation.z])
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        R = tf2_ros.transformations.quaternion_matrix(quaternion)[:3, :3]      
        return R,T
    
    def stereo(self):
        # Extract left and right images
        if self.img_left is None or self.img_right is None:
            rospy.logerr("Images not received")
            return
        
        imgL = cv2.cvtColor(self.img_left, cv2.COLOR_BGR2GRAY)
        imgL = cv2.undistort(imgL, self.camera_matrix_left, self.dist_coeffs_left)
        img_sizeL = (imgL.shape[1], imgL.shape[0])
        
        imgR = cv2.cvtColor(self.img_right, cv2.COLOR_BGR2GRAY)
        imgR = cv2.undistort(imgR, self.camera_matrix_right, self.dist_coeffs_right)
        img_sizeR = (imgR.shape[1], imgR.shape[0])
        
        if img_sizeL != img_sizeR:
            rospy.logerr("Image sizes do not match")
            return
        
        R,T = self.get_tf()
        if R is None or T is None:
            rospy.logerr("Failed to get transform")
            return 
        
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(self.camera_matrix_left, self.dist_coeffs_left,
                                                    self.camera_matrix_right, self.dist_coeffs_right,
                                                    img_sizeL, R, T)
        
        # Rectification maps
        map1x, map1y = cv2.initUndistortRectifyMap(self.camera_matrix_left, self.dist_coeffs_left, R1, P1, img_sizeL, cv2.CV_32FC1)
        map2x, map2y = cv2.initUndistortRectifyMap(self.camera_matrix_right, self.dist_coeffs_right, R2, P2, img_sizeR, cv2.CV_32FC1)

        rectified_left = cv2.remap(imgL, map1x, map1y, cv2.INTER_LINEAR)
        rectified_right = cv2.remap(imgR, map2x, map2y, cv2.INTER_LINEAR)

        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        
        disparity = stereo.compute(rectified_left, rectified_right)
        
        disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        disparity = np.uint8(disparity)
        
        # Publish the disparity image
        self.publisher_depth.publish(self.bridge.cv2_to_imgmsg(disparity, encoding='mono8'))
        
        
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img_copy = img.copy()
        img = cv2.resize(img, (640, 640))  # Adjust size as needed
        results = self.model(img,iou=0.7, conf=0.5)
        results = results[0]
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = self.class_names[class_id]
                confidence = box.conf[0]

                # Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw label and confidence
                # cv2.putText(img, f'{label} {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(img, f'{label}', (int((x1+x2)/2),int((y1+y2)/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                # Publish the image with bounding boxes
        self.publisher.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        return img_copy
def main():
    rospy.init_node('yolo_detector', anonymous=True)
    detector = Detector()
    
    while not rospy.is_shutdown():
        if detector.img_left is None or detector.img_right is None:
            rospy.loginfo("Waiting for images...")
            rospy.sleep(0.1)
            continue
        
        rospy.loginfo("Images received")
        detector.stereo()
        rospy.sleep(0.1)
        
    rospy.spin()
    
if __name__ == '__main__':
    main()
        
        
