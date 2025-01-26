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
        self.input_topic = rospy.get_param('~input_topic', '/camera_base_right/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera_base_right/camera_info')
        self.output_topic = rospy.get_param('~output_topic', '/camera_base_right/processed_image')
        self.class_to_detect = rospy.get_param('~class_to_detect', 66)
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link_base_right')
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
        with open('/home/ub20/Robotic_Arm/src/autonomous_typing/src/keyboard_layout.json', 'r') as f:
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
                            projected_points = projected_points.reshape(-1, 3)
                            depth = float(tvec[2] + model_points[i, 2])  # Z component
                            # X_ros = Z_camera
                            # Y_ros = -X_camera
                            # Z_ros = -Y_camera

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




if __name__ == '__main__':
    try:
        node = YoloPixelSegmentationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
