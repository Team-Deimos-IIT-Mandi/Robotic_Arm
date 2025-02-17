
#!/usr/bin/env python3

import moveit_msgs.msg
import geometry_msgs.msg
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
from sklearn.linear_model import LinearRegression
from scipy.spatial.distance import cdist
import logging
import moveit_commander 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import sys
# Camera parameters
color_info = {
    'K': [1297.672904, 0.0, 620.914026, 0.0, 1298.631344, 238.280325, 0.0, 0.0, 1.0]
}

depth_info = {
    'K': [360.01333, 0.0, 243.87228, 0.0, 360.013366699, 137.9218444, 0.0, 0.0, 1.0]
}

class AutonomousTyping:
    def __init__(self):
        rospy.init_node('autonomous_typing', anonymous=True)
        
        self.model = YOLO('trained_yolov8n.pt')
        rospy.loginfo("YOLOv8n is loaded")
        rospy.sleep(1)
        os.environ['YOLO_LOG_LEVEL'] = 'error'
        logging.getLogger("ultralytics").setLevel(logging.ERROR)
        
        self.camera_view = rospy.get_param('~camera_view', '/camera/color/image_raw')
        self.camera_depth = rospy.get_param('~camera_depth', '/camera/depth/image_raw')
        self.camera_info = rospy.get_param('~camera_info', '/camera/color/camera_info')
        
        self.camera_view_sub = rospy.Subscriber(self.camera_view, Image, self.image_callback)
        self.camera_depth_sub = rospy.Subscriber(self.camera_depth, Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info, CameraInfo, self.camera_info_callback)
        
        self.camera_view_pub = rospy.Publisher('yolo', Image, queue_size=10)
        
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        
        self.img = None
        self.depth_img = None
        self.result = None
        self.points = None
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        
    def image_callback(self, msg):
        try: 
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.img = img
            # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # img = cv2.resize(img, (640, 640))
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
            self.camera_view_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
            self.result = results
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
            
    def depth_callback(self, msg):
        if msg.encoding == "16UC1":
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "16UC1").astype(np.float32) * 0.001  # Convert mm to meters
        elif msg.encoding == "32FC1":
            self.depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        # cv2.imshow('Depth Image', self.depth_img)
        # cv2.waitKey(1)
        
    def camera_info_callback(self, msg):
        self.camera_info = msg
    
    def get_points(self, results):
        points = {key: None for key in self.class_names}  # Initialize all keys with None
        confidences = {key: 0 for key in self.class_names}  # Track max confidence per key
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                class_id = int(box.cls[0])
                label = self.class_names[class_id]
                confidence = box.conf[0]
                
                if confidence > confidences[label]:
                    confidences[label] = confidence
                    x = 270/720
                    y = 480/1280
                    depth = self.depth_img[int(y*(y1+y2)/2), int(x*(x1+x2)/2)]
                    X,Y,Z = (int((x1+x2)/2), int((y1+y2)/2),depth)
                    if depth>0:
                        x, y, z = self.compute_3d_coordinates(X, Y, Z)
                        world_coords = self.transform_to_world(x, y, z)
                        points[label] = world_coords
                    
                        
                    
        
        return points

    def transform_to_world(self, x, y, z, camera_frame="camera_depth_frame"):
        """Transform the 3D point from the camera frame to the world frame using TF2."""
        try:
            transform = self.tf_buffer.lookup_transform("world", camera_frame, rospy.Time(0), rospy.Duration(1.0))
            point = PointStamped()
            point.header.frame_id = camera_frame
            point.point.x, point.point.y, point.point.z = x, y, z
            transformed_point = do_transform_point(point, transform)
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        except tf2_ros.LookupException as e:
            rospy.logerr(f"TF Lookup failed: {e}")
            return None
        
    def compute_3d_coordinates(self, u, v, depth):
        """Compute 3D world coordinates from pixel coordinates using camera intrinsics."""
        K = np.array(depth_info["K"]).reshape(3, 3)
        uv1 = np.array([u, v, 1.0])
        xyz = np.linalg.inv(K) @ uv1 * depth
        return xyz[0], xyz[1], xyz[2]


    
        
def main():
    autonomous_typing = AutonomousTyping()
    while not rospy.is_shutdown():
        if autonomous_typing.result is not None:
            points = autonomous_typing.get_points(autonomous_typing.result)
            for pt in points:
                print(f"{pt}: {points[pt]}")
        rospy.sleep(1)
    #     rospy.loginfo("hhahaha")
    # #     if autonomous_typing.img is not None and autonomous_typing.depth_img is not None and autonomous_typing.result is not None:
    # #         points = autonomous_typing.get_points(autonomous_typing.result)
    # #         autonomous_typing.points = points
    #     rospy.sleep(0.1)
    rospy.spin()
    
if __name__=="__main__":
    main()