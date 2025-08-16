#!/usr/bin/env python3

import rospy
import rospkg
import os
import cv2
import numpy as np
import json
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Header
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

# Import custom messages
from keyboard_detector.msg import KeyboardDetection, KeyboardDetectionArray

class KeyboardDetectorNode:
    def __init__(self):
        rospy.init_node('keyboard_detector_node', anonymous=True)
        
        # Get package path for model loading
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('keyboard_detector')
        
        # Model path
        model_path = rospy.get_param('~model_path', 
                                   os.path.join(self.package_path, 'models', 'best.pt'))
        
        if not os.path.exists(model_path):
            rospy.logerr(f"Model not found at: {model_path}")
            rospy.signal_shutdown("Model not found")
            return
        
        # Initialize YOLO model
        try:
            self.model = YOLO(model_path)
            rospy.loginfo(f"Successfully loaded YOLO model from: {model_path}")
            rospy.loginfo(f"Model classes: {self.model.names}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {e}")
            rospy.signal_shutdown("Model loading failed")
            return
        
        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        
        # Publishers
        self.detection_pub = rospy.Publisher('/keyboard_detections', KeyboardDetectionArray, queue_size=10)
        self.json_pub = rospy.Publisher('/keyboard_detections/json', String, queue_size=10)
        self.annotated_pub = rospy.Publisher('/keyboard_detections/annotated_image', Image, queue_size=10)
        
        rospy.loginfo("Keyboard Detector Node initialized and ready!")
    
    def image_callback(self, data):
        """Handle raw image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image(cv_image, data.header)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def process_image(self, cv_image, header):
        """Main image processing function"""
        try:
            # Run YOLO inference
            results = self.model(cv_image, 
                               conf=self.confidence_threshold, 
                               iou=self.iou_threshold, 
                               verbose=False)
            
            # Process and publish detections
            detections = self.process_detections(results[0], header)
            self.publish_detections(detections, header)
            
            # Create and publish annotated image
            annotated_image = self.create_annotated_image(cv_image, results)
            self.publish_annotated_image_msg(annotated_image, header)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")
    
    def process_detections(self, result, header):
        """Convert YOLO results to ROS messages"""
        detection_array = KeyboardDetectionArray()
        detection_array.header = header
        
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            
            detected_keys = []
            for box, conf, cls in zip(boxes, confidences, classes):
                detection = KeyboardDetection()
                detection.class_name = self.model.names[cls]
                detection.class_id = int(cls)
                detection.confidence = float(conf)
                detection.x1 = float(box)
                detection.y1 = float(box[1])
                detection.x2 = float(box[2])
                detection.y2 = float(box[3])
                
                detection_array.detections.append(detection)
                detected_keys.append(self.model.names[cls])
            
            if detected_keys:
                rospy.loginfo(f"Detected keys: {detected_keys}")
        
        return detection_array
    
    def publish_detections(self, detections, header):
        """Publish detection results"""
        self.detection_pub.publish(detections)
        
        # Also publish as JSON
        json_data = {
            'header': {
                'stamp': {'secs': header.stamp.secs, 'nsecs': header.stamp.nsecs},
                'frame_id': header.frame_id
            },
            'detections': []
        }
        
        for det in detections.detections:
            json_data['detections'].append({
                'class_name': det.class_name,
                'class_id': det.class_id,
                'confidence': det.confidence,
                'bbox': [det.x1, det.y1, det.x2, det.y2]
            })
        
        json_msg = String()
        json_msg.data = json.dumps(json_data)
        self.json_pub.publish(json_msg)
    
    def create_annotated_image(self, image, result):
        """Create annotated image with bounding boxes and labels"""
        annotated_img = image.copy()
        
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            
            for box, conf, cls in zip(boxes, confidences, classes):
                # Draw bounding box
                cv2.rectangle(annotated_img,
                            (int(box[0]), int(box[1])),
                            (int(box[2]), int(box[3])),
                            (0, 255, 0), 2)
                
                # Draw label
                label = f'{self.model.names[cls]}: {conf:.2f}'
                cv2.putText(annotated_img, label,
                          (int(box[0]), int(box[1]) - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return annotated_img
    
    def publish_annotated_image_msg(self, annotated_image, header):
        """Publish the annotated image"""
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error publishing annotated image: {e}")

if __name__ == '__main__':
    try:
        detector = KeyboardDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard detector node shutting down")
