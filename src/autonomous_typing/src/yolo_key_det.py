
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import json
import os

# current_dir = os.path.dirname(__file__)

class Detector:
    def __init__(self):
        self.model = YOLO('trained_yolov8n.pt')
        self.camera_topic = rospy.get_param('~camera_topic', '/camera_gripper_left/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera_gripper_left/camera_info')
        # self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        self.publisher = rospy.Publisher('yolo_output', Image, queue_size=10)
        self.class_names = [
            'caret', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'ss', 'accent', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', 'ue', 'plus', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', 'oe', 
            'ae', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '-', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]

        
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
        self.dist_coeffs = np.array(msg.D)
        
        
    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
        
def main():
    rospy.init_node('yolo_detector', anonymous=True)
    detector = Detector()
    rospy.spin()
    
if __name__ == '__main__':
    main()
        
        
