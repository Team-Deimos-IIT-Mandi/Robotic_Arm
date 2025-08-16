#!/usr/bin/env python3

import rospy
import rospkg
import os
import cv2
import numpy as np
import json
import threading
import queue
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from keyboard_detector.msg import KeyboardDetection, KeyboardDetectionArray

class KeyboardDetectorNode:
    def __init__(self):
        rospy.init_node('keyboard_detector_node', anonymous=True)

        # Paths and Model
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('keyboard_detector')
        model_path = rospy.get_param('~model_path',
                os.path.join(self.package_path, 'models', 'best.pt'))

        if not os.path.exists(model_path):
            rospy.logerr(f"Model not found at: {model_path}")
            rospy.signal_shutdown("Model not found")
            return

        try:
            self.model = YOLO(model_path)
            self.model.to('cuda')
            rospy.loginfo(f"Loaded YOLO from: {model_path}")
            rospy.loginfo(f"Model classes: {self.model.names}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO: {e}")
            rospy.signal_shutdown("Model loading failed")
            return

        # Parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)

        self.bridge = CvBridge()
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        # Use queue=1 for always-latest; does not block ROS callback
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

        self.detection_pub = rospy.Publisher('/keyboard_detections', KeyboardDetectionArray, queue_size=1)
        self.json_pub = rospy.Publisher('/keyboard_detections/json', String, queue_size=1)
        self.annotated_pub = rospy.Publisher('/keyboard_detections/annotated_image', Image, queue_size=1)

        # Frame queue and processing thread
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        self.worker_thread = threading.Thread(target=self.worker_loop)
        self.worker_thread.daemon = True
        self.worker_thread.start()

        rospy.loginfo("Keyboard Detector Node initialized and ready!")

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Super-low-latency: shrink to speed up further if needed!
            cv_image = cv2.resize(cv_image, (1920, 1080))  # CHANGE HERE if desired
            # Only keep the latest, drop previous if behind
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                pass
            self.frame_queue.put_nowait((cv_image, data.header))
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def worker_loop(self):
        while not rospy.is_shutdown() and self.running:
            try:
                cv_image, header = self.frame_queue.get(timeout=1)
            except queue.Empty:
                continue
            t0 = time.time()
            results = self.model(
                cv_image,
                device='cuda',
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                verbose=False
            )
            rospy.logdebug(f"YOLO inference took {time.time()-t0:.3f} seconds")
            
            # Fix: Handle results as list and pass individual result
            if len(results) > 0:
                detections = self.process_detections(results[0], header)
                self.publish_detections(detections, header)
                annotated_image = self.create_annotated_image(cv_image, results[0])
                self.publish_annotated_image_msg(annotated_image, header)

    def process_detections(self, result, header):
        detection_array = KeyboardDetectionArray()
        detection_array.header = header
        
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            detected_keys = []
            
            for box, conf, cls in zip(boxes, confidences, classes):
                det = KeyboardDetection()
                det.class_name = self.model.names[cls]
                det.class_id = int(cls)
                det.confidence = float(conf)
                # Fix: Correct box indexing
                det.x1 = float(box[0])
                det.y1 = float(box[1])
                det.x2 = float(box[2])
                det.y2 = float(box[3])
                detection_array.detections.append(det)
                detected_keys.append(self.model.names[cls])
                
            if detected_keys:
                rospy.loginfo_throttle(2, f"Detected keys: {detected_keys}")
                
        return detection_array

    def publish_detections(self, detections, header):
        self.detection_pub.publish(detections)
        # Also as JSON:
        json_data = {
            'header': {'stamp': {'secs': header.stamp.secs, 'nsecs': header.stamp.nsecs},
                       'frame_id': header.frame_id},
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
        annotated_img = image.copy()
        
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            
            for box, conf, cls in zip(boxes, confidences, classes):
                # Fix: Correct box indexing for rectangle drawing
                cv2.rectangle(annotated_img,
                              (int(box[0]), int(box[1])),
                              (int(box[2]), int(box[3])),
                              (0, 255, 0), 2)
                label = f'{self.model.names[cls]}: {conf:.2f}'
                # Fix: Correct text positioning
                cv2.putText(annotated_img, label,
                            (int(box[0]), int(box[1]) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return annotated_img

    def publish_annotated_image_msg(self, annotated_image, header):
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error publishing annotated image: {e}")

    def shutdown(self):
        self.running = False
        # self.worker_thread.join()  # Only if you block on exit

if __name__ == '__main__':
    try:
        detector = KeyboardDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        detector.shutdown()
        rospy.loginfo("Keyboard detector node shutting down")