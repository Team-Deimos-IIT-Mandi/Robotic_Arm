#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
import message_filters
import cv2

class Detector:
    def __init__(self):
        rospy.init_node('detector', anonymous=True)
        
        # Initialize YOLO model
        self.model = YOLO('trained_yolov8n.pt')
        
        # Camera intrinsics matrix
        self.K = np.array([
            [360.01333, 0.0, 243.87228],
            [0.0, 360.013366699, 137.9218444],
            [0.0, 0.0, 1.0]
        ])
        self.K_inv = np.linalg.inv(self.K)
        
        # TF buffer (lazy initialization for listener)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = None  # Delayed initialization

        # Bridge for image conversion
        self.bridge = CvBridge()

        # Publisher
        self.poses_pub = rospy.Publisher("/keyboard_key_poses", PoseArray, queue_size=1)

        # Subscribers (message filter for synchronized callbacks)
        color_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        depth_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.image_callback)

        # Class labels
        self.class_names = [
            'accent', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', 'minus', 'plus', 'del', 'tab', 'q', 'w', 'e', 'r', 't', 
            'y', 'u', 'i', 'o', 'p', '[', ']', 'enter', 'caps', 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ':', 
            '"', '\\', 'shift-left', 'less', 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', 'shift-right', 
            'ctrl-left', 'alt-left', 'space', 'alt-right', 'ctrl-right', 'keyboard'
        ]
        self.num_keys = len(self.class_names)

    def image_callback(self, color_msg, depth_msg):
        """Callback for synchronized color and depth images."""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            if depth_msg.encoding == "16UC1":
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1").astype(np.float32) * 0.001
            else:  # "32FC1"
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            
            # Resize depth to match color image
            self.depth_image = cv2.resize(self.depth_image, (self.color_image.shape[1], self.color_image.shape[0]))

            # Process the frame
            self.process_frame()
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")

    def compute_3d_point(self, u, v, depth):
        """Convert pixel coordinates and depth to 3D point."""
        if depth <= 0:
            return None
        uv1 = np.array([u, v, 1.0])
        return self.K_inv @ uv1 * depth

    def transform_point(self, point, source_frame="camera_depth_frame", target_frame="world"):
        """Transform point between coordinate frames."""
        if self.tf_listener is None:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.point.x, point_stamped.point.y, point_stamped.point.z = point
            transformed = do_transform_point(point_stamped, transform)
            return transformed.point.x, transformed.point.y, transformed.point.z
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF transform error: {e}")
            return None

    def process_frame(self):
        """Process the current frame and publish results."""
        try:
            pose_array = PoseArray()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "world"

            poses = [Pose() for _ in range(self.num_keys)]
            confidences = [-1] * self.num_keys  # Use -1 to allow low-confidence detections
            
            # Run YOLO detection
            results = self.model(self.color_image, iou=0.7, conf=0.5)[0]

            for result in results.boxes:
                class_id = int(result.cls[0])
                confidence = float(result.conf[0])
                
                if confidence <= confidences[class_id]:
                    continue  # Keep the highest confidence detection for each class
                    
                confidences[class_id] = confidence
                x1, y1, x2, y2 = map(int, result.xyxy[0])
                center_x, center_y = (x1 + x2) / 2, (y1 + y2) / 2
                
                # Get depth and compute 3D point
                depth = self.depth_image[int(center_y), int(center_x)]
                point_3d = self.compute_3d_point(center_x, center_y, depth)
                
                if point_3d is not None:
                    world_point = self.transform_point(point_3d)
                    if world_point is not None:
                        poses[class_id].position.x, poses[class_id].position.y, poses[class_id].position.z = world_point
                        poses[class_id].orientation.w = 1.0  # Default orientation

            pose_array.poses = poses
            self.poses_pub.publish(pose_array)

        except Exception as e:
            rospy.logerr(f"Frame processing error: {e}")

    def run(self):
        """ROS spin loop."""
        rospy.spin()

if __name__ == '__main__':
    try:
        Detector().run()
    except rospy.ROSInterruptException:
        pass
