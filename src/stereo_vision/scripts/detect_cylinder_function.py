#!/usr/bin/env python3

import tf2_ros
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import signal
import sys

# Initialize global variables
bridge = CvBridge()
left_image = None
right_image = None
stop_program = False


def signal_handler(sig, frame):
    """Handle Ctrl+C signal to exit gracefully."""
    global stop_program
    rospy.loginfo("Ctrl+C pressed. Shutting down...")
    stop_program = True
    cv2.destroyAllWindows()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def transform_point(camera_point, camera_frame="camera_link1"):
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

def left_image_callback(msg):
    """Callback for the left camera."""
    global left_image
    left_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def right_image_callback(msg):
    """Callback for the right camera."""
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def detect_red_object(image):
    """Detect a red object in the given image."""
    if image is None:
        rospy.loginfo("Image is None")
    else:
        rospy.loginfo("Image is not None")
        rospy.loginfo(f"Red object detection : {type(image)}")
        rospy.loginfo(f"Red object detection : {len(image)}")
    # rospy.loginfo(f"Red object detection : {len(image)}")
    rospy.loginfo("detect_red_object started")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    rospy.loginfo(f"Red object detection test2")
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        rospy.loginfo(f"Red object detection loop")
        if w > 20 and h > 20:  # Filter noise
            rospy.loginfo(f"Red object detected: x={x}, y={y}, w={w}, h={h}")
            return x, y, w, h
    return None

def compute_depth(x_left, x_right, focal_length, baseline):
    """Compute the depth (Z) using disparity."""
    disparity = x_left - x_right
    if disparity > 0:
        return (focal_length * baseline) / disparity
    return None

def compute_coordinates(x_left, y_left, depth, cx, cy, focal_length):
    """Compute the real-world coordinates (X, Y, Z) in the camera frame."""
    X_cam = (x_left - cx) * depth / focal_length
    Y_cam = (y_left - cy) * depth / focal_length
    Z_cam = depth
    return X_cam, Y_cam, Z_cam

def get_object_coordinates(left_image, right_image, cx, cy, focal_length, baseline, camera_frame="camera_link_gripper_left"):
    """Detect a red object and compute its real-world coordinates."""
    rospy.loginfo("get_object_coordinates started")
    
    
    left_result = detect_red_object(left_image)
    right_result = detect_red_object(right_image)
    
    rospy.loginfo(f"get_object_coordinates: {len(left_result)}, {len(right_result)}")
    

    if left_result and right_result:
        x_left, y_left, w_left, h_left = left_result
        x_right, y_right, w_right, h_right = right_result

        depth = compute_depth(x_left, x_right, focal_length, baseline)
        if depth:
            X_camera, Y_camera, Z_camera = compute_coordinates(
                x_left + w_left // 2, y_left + h_left // 2, depth, cx, cy, focal_length
            )
            X_ros = Z_camera
            Y_ros = -X_camera
            Z_ros = -Y_camera

            camera_point = PointStamped()
            camera_point.header.frame_id = camera_frame
            camera_point.point.x = X_ros
            camera_point.point.y = Y_ros
            camera_point.point.z = Z_ros
            
            rospy.loginfo("get_object_coordinates")

            world_point = transform_point(camera_point, camera_frame)
            if world_point:
                return world_point.point.x, world_point.point.y, world_point.point.z
            
            return camera_point.point.x, camera_point.point.y, camera_point.point.z
    else:
        rospy.loginfo("no left and right result")
    return None

def main():
    rospy.init_node("stereo_vision_red_object_detection")

    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)

    cx = 400  # Principal point x-coordinate
    cy = 400  # Principal point y-coordinate
    focal_length = 500  # Focal length in pixels
    baseline = 0.16  # Distance between cameras in meters

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not stop_program:
        if left_image is not None and right_image is not None:
            result = get_object_coordinates(left_image, right_image, cx, cy, focal_length, baseline)
            if result:
                x, y, z = result
                rospy.loginfo(f"Object in world frame: x: {x:.2f}, y: {y:.2f}, z: {z:.2f}")
            else:
                rospy.loginfo("Object detection or transformation failed.")
        else:
            rospy.loginfo("Waiting for images...")

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
