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
depth_image = None
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
    
def depth_image_callback(msg):
    """Callback for the depth image."""
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    rospy.loginfo(f"Depth image shape: {depth_image.shape}")


def right_image_callback(msg):
    """Callback for the right camera."""
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def detect_red_object(image):
    """Detect a red object in the given image."""
    if image is None:
        rospy.loginfo("Image is None")
        return None

    rospy.loginfo("detect_red_object started")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
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
        if w > 20 and h > 20:  # Filter noise
            rospy.loginfo(f"Red object detected: x={x}, y={y}, w={w}, h={h}")
            return x, y, w, h
    return None

def compute_depth(x_left, x_right, focal_length, baseline):
    """Compute the depth (Z) using disparity."""
    disparity = x_left - x_right
    if disparity > 0:
        rospy.loginfo(f"depth: {(focal_length * baseline) / disparity}")
        return (focal_length * baseline) / disparity
    return None

def get_depth_from_depth_camera(depth_image, x, y):
    """
    Get the depth (Z) value from the depth image at the specified pixel coordinates (x, y).

    Args:
        depth_image (numpy.ndarray): The depth image from the depth camera.
        x (int): The x-coordinate of the pixel.
        y (int): The y-coordinate of the pixel.

    Returns:
        float: The depth value in meters, or None if the coordinates are invalid or no depth is available.
    """
    if depth_image is None:
        rospy.logwarn("Depth image is None.")
        return None

    # Check if the coordinates are within the image bounds
    height, width = depth_image.shape
    if x < 0 or x >= width or y < 0 or y >= height:
        rospy.logwarn(f"Invalid coordinates: x={x}, y={y}. Image dimensions are {width}x{height}.")
        return None

    # Get the depth value at the specified pixel
    depth_value = depth_image[y, x]

    # Check if the depth value is valid (non-zero and finite)
    if depth_value > 0 and np.isfinite(depth_value):
        return depth_value
    else:
        rospy.logwarn(f"Invalid depth value at x={x}, y={y}: {depth_value}")
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

    if left_result and right_result:
        x_left, y_left, w_left, h_left = left_result
        x_right, y_right, w_right, h_right = right_result
        
        object_center_x = x_left + w_left // 2
        object_center_y = y_left + h_left // 2
        # depth = get_depth_from_depth_camera(depth_image, object_center_x, object_center_y)

        depth = compute_depth(x_left, x_right, focal_length, baseline)
        
        if depth:
            X_camera, Y_camera, Z_camera = compute_coordinates(
                x_left + w_left // 2, y_left + h_left // 2, depth, cx, cy, focal_length
            )
            
            # Convert image frame to camera frame
            X_ros = Z_camera
            Y_ros = -X_camera
            Z_ros = -Y_camera

            camera_point = PointStamped()
            camera_point.header.frame_id = camera_frame
            camera_point.point.x = X_ros
            camera_point.point.y = Y_ros
            camera_point.point.z = Z_ros

            world_point = transform_point(camera_point, camera_frame)
            if world_point:
                return world_point.point.x, world_point.point.y, world_point.point.z

            return camera_point.point.x, camera_point.point.y, camera_point.point.z
    else:
        rospy.loginfo("No results from left or right image detection.")
    return None

def main():
    """Main function to initialize the node and process stereo vision."""
  
    rospy.init_node("stereo_vision_red_object_detection")

    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)
    # rospy.Subscriber("/depth_camera/image_raw", Image, depth_image_callback)

    cx = 400  # Principal point x-coordinate
    cy = 400  # Principal point y-coordinate
    focal_length = 450  # Focal length in pixels
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
