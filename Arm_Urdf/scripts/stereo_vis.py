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
from std_msgs.msg import Float32,Float64MultiArray

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

# def transform_point(camera_point, camera_frame="camera_link1"):
#     # Initialize TF2 buffer and listener
#     tf_buffer = tf2_ros.Buffer()
#     listener = tf2_ros.TransformListener(tf_buffer)

#     try:
#         # Wait for the transform to be available
#         transform = tf_buffer.lookup_transform("world", camera_frame, rospy.Time(0), rospy.Duration(1.0))

#         # Transform the point
#         world_point = do_transform_point(camera_point, transform)
#         return world_point

#     except tf2_ros.LookupException as e:
#         rospy.logerr(f"Transform lookup failed: {e}")
#         return None

def left_image_callback(msg):
    """Callback for the left camera."""
    global left_image
    # rospy.loginfo("left_image_callback")
    left_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def right_image_callback(msg):
    """Callback for the right camera."""
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def detect_red_object(image):
    """Detect a red object in the given image."""
    # Convert to HSV color space
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define red color range
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for both red ranges
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        # Calculate the bounding rectangle for each contour
        x, y, w, h = cv2.boundingRect(contour)

        # Set a minimum size to filter noise
        if w > 20 and h > 20:  # Adjust these thresholds based on your needs
            return x, y, w, h
    return None

def compute_depth(x_left, x_right, focal_length, baseline):
    """Compute the depth (Z) using disparity."""
    disparity = x_left - x_right
    if disparity > 0:
        depth = (focal_length * baseline) / disparity
        return depth
    return None

# def compute_coordinates(x_left, y_left, depth, cx, cy, focal_length):
#     """Compute the real-world coordinates (X, Y, Z) in the world frame."""
#     # Coordinates relative to the left camera
#     X_cam = (x_left - cx) * depth / focal_length
#     Y_cam = (y_left - cy) * depth / focal_length
#     Z_cam = depth
#     return X_cam, Y_cam, Z_cam

def main():
    rospy.init_node("stereo_vision_red_object_detection")

    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)
    depth_pub=rospy.Publisher("/image_depth",Float32,queue_size=10)
    coordinates=rospy.Publisher("/detected_corners",Float64MultiArray,queue_size=10)

    focal_length = 476.70143780997665  # Focal length in pixels (adjust based on your camera)
    baseline = 0.06  # Distance between cameras in meters


    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not stop_program:
        if left_image is not None and right_image is not None:
            # Detect red object in both images
            left_result = detect_red_object(left_image)
            right_result = detect_red_object(right_image)

            if left_result and right_result:
                x_left, y_left, w_left, h_left = left_result
                x_right, y_right, w_right, h_right = right_result

                # Compute depth (Z)
                depth = compute_depth(x_left, x_right, focal_length, baseline)
                depth_pub.publish(depth)
                if depth:
                    # Compute real-world coordinates (X, Y, Z)
                    # X_camera, Y_camera, Z_camera = compute_coordinates(x_left + w_left // 2, y_left + h_left // 2, depth, cx, cy, focal_length)

                    # label = f"X: {X_camera:.2f} m, Y: {Y_camera:.2f} m, Z: {Z_camera:.2f} m"
                    # rospy.loginfo(f"Object in camera frame: {label}")

                    # Transform to ROS coordinate system
                    # X_ros = Z_camera
                    # Y_ros = -X_camera
                    # Z_ros = -Y_camera

                    # camera_point = PointStamped()
                    # camera_point.header.frame_id = "camera_link_gripper_left"
                    # camera_point.point.x = X_ros
                    # camera_point.point.y = Y_ros
                    # camera_point.point.z = Z_ros

                    # link = "camera_link_gripper_left"
                    # world_point = transform_point(camera_point, link)
                    # if world_point:
                    #     rospy.loginfo(f"Object in world frame: x: {world_point.point.x:.2f}, y: {world_point.point.y:.2f}, z: {world_point.point.z:.2f}")
                    # else:
                    #     rospy.loginfo("transform_point failed")
                        
                    # rospy.loginfo("")

                    # Draw bounding box and label on the left image
                    cv2.rectangle(left_image, (x_left, y_left), (x_left + w_left, y_left + h_left), (0, 255, 0), 2)
                msg=Float64MultiArray()
                msg.data=[x_left,y_left,x_left+w_left,y_left,x_left,y_left+h_left,x_left+w_left,y_left+h_left]
                coordinates.publish(msg)

            # Show the left image with annotations
            cv2.imshow("Left Camera - Red Object Detection", left_image)


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        else:
            rospy.loginfo("Waiting for images...")

        rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()