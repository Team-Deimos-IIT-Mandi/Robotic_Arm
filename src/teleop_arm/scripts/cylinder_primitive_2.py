#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import PointStamped, Pose
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import signal
import math

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
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        transform = tf_buffer.lookup_transform("world", camera_frame, rospy.Time(0), rospy.Duration(1.0))
        world_point = do_transform_point(camera_point, transform)
        return world_point

    except tf2_ros.LookupException as e:
        rospy.logerr(f"Transform lookup failed: {e}")
        return None

def move_robot_arm(move_group, x, y, z):
    """Move the robotic arm to the target position."""
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    target_pose.orientation.w = 1.0  # Default orientation (adjust as needed)

    move_group.set_pose_target(target_pose)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"Arm moved to position: x={x}, y={y}, z={z}")
    else:
        rospy.logerr("Failed to move the robotic arm!")

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
        if w > 20 and h > 20:
            return x, y, w, h
    return None

def compute_depth(x_left, x_right, focal_length, baseline):
    disparity = x_left - x_right
    if disparity > 0:
        depth = (focal_length * baseline) / disparity
        return depth
    return None

def compute_coordinates(x_left, y_left, depth, cx, cy, focal_length):
    X_cam = (x_left - cx) * depth / focal_length
    Y_cam = (y_left - cy) * depth / focal_length
    Z_cam = depth
    return X_cam, Y_cam, Z_cam

def main():
    rospy.init_node("stereo_vision_red_object_detection")

    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("body")  # Replace "arm" with your MoveIt group name

    rospy.Subscriber("/camera_base_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_base_right/image_raw", Image, right_image_callback)

    cx = 400
    cy = 400
    focal_length = 500
    baseline = 0.24

    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not stop_program:
        if left_image is not None and right_image is not None:
            left_result = detect_red_object(left_image)
            right_result = detect_red_object(right_image)

            if left_result and right_result:
                x_left, y_left, w_left, h_left = left_result
                x_right, y_right, w_right, h_right = right_result

                depth = compute_depth(x_left, x_right, focal_length, baseline)
                if depth:
                    X_cam, Y_cam, Z_cam = compute_coordinates(x_left + w_left // 2, y_left + h_left // 2, depth, cx, cy, focal_length)

                    camera_point = PointStamped()
                    camera_point.header.frame_id = "camera_base_left"
                    camera_point.point.x = Z_cam
                    camera_point.point.y = -X_cam
                    camera_point.point.z = -Y_cam

                    world_point = transform_point(camera_point, "camera_link_base_left")
                    if world_point:
                        x_world = world_point.point.x
                        y_world = world_point.point.y
                        z_world = world_point.point.z
                        rospy.loginfo(f"Moving arm to world coordinates: x={x_world}, y={y_world}, z={z_world}")
                        move_robot_arm(move_group, x_world, y_world, z_world)
                    

            cv2.imshow("Left Camera - Red Object Detection", left_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        rate.sleep()

    moveit_commander.roscpp_shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
