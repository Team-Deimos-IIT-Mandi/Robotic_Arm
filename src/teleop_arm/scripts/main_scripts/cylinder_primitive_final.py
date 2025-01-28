#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pynput import keyboard
from cv_bridge import CvBridge
import signal

from detect_cylinder_function import get_object_coordinates
from sensor_msgs.msg import Image

# Global variables for images
bridge = CvBridge()
left_image = None
right_image = None

# Signal handler for clean shutdown
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down teleoperation node...")
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

# Callback for the left camera image topic
def left_image_callback(msg):
    global left_image
    left_image = bridge.imgmsg_to_cv2(msg, "bgr8")

# Callback for the right camera image topic
def right_image_callback(msg):
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, "bgr8")

# Function to log the current position of the robot's end effector
def log_current_position(move_group):
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(f"Current Position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")

# Function to move the robot towards a target position within a specified tolerance
def move_robot_to_target(move_group, target_x, target_y, target_z, tolerance=0.3):
    rospy.loginfo("Starting to move towards the target...")
    while not rospy.is_shutdown():
        current_pose = move_group.get_current_pose().pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_z = current_pose.position.z

        # Calculate differences between current and target positions
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        log_current_position(move_group)
        rospy.loginfo(f"Target Position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")

        # Check if the robot is within the tolerance range
        if abs(dx) < tolerance and abs(dy) < tolerance and abs(dz) < tolerance:
            rospy.loginfo(f"Already within tolerance: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
            rospy.loginfo("Reached the target within tolerance range.")
            rospy.loginfo("Exiting program as movement is successful.")
            sys.exit(0)

        # Move the robot in small steps towards the target
        step_size = 0.02
        move_robot(
            move_group,
            dx=min(step_size, max(-step_size, dx)),
            dy=min(step_size, max(-step_size, dy)),
            dz=min(step_size, max(-step_size, dz)),
        )

# Function to move the robot by a small step with retry logic
def move_robot(move_group, dx=0.0, dy=0.0, dz=0.0, max_retries=5, tolerance=0.01):
    retries = 0

    while not rospy.is_shutdown():
        try:
            current_pose = move_group.get_current_pose().pose
            target_pose = Pose()
            # Set the target pose based on the current pose and step sizes
            target_pose.position.x = current_pose.position.x + dx
            target_pose.position.y = current_pose.position.y + dy
            target_pose.position.z = current_pose.position.z + dz
            target_pose.orientation = current_pose.orientation

            # Calculate differences to check if within tolerance
            diff_x = abs(target_pose.position.x - current_pose.position.x)
            diff_y = abs(target_pose.position.y - current_pose.position.y)
            diff_z = abs(target_pose.position.z - current_pose.position.z)

            if diff_x < tolerance and diff_y < tolerance and diff_z < tolerance:
                rospy.loginfo(f"Already within tolerance: dx={diff_x:.3f}, dy={diff_y:.3f}, dz={diff_z:.3f}")
                rospy.loginfo("Movement successful.")
                return

            rospy.loginfo(f"Attempting movement: dx={dx}, dy={dy}, dz={dz} (Retry {retries + 1}/{max_retries})")

            move_group.set_pose_target(target_pose)
            success = move_group.go(wait=True)

            if success:
                rospy.loginfo("Movement step successful.")
                move_group.stop()
                move_group.clear_pose_targets()
                log_current_position(move_group)
                return
            else:
                rospy.logwarn("Movement aborted. Retrying...")
                retries += 1
                if retries >= max_retries:
                    rospy.logerr("Maximum retries reached. Aborting movement.")
                    move_group.stop()
                    move_group.clear_pose_targets()
                    break

        except Exception as e:
            rospy.logerr(f"Exception during movement: {e}")
            retries += 1
            if retries >= max_retries:
                rospy.logerr("Maximum retries reached. Aborting movement.")
                move_group.stop()
                move_group.clear_pose_targets()
                break

        rospy.sleep(1)

# Main function to initialize the ROS node and handle keyboard inputs
def main():
    # Set up signal handling for clean shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Initialize MoveIt and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_move_arm', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("body")  # Replace "body" with the correct group name

    # Subscribe to stereo camera image topics
    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)

    rospy.loginfo("""
    Press 't' to automatically move towards the detected object.
    """)

    # Parameters for object detection and movement
    cx = 400  # Principal point x-coordinate
    cy = 400  # Principal point y-coordinate
    focal_length = 500
    baseline = 0.16
    tolerance = 0.2

    # Keyboard listener callback
    def on_press(key):
        try:
            if key.char == 't':
                rospy.loginfo("Auto-moving towards the object...")
                if left_image is not None and right_image is not None:
                    result = get_object_coordinates(left_image, right_image, cx, cy, focal_length, baseline)
                    if result:
                        target_x, target_y, target_z = result
                        rospy.loginfo(f"Moving towards object at: x={target_x}, y={target_y}, z={target_z}")
                        move_robot_to_target(move_group, target_x, target_y, target_z, tolerance)
                    else:
                        rospy.loginfo("Object not detected or transformation failed.")
                else:
                    rospy.loginfo("Waiting for stereo images...")
        except AttributeError:
            pass

    # Start the keyboard listener and ROS spin loop
    with keyboard.Listener(on_press=on_press) as listener:
        rospy.spin()
        listener.join()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
