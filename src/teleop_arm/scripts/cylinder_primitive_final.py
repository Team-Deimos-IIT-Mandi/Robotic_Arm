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

def left_image_callback(msg):
    """Callback for the left camera."""
    global left_image
    left_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def right_image_callback(msg):
    """Callback for the right camera."""
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def log_current_position(move_group):
    """Log the current position of the end effector."""
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(f"Current Position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")

def move_robot_to_target(move_group, target_x, target_y, target_z, tolerance=0.3):
    """Move the robot towards the target coordinates with a given tolerance."""
    rospy.loginfo("Starting to move towards the target...")
    while not rospy.is_shutdown():
        current_pose = move_group.get_current_pose().pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_z = current_pose.position.z

        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        log_current_position(move_group)
        rospy.loginfo(f"Target Position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")

        # Check if the robot is within the tolerance range
        if abs(dx) < tolerance and abs(dy) < tolerance and abs(dz) < tolerance:
            
            rospy.loginfo(f"Already within tolerance: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f} test1")
            
            
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

def move_robot(move_group, dx=0.0, dy=0.0, dz=0.0, max_retries=5, tolerance=0.01):
    """Move the robot by a given step with retry logic and tolerance check."""
    retries = 0

    while not rospy.is_shutdown():
        try:
            current_pose = move_group.get_current_pose().pose
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + dx
            target_pose.position.y = current_pose.position.y + dy
            target_pose.position.z = current_pose.position.z + dz
            target_pose.orientation = current_pose.orientation

            diff_x = abs(target_pose.position.x - current_pose.position.x)
            diff_y = abs(target_pose.position.y - current_pose.position.y)
            diff_z = abs(target_pose.position.z - current_pose.position.z)

            # Check if the robot is already within tolerance
            if diff_x < tolerance and diff_y < tolerance and diff_z < tolerance:
                rospy.loginfo(f"Already within tolerance: dx={diff_x:.3f}, dy={diff_y:.3f}, dz={diff_z:.3f}")
                rospy.loginfo("Movement successful.")
                return  # Stop further retries and exit function

            rospy.loginfo(f"Attempting movement: dx={dx}, dy={dy}, dz={dz} (Retry {retries + 1}/{max_retries})")

            move_group.set_pose_target(target_pose)
            success = move_group.go(wait=True)

            if success:
                rospy.loginfo("Movement step successful.")
                move_group.stop()
                move_group.clear_pose_targets()
                log_current_position(move_group)
                return  # Exit function after successful movement
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

def main():
    signal.signal(signal.SIGINT, signal_handler)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_move_arm', anonymous=True)

    move_group = moveit_commander.MoveGroupCommander("body")  # Use the correct group name
    # move_group.set_goal_tolerance(0.01)

    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)

    rospy.loginfo("""
    Press 't' to automatically move towards the detected object.
    """)

    cx = 400  # Principal point x-coordinate
    cy = 400  # Principal point y-coordinate
    focal_length = 500
    baseline = 0.16
    tolerance = 0.2

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

    with keyboard.Listener(on_press=on_press) as listener:
        rospy.spin()
        listener.join()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
