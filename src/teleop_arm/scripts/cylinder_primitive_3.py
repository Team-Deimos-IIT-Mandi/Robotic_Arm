#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pynput import keyboard
import signal
from stereo_vision.scripts.detect_cylinder_function import get_object_coordinates

from sensor_msgs.msg import Image

# Global variables for images
left_image = None
right_image = None

# Signal handler for clean shutdown
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down teleoperation node...")
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

# Callback functions for stereo images
def left_image_callback(msg):
    global left_image
    left_image = msg

def right_image_callback(msg):
    global right_image
    right_image = msg

def log_current_position(move_group):
    """Log the current position of the end effector."""
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(f"Current Position: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")

def move_robot_to_target(move_group, target_x, target_y, target_z, tolerance=0.3):
    """Move the robot towards the target coordinates with a given tolerance."""
    while not rospy.is_shutdown():
        # Get the current position of the end effector
        current_pose = move_group.get_current_pose().pose
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        current_z = current_pose.position.z

        # Calculate the difference
        dx = target_x - current_x
        dy = target_y - current_y
        dz = target_z - current_z

        # Log current and target positions
        log_current_position(move_group)
        rospy.loginfo(f"Target Position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")

        # Check if the robot is within the tolerance
        if abs(dx) < tolerance and abs(dy) < tolerance and abs(dz) < tolerance:
            rospy.loginfo("Reached the target within tolerance.")
            break

        # Move the robot in small steps towards the target
        step_size = 0.02  # Adjust as needed for smooth movement
        move_robot(
            move_group,
            dx=min(step_size, max(-step_size, dx)),
            dy=min(step_size, max(-step_size, dy)),
            dz=min(step_size, max(-step_size, dz)),
        )

def move_robot(move_group, dx=0.0, dy=0.0, dz=0.0):
    """Move the robot by a given step."""
    current_pose = move_group.get_current_pose().pose
    target_pose = Pose()
    target_pose.position.x = current_pose.position.x + dx
    target_pose.position.y = current_pose.position.y + dy
    target_pose.position.z = current_pose.position.z + dz
    target_pose.orientation = current_pose.orientation  # Keep the orientation unchanged
    
    rospy.loginfo(f"dx: {dx} , dy : {dy}, dz: {dz}")

    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Log the new position of the end effector
    log_current_position(move_group)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_move_arm', anonymous=True)

    # Set up MoveIt objects
    move_group = moveit_commander.MoveGroupCommander("body")  # Use the correct group name
    rospy.Subscriber("/camera_gripper_left/image_raw", Image, left_image_callback)
    rospy.Subscriber("/camera_gripper_right/image_raw", Image, right_image_callback)

    rospy.loginfo("""
    Teleoperation started. Use the following keys to move the arm manually:
        w → Move in +X direction
        s → Move in -X direction
        a → Move in +Y direction
        d → Move in -Y direction
        q → Move in +Z direction
        e → Move in -Z direction
    Press 't' to automatically move towards the detected object.
    """)

    cx = 400  # Principal point x-coordinate
    cy = 400  # Principal point y-coordinate
    focal_length = 500
    baseline = 0.16
    tolerance = 0.3  # Tolerance level

    def on_press(key):
        try:
            if key.char == 't':  # Auto-move towards the object
                rospy.log
                if left_image is not None and right_image is not None:
                    result = get_object_coordinates(left_image, right_image, cx, cy, focal_length, baseline)
                    rospy.loginfo("result")
                    if result:
                        target_x, target_y, target_z = result
                        rospy.loginfo(f"Moving towards object at: x={target_x}, y={target_y}, z={target_z}")
                        move_robot_to_target(move_group, target_x, target_y, target_z, tolerance)
                    else:
                        rospy.loginfo("Object not detected or transformation failed.")
                else:
                    rospy.loginfo("Waiting for stereo images...")
            else:
                # Manual control
                step = 0.05  # 5 cm step size
                if key.char == 'w':  # Move +X
                    move_robot(move_group, dx=step)
                elif key.char == 's':  # Move -X
                    move_robot(move_group, dx=-step)
                elif key.char == 'a':  # Move +Y
                    move_robot(move_group, dy=step)
                elif key.char == 'd':  # Move -Y
                    move_robot(move_group, dy=-step)
                elif key.char == 'q':  # Move +Z
                    move_robot(move_group, dz=step)
                elif key.char == 'e':  # Move -Z
                    move_robot(move_group, dz=-step)
        except AttributeError:
            pass
        
    rospy.loginfo("test1")

    with keyboard.Listener(on_press=on_press) as listener:
        rospy.spin()
        listener.join()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
