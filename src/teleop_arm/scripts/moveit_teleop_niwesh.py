#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pynput import keyboard
import signal

# Signal handler for clean shutdown
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down teleoperation node...")
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

# Function to move the robot arm
def move_robot_final(move_group, direction):
    # Define the step size for movement
    step = 0.05  # 5 cm step size

    # Mapping directions to deltas
    deltas = {
        'w': (step, 0.0, 0.0),   # +X
        's': (-step, 0.0, 0.0),  # -X
        'a': (0.0, step, 0.0),   # +Y
        'd': (0.0, -step, 0.0),  # -Y
        'q': (0.0, 0.0, step),   # +Z
        'e': (0.0, 0.0, -step)   # -Z
    }

    # Get the corresponding deltas for the direction
    dx, dy, dz = deltas.get(direction, (0.0, 0.0, 0.0))

    # Get the current pose of the end effector
    current_pose = move_group.get_current_pose().pose

    # Define the new target pose
    target_pose = Pose()
    target_pose.position.x = current_pose.position.x + dx
    target_pose.position.y = current_pose.position.y + dy
    target_pose.position.z = current_pose.position.z + dz
    target_pose.orientation = current_pose.orientation  # Keep the same orientation

    # Set and execute the target pose
    move_group.set_pose_target(target_pose)
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Log the new pose
    rospy.loginfo(f"Moved to: {move_group.get_current_pose().pose}")

# Keyboard press handler
def on_press(key, move_group):
    try:
        if key.char in ['w', 's', 'a', 'd', 'q', 'e']:
            move_robot_final(move_group, key.char)
    except AttributeError:
        pass

# Main function
def main():
    signal.signal(signal.SIGINT, signal_handler)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_move_arm', anonymous=True)

    # Set up MoveIt objects
    move_group = moveit_commander.MoveGroupCommander("body")  # "body" is the group name in MoveIt configuration

    rospy.loginfo("""
    Teleoperation started. Use the following keys to move the arm:
        w → Move in +X direction
        s → Move in -X direction
        a → Move in +Y direction
        d → Move in -Y direction
        q → Move in +Z direction
        e → Move in -Z direction
    """)

    with keyboard.Listener(on_press=lambda key: on_press(key, move_group)) as listener:
        rospy.spin()  # Keep the node running
        listener.join()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
