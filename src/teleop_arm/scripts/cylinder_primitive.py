#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from pynput import keyboard
import signal
import math

# Signal handler for clean shutdown
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down teleoperation node...")
    moveit_commander.roscpp_shutdown()
    sys.exit(0)

def move_to_goal(move_group, goal_x, goal_y, goal_z, tolerance=0.1):
    """
    Moves the robot to the specified goal position (goal_x, goal_y, goal_z) with a given tolerance.
    """
    current_pose = move_group.get_current_pose().pose

    # Compute the distance to the goal
    dx = goal_x - current_pose.position.x
    dy = goal_y - current_pose.position.y
    dz = goal_z - current_pose.position.z
    distance = math.sqrt(dx**2 + dy**2 + dz**2)

    # Move in small steps until within tolerance
    step_size = 0.05  # 5 cm per step
    while distance > tolerance and not rospy.is_shutdown():
        step_dx = dx * min(step_size / distance, 1.0)
        step_dy = dy * min(step_size / distance, 1.0)
        step_dz = dz * min(step_size / distance, 1.0)

        move_robot(move_group, dx=step_dx, dy=step_dy, dz=step_dz)

        # Update the current position and distance
        current_pose = move_group.get_current_pose().pose
        dx = goal_x - current_pose.position.x
        dy = goal_y - current_pose.position.y
        dz = goal_z - current_pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

    rospy.loginfo(f"Reached goal position: x={goal_x}, y={goal_y}, z={goal_z} (tolerance: {tolerance})")

def move_robot(move_group, dx=0.0, dy=0.0, dz=0.0):
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
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Log the new pose
    rospy.loginfo(f"Moved to: {move_group.get_current_pose().pose}")

def on_press(key, move_group):
    try:
        if key.char == 'g':  # Move to a goal
            # Define goal position (example goal)
            goal_x = 0 # Replace with your desired x-coordinate
            goal_y = -0.7  # Replace with your desired y-coordinate
            goal_z = 0.5  # Replace with your desired z-coordinate
            move_to_goal(move_group, goal_x, goal_y, goal_z)

    except AttributeError:
        pass

def main():
    
    signal.signal(signal.SIGINT, signal_handler)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cylinder_primitive', anonymous=True)

    # Set up MoveIt objects
    move_group = moveit_commander.MoveGroupCommander("body")  # Replace "body" with your MoveIt group name

    rospy.loginfo("""
    Teleoperation started. Use the following keys to move the arm:
        g → Move to a specific goal position
    """)

    with keyboard.Listener(on_press=lambda key: on_press(key, move_group)) as listener:
        rospy.spin()  # Keep the node running
        listener.join()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
