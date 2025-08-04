#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
import tkinter as tk
import signal
from pynput import keyboard
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Signal handler for clean shutdown
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down teleoperation node...")
    moveit_commander.roscpp_shutdown()
    sys.exit(0)


# Function to move the robot arm in specified directions
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
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Log the new pose
    rospy.loginfo(f"Moved to: {move_group.get_current_pose().pose}")


# Class for Gripper Control
class GripperController:
    def __init__(self):
        self.joint_names = ['Finger_1', 'Finger_2']
        self.joint_angles = [0.0, 0.0]  # Initial joint angles for open position
        self.step_size = 0.001  # Incremental step size

        self.pub = rospy.Publisher('/end_effector_controller/command', JointTrajectory, queue_size=10)

    def open_gripper(self):
        self.joint_angles[0] = max(0.0, self.joint_angles[0] - self.step_size)
        self.joint_angles[1] = min(0.0, self.joint_angles[1] + self.step_size)
        self._publish()

    def close_gripper(self):
        self.joint_angles[0] = min(0.0354, self.joint_angles[0] + self.step_size)
        self.joint_angles[1] = max(-0.0354, self.joint_angles[1] - self.step_size)
        self._publish()

    def _publish(self):
        # Create and publish the JointTrajectory message
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.joint_angles
        point.time_from_start = rospy.Duration(0.1)

        trajectory_msg.points = [point]
        self.pub.publish(trajectory_msg)

        rospy.loginfo(f"Gripper joint angles: {self.joint_angles}")


# Class for Virtual Joystick control
class VirtualJoystick:
    def __init__(self, move_group, gripper):
        self.move_group = move_group
        self.gripper = gripper
        self.step_scale = 0.1  # Scale joystick movement to robot motion

        # Create GUI window
        self.root = tk.Tk()
        self.root.title("Virtual Joystick")

        # Create a canvas for the joystick
        self.canvas = tk.Canvas(self.root, width=300, height=300, bg="lightgray")
        self.canvas.pack()

        # Draw the joystick base and knob
        self.base = self.canvas.create_oval(50, 50, 250, 250, fill="white", outline="black")
        self.knob = self.canvas.create_oval(125, 125, 175, 175, fill="blue")

        # Bind mouse events to the joystick knob
        self.canvas.bind("<B1-Motion>", self.drag_knob)
        self.canvas.bind("<ButtonRelease-1>", self.reset_knob)

        # Start the GUI loop
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.update_robot_motion)
        self.root.mainloop()

    def drag_knob(self, event):
        # Constrain the knob movement within the base
        x, y = event.x, event.y
        cx, cy = 150, 150  # Center of the joystick base
        dx, dy = x - cx, y - cy
        distance = (dx**2 + dy**2)**0.5
        max_distance = 100  # Radius of the joystick base

        if distance > max_distance:
            # Scale dx and dy to stay within the base
            dx = dx / distance * max_distance
            dy = dy / distance * max_distance

        # Update the position of the knob
        self.canvas.coords(self.knob, cx + dx - 25, cy + dy - 25, cx + dx + 25, cy + dy + 25)

    def reset_knob(self, event):
        # Reset the knob to the center
        self.canvas.coords(self.knob, 125, 125, 175, 175)

    def update_robot_motion(self):
        # Get the current position of the knob
        knob_coords = self.canvas.coords(self.knob)
        knob_x = (knob_coords[0] + knob_coords[2]) / 2
        knob_y = (knob_coords[1] + knob_coords[3]) / 2

        # Calculate the displacement from the center
        cx, cy = 150, 150  # Center of the joystick base
        dx = -(knob_x - cx) / 100 * self.step_scale  # Scale displacement to step size
        dy = (knob_y - cy) / 100 * self.step_scale  # Invert Y-axis for correct motion

        # Only move the robot if the displacement is significant
        if abs(dx) > 0.01 or abs(dy) > 0.01:
            move_robot(self.move_group, dx=dx, dy=dy)

        # Schedule the next update
        self.root.after(100, self.update_robot_motion)

    def on_close(self):
        self.root.destroy()
        sys.exit(0)


# Keyboard press handler for Z-axis and gripper movement
def on_press(key, move_group, gripper):
    try:
        if key.char == 'w':  # Move in +Z direction
            rospy.loginfo("Moving up in +Z direction")
            move_robot(move_group, dz=0.05)
        elif key.char == 's':  # Move in -Z direction
            rospy.loginfo("Moving down in -Z direction")
            move_robot(move_group, dz=-0.05)
        elif key.char == 'o':  # Open gripper
            rospy.loginfo("Opening gripper")
            gripper.open_gripper()
        elif key.char == 'c':  # Close gripper
            rospy.loginfo("Closing gripper")
            gripper.close_gripper()
    except AttributeError:
        pass


# Main function
def main():
    signal.signal(signal.SIGINT, signal_handler)

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('teleop_virtual_joystick_with_gripper', anonymous=True)

    # Set up MoveIt objects
    move_group = moveit_commander.MoveGroupCommander("body")  # Replace "body" with your group name in MoveIt
    gripper = GripperController()

    rospy.loginfo("""
    Teleoperation started. Use the following controls:
        - Virtual Joystick (GUI) for X and Y movements
        - Terminal keys:
            w → Move up (+Z direction)
            s → Move down (-Z direction)
            o → Open gripper
            c → Close gripper
    Press Ctrl+C to exit.
    """)

    # Start keyboard listener for Z-axis and gripper movement
    with keyboard.Listener(on_press=lambda key: on_press(key, move_group, gripper)) as listener:
        # Start the Virtual Joystick GUI
        VirtualJoystick(move_group, gripper)
        listener.join()

    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
