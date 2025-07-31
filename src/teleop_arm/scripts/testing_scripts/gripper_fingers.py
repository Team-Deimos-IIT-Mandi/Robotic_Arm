#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import sys, select, termios, tty


class TeleopGripperController:
    def __init__(self):
        # Define the joint names for the gripper
        self.joint_names = ['Finger_1', 'Finger_2']
        self.joint_angles = [0.0, 0.0]  # Placeholder for joint angles
        self.step_size = 0.001  # Incremental step for each key press
        self.settings = termios.tcgetattr(sys.stdin)

        rospy.init_node('teleop_gripper')
        self.pub = rospy.Publisher('/end_effector_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscribe to joint_states to get initial joint angles
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        self.received_joint_states = False
        rospy.loginfo("Teleoperation node started. Waiting for initial joint angles...")

        # Wait until the joint states are received
        while not self.received_joint_states and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def joint_state_callback(self, msg):
        """Callback function to receive joint states and update initial angles."""
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_names:
                index = self.joint_names.index(joint_name)
                self.joint_angles[index] = msg.position[i]
        self.received_joint_states = True
        rospy.loginfo(f"Initial joint angles received: {self.joint_angles}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        while not rospy.is_shutdown():
            key = self.get_key()
            command_executed = False

            if key == 'o':  # Incrementally open gripper
                self.joint_angles[0] = max(0.0, self.joint_angles[0] - self.step_size)
                self.joint_angles[1] = min(0.0, self.joint_angles[1] + self.step_size)
                command_executed = True
                rospy.loginfo("Gripper opening incrementally.")

            elif key == 'c':  # Incrementally close gripper
                self.joint_angles[0] = min(0.0354, self.joint_angles[0] + self.step_size)
                self.joint_angles[1] = max(-0.0354, self.joint_angles[1] - self.step_size)
                command_executed = True
                rospy.loginfo("Gripper closing incrementally.")

            elif key == '\x03':  # Ctrl+C
                rospy.loginfo("Shutting down teleoperation node.")
                break

            if command_executed:
                # Create the JointTrajectory message
                trajectory_msg = JointTrajectory()
                trajectory_msg.joint_names = self.joint_names

                # Create a single point in the trajectory
                point = JointTrajectoryPoint()
                point.positions = self.joint_angles
                point.time_from_start = rospy.Duration(0.1)  # Small duration to indicate immediate movement

                trajectory_msg.points = [point]

                # Publish the trajectory message
                self.pub.publish(trajectory_msg)

                rospy.loginfo(f"Published joint angles: {self.joint_angles}")

            self.rate.sleep()

    def shutdown(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    teleop_gripper = TeleopGripperController()
    try:
        teleop_gripper.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        teleop_gripper.shutdown()
