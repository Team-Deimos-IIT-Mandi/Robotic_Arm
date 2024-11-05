#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

def move_arm_to_position(x, y, z):
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_ik', anonymous=True)

    # Instantiate a `RobotCommander` object. Provides information about the robot's kinematic model.
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface` object. Provides information about the world.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander` object. This object is an interface to one group of joints.
    group_name = "plan"  # Replace with your planning group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Display information about the robot
    rospy.loginfo("Reference frame: %s" % move_group.get_planning_frame())
    rospy.loginfo("End effector: %s" % move_group.get_end_effector_link())
    rospy.loginfo("Robot Groups: %s" % robot.get_group_names())

    # Set the target pose for the end effector
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0  # Assuming upright orientation; modify as needed
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z
    move_group.set_pose_target(pose_goal)

    # Plan the motion
    success, plan = move_group.plan()
    if success:
        rospy.loginfo("Planning succeeded.")
        if plan.joint_trajectory.points:
            # Execute the plan
            move_group.execute(plan, wait=True)
            move_group.stop()
            move_group.clear_pose_targets()
            rospy.loginfo("Motion successfully executed!")
        else:
            rospy.logwarn("Planning succeeded but no trajectory points found.")
    else:
        rospy.logwarn("Motion planning failed.")

    # Shutdown MoveIt! properly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        # Ask for the target end effector position
        x = float(input("Enter the X position of the end effector: "))
        y = float(input("Enter the Y position of the end effector: "))
        z = float(input("Enter the Z position of the end effector: "))

        # Move the arm to the desired position
        move_arm_to_position(x, y, z)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
