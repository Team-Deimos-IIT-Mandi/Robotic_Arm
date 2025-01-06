#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# Global variable for MoveGroupCommander
move_group = None
joint_pub = None

def ik_and_control():
    global move_group, joint_pub
    

    rospy.init_node('node_ik', anonymous=True)

    # Initialize MoveIt! Commander
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("plan")  # Adjust group name
    # Initialize the publisher
    joint_pub = rospy.Publisher('/joint_trajectory_controller/command', JointState, queue_size=10)

    # Subscribe to the end-effector goal topic
    rospy.Subscriber('/end_effector_goal', PoseStamped, goal_callback)

    rospy.spin()

def goal_callback(goal):
    global move_group, joint_pub
    
    rospy.loginfo("Received goal: %s", goal)
    
    move_group.set_pose_target(goal.pose)
    
    # Perform IK and plan
    plan_result = move_group.plan()
    
    # Log the type and content of plan_result
    rospy.loginfo("Plan result type: %s", type(plan_result))
    rospy.loginfo("Plan result content: %s", plan_result)
    
    # Inspect each element of the tuple
    for i, element in enumerate(plan_result):
        rospy.loginfo("Element %d type: %s", i, type(element))
        rospy.loginfo("Element %d content: %s", i, element)
        
        if hasattr(element, 'joint_trajectory'):
            rospy.loginfo("Element %d has joint_trajectory attribute", i)
            plan_result = element
            break
    else:
        rospy.logwarn("No element with joint_trajectory found")
        return
    
    # Ensure we have a valid RobotTrajectory
    if len(plan_result.joint_trajectory.points) > 0:
        rospy.loginfo("Plan result: %s", plan_result)
        
        # Publish joint states to control the robot
        joint_state = JointState()
        joint_state.name = move_group.get_joints()
        joint_state.position = plan_result.joint_trajectory.points[0].positions
        joint_state.header.stamp = rospy.Time.now()
        
        rospy.loginfo("Publishing joint states: %s", joint_state)
        joint_pub.publish(joint_state)
    else:
        rospy.logwarn("No joint trajectory points found in the plan")

    # Ensure we have a valid RobotTrajectory
    if hasattr(plan_result, 'joint_trajectory'):
        if len(plan_result.joint_trajectory.points) > 0:
            rospy.loginfo("Plan result: %s", plan_result)
            
            # Publish joint states to control the robot
            joint_state = JointState()
            joint_state.name = move_group.get_joints()
            joint_state.position = plan_result.joint_trajectory.points[0].positions
            joint_state.header.stamp = rospy.Time.now()
            
            rospy.loginfo("Publishing joint states: %s", joint_state)
            joint_pub.publish(joint_state)
        else:
            rospy.logwarn("No joint trajectory points found in the plan")
    else:
        rospy.logwarn("Planning result does not have joint_trajectory attribute")


if __name__ == '__main__':
    try:
        ik_and_control()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
