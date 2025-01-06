#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

def publish_goal(x, y, z, x_o, y_o, z_o, w):
    rospy.init_node('end_effector_publisher', anonymous=True)
    pub = rospy.Publisher('/end_effector_goal', PoseStamped, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.frame_id = "base_link"  # Adjust frame_id as needed
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.x = x_o
        goal.pose.orientation.y = y_o
        goal.pose.orientation.z = z_o
        goal.pose.orientation.w = w  # Adjust orientation if needed
        
        rospy.loginfo("Publishing goal: %s", goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        x = float(input("Enter the X position of the end effector: "))
        y = float(input("Enter the Y position of the end effector: "))
        z = float(input("Enter the Z position of the end effector: "))
        x_o = float(input("Enter the X orien of the end effector: "))
        y_o = float(input("Enter the Y orien of the end effector: "))
        z_o = float(input("Enter the Z orien of the end effector: "))
        w = float(input("Enter the W orien of the end effector: "))

        publish_goal(x, y, z,x_o,y_o,z_o,w)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
