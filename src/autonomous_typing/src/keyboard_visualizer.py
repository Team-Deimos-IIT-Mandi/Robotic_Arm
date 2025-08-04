#!/usr/bin/env python3
# filepath: /root/ros_ws/src/Robotic_Arm/src/autonomous_typing/src/keyboard_visualizer.py

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_keyboard_visualization():
    rospy.init_node('keyboard_visualizer', anonymous=True)
    pub = rospy.Publisher('/keyboard_model_marker', Marker, queue_size=10)
    
    rospy.loginfo("🎹 Starting keyboard visualizer for RViz...")
    
    # Wait for subscribers
    rospy.sleep(2)
    
    rate = rospy.Rate(2)  # 2 Hz
    
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = "base_link"  # Use base_link instead of keyboard_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "keyboard_model"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Position (same as Gazebo spawn position)
        marker.pose.position.x = 0.0
        marker.pose.position.y = -0.75
        marker.pose.position.z = 0.3
        
        # Orientation (same as Gazebo: R=1.57, P=0, Y=3.14)
        # Convert to quaternion: roll=1.57, pitch=0, yaw=3.14
        marker.pose.orientation.x = 0.707  # sin(1.57/2)
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.707  # sin(3.14/2)
        marker.pose.orientation.w = 0.0    # cos(total_rotation/2)
        
        # Scale (keyboard dimensions)
        marker.scale.x = 0.45  # 45cm length
        marker.scale.y = 0.15  # 15cm width  
        marker.scale.z = 0.02  # 2cm height
        
        # Color (dark gray keyboard)
        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.color.a = 0.8
        
        marker.lifetime = rospy.Duration(0)  # Never expire
        
        pub.publish(marker)
        rospy.loginfo_throttle(5, "📺 Publishing keyboard model marker to /keyboard_model_marker")
        rate.sleep()

if __name__ == '__main__':
    try:
        create_keyboard_visualization()
    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard visualizer shutting down...")
        pass