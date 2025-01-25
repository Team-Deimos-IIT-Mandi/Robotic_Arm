#!/usr/bin/env python3

import rospy
import tf
import math

def calculate_distance(link1, link2):
    rospy.init_node('cam_distance')
    listener = tf.TransformListener()

    try:
        # Wait for the transform to be available
        listener.waitForTransform(link1, link2, rospy.Time(0), rospy.Duration(5.0))
        
        # Lookup the transform between the two links
        (translation, rot) = listener.lookupTransform(link1, link2, rospy.Time(0))
        Re_c = tf.transformations.quaternion_matrix(rot)[:3,:3]
        
        # Calculate Euclidean distance
        return translation,Re_c
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Transform not found between {link1} and {link2}: {e}")
        return None

if __name__ == '__main__':
    link1 = "Link_6"
    link2 = "base_link"
    while not rospy.is_shutdown():    
        distance,r = calculate_distance(link1, link2)
        print(distance,r)