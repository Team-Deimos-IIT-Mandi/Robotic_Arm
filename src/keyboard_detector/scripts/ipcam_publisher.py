#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main():
    rospy.init_node('ipcam_publisher')
    ipcam_url = rospy.get_param("~ipcam_url", "http://172.16.8.206:8080/video")  # Replace with your phone's IP and port
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    bridge = CvBridge()
    cap = cv2.VideoCapture(ipcam_url)

    if not cap.isOpened():
        rospy.logerr("Cannot open IP camera stream: {}".format(ipcam_url))
        return

    rate = rospy.Rate(60)  # 10 Hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame")
            continue
        msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
