#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

CAP_ID = rospy.get_param("/CAP_ID")

def video_pub():
    rospy.init_node("video_publisher_node")
    pub = rospy.Publisher("video_publisher", Image, queue_size=1)

    cap = cv2.VideoCapture(CAP_ID)
    bridge = CvBridge()

    rate = rospy.Rate(60) #fps
    while not rospy.is_shutdown():
        if cap.isOpened():
            _, frame = cap.read()
            img_msg = bridge.cv2_to_imgmsg(frame, encoding="passthrough")
            
            try:
                pub.publish(img_msg)

            except CvBridgeError as e:
                rospy.logerror(str(e))


            rate.sleep()

    # cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        video_pub()

    except rospy.ROSInterruptException:
        pass

    # rospy.on_shutdown(cv2.destroyAllWindows())
