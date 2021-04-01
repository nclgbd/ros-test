#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

### --- PARAMETERS --- ###
HAAR_CASCADE_FILE = rospy.get_param("/HAAR_CASCADE_FILE")


def get_faces(raw_img):
    bridge = CvBridge()
    cv2_img = bridge.imgmsg_to_cv2(raw_img, "8UC3")
    faces = detect_faces((cv2_img))
    face_pub = rospy.Publisher("face_publisher", BoundingBox2D, queue_size=1)

    rate = rospy.Rate(60) #fps
    if len(faces) > 0: #stupid stupid line of stupid code STUPIDDDD
        rospy.loginfo(faces)
        for (x,y,w,h) in faces:
            centered = Pose2D((x+w)/2, (y+h)/2, 0.0)
            bbox = BoundingBox2D(center=centered, size_x=w, size_y=h)
            face_pub.publish(bbox)

    else:
        rospy.loginfo("No faces detected")

    rate.sleep()


def detect_faces(cv2_img):
    face_cascade = cv2.CascadeClassifier(HAAR_CASCADE_FILE)
    gray = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2GRAY)
    return face_cascade.detectMultiScale(gray, 1.3, 5)


if __name__ == '__main__':
    rospy.init_node("face_sub_node")
    sub = rospy.Subscriber("/video_publisher/output_video", Image, get_faces)
    rospy.spin()
