#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

cv2.resizeWindow("Detected face", 600,600)

class ImageWrapper():

    def __init__(self):
        self.bridge = CvBridge()
        self.x = 0
        self.y = 0
        self.w = 0
        self.h = 0
        rospy.Subscriber("face_publisher", BoundingBox2D, self.get_bbox)
        rospy.Subscriber("/video_publisher/output_video", Image, self.detect_frame)
        

    def get_bbox(self, bbox):
        center = bbox.center
        x, y = center.x, center.y
        w, h = bbox.size_x, bbox.size_y
        self.x = int(2*x-w)
        self.y = int(2*y-h)
        self.w = int(w)
        self.h = int(h)
        rospy.loginfo([self.x, self.y, self.w, self.h])


    def detect_frame(self, frame):
        cv2_frame = self.bridge.imgmsg_to_cv2(frame, "8UC3")
        self.frame = cv2_frame if self.x == 0 else cv2.rectangle(cv2_frame, 
                                                                 pt1=(self.x, self.y), 
                                                                 pt2=(self.x+self.w, self.y+self.h), 
                                                                 color=(0, 100, 255), 
                                                                 thickness=5)
        
        try:
            cv2.imshow("Detected face", self.frame)
            if cv2.waitKey(20) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                rospy.signal_shutdown("Exiting...")
                # exit()
            
        except:
            rospy.logerr("Error in loading image. Trying again...")
            rospy.sleep(1)
            


if __name__ == "__main__":
    rospy.init_node("video_gui_node")
    rospy.sleep(2)
    img_wrapper = ImageWrapper()    
    rospy.spin()
    