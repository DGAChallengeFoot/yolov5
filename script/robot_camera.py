#!/usr/bin/env python

import rospy
import cv2, cv_bridge
from sensor_msgs.msg import Image

class robot_camera:
    def __init__(self):
        rospy.init_node("robot_camera", anonymous=False)
        self.track_flag = False
        self.default_pose_flag = True

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/husky/front_cam/camera/image', Image, self.image_callback)


    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        cv2.namedWindow("robot_camera_window", 1)
        cv2.imshow("robot_camera_window", image)
        cv2.waitKey(1)


if __name__ == '__main__':
    print("Let's view your robot camera")
    follower = robot_camera()
    rospy.spin()


