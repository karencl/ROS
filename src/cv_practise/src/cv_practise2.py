#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np


min_green = np.array([50, 220, 220])
max_green = np.array([60, 255, 255])
min_red = np.array([170, 220, 220])
max_red = np.array([180, 255, 255])
min_blue = np.array([110, 220, 220])
max_blue = np.array([120, 255, 255])


#!/usr/bin/env python


class ShowingImage(object):

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.bridge_object = CvBridge()

        # Flag to make sure we received an image before showing it.
        self.img_received = 0

        r = rospy.Rate(10)  # 10Hz
        image = cv2.imread(
            '/home/estebanp/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/Filtering.png')
        image = cv2.resize(image, (300, 300))
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_g = cv2.inRange(hsv, min_green, max_green)
        mask_r = cv2.inRange(hsv, min_red, max_red)
        mask_b = cv2.inRange(hsv, min_blue, max_blue)

        res_b = cv2.bitwise_and(image, image, mask=mask_b)
        res_g = cv2.bitwise_and(image, image, mask=mask_g)
        res_r = cv2.bitwise_and(image, image, mask=mask_r)

        cv2.imshow('Green', res_g)
        cv2.imshow('Red', res_r)
        cv2.imshow('Blue', res_b)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

    def cleanup(self):
        cv2.destroyAllWindows()


if __name__ == '__main__':

    rospy.init_node('load_image_2', anonymous=True)

    showing_image_object = ShowingImage()
