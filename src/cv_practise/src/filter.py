#!/usr/bin/env python

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospy
import cv2
import numpy as np

H_min = 0
H_max = 22
S_min = 1
S_max = 255
V_min = 35
V_max = 255


class Filter(object):

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.bridge_object = CvBridge()

        r = rospy.Rate(10)  # 10Hz
        self.image_sub = rospy.Subscriber(
            "camera/image_raw", Image, self.camera_callback)

        self.mask_pub = rospy.Publisher(
            "/segmented_image", Image, queue_size=1)

        self.img_received = 0

        while not rospy.is_shutdown():
            if self.img_received:

                image = cv2.resize(self.cv_image, (300, 300))
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask_r = cv2.inRange(
                    hsv, (H_min, S_min, V_min), (H_max, S_max, V_max))

                res_r = cv2.bitwise_and(image, image, mask=mask_r)
                cv2.imshow('Original', image)
                cv2.imshow('Red', res_r)

                image_message = self.bridge_object.cv2_to_imgmsg(
                    res_r, encoding="passthrough")

                self.mask_pub.publish(image_message)

            cv2.waitKey(1)
            r.sleep()

        cv2.destroyAllWindows()

    def cleanup(self):
        cv2.destroyAllWindows()

    def camera_callback(self, data):

        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")

        except CvBridgeError as e:
            print(e)

        self.img_received = 1


if __name__ == '__main__':

    rospy.init_node('load_image_2', anonymous=True)

    showing_image_object = Filter()
