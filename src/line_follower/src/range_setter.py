#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
import cv2
import imutils


H_min = 0  # HUE (0-180) = (0-360)
S_min = 0  # SATURATION (0-255)
V_min = 0  # VALUE (0-255)

H_max = 255
S_max = 255
V_max = 255


def cb(_):
    pass


cv2.namedWindow("Sliders")
cv2.resizeWindow("Sliders", 500, 500)

cv2.createTrackbar("H Min", "Sliders", 0, 180, cb)
cv2.setTrackbarPos("H Min", "Sliders", H_min)
cv2.createTrackbar("H Max", "Sliders", 0, 180, cb)
cv2.setTrackbarPos("H Max", "Sliders", H_max)

cv2.createTrackbar("S Min", "Sliders", 0, 255, cb)
cv2.setTrackbarPos("S Min", "Sliders", S_min)
cv2.createTrackbar("S Max", "Sliders", 0, 255, cb)
cv2.setTrackbarPos("S Max", "Sliders", S_max)

cv2.createTrackbar("V Min", "Sliders", 0, 255, cb)
cv2.setTrackbarPos("V Min", "Sliders", V_min)
cv2.createTrackbar("V Max", "Sliders", 0, 255, cb)
cv2.setTrackbarPos("V Max", "Sliders", V_max)


class LightDetector():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.image_sub = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()
        self.center_ros = Point()

        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0

        ros_rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.image_received_flag:
                H_min = cv2.getTrackbarPos("H Min", "Sliders")
                H_max = cv2.getTrackbarPos("H Max", "Sliders")
                S_min = cv2.getTrackbarPos("S Min", "Sliders")
                S_max = cv2.getTrackbarPos("S Max", "Sliders")
                V_min = cv2.getTrackbarPos("V Min", "Sliders")
                V_max = cv2.getTrackbarPos("V Max", "Sliders")

                median = cv2.medianBlur(self.frame, 7)
                hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

                # construct a mask for the color, then perform a series of dilations and
                # erosions to remove any small blobs left in the mask
                mask = cv2.inRange(hsv, (H_min, S_min, V_min),
                                   (H_max, S_max, V_max))

                # mask2 = cv2.inRange(hsv, (0, 115, 0), (5, 255, 255))

                # mask = cv2.bitwise_or(mask, mask2)

                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                cv2.imshow("Image", mask)

                if (cv2.waitKey(1) & 0xFF == ord('l')):
                    cv2.destroyAllWindows()

            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def cleanup(self):
        print("Shutting range setter node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('range_setter', anonymous=True)
    print("Running range setter")
    LightDetector()
