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
import rospy
import numpy as np


class LightDetector():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.traffic_light_pub = rospy.Publisher(
            "traffic_light", String, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image_traffic", Image, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()
        self.center_ros = Point()

        r_radius = 0
        g_radius = 0
        y_radius = 0
        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0
        self.radius_ros = 0

        # define the lower and upper boundaries for each color
        # The parameters are also here, in case the program do not find the yaml file
        redColorLower = tuple(rospy.get_param(
            "/redColorLower", (90, 120, 170)))
        redColorUpper = tuple(rospy.get_param(
            "/redColorUpper", (180, 255, 255)))
        greenColorLower = tuple(rospy.get_param(
            "/greenColorLower", (40, 20, 0)))
        greenColorUpper = tuple(rospy.get_param(
            "/greenColorUpper", (80, 255, 255)))
        yellowColorLower = tuple(rospy.get_param(
            "/yellowColorLower", (20, 120, 190)))
        yellowColorUpper = tuple(rospy.get_param(
            "/yellowColorUpper", (60, 255, 255)))
        secondRedColorLower = tuple(rospy.get_param(
            "/secondRedColorLower", (20, 120, 190)))
        secondRedColorUpper = tuple(rospy.get_param(
            "/secondRedColorUpper", (20, 120, 190)))

        self.min_radius = 120
        self.pts = deque(maxlen=64)

        ros_rate = rospy.Rate(50)
        while not rospy.is_shutdown():

            if self.image_received_flag == 1:
                # resize the.self.frame, blur it, and convert it to the HSV color space
                self.frame = cv2.resize(self.frame, (600, 400))
                # blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
                median = cv2.medianBlur(self.frame, 7)
                hsv = cv2.cvtColor(median, cv2.COLOR_BGR2HSV)

                r_radius = self.find_ball(
                    hsv, redColorLower, redColorUpper, secondRedColorLower,  secondRedColorUpper)

                g_radius = self.find_ball(
                    hsv, greenColorLower, greenColorUpper)
                y_radius = self.find_ball(
                    hsv, yellowColorLower, yellowColorUpper)
                print("Red r: " + str(r_radius) + " Green r: " +
                      str(g_radius) + " Yellow r: " + str(y_radius))

                # cv2.imshow("Frame", self.frame)

                image_topic = self.bridge_object.cv2_to_imgmsg(   self.frame, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.image_received_flag = 0

                # Publish the color of the traffic light
                if r_radius or g_radius or y_radius:
                    if r_radius > g_radius and r_radius > y_radius:
                        self.traffic_light_pub.publish("red")
                    elif g_radius > r_radius and g_radius > y_radius:
                        self.traffic_light_pub.publish("green")
                    else:
                        self.traffic_light_pub.publish("yellow")
                else:
                    self.traffic_light_pub.publish("none")

            key = cv2.waitKey(1)
            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def find_ball(self, hsv, colorLower, colorUpper, secondLower=None, secondUpper=None):

        # construct a mask for the color, then perform a series of dilations and
        # erosions to remove any small blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        if secondLower is not None and secondUpper is not None:
            mask2 = cv2.inRange(hsv, secondLower, secondUpper)
            mask = cv2.bitwise_or(mask, mask2)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        gray_blurred = cv2.blur(mask, (3, 3))
        # cv2.imshow("mask", gray_blurred)

        contours = cv2.findContours(
            gray_blurred.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        largest_circle = None
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and centroid
            largest_circle = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_circle)
            M = cv2.moments(largest_circle)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            cv2.circle(self.frame, (int(x), int(y)),
                       int(radius), (0, 255, 0), 2)
            cv2.circle(self.frame, (int(x), int(y)),
                       2, (0, 0, 255), 3)
            if radius > self.min_radius:
                return radius
            else:
                return 0
        return 0

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('light_detector', anonymous=True)
    print("Running light detector")
    LightDetector()
