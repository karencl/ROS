#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
from collections import deque
import cv2

import rospy
import numpy as np

class LineDetector():
    def __init__(self):

        self.bridge_object = CvBridge()

        self.image_received_flag = 0

        # The value for the threshold that will highlight black lines
        self.line_threshold = 170

        rospy.on_shutdown(self.cleanup)
        self.line_centroid_pub = rospy.Publisher(
            "/line_centroid", Float32, queue_size=1)
        self.image_pub = rospy.Publisher(
            "/processed_image_line", Image, queue_size=1)

        # We subscribe to three topics to account for simulation, physical camera and launch file
        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        # We can modify the threshold through a topic
        self.threshold_sub = rospy.Subscriber(
            "/line_threshold", Int32, self.threshold_callback)

        ros_rate = rospy.Rate(50)

        # We start by assuming that the line centroid is in the center of the image
        # The image is of shape 500x166 (166 is a third of 500)
        last_centroid = (250, 83)

        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                resized = cv2.resize(self.frame, (500, 500))

                # We get the lower third of the image to remove unnecessary information
                resized = resized[500/3*2:500, 0:500]

                # Convert to grayscale
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

                # Gaussian blurr
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # Dilate and erode
                blurred = cv2.erode(cv2.dilate(
                    blurred, None, iterations=2), None, iterations=2)

                # With an inverted threshold, we turn black objects white and everything else black
                _, thresh = cv2.threshold(
                    blurred, self.line_threshold, 255, cv2.THRESH_BINARY_INV)

                # We find the contours of the lines
                # We use the try/except block to account for different versions of OpenCV
                try:
                    _, cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
                except:
                    cnts, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)

                # Filter out small contours
                cnts = [c for c in cnts if cv2.contourArea(c) > 500]

                # Get all the centroids of the contours and draw them
                centers = []
                for c in cnts:
                    M = cv2.moments(c)
                    if M["m00"] != 0:
                        centers.append(
                            (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                    else:
                        centers.append((0, 0))


                # Turn thresh into color image
                thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                # Draw all the contours and centroids
                cv2.drawContours(thresh, cnts, -1, (0, 255, 0), 2)
                for center in centers:
                    cv2.circle(thresh, center, 5, (255, 0, 0), -1)

                # Find the centroid that is closest to the last centroid
                min_dist = 100000
                for center in centers:
                    dist = np.linalg.norm(
                        np.array(last_centroid) - np.array(center))
                    if dist < min_dist:
                        min_dist = dist
                        min_center = center
                last_centroid = min_center

                # Highlight the closest centroid
                cv2.circle(thresh, min_center, 5, (0, 0, 255), -1)

                #cv2.imshow("output", thresh)

                # Public the processed image to be able to view it in rqt
                image_topic = self.bridge_object.cv2_to_imgmsg(
                    thresh, encoding="bgr8")
                self.image_pub.publish(image_topic)

                # Publish the centroid, relative to the center of the image
                self.line_centroid_pub.publish(min_center[0] - 250)

                self.image_received_flag = 0

            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def threshold_callback(self, data):
        self.line_threshold = data.data

    def cleanup(self):
        print("Shutting down vision node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    print("Running line detector")
    LineDetector()
