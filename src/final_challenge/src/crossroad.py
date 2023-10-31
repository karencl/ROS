#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import the necessary packages
import cv2
import sys
import numpy as np
import itertools

def find_lines(coordinates, min_length = 4, line_threshold = 1, slope_threshold = 0.1, intercept_threshold = 5):
    """
        Gets all the lines of length min_length or greater from the given coordinates using linear regression.
    """

    # Gets all combinations of coordinates of length min_length
    combinations = list(itertools.combinations(coordinates, min_length))

    # Linear regression of degree 1 for each combination
    lines = []
    for combination in combinations:
        x = [c[0] for c in combination]
        y = [c[1] for c in combination]
        z = np.polyfit(x, y, 1)


        # Get the average error of the line
        error = 0
        for i in range(len(x)):
            error += abs(y[i] - (z[0] * x[i] + z[1]))
        error /= len(x)

        # If the error is small enough, we consider it a line
        if error < line_threshold:
            lines.append((z[0], z[1]))

    # Combine lines with similar slopes and intercepts
    groups = []
    for line in lines:
        slope = line[0]
        intercept = line[1]
        found = False
        for group in groups:
            if abs(slope - group[0]) < slope_threshold and abs(intercept - group[1]) < intercept_threshold:
                group[0] = (group[0] + slope) / 2
                group[1] = (group[1] + intercept) / 2
                found = True
                break
        if not found:
            groups.append([slope, intercept])

    # Get the coordinates of the lines
    lines = []
    for group in groups:
        x1 = 0
        y1 = int(group[1])
        x2 = 500
        y2 = int(group[0] * x2 + group[1])
        lines.append((x1, y1, x2, y2, group[0], group[1]))

    return lines

class CrossroadDetector():
    def __init__(self):

        self.system = sys.argv[1]

        self.bridge_object = CvBridge()

        self.image_received_flag = 0

        # The value for the threshold that will highlight black lines
        self.line_threshold = rospy.get_param("/{}_line_threshold".format(self.system), 100)

        rospy.on_shutdown(self.cleanup)

        self.image_pub = rospy.Publisher(
            "/processed_image_crossroad", Image, queue_size=1)
        self.crossroad_pub = rospy.Publisher(
            "/crossroad", Int32, queue_size=10)

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

        WIDTH = 500

        KERNEL = np.ones((5, 5), np.uint8)

        while not rospy.is_shutdown():

            if self.image_received_flag == 1:

                resized = cv2.resize(self.frame, (WIDTH, WIDTH))

                # We get the lower third of the image to remove unnecessary information
                resized = resized[WIDTH/2:WIDTH, :WIDTH]

                # Convert to grayscale
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

                # Gaussian blurr
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                # Dilate and erode
                blurred = cv2.erode(cv2.dilate(
                    blurred, KERNEL, iterations=2), KERNEL, iterations=2)

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


                # Turn thresh into color image
                thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)

                # Filter out small contours
                cnts = [c for c in cnts if cv2.contourArea(
                    c) > 400]

                # filter out tall or wide contours
                cnts = [c for c in cnts if
                    cv2.boundingRect(c)[3] < 100 and cv2.boundingRect(c)[2] < 150]

                # get only rectangles
                cnts = [c for c in cnts if cv2.boundingRect(c)[2] > 0.5 * cv2.boundingRect(c)[3]]

                vertical_pos = -1

                # draw contours
                cv2.drawContours(thresh, cnts, -1, (0, 255, 0), 1)

                if len(cnts) >= 4:

                    # Get all the centroids of the contours and draw them
                    centers = []
                    for c in cnts:
                        M = cv2.moments(c)
                        if M["m00"] != 0:
                            centers.append(
                                (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])))
                        else:
                            centers.append((0, 0))

                    # Find all lines of 4 or more centroids
                    groups = find_lines(centers, min_length=4, line_threshold=5, slope_threshold=10, intercept_threshold=10)

                    if len(groups):

                        # Get the lowest line
                        lowest = sorted(groups, key=lambda x: x[1], reverse=True)[0]
                        x1, y1, x2, y2, _, _ = lowest
                        cv2.line(thresh, (x1, y1), (x2, y2), (0, 0, 255), 4)
                        vertical_pos = int((y1 + y2) / 2)

                        cv2.putText(thresh, str(vertical_pos), (vertical_pos, 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


                # Public the processed image to be able to view it in rqt
                image_topic = self.bridge_object.cv2_to_imgmsg(
                    thresh, encoding="bgr8")
                self.image_pub.publish(image_topic)

                self.crossroad_pub.publish(vertical_pos)

                self.image_received_flag = 0

            key = cv2.waitKey(1) & 0xFF
            ros_rate.sleep()

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print("[CROSSROAD]",e)

    def threshold_callback(self, data):
        self.line_threshold = data.data

    def cleanup(self):
        print("[CROSSROAD] Shutting down crossroad detector")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('crossroad_detector', anonymous=True)
    print("Running crossroad detector")
    CrossroadDetector()
