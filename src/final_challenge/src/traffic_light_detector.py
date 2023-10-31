#!/usr/bin/env python

# import ROS stuff
import rospy
from std_msgs.msg import Float32MultiArray
from final_challenge.msg import detected_object
import cv2
import rospy
import numpy as np

class TrafficLightDetector():



    def __init__(self):

        self.MIN_DETECTION_AREA = rospy.get_param("/light_detection_area", 700)

        rospy.on_shutdown(self.cleanup)

        self.traffic_light_pub = rospy.Publisher(
            "/traffic_light", detected_object, queue_size=1)
        self.yolo_sub = rospy.Subscriber("/yolo_results", Float32MultiArray, self.yolo_cb)

        self.yolo_matrix = []
        ros_rate = rospy.Rate(50)

        categories = [ "green", "red", "yellow", "none"]

        while not rospy.is_shutdown():

            min_confidence = 0
            best_area = 0
            best_light = None

            if len(self.yolo_matrix):

                for light in self.yolo_matrix:

                    width = light[2] - light[0]
                    height = light[3] - light[1]
                    confidence = float(light[4])

                    # Ignore signs
                    if int(light[5]) < 6:
                        continue

                    # Ignore unsure reds and yellows
                    if int(light[5]) != 6 and confidence < 0.6:
                        continue


                    category = categories[int(light[5] - 6)]

                    area = width * height

                    if area > self.MIN_DETECTION_AREA and confidence > min_confidence:
                        min_confidence = confidence
                        best_light = category
                        best_area = area

            self.traffic_light_pub.publish(best_light, best_area)

            ros_rate.sleep()

    def yolo_cb(self, data):
        flat = data.data
        self.yolo_matrix = np.reshape(flat, (-1, 6))

    def cleanup(self):
        print("[TRAFFIC] Shutting down traffic node")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('light_detector', anonymous=True)
    print("[TRAFFIC] Running traffic light detector")
    TrafficLightDetector()
