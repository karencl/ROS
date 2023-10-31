#!/usr/bin/env python

from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy
from time import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class Robot():

    LINEAR_VELOCITY = 0.5
    def __init__(self):

        self.linear_vel = 0
        self.angular_vel = 0

        self.sign_class = "chess"

        self.photo_number = 99

        self.frame = None

        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/camera/image_raw", Image, self.camera_callback)
        self.image_sub_jetson = rospy.Subscriber(
            "/video_source/raw", Image, self.camera_callback)
        self.image_sub_jetson_launch = rospy.Subscriber(
            "/camera/video_source/raw", Image, self.camera_callback)

        self.bridge_object = CvBridge()

        # This flag is to ensure we received at least one self.frame
        self.image_received_flag = 0

        start_time = rospy.get_time()

        while not rospy.is_shutdown():

            if rospy.get_time() - start_time > 2:
                print("Taking picture")
                self.take_pictures()
                start_time = rospy.get_time()

            self.rate.sleep()

    def take_pictures(self):
        if self.image_received_flag == 1 and self.frame is not None:
            # Save image
            cv2.imwrite(
                "/home/puzzlebot/catkin_ws/src/final_challenge/images/{0}_{1}.jpg".format(self.sign_class, self.photo_number), self.frame)
            self.photo_number += 1
            self.image_received_flag = 0

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = np.clip(angular, -5, 5)
        # self.cmd_vel_pub.publish(vel)

    def camera_callback(self, data):
        try:
            self.frame = self.bridge_object.imgmsg_to_cv2(
                data, desired_encoding="bgr8")
            self.image_received_flag = 1
        except CvBridgeError as e:
            print(e)

    def cleanup(self):
        """
            Stops the robot when the program is interrupted.
        """
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

        print("My battery is low and it's getting dark")




############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("final_challenge", anonymous=True)
    Robot()
