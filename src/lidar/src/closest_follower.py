#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class LaserSubClass():

    SAFETY_DISTANCE = 0.20
    DISTANCE_GAIN = 0.5
    ANGLE_GAIN = 1

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("base_scan", LaserScan, self.lidar_cb)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.lidar = LaserScan()

        r = rospy.Rate(20)

        print("Node initialized 20HZ")

        while not rospy.is_shutdown():
            if (self.lidar.ranges):

                distance = min(self.lidar.ranges)

                if (np.isinf(distance)):
                    print("No object")
                    self.publish_vel(0, 0)
                    continue

                index = self.lidar.ranges.index(distance)

                step = self.lidar.angle_increment
                angle = self.lidar.angle_min + (index * step)

                theta = np.arctan2(
                    np.sin(angle), np.cos(angle))

                print("{:.2f}".format(distance) + "m " +
                      "{:.2f}".format(theta))

                linear = np.clip(distance*self.DISTANCE_GAIN, -0.7, 0.7)
                angular = np.clip(theta*self.ANGLE_GAIN, -3, 3)

                if (distance < self.SAFETY_DISTANCE):
                    self.publish_vel(0, angular)
                else:
                    self.publish_vel(linear, angular)

            else:
                print("No object")
                self.publish_vel(0, 0)

            r.sleep()

    def publish_vel(self, linear, angular):
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.cmd_vel_pub.publish(vel)

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg

    def cleanup(self):
        print("I'm dying, bye bye!!!")
        self.publish_vel(0, 0)


if __name__ == "__main__":
    rospy.init_node("laser_scan_subscriber", anonymous=True)
    LaserSubClass()
