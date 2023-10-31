#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


# This class implements a simple obstacle avoidance algorithm
class AvoidObstacleClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.closest_angle = 0.0
        self.closest_range = np.inf
        robot_vel = Twist()
        robot_vel.linear.x = 0.5
        robot_vel.angular.z = 0.0

        self.dAO = 1
        self.min_obstacle_range = 0.15

        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            #print("closest object distance: " + str(self.closest_range) +
                #  " || theta_closest: " + str(self.closest_angle))

            if self.closest_range <= self.dAO and self.closest_range > self.min_obstacle_range:
                robot_vel.linear.x, robot_vel.angular.z = self.avoid_obstacles_controller()
            elif self.closest_range <= self.min_obstacle_range:
                robot_vel.linear.x = 0.0
                robot_vel.angular.z = 0.0
            else:
                robot_vel.linear.x = 0.5
                robot_vel.angular.z = 0.0

            self.cmd_vel_pub.publish(robot_vel)
            r.sleep()

    def avoid_obstacles_controller(self):

        theta_AO = self.closest_angle + np.pi/2

        theta_AO = np.arctan2(np.sin(theta_AO), np.cos(theta_AO))
        v = 0.1
        kAO = 3
        w = kAO * theta_AO
        return v, w

    def laser_cb(self, msg):

        # Ignore everything behind the robot
        # Which is the first and last quarters of the scan array

        if msg.ranges:
            self.closest_range = min(msg.ranges)
            idx = msg.ranges.index(self.closest_range)
            self.closest_angle = msg.angle_min + idx * msg.angle_increment
            self.closest_angle = np.arctan2(
                np.sin(self.closest_angle), np.cos(self.closest_angle))

    def cleanup(self):

        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("avoid_obstacle", anonymous=True)
    AvoidObstacleClass()
