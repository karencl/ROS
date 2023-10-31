#!/usr/bin/env python

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist, Pose
import numpy as np
import rospy
import time


class PathFollower():

    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05

    DISTANCE_TOLERANCE = 0.5

    def __init__(self):

        self.path = rospy.get_param("/path")
        self.current_goal = 0

        self.pose = Pose()
        self.goal = Pose()
        self.goal.position.x = self.path[0][0]
        self.goal.position.y = self.path[0][1]

        self.linear_vel = 0
        self.angular_vel = 0
        self.wl = 0
        self.wr = 0

        self.init_time = rospy.get_time()

        # Setup ROS node
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pose_pub = rospy.Publisher(
            'robot_pose', Pose, queue_size=1)
        self.reached_pub = rospy.Publisher(
            'reached_goal', Bool, queue_size=1)
        self.reached_pub.publish(False)

        # Subscribers
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)
        rospy.Subscriber("/linear_vel", Float32, self.linear_vel_cb)
        rospy.Subscriber("/angular_vel", Float32, self.angular_vel_cb)
        rospy.Subscriber("/goal", Pose, self.goal_cb)

        print("Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

        print("Running...")

        print_time = time.time()

        while not rospy.is_shutdown():
            if self.follow_path():
                break
            self.rate.sleep()

    def follow_path(self):
        """
            Updates the robot position, checks if it has reached the goal. If not, it sends the velocities sent by the PID controlers.
        """

        self.update_position()

        dx = self.goal.position.x - self.pose.position.x
        dy = self.goal.position.y - self.pose.position.y

        distance = np.sqrt(dx**2 + dy**2)

        if distance < self.DISTANCE_TOLERANCE:
            self.publish_vel(0, 0)
            self.reached_pub.publish(True)
            self.current_goal += 1
            if self.current_goal < len(self.path):
                coords = self.path[self.current_goal]
                self.goal.position.x = coords[0]
                self.goal.position.y = coords[1]
        else:
            self.publish_vel(self.linear_vel, self.angular_vel)
            self.reached_pub.publish(False)

        return False

    def update_position(self):
        """
            Updates the current heading and position of the robot based on the current wheel velocities.
        """

        dt = rospy.get_time()-self.init_time

        linear_vel = self.WHEEL_RADIUS * (self.wr+self.wl)/2
        angular_vel = self.WHEEL_RADIUS * \
            (self.wr-self.wl)/self.TRACK_WIDTH

        theta = angular_vel * dt

        self.pose.orientation.z += np.arctan2(
            np.sin(theta), np.cos(theta))

        self.pose.position.x += linear_vel * \
            dt * np.cos(self.pose.orientation.z)
        self.pose.position.y += linear_vel * \
            dt * np.sin(self.pose.orientation.z)

        self.init_time = rospy.get_time()

        self.pose_pub.publish(self.pose)

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.cmd_vel_pub.publish(vel)

    def cleanup(self):
        """
            Stops the robot when the program is interrupted.
        """
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)

        print("My battery is low and it's getting dark")

    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def linear_vel_cb(self, msg):
        self.linear_vel = msg.data

    def angular_vel_cb(self, msg):
        self.angular_vel = msg.data

    def goal_cb(self, msg):
        self.goal = msg


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("path_follower", anonymous=True)
    PathFollower()
