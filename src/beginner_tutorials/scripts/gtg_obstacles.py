#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan  # Lidar
import numpy as np


class Robot():

    # This class implements the differential drive model of the robot
    def __init__(self):

        ############ ROBOT CONSTANTS ################
        self.r = 0.05  # wheel radius [m]
        self.L = 0.19  # wheel separation [m]

        ############ Variables ###############
        self.x = 0.0  # x position of the robot [m]
        self.y = 0.0  # y position of the robot [m]
        self.theta = 0.0  # angle of the robot [rad]

    def update_state(self, wr, wl, delta_t):

        # This function updates the robot's state
        # This functions receives the wheel speeds wr and wl in [rad/sec]
        # and returns the robot's state

        v = self.r*(wr+wl)/2
        w = self.r*(wr-wl)/self.L

        self.theta = self.theta + w*delta_t

        # Crop theta_r from -pi to pi
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))
        vx = v*np.cos(self.theta)
        vy = v*np.sin(self.theta)
        self.x = self.x+vx*delta_t
        self.y = self.y+vy*delta_t

# This class will make the puzzlebot move to a given goal


class AutonomousNav():

    def __init__(self):

        rospy.on_shutdown(self.cleanup)
        self.robot = Robot()  # create an object of the Robot class

        ############ Variables ###############
        self.x_target = 4.0  # x position of the goal
        self.y_target = 0.0  # y position of the goal

        self.goal_received = 1  # flag to indicate if the goal has been received
        self.lidar_received = 0  # flag to indicate if the laser scan has been received

        # acceptable distance to the goal to declare the robot has arrived to it [m]
        self.target_position_tolerance = 0.10
        # distance from closest obstacle to activate the avoid obstacle behavior [m]
        ao_distance = 1.0
        # distance from closest obstacle to stop the robot [m]
        stop_distance = 0.2

        eps = 0.25  # Fat guard epsilon
        v_msg = Twist()  # Robot's desired speed
        self.wr = 0  # right wheel speed [rad/s]
        self.wl = 0  # left wheel speed [rad/s]
        self.current_state = 'GoToGoal'  # Robot's current state

        ### ******* INIT PUBLISHERS *******###

        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        ############################### SUBSCRIBERS #####################################

        rospy.Subscriber("wl", Float32, self.wl_cb)
        rospy.Subscriber("wr", Float32, self.wr_cb)
        rospy.Subscriber("move_base_simple/goal", PoseStamped, self.goal_cb)
        rospy.Subscriber("base_scan", LaserScan, self.laser_cb)

        # ********** INIT NODE **********###

        freq = 20

        rate = rospy.Rate(freq)  # freq Hz

        # Dt is the time between one calculation and the next one
        Dt = 1.0/float(freq)

        ################ MAIN LOOP ################

        while not rospy.is_shutdown():

            # update the robot's state
            self.robot.update_state(self.wr, self.wl, Dt)

            if self.lidar_received:
                closest_range, closest_angle = self.get_closest_object(
                    self.lidar_msg)  # get the closest object range and angle

                if self.current_state == 'Stop':
                    if self.goal_received:
                        print("Going to goal")
                        self.current_state = "GoToGoal"
                    v_msg.linear.x = 0
                    v_msg.angular.z = 0

                elif self.current_state == 'GoToGoal':

                    if self.at_goal():
                        print("At goal")
                        self.current_state = "Stop"
                        self.goal_received = 0

                    elif closest_range < stop_distance:
                        print("Too close")
                        self.current_state = "Stop"

                    elif closest_range < ao_distance:
                        print("Avoid obstacle")
                        self.current_state = "AvoidObstacle"
                    else:
                        v_gtg, w_gtg = self.compute_gtg_control(
                            self.x_target, self.y_target, self.robot.x, self.robot.y, self.robot.theta)
                        v_msg.linear.x = v_gtg
                        v_msg.angular.z = w_gtg

                elif self.current_state == 'AvoidObstacle':
                    if closest_range > ao_distance:
                        self.current_state = "GoToGoal"
                        print("Going to goal")
                    if self.at_goal():
                        print("At goal")
                        self.current_state = "Stop"
                        self.goal_received = 0

                    elif closest_range < stop_distance:
                        print("Too close")
                        self.current_state = "Stop"
                    else:
                        v_ao, w_ao = self.compute_ao_control(closest_angle)
                        v_msg.linear.x = v_ao
                        v_msg.angular.z = w_ao

            self.pub_cmd_vel.publish(v_msg)
            rate.sleep()

    def at_goal(self):
        # This function returns true if the robot is close enough to the goal
        # This functions receives the goal's position and returns a boolean
        # This functions returns a boolean
        return np.sqrt((self.x_target-self.robot.x)**2+(self.y_target-self.robot.y)**2) < self.target_position_tolerance

    def get_closest_object(self, lidar_msg):
        # This function returns the closest object to the robot
        # This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
        # returns  closest_range [m], closest_angle [rad],
        new_angle_min = -np.pi/2.0
        ranges_size = len(lidar_msg.ranges)
        cropped_ranges = lidar_msg.ranges[ranges_size/4:3*ranges_size/4]
        min_idx = np.argmin(cropped_ranges)
        closest_range = cropped_ranges[min_idx]
        closest_angle = new_angle_min + min_idx * lidar_msg.angle_increment
        # limit the angle to [-pi, pi]
        closest_angle = np.arctan2(
            np.sin(closest_angle), np.cos(closest_angle))
        return closest_range, closest_angle

    def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
        # This function returns the linear and angular speed to reach a given goal
        # This functions receives the goal's position (x_target, y_target) [m]
        #  and robot's position (x_robot, y_robot, theta_robot) [m, rad]
        # This functions returns the robot's speed (v, w) [m/s] and [rad/s]
        kvmax = 0.3  # linear speed maximum gain
        kwmax = 1.0  # angular angular speed maximum gain
        # kw=0.5
        av = 2.0  # Constant to adjust the exponential's growth rate
        aw = 2.0  # Constant to adjust the exponential's growth rate
        ed = np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2)
        # Compute angle to the target position
        theta_target = np.arctan2(y_target-y_robot, x_target-x_robot)
        e_theta = theta_target-theta_robot
        # limit e_theta from -pi to pi
        # This part is very important to avoid abrupt changes when error switches between 0 and +-2pi
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
        # Compute the robot's angular speed
        kw = kwmax*(1-np.exp(-aw*e_theta**2)) / \
            abs(e_theta)  # Constant to change the speed
        w = kw*e_theta
        if abs(e_theta) > np.pi/8:
            # we first turn to the goal
            v = 0  # linear speed
        else:
            # Make the linear speed gain proportional to the distance to the target position
            # Constant to change the speed
            kv = kvmax*(1-np.exp(-av*ed**2))/abs(ed)
            v = kv*ed  # linear speed
        return v, w

    def compute_ao_control(self, closest_angle):
        # This function computes the linear and angular speeds for the robot
        # given the distance and the angle theta as error references
        v_desired = 0.3  # [m/s] desired speed when there are no obstacles
        kv = 0.2  # Angular speed gain
        kw = 0.3  # Angular speed gain
        thetaAO = closest_angle-np.pi
        # limit the angle to [-pi,pi]
        thetaAO = np.arctan2(np.sin(thetaAO), np.cos(thetaAO))
        v_ao = v_desired
        w_ao = kw*thetaAO
        return v_ao, w_ao

    def laser_cb(self, msg):
        # This function receives a message of type LaserScan
        self.lidar_msg = msg
        self.lidar_received = 1

    def wl_cb(self, wl):

        # This function receives a the left wheel speed [rad/s]

        self.wl = wl.data

    def wr_cb(self, wr):

        # This function receives a the right wheel speed.

        self.wr = wr.data

    def goal_cb(self, goal):

        # This function receives a the goal from rviz.

        print("Goal received")

        # assign the goal position
        self.x_target = goal.pose.position.x
        self.y_target = goal.pose.position.y
        self.goal_received = 1

    def cleanup(self):

        # This function is called just before finishing the node
        # You can use it to clean things up before leaving
        # Example: stop the robot before finishing a node.

        vel_msg = Twist()
        self.pub_cmd_vel.publish(vel_msg)


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)

    AutonomousNav()
