#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

distance = 0.0
x_pose = 0.0
y_pose = 0.0
x_goal = 0.0
y_goal = 0.0
z_orientation = 0.0

e = [0, 0, 0]
u = [0, 0]


def stop():
    print("Stopping...")


def callback_robot_pose(r_pose):
    global x_pose, y_pose, z_orientation
    x_pose = r_pose.position.x
    y_pose = r_pose.position.y
    z_orientation = r_pose.orientation.z


def callback_goal(r_goal):
    global x_goal, y_goal
    x_goal = r_goal.position.x
    y_goal = r_goal.position.y


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("controller")
    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    # linear or angular
    mode = rospy.get_name().split("_")[0][1:]

    kP = rospy.get_param("/"+mode+"_controller_kP", 0)
    kI = rospy.get_param("/"+mode+"_controller_kI", 0)
    kD = rospy.get_param("/"+mode+"_controller_kD", 0)
    Ts = rospy.get_param("/controller_Ts", 0.02)

    K1 = kP + Ts * kI + kD / Ts
    K2 = -kP - 2.0 * kD / Ts
    K3 = kD / Ts

    max_speed = rospy.get_param("/"+mode+"_controller_max_speed", 0)

    # Subscribers
    rospy.Subscriber("/robot_pose", Pose, callback_robot_pose)
    rospy.Subscriber("/goal", Pose, callback_goal)

    # Publishers
    pub = rospy.Publisher("/"+mode+"_vel", Float32, queue_size=10)

    while not rospy.is_shutdown():

        dx = x_goal - x_pose
        dy = y_goal - y_pose

        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) - z_orientation
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        e[0] = distance if mode == "linear" else angle

        u[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + u[1]

        e[2] = e[1]
        e[1] = e[0]
        u[1] = u[0]

        result = np.clip(u[0], -max_speed, max_speed)
        pub.publish(result)
        rate.sleep()
