#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
from wang_mendel import wang_mendel


def stop():
    print("Stopping...")


x_pose = 0
y_pose = 0
z_orientation = 0
x_goal = 0
y_goal = 0


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

    max_speed = rospy.get_param("/"+mode+"_controller_max_speed", 0)

    # Subscribers
    rospy.Subscriber("/robot_pose", Pose, callback_robot_pose)
    rospy.Subscriber("/goal", Pose, callback_goal)

    # Publishers
    pub = rospy.Publisher("/"+mode+"_vel", Float32, queue_size=10)

    antecedents = rospy.get_param(
        "/"+mode+"_controller_antecedents", [0, 0, 0])

    distance_range = rospy.get_param("/distance_range", [0, 0])

    angle_range = [-np.pi, np.pi]

    output_range = rospy.get_param("/"+mode+"_output_range", [0, 0])

    rules = np.load(
        "/home/estebanp/catkin_ws/src/fuzzy_path_follower/src/"+mode+"_rules.npy")

    controller = wang_mendel(antecedents,
                             distance_range,
                             angle_range,
                             output_range,
                             rules)

    while not rospy.is_shutdown():

        dx = x_goal - x_pose
        dy = y_goal - y_pose

        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) - z_orientation
        angle = np.arctan2(np.sin(angle), np.cos(angle))

        result = controller.get_output(distance, angle)
        pub.publish(result)
        rate.sleep()
