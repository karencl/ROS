#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

e = [0, 0, 0]
u = [0, 0]

centroid = 0

def stop():
    print("Stopping...")


def callback_centroid(cent):
    global centroid
    centroid = cent.data


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("controller")
    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    kP = rospy.get_param("/controller_kP", 0)
    kI = rospy.get_param("/controller_kI", 0)
    kD = rospy.get_param("/controller_kD", 0)
    Ts = rospy.get_param("/controller_Ts", 0.02)

    K1 = kP + Ts * kI + kD / Ts
    K2 = -kP - 2.0 * kD / Ts
    K3 = kD / Ts

    max_speed = rospy.get_param("/controller_max_speed", 0)

    # Subscribers
    rospy.Subscriber("/line_centroid", Float32, callback_centroid)

    # Publishers
    pub = rospy.Publisher("/angular_vel", Float32, queue_size=10)

    while not rospy.is_shutdown():


        e[0] = -centroid

        u[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + u[1]

        e[2] = e[1]
        e[1] = e[0]
        u[1] = u[0]

        result = np.clip(u[0], -max_speed, max_speed)
        pub.publish(result)
        rate.sleep()
