#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def stop():
    print("Stopping...")


x_joystick = 0.0
y_joystick = 0.0
right_x_joystick = 0.0


def callback_joystick(stick):
    global x_joystick, y_joystick, right_x_joystick
    x_joystick = stick.axes[0]
    if abs(x_joystick) < 0.2:
        x_joystick = 0.0
    y_joystick = stick.axes[1]
    if abs(y_joystick) < 0.2:
        y_joystick = 0.0

    right_x_joystick = stick.axes[3]
    if abs(right_x_joystick) < 0.2:
        right_x_joystick = 0.0


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("joystick")
    rospy.on_shutdown(stop)
    rate = rospy.Rate(50)

    max_linear_vel = rospy.get_param("/linear_controller_max_speed", 0.5)

    max_angular_vel = rospy.get_param("/angular_controller_max_speed", 1)

    rospy.Subscriber("/joy", Joy, callback_joystick)

    # Publishers
    linear_pub = rospy.Publisher("/linear_vel", Float32, queue_size=10)
    angular_pub = rospy.Publisher("/angular_vel", Float32, queue_size=10)

    while not rospy.is_shutdown():
        linear_pub.publish(y_joystick*max_linear_vel)
        angular_pub.publish(x_joystick*max_angular_vel/5 +
                            right_x_joystick*max_angular_vel)
        rate.sleep()
