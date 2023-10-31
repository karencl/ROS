#!/usr/bin/env python
import rospy
import numpy as np
from reto.msg import set_point

AMPLITUDE = rospy.get_param("/setpoint_amplitude",0)
MAX_AMPLITUDE = rospy.get_param("/setpoint_max_amplitude",0)
PERIOD = rospy.get_param("/setpoint_period",0)
MODE = rospy.get_param("/setpoint_mode",0)


#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("setpoint")
    rate = rospy.Rate(50)
    rospy.on_shutdown(stop)

    pub = rospy.Publisher("/set_point", set_point, queue_size=10)

    print("The Setpoint is Running")

    time = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():

        t = rospy.get_time() - time

        if MODE == 0:
            setpoint = input("Provide speed in Radians per Second from -{0} RPS to {0} RPS: ".format(MAX_AMPLITUDE))
            setpoint = np.clip(setpoint, -MAX_AMPLITUDE, MAX_AMPLITUDE)

        elif MODE == 1:
            setpoint = np.sin(t*2*np.pi/PERIOD)*AMPLITUDE

        elif MODE == 2:
            setpoint = np.sin(t*2*np.pi/PERIOD)*AMPLITUDE
            setpoint = np.sign(setpoint) * AMPLITUDE

        else:
            print("Mode {} is not allowed.".format(MODE))
            break

        pub.publish(setpoint, MODE)
        rospy.loginfo(setpoint)
        rate.sleep()
