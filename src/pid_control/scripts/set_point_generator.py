#!/usr/bin/env python
import rospy
import numpy as np
from math import sin
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)



#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")


if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("setPoint")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    pub = rospy.Publisher("/set_point", set_point, queue_size=10)
    time = rospy.get_time()

    print("The Set Point Genertor is Running")
    t = 0
    #Run the node
    while not rospy.is_shutdown():
        output = sin(t)*rospy.get_param("/setpoint_amplitude")
        
        t+=0.01
        
        pub.publish(output,rospy.get_time() - time)

        rate.sleep()