#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)

m_motor_output = 0
m_setpoint = 0
m_time = 0

kP = rospy.get_param("/controller_kP")
kI = rospy.get_param("/controller_kI")
kD = rospy.get_param("/controller_kD")
Ts = rospy.get_param("/controller_Ts")

e = [0,0,0]
u = [0,0]

K1 = kP + Ts * kI + kD / Ts;
K2 = -kP - 2.0 * kD / Ts;
K3 = kD / Ts;


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

def callback_motor_output(msg_motor_output):
  global m_motor_output, m_time
  m_motor_output = msg_motor_output.output
  m_time = msg_motor_output.time

def callback_setpoint(msg_setpoint):
  global m_setpoint
  m_setpoint = msg_setpoint.value
  

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)
    rospy.Subscriber("/motor_output", motor_output, callback_motor_output)
    rospy.Subscriber("/set_point", set_point, callback_setpoint)
    pub = rospy.Publisher("/motor_input", motor_input, queue_size=10)
    time = rospy.get_time()
    
    #Setup Publishers and subscribers here
    
    print("The Controller is Running")
    
    #Run the node
    while not rospy.is_shutdown():
        # To Do : Generate motor_input
        
        vel = m_motor_output;

        e[0] = m_setpoint - vel;

        u[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + u[1];

        e[2] = e[1];
        e[1] = e[0];
        u[1] = u[0];

        result = u[0]
        
        if result > 1:
          result = 1
        elif result < -1:
          result = -1
        pub.publish(result,rospy.get_time()-time)
        rate.sleep()