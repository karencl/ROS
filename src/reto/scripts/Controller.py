#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from reto.msg import set_point

#from pid_control.msg import setpoint

#Setup parameters, vriables and callback functions here (if required)

motor_output = 0
setpoint = 0
mode = 0

TS = rospy.get_param("/controller_Ts",0)
KP_STEP = rospy.get_param("/controller_kP_step",0)
KP_STEP = rospy.get_param("/controller_kP_step",0)
KI_STEP = rospy.get_param("/controller_kI_step",0)
KD_STEP = rospy.get_param("/controller_kD_step",0)

KP_SINE = rospy.get_param("/controller_kP_sine",0)
KI_SINE = rospy.get_param("/controller_kI_sine",0)
KD_SINE = rospy.get_param("/controller_kD_sine",0)

KP_SQUARE = rospy.get_param("/controller_kP_square",0)
KI_SQUARE = rospy.get_param("/controller_kI_square",0)
KD_SQUARE = rospy.get_param("/controller_kD_square",0)

e = [0,0,0]
u = [0,0]

#Stop Condition
def stop():
    #Setup the stop message (can be the same as the control message)
    print("Stopping")

def callback_motor_output(msg_motor_output):
    rospy.loginfo("RPS: {}".format(msg_motor_output.data))
    global motor_output
    motor_output = msg_motor_output.data

def callback_setpoint(msg_setpoint):
    rospy.loginfo("Setpoint: {}".format(msg_setpoint.value))
    global setpoint
    global mode
    setpoint = msg_setpoint.value
    mode =  msg_setpoint.mode

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(50)
    rospy.on_shutdown(stop)
    rospy.Subscriber("/motor_output", Float32, callback_motor_output)
    rospy.Subscriber("/set_point", set_point, callback_setpoint)
    pub = rospy.Publisher("/motor_input", Float32, queue_size=10)

    print("The Controller is Running")

    #Run the node
    while not rospy.is_shutdown():

        kP,kI,kD = [(KP_STEP, KI_STEP, KD_STEP),
                    (KP_SINE, KI_SINE, KD_SINE),
                    (KP_SQUARE, KI_SQUARE, KD_SQUARE)][mode]

        K1 = kP + TS * kI + kD / TS
        K2 = -kP - 2.0 * kD / TS
        K3 = kD / TS

        vel = motor_output

        e[0] = setpoint - vel

        u[0] = K1 * e[0] + K2 * e[1] + K3 * e[2] + u[1]

        e[2] = e[1]
        e[1] = e[0]
        u[1] = u[0]

        result = u[0]

        result = np.clip(result, -1, 1)

        pub.publish(result)
        rate.sleep()
