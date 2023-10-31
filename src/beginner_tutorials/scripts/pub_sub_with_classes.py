#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

from std_msgs.msg import String

# This class will subscribe to the /message topic and publish to the /cmd_vel topic


class PubSubClass():

    def __init__(self):

        # This function will be called before killing the node.
        rospy.on_shutdown(self.cleanup)

        ######### PUBLISHERS AND SUBSCRIBERS #################

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("string_topic", String, self.message_cb)

        ############ CONSTANTS AND VARIABLES ################

        vel = Twist()

        self.my_string = "stop"

        r = rospy.Rate(2)  # 2 Hz

        while not rospy.is_shutdown():

            if self.my_string == "move forward":

                print("moving forward")

                vel.linear.x = 0.1

                vel.angular.z = 0.0

            elif self.my_string == "move back":

                print("moving back")

                vel.linear.x = -0.1

                vel.angular.z = 0.0

            elif self.my_string == "turn right":

                print("turning right")

                vel.linear.x = 0.0

                vel.angular.z = 0.1

            elif self.my_string == "turn left":

                print("turning left")

                vel.linear.x = 0.0

                vel.angular.z = -0.1

            else:  # stop

                print("stopped")

                vel.linear.x = 0.0

                vel.angular.z = 0.0

            self.cmd_vel_pub.publish(vel)  # publish the message

            # It is very important that the r.sleep function is called at least onece every cycle.
            r.sleep()

    def message_cb(self, msg):

        # This function receives the ROS message as the msg variable.

        self.my_string = msg.data  # msg.data is the string contained inside the ROS message

        print("I received this message in the callback: " + self.my_string)

    def cleanup(self):

        # This function is called just before finishing the node

        # You can use it to clean things up before leaving

        # Example: stop the robot before finishing a node.

        print("I'm dying, bye bye!!!")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("pub_sub_with_classes", anonymous=True)

    PubSubClass()
