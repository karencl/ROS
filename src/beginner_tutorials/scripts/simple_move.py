#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

from std_msgs.msg import Float32


class SimpleMove():

    VELOCITY_MS = 0.5

    target_distance = 0

    init_time = 0

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        rospy.Subscriber("distance_topic", Float32, self.message_cb)

        vel = Twist()

        r = rospy.Rate(2)

        print("Running...")

        while not rospy.is_shutdown():

            direction = self.target_distance / abs(self.target_distance or 1)

            current_distance = (
                rospy.get_time() - self.init_time) * self.VELOCITY_MS * direction

            if abs(current_distance) < abs(self.target_distance):

                vel.linear.x = self.VELOCITY_MS * direction
                vel.angular.z = 0.0

            else:
                vel.linear.x = 0.0
                vel.angular.z = 0.0

            self.cmd_vel_pub.publish(vel)  # publish the message

            r.sleep()

    def message_cb(self, msg):

        self.target_distance = msg.data

        self.init_time = rospy.get_time()

        print("I received this message in the callback: " +
              str(self.target_distance))

    def cleanup(self):
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        print("I'm dying, bye bye!!!")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":

    rospy.init_node("simple_move", anonymous=True)
    SimpleMove()
