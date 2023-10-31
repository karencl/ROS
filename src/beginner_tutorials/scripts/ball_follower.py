#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Int32


class BallFollowerClass():

    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("radius", Int32, self.radius_cb)
        rospy.Subscriber("center", Point, self.center_cb)

        vel = Twist()
        kv = 5.0
        kw = 0.002
        self.radius = 0.0
        self.xc = 0.0

        r = rospy.Rate(10)

        print("Node initialized")
        while not rospy.is_shutdown():
            if self.radius > 0 and self.radius < 200:
                vel.linear.x = kv*(1.0/self.radius)
            else:
                vel.linear.x = 0.0

            if self.radius == 0:
                vel.angular.z = 1
            else:
                vel.angular.z = kw*(300-self.xc)

            self.cmd_vel_pub.publish(vel)
            r.sleep()

    def radius_cb(self, msg):
        self.radius = msg.data

    def center_cb(self, msg):
        self.xc = msg.x  # The x coordinate of the object's center

    def cleanup(self):
        print("I'm dying, bye bye!!!")
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)


if __name__ == "__main__":
    rospy.init_node("ball_follower", anonymous=True)
    BallFollowerClass()
