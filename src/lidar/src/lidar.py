#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan


class LaserSubClass():

    def __init__(self):

        rospy.on_shutdown(self.cleanup)

        rospy.Subscriber("base_scan", LaserScan, self.lidar_cb)

        self.lidar = LaserScan()

        r = rospy.Rate(1)

        print("Node initialized 1hz")

        while not rospy.is_shutdown():

            # print("ranges:")
            # print(self.lidar.ranges)

            # print("angle_min: " + str(self.lidar.angle_min))
            # print("angle_max: " + str(self.lidar.angle_max))
            # print("range_min: " + str(self.lidar.range_min))
            # print("range_max: " + str(self.lidar.range_max))

            if (self.lidar.ranges):
                # print("First range: " + str(self.lidar.ranges[0]))

                closest = min(self.lidar.ranges)
                index = self.lidar.ranges.index(closest)

                step = self.lidar.angle_increment
                closest_angle = self.lidar.angle_min + (index * step)
                print("Closest range: " + str(closest) +
                      " at angle: " + str(closest_angle))

            if (self.lidar.intensities):
                # print("Last intensity: " + str(self.lidar.intensities[-1]))
                pass

            # print("Header: " + str(self.lidar.header.frame_id))

            r.sleep()

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg

    def cleanup(self):
        print("I'm dying, bye bye!!!")


if __name__ == "__main__":

    rospy.init_node("laser_scan_subscriber", anonymous=True)

    LaserSubClass()
