#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool


class PathGenerator():
    def __init__(self):

        # Initialize ROS node
        rospy.on_shutdown(self.cleanup)
        r = rospy.Rate(50)  # 50 Hz

        # Publishers and subscribers
        self.path_goal = rospy.Publisher('/goal', Pose, queue_size=1)
        rospy.Subscriber('/reached_goal', Bool, self.bandera_cb)

        goal = Pose()
        path = rospy.get_param("/path")
        i = 0
        self.flag = True

        while not rospy.is_shutdown():

            if i >= len(path):
                print("Path completed")
                break

            if (self.flag == True):
                goal.position.x = path[i][0]
                goal.position.y = path[i][1]
                i += 1
                self.flag = False
                print("Goal: ", goal.position.x, goal.position.y)

            self.path_goal.publish(goal)
            r.sleep()

    def bandera_cb(self, msg):
        self.flag = msg.data

    def cleanup(self):
        print("Path generator node terminated.")


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("path_generator", anonymous=True)
    PathGenerator()
