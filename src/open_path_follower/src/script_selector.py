#!/usr/bin/env python

from open_path_follower.msg import script_select
import numpy as np
import rospy

# Stop Condition


def stop():
 # Setup the stop message (can be the same as the control message)
    print("Stopping")


if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("script_selector")
    rate = rospy.Rate(50)
    rospy.on_shutdown(stop)
    pub = rospy.Publisher("/script_select", script_select, queue_size=1)
    time = rospy.get_time()

    while not rospy.is_shutdown():

        script_select = input("Enter script type: \n 1: Square \n 2: Path \n")

        while script_select != 1 and script_select != 2:
            script_select = input(
                "That is not an option. \n\n Enter script type: \n 1: Square \n 2: Path \n")

        if script_select == 1:
            script_select = "SQUARE"
        elif script_select == 2:
            script_select = "PATH"

        type_select = input("Enter type: \n 1: Velocity \n 2: Time \n")

        while type_select != 1 and type_select != 2:
            type_select = input(
                "That is not an option. \n\n Enter type: \n 1: Velocity \n 2: Time \n"
            )

        if type_select == 1:
            type_select = "VELOCITY"
        elif type_select == 2:
            type_select = "TIME"

        total_time = input(
            "Enter total time in seconds: ") if type_select == "TIME" else 0

        turn_velocity = input(
            "Enter turn velocity in r/s: ") if type_select == "VELOCITY" else 0

        move_velocity = input(
            "Enter move velocity in m/s: ") if type_select == "VELOCITY" else 0

        square_length = input(
            "Enter square length in meters: ") if script_select == "SQUARE" else 0

        pub.publish(script_select, type_select, turn_velocity,
                    move_velocity, total_time, square_length)

        rate.sleep()
