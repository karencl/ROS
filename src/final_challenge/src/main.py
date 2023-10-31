#!/usr/bin/env python

from std_msgs.msg import Float32, Int32, String, Bool
from geometry_msgs.msg import Twist
from final_challenge.msg import detected_object
import numpy as np
import rospy
import sys

class Robot():

    # Simulation or Jetson
    SYSTEM = sys.argv[1]

    # ------------------- Odometry -------------------
    TRACK_WIDTH = 0.19
    WHEEL_RADIUS = 0.05


    # ------------------- Timeouts -------------------
    STOP_TIME = 10
    GIVE_WAY_TIME = 5
    ROAD_WORK_TIME = 2

    LOGGING = True

    def __init__(self):

        self.FULL_VELOCITY = rospy.get_param("/"+self.SYSTEM+"_linear_velocity", 0.5)

        self.FIRST_DRIVE_DISTANCE = rospy.get_param("/first_drive_distance", 0.2)
        self.SECOND_DRIVE_DISTANCE = rospy.get_param("/second_drive_distance", 0.3)
        self.CROSS_DRIVE_DISTANCE = rospy.get_param("/cross_drive_distance", 0.5)

        # ------------------- Commands -------------------
        self.TURN_RIGHT_CMD = [("drive",
                        self.FIRST_DRIVE_DISTANCE,
                        self.FULL_VELOCITY),

                            ("turn", -np.pi/2, 0.8),

                            ("drive",
                            self.SECOND_DRIVE_DISTANCE,
                            self.FULL_VELOCITY)]


        self.TURN_LEFT_CMD = [("drive",
                            self.FIRST_DRIVE_DISTANCE,
                            self.FULL_VELOCITY),

                            ("turn", np.pi/2, 0.8),

                            ("drive",
                            self.SECOND_DRIVE_DISTANCE,
                            self.FULL_VELOCITY)]

        self.DRIVE_FORWARD_CMD = [("drive",
                                self.CROSS_DRIVE_DISTANCE,
                                self.FULL_VELOCITY)]

        # ------------------- Obey Thresholds -------------------
        self.CROSSROAD_OBEY_THRESHOLD = rospy.get_param("/crossroad_obey_threshold", 80)
        self.SIGN_OBEY_THRESHOLD = rospy.get_param("/sign_obey_threshold", 800)
        self.LIGHT_OBEY_THRESHOLD = rospy.get_param("/light_obey_threshold", 500)

        # YOLO started
        self.yolo_started = False

        # Angular control velocities
        self.line_angular_vel = 0
        self.turn_angular_vel = 0

        # Crossroad detection
        self.crossroad = -1

        # Road detections
        self.traffic_sign = detected_object("none", 0)
        self.action_sign = detected_object("none", 0)
        self.traffic_light = detected_object("none", 0)
        self.last_sign = detected_object("none", 0)

        # Odometry variables
        self.wl = 0
        self.wr = 0
        self.heading = 0
        self.distance_time = 0
        self.init_time = rospy.get_time()
        self.current_action = 0

        # State machine states
        self.turning = False
        self.driving = False
        self.crossing = False
        self.stopping = False
        self.giving_way = False
        self.road_working = False

        # State machine times
        self.giving_way_time = 0
        self.road_work_time = 0
        self.stopping_time = 0

        self.setup_node()

        raw_input("Press enter to start:")

        self.LOG("Running...")

        while not rospy.is_shutdown():

            if self.crossing:
                self.LOG("Crossing "+self.action_sign.name)
                self.cross_road()

            else:
                # Check if we are close to a crossroad
                self.crossing = self.crossroad > self.CROSSROAD_OBEY_THRESHOLD

                # Get the velocities according to the traffic signs and lights
                (linear_vel, angular_vel) = self.obey_traffic()
                self.publish_vel(linear_vel, angular_vel)

            # Tell the line follower that we are crossing
            # So it can stop tracking the line
            self.crossing_pub.publish(self.crossing)
            self.rate.sleep()


    def setup_node(self):
        """
            Sets up the ROS node, publishers and subscribers.
            Waits for simulation time and YOLO before starting.
        """

        self.rate = rospy.Rate(50)
        rospy.on_shutdown(self.cleanup)
        self.setup_publishers()
        self.setup_subscribers()

        # Setup ROS node
        self.LOG("Waiting for time to be set...")
        while rospy.get_time() == 0:
            pass

    def setup_publishers(self):
        """
            Sets up the publishers for the node.
        """

        self.cmd_vel_pub = rospy.Publisher(
            '/puzzlebot/cmd_vel', Twist, queue_size = 10)
        self.turn_error_pub = rospy.Publisher(
            '/turn_error', Float32, queue_size = 10)
        self.crossing_pub = rospy.Publisher(
            "/crossing", Bool, queue_size = 10)

    def setup_subscribers(self):
        """
            Sets up the subscribers for the node.
        """

        rospy.Subscriber("/puzzlebot/wl", Float32, self.wl_cb)
        rospy.Subscriber("/puzzlebot/wr", Float32, self.wr_cb)
        rospy.Subscriber("/traffic_light", detected_object, self.traffic_light_cb)
        rospy.Subscriber("/line_angular_vel", Float32,
                         self.line_angular_vel_cb)
        rospy.Subscriber("/turn_angular_vel", Float32,
                         self.turn_angular_vel_cb)
        rospy.Subscriber("/crossroad", Int32, self.crossroad_cb)
        rospy.Subscriber("/sign", detected_object, self.traffic_sign_cb)
        rospy.Subscriber("/yolo_started", Bool, self.yolo_started_cb)

    # ---------------------- Control Methods ----------------------
    def cross_road(self):
        """
            Executes a series of actions to cross the crossroad, according to the
            last traffic sign.
        """

        if self.action_sign.name == "left":
            command = self.TURN_LEFT_CMD
        elif self.action_sign.name == "right":
            command = self.TURN_RIGHT_CMD
        else:
            command = self.DRIVE_FORWARD_CMD

        action, target, speed = command[self.current_action]

        if action == "drive":
            if self.drive(target, speed):
                self.current_action += 1
        elif action == "turn":
            if self.turn(target, speed):
                self.current_action += 1

        if self.current_action >= len(command):
            self.crossing = False
            self.current_action = 0
            self.LOG("Done crossing")

    def obey_traffic(self):
        """
            Returns the linear and angular velocities according to the traffic
            signs and lights.
        """

        if (self.sign_is("right", min_area = 500)
            or self.sign_is("left", min_area = 500)
            or self.sign_is("forward", min_area = 500)):

            self.action_sign = self.traffic_sign

        if self.light_is("red"):
            self.LOG("Stopped at red")
            return (0, 0)

        if self.light_is("yellow"):
            self.LOG("Slowing at yellow.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        if self.sign_is("stop") and not self.stopping and self.stop_released:
            self.LOG("Stopped at sign.")
            self.stopping_time = rospy.get_time()
            self.stopping = True

        if self.stopping:
            if rospy.get_time() - self.stopping_time > self.STOP_TIME:
                self.stopping = False
                self.stop_released = False
                self.LOG("Stopped finished.")
            return (0, 0)

        if not self.sign_is("stop"):
            self.stop_released = True

        if self.sign_is("give_way") and not self.giving_way:
            self.LOG("Giving way.")
            self.giving_way_time = rospy.get_time()
            self.giving_way = True

        if self.giving_way:
            if rospy.get_time() - self.giving_way_time > self.GIVE_WAY_TIME:
                self.giving_way = False
                self.LOG("Giving way finished.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        if self.sign_is("road_work") and not self.road_working:
            self.LOG("Slowing for work.")
            self.road_work_time = rospy.get_time()
            self.road_working = True

        if self.road_working:
            if rospy.get_time() - self.road_work_time > self.road_working:
                self.road_working = False
                self.LOG("Road work finished.")
            return (self.FULL_VELOCITY / 2, self.line_angular_vel)

        return (self.FULL_VELOCITY, self.line_angular_vel)


    # ---------------------- Commands ----------------------
    def drive(self, target, speed):
        """
            Drives the robot forward by a given distance.
        """

        if not self.driving:
            self.LOG("Driving forward for {}m".format(target))
            self.distance_time = rospy.get_time()
            self.driving = True

        self.publish_vel(speed, 0)
        vel = self.WHEEL_RADIUS * (self.wr+self.wl)/2
        distance = vel * (rospy.get_time() - self.distance_time)

        if distance > target:
            self.driving = False
            return True

        return False

    def turn(self, angle, speed):
        """
            Turns the robot by a given angle.
        """

        if not self.turning:
            self.heading = 0
            self.turning = True
            self.init_time = rospy.get_time()

        self.update_heading()

        error = angle - self.heading
        error = np.arctan2(np.sin(error), np.cos(error))

        self.turn_error_pub.publish(error)
        self.publish_vel(0, self.turn_angular_vel*speed)

        if np.abs(error) < 0.05:
            self.turning = False
            return True

        return False

    # ---------------------- Helper Methods ----------------------
    def LOG(self, msg):
        """
            Prints a tagged message if logging is enabled
        """

        if self.LOGGING:
            print("[MAIN] " + str(msg))

    def cleanup(self):
        """
            Stops the robot when the program is interrupted.
        """

        self.publish_vel(0, 0)
        self.LOG("My battery is low and it's getting dark")

    def publish_vel(self, linear, angular):
        """
            Sends a velocity command to the robot through the cmd_vel topic.
        """
        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = np.clip(angular, -5, 5)
        self.cmd_vel_pub.publish(vel)

    def update_heading(self):
        """
            Updates the current heading of the robot based on the current wheel velocities.
        """

        dt = rospy.get_time()-self.init_time

        angular_vel = self.WHEEL_RADIUS * \
            (self.wr-self.wl)/self.TRACK_WIDTH

        theta = angular_vel * dt

        self.heading += np.arctan2(
            np.sin(theta), np.cos(theta))

        self.init_time = rospy.get_time()

    def sign_is(self, sign, min_area = 0):
        """
            Checks if the current sign is the given sign and if it is big enough. Ensures that the same sign is not detected twice.
        """
        if min_area == 0:
            min_area = self.SIGN_OBEY_THRESHOLD

        is_sign = (self.traffic_sign.name == sign
                and self.traffic_sign.area > min_area
                and self.traffic_sign.name != self.last_sign.name)
        if is_sign:
            self.last_sign = self.traffic_sign
        return is_sign

    def light_is(self, light, min_area = 0):
        """
            Checks if the current light is the given light andif it is big enough
        """
        if min_area == 0:
            min_area = self.LIGHT_OBEY_THRESHOLD

        return self.traffic_light.name == light and self.traffic_light.area > min_area

    # ---------------------- Callback Methods ----------------------
    def wl_cb(self, wl):
        self.wl = wl.data

    def wr_cb(self, wr):
        self.wr = wr.data

    def line_angular_vel_cb(self, msg):
        self.line_angular_vel = msg.data

    def turn_angular_vel_cb(self, msg):
        self.turn_angular_vel = msg.data

    def crossroad_cb(self, msg):
        self.crossroad = msg.data

    def traffic_sign_cb(self, msg):
        self.traffic_sign = msg

    def traffic_light_cb(self, msg):
        self.traffic_light = msg

    def yolo_started_cb(self, msg):
        self.yolo_started = msg.data


if __name__ == "__main__":
    rospy.init_node("final_challenge", anonymous=True)
    Robot()
