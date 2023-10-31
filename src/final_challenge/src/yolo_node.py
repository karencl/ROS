#!/usr/bin/env python3
import cv2
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import rospy
import time
import sys

print("[YOLO] Importing yolo... ")

HOME = "/"+sys.path[0].split("/")[1]+"/"+sys.path[0].split("/")[2]
PACKAGE_PATH = sys.path[0][:-3]

sys.path.append(HOME+'/yolov5')
from detection import detector
print("[YOLO] Imported yolo!")

def string_to_rgb_color(string):
    # Compute the hash value of the string
    hash_value = hash(string)

    # Extract the RGB components from the hash value
    red = (hash_value & 0xFF0000) >> 16
    green = (hash_value & 0x00FF00) >> 8
    blue = hash_value & 0x0000FF

    return (blue, green, red)

def image_to_msg(image):
    # Convert the image to a ROS Image message

    if len(image.shape) == 3:
        height, width, channels = image.shape
        encoding = 'bgr8'  # Specify the image encoding
    else:
        height, width = image.shape
        encoding = "mono8"
        channels = 1

    # Create the Image message
    img_msg = Image()
    img_msg.header.stamp = rospy.Time.now()
    img_msg.header.frame_id = 'camera_frame'
    img_msg.height = height
    img_msg.width = width
    img_msg.encoding = encoding
    img_msg.step = width * channels
    img_msg.data = image.tobytes()

    return img_msg

def msg_to_image(msg):
    width = msg.width
    height = msg.height
    channels = msg.step // msg.width
    img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, channels))
    return img_data

def distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2)**2 + (y1 - y2)**2)

class Robot():

    GHOST_FRAME_THRESHOLD = 3
    SIGN_CONFIDENCE_THRESHOLD = 0.7
    LIGHT_CONFIDENCE_THRESHOLD = 0.4

    WIDTH = 224

    frame = None
    pred = []

    tracked_objects = {}

    def __init__(self):

        self.setup_node()
        self.image_received_flag = 0

        # folder path

        weights = PACKAGE_PATH+"/models/best.onnx"

        print("[YOLO] Loading network...")
        t = time.time()
        yolo = detector(weights, 0.5)
        print("[YOLO] Loaded in", time.time() - t)
        self.yolo_started.publish(True)

        self.names = ["forward", "give_way", "left", "right", "road_work", "stop", "green", "red", "yellow"]

        time_avg = 0
        time_count = 0

        print_time = time.time()
        print_time = time.time()
        data_sender = Float32MultiArray()

        print("[YOLO] Reading... ")

        for i in range(len(self.names)):
            self.tracked_objects[i] = []

        while not rospy.is_shutdown():

            for key in self.tracked_objects:
                for obj in self.tracked_objects[key]:
                    obj["detected"] = False

            if self.frame is not None and self.image_received_flag == 1:
                t = time.time()
                #try:

                pred = yolo.detect(self.frame)

                time_avg += time.time()-t
                time_count += 1

                if time.time() - print_time > 5:
                    print("[YOLO] Latency: ", time_avg/time_count)
                    time_avg = 0
                    time_count = 0
                    print_time = time.time()

                for det in pred:
                    for d in det:

                        x1, y1, x2, y2, confidence, key = d
                        key = int(key)

                        if key < 5:
                            threshold = self.SIGN_CONFIDENCE_THRESHOLD
                        else:
                            threshold = self.LIGHT_CONFIDENCE_THRESHOLD

                        if confidence < threshold:
                            continue

                        center_x = (x1 + x2) / 2
                        center_y = (y1 + y2) / 2


                        unique = True
                        for i, obj in enumerate(self.tracked_objects[key]):
                            unique = False
                            self.tracked_objects[key][i] = {
                                    "x":center_x,
                                    "y":center_y,
                                    "detected_frames": obj["detected_frames"] + 1,
                                    "lost_frames": self.GHOST_FRAME_THRESHOLD,
                                    "detected": True,
                                    "d":d
                                }

                        if unique:
                            self.tracked_objects[key].append({
                                "x":center_x,
                                "y":center_y,
                                "detected_frames": 1,
                                "lost_frames": self.GHOST_FRAME_THRESHOLD,
                                "detected": True,
                                "d":d
                            })

            result = np.array([])

            for key in self.tracked_objects:
                for obj in self.tracked_objects[key]:
                    if obj["detected_frames"] >= self.GHOST_FRAME_THRESHOLD:
                        result = np.append(result, obj["d"])
                        d = obj["d"]
                        x1, y1, x2, y2, _, _ = d
                        area = abs(x2-x1)*abs(y2-y1)


                    if not obj["detected"]:
                        obj["lost_frames"] -= 1
                        if obj["lost_frames"] == 0:
                            self.tracked_objects[key].remove(obj)

            data_sender.data = result
            self.yolo_pub.publish(data_sender)


                #except Exception as e:
                    #print(e)

            self.image_received_flag == 0

    def setup_node(self):
        rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(50)

        self.image_sub = rospy.Subscriber(
        "/video_source/raw", Image, self.camera_callback)
        self.image_sub = rospy.Subscriber(
        "/camera/video_source/raw", Image, self.camera_callback)

        self.image_pub = rospy.Publisher("/processed_image_yolo", Image, queue_size=1)
        self.yolo_pub = rospy.Publisher("/yolo_results", Float32MultiArray, queue_size = 100)
        self.yolo_started = rospy.Publisher("/yolo_started", Bool, queue_size = 1)

        self.yolo_started.publish(False)

    def camera_callback(self, data):
        #try:
        if self.frame is not None:
            for key in self.tracked_objects:
                for obj in self.tracked_objects[key]:
                    d = obj["d"]
                    top_left = (int(d[0]), int(d[1]))
                    bottom_right = (int(d[2]), int(d[3]))
                    class_name = self.names[int(d[5])]
                    confidence = round(float(d[4]),3)

                    area = abs(bottom_right[0]-top_left[0])*abs(bottom_right[1]-top_left[1])

                    color_hash = string_to_rgb_color(class_name)

                    cv2.rectangle(self.frame, top_left, bottom_right, color_hash, 2)
                    cv2.putText(self.frame, class_name + " " + str(confidence), (top_left[0], top_left[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_hash, 1)
                    cv2.putText(self.frame, str(area), (bottom_right[0], bottom_right[1]+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_hash, 1)


            self.image_pub.publish(image_to_msg(self.frame))

        img = msg_to_image(data)
        self.frame = cv2.resize(img, (self.WIDTH, self.WIDTH))
        self.image_received_flag = 1
        #except Exception as e:
            #print(e)

    def cleanup(self):
        print("[YOLO] Shutting down YOLO node")
        cv2.destroyAllWindows()


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("yolo", anonymous=True)
    Robot()
