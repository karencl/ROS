# Importando las librerias del dron
from djitellopy import Tello
import cv2
import numpy as np

MODE = "DRONE"  # "WEBCAM" or "DRONE"

H_min = 0  # HUE (0-180) = (0-360)
S_min = 21  # SATURATION (0-255)
V_min = 173  # VALUE (0-255)

H_max = 255
S_max = 123
V_max = 255


# Connecting to the drone
drone = Tello()
capture = None

if MODE == "DRONE":
    drone.connect()
    drone.streamoff()
    drone.streamon()
else:
    capture = cv2.VideoCapture(0)

# Drone speeds initialize
drone.left_right_velocity = 0
drone.for_back_velocity = 0
drone.up_down_velocity = 0
drone.yaw_velocity = 0


def save_value(name, value):
    with open("hsv_values.txt", "r+") as f:
        # Find the line and replace it with the new value
        lines = f.readlines()
        for i, line in enumerate(lines):
            if name in line:
                lines[i] = line.replace(line.split(":")[1], str(value)+"\n")

        f.seek(0)
        f.truncate()
        f.writelines(lines)


def get_value(name):
    with open("hsv_values.txt", "r") as f:
        # Find the line with the name and read the value
        lines = f.readlines()
        for line in lines:
            if name in line:
                return int(line.split(":")[1])


cv2.namedWindow("Sliders")
cv2.resizeWindow("Sliders", 1000, 1000)

cv2.createTrackbar("H Min", "Sliders", 0, 180,
                   lambda x: save_value("H_min", x))
cv2.setTrackbarPos("H Min", "Sliders", get_value("H_min"))
cv2.createTrackbar("H Max", "Sliders", 0, 180,
                   lambda x: save_value("H_max", x))
cv2.setTrackbarPos("H Max", "Sliders", get_value("H_max"))

cv2.createTrackbar("S Min", "Sliders", 0, 255,
                   lambda x: save_value("S_min", x))
cv2.setTrackbarPos("S Min", "Sliders", get_value("S_min"))
cv2.createTrackbar("S Max", "Sliders", 0, 255,
                   lambda x: save_value("S_max", x))
cv2.setTrackbarPos("S Max", "Sliders", get_value("S_max"))

cv2.createTrackbar("V Min", "Sliders", 0, 255,
                   lambda x: save_value("V_min", x))
cv2.setTrackbarPos("V Min", "Sliders", get_value("V_min"))
cv2.createTrackbar("V Max", "Sliders", 0, 255,
                   lambda x: save_value("V_max", x))
cv2.setTrackbarPos("V Max", "Sliders", get_value("V_max"))


def main():
    print("Main program inicialized")
    while True:

        img = None

        if MODE == "DRONE":
            frame_read = drone.get_frame_read()
            img = frame_read.frame
        else:
            ret, img = capture.read()

        H_min = cv2.getTrackbarPos("H Min", "Sliders")
        H_max = cv2.getTrackbarPos("H Max", "Sliders")
        S_min = cv2.getTrackbarPos("S Min", "Sliders")
        S_max = cv2.getTrackbarPos("S Max", "Sliders")
        V_min = cv2.getTrackbarPos("V Min", "Sliders")
        V_max = cv2.getTrackbarPos("V Max", "Sliders")

        img = cv2.resize(img, (1000, 1000))
        img = cv2.flip(img, 1)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, (H_min, S_min, V_min),
                           (H_max, S_max, V_max))

        cv2.imshow("Image", mask)

        if MODE == "DRONE":
            drone.send_rc_control(drone.left_right_velocity,
                                  drone.for_back_velocity,
                                  drone.up_down_velocity,
                                  drone.yaw_velocity)

        if (cv2.waitKey(1) & 0xFF == ord('l')):
            cv2.destroyAllWindows()


try:
    main()

except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    cv2.destroyAllWindows()

    if MODE == "DRONE":
        print("Landing")
        drone.land()
        drone.streamoff()

else:
    print('No exceptions are caught')
