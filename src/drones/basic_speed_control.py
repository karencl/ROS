# Importando las librerias del dron
from djitellopy import Tello
import cv2
import numpy as np

MODE = "WEBCAM"  # or "DRONE"

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

INIT_SPEED = 50
drone_speed = INIT_SPEED

cv2.namedWindow("Slider")
cv2.resizeWindow("Slider", 500, 500)
cv2.createTrackbar("Speed", "Slider", 0, 100, lambda x: x)
cv2.setTrackbarPos("Speed", "Slider", INIT_SPEED)


def main():
    print("Main program inicialized")
    while True:

        img = None

        if MODE == "DRONE":
            frame_read = drone.get_frame_read()
            img = frame_read.frame
        else:
            ret, img = capture.read()

        img = cv2.resize(img, (500, 500))

        drone_speed = cv2.getTrackbarPos("Speed", "Slider")
        cv2.putText(img, "Speed: " + str(drone_speed), (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)
        cv2.imshow("Image", img)

        var = cv2.waitKey(10) & 0xFF

        drone.left_right_velocity = 0
        drone.for_back_velocity = 0
        drone.up_down_velocity = 0
        drone.yaw_velocity = 0

        if var == ord('q'):
            print("Landing...")
            drone.land()

        elif var == ord('p'):
            print("Starting takeoff...")
            drone.takeoff()

        elif var == ord('w'):
            drone.for_back_velocity = drone_speed

        elif var == ord('s'):
            drone.for_back_velocity = -drone_speed

        elif var == ord('a'):
            drone.left_right_velocity = -drone_speed

        elif var == ord('d'):
            drone.left_right_velocity = drone_speed

        elif var == ord('e'):
            drone.up_down_velocity = drone_speed

        elif var == ord('c'):
            drone.up_down_velocity = -drone_speed

        elif var == ord('z'):
            drone.yaw_velocity = -drone_speed

        elif var == ord('x'):
            drone.yaw_velocity = drone_speed

        if MODE == "DRONE":
            drone.send_rc_control(drone.left_right_velocity,
                                  drone.for_back_velocity,
                                  drone.up_down_velocity,
                                  drone.yaw_velocity)


try:
    main()

except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    print("Landing")
    cv2.destroyAllWindows()
    drone.land()

else:
    print('No exceptions are caught')
