import cv2
import mediapipe as mp
import numpy as np
from djitellopy import tello

# drone = tello.Tello()
# drone.connect()
# drone.streamon()
capture = cv2.VideoCapture(0)
w, h = 360, 240
fbRange = [6200, 6800]
pid = [0.4, 0.4, 0]

mp_face_detection = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils


def main():
    pError = 0
    status = 0
    cx = 0
    area = 0
    with mp_face_detection.FaceDetection(min_detection_confidence=0.75) as face_detection:
        while True:
            ret, img = capture.read()
            # img = me.get_frame_read().frame
            img = cv2.resize(img, (w, h))
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = face_detection.process(img)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if results.detections:
                for face_no, detection in enumerate(results.detections):
                    mp_drawing.draw_detection(img, detection)
                    if face_no == 0:
                        xmin = int(
                            detection.location_data.relative_bounding_box.xmin * w)
                        ymin = int(
                            detection.location_data.relative_bounding_box.ymin * h)
                        xd = int(
                            detection.location_data.relative_bounding_box.width * w)
                        yd = int(
                            detection.location_data.relative_bounding_box.height * h)
                        cx, cy, area = (xmin + (xd / 2),
                                        ymin + (yd / 2), xd * yd)
                        cv2.putText(img, "Tracking", (xmin, ymin),
                                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)
            fb_speed = 0
            error = cx - w // 2
            yaw_speed = pid[0] * error + pid[1] * (error - pError)
            yaw_speed = int(np.clip(yaw_speed, -100, 100))
            if area > fbRange[0] and area < fbRange[1]:
                fb_speed = 0
            elif area > fbRange[1]:
                fb_speed = -20
            elif area < fbRange[0] and area != 0:
                fb_speed = 20
            if cx == 0:
                yaw_speed = 0
                error = 0

            cv2.imshow("Output", img)
            if (cv2.waitKey(10) == ord('t')):
                if status == 0:
                    status = 1
                    # drone.takeoff()
            if (cv2.waitKey(10) == ord('l')):
                if status == 1:
                    status = 0
                    # drone.land()

            if (cv2.waitKey(20) == ord('r')):
                ud_speed = 40
            elif (cv2.waitKey(20) == ord('f')):
                ud_speed = -40
            else:
                ud_speed = 0

            print("u/d speed: " + str(ud_speed), "yaw_speed: " + str(yaw_speed), "f/b speed: " + str(fb_speed),
                  "Status: " + str(status))
            # print("u/d speed: " + str(ud_speed), "yaw_speed: " + str(yaw_speed), "f/b speed: " + str(fb_speed), "Status: " + str(status), "Battery: " + str(drone.get_Battery()))

            # if status == 1:
            # drone.send_rc_control(0, fb_speed, ud_speed, yaw_speed)

            if (cv2.waitKey(10) == ord('q')):
                break


try:
    main()
except KeyboardInterrupt:
    print('KeyboardInterrupt exception is caught')
    cv2.destroyAllWindows()
    # drone.streamoff()
    # drone.land()
else:
    print('No exceptions are caught')
