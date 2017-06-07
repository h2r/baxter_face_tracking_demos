#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from baxter_interface import CameraController
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep


def send(data):
    face_cascade = cv2.CascadeClassifier(
        'src/baxter_face_tracking_demos/src' +
        '/haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(
        'src/baxter_face_tracking_demos/src/haarcascade_eye.xml')

    # The image from the head_camera
    img = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray,
                                          scaleFactor=1.35, minNeighbors=4,
                                          minSize=(10, 10),
                                          flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

    num_eyes = 0
    for (x, y, w, h) in faces:
        # Have a lighter and darker colored box so that you see it anywhere.
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
        cv2.rectangle(img, (x - 1, y - 1), (x + w + 1, y + h + 1), (119, 0, 0),
                      1)

        roi_gray = gray[y:y + h, x:x + w]
        roi_color = img[y:y + h, x:x + w]

        num_eyes = 0

        eyes = eye_cascade.detectMultiScale(roi_gray,
                                            maxSize=((w / 2), (h / 2)))
        for (ex, ey, ew, eh) in eyes:
            # Have a lighter and darker colored box so that you see it anywhere.
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0),
                          1)
            cv2.rectangle(roi_color, (ex - 1, ey - 1),
                          (ex + ew + 1, ey + eh + 1), (11, 86,
                                                       11), 1)
            num_eyes += 1

    # If there are at least two eyes then it saw a face
    if num_eyes >= 2:

        # The wave position
        wave_1 = '0.612655 -0.559410 0.768578 0.223566' + \
                 ' 0.012220 0.146616 0.963521'
        wave_2 = '0.489536 -0.188734 0.725782 -0.33575' + \
                 '4 -0.076261 0.088122 0.934713'

        # The first wave takes longer to get to
        right_hand_pub.publish(
            String(wave_2 + ' createEEPose moveEeToPoseWord'))
        sleep(4)

        # Wave
        for _move in range(2):
            right_hand_pub.publish(
                String(wave_1 + ' createEEPose moveEeToPoseWord'))
            sleep(2)

            right_hand_pub.publish(
                String(wave_2 + ' createEEPose moveEeToPoseWord'))
            sleep(2)

        # Goes home after waving        
        right_hand_pub.publish(String('goHome'))
        # Needs time to move there
        sleep(3)

    msg = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')

    display_pub.publish(msg)


def track():
    def leave_subs_n_pubs():
        subscription.unregister()
        right_hand_pub.unregister()
        display_pub.unregister()

    # Initializing nodes
    rospy.init_node('wave_for_faces', anonymous=True)

    # Defining global publishers
    global display_pub
    display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=0)
    global right_hand_pub
    right_hand_pub = rospy.Publisher('/ein/right/forth_commands', String,
                                     queue_size=0)
    # Needs time to start up
    sleep(3)
    right_hand_pub.publish(String('goHome'))
    # Needs time to go home
    sleep(3)

    # Setting and opening camera
    global camera_name
    camera_name = 'head_camera'
    head_cam = CameraController(camera_name)
    head_cam.resolution = (1280, 800)
    head_cam.open()

    rospy.on_shutdown(leave_subs_n_pubs)

    # Setting subscription
    subscription = rospy.Subscriber(
        '/cameras/' + camera_name + '/image', Image, send)
    rospy.spin()


if __name__ == '__main__':
    # print "main running"
    track()
