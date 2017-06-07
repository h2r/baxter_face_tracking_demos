#!/usr/bin/env python

from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image


def send(data):
    face_cascade = cv2.CascadeClassifier(
        'src/baxter_face_tracking_demos/src' +
        '/haarcascade_frontalface_default.xml')
    eye_cascade = cv2.CascadeClassifier(
        'src/baxter_face_tracking_demos/src/haarcascade_eye.xml')

    # Getting the image
    img = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detecting faces
    faces = face_cascade.detectMultiScale(gray,
                                          scaleFactor=1.35, minNeighbors=4,
                                          minSize=(10, 10),
                                          flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

    # Iterating through all faces
    for (x, y, w, h) in faces:

        # Have a lighter and darker colored box so that you see it anywhere.
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
        cv2.rectangle(img, (x - 1, y - 1), (x + w + 1, y + h + 1), (119, 0, 0),
                      1)
        roi_gray = gray[y:y + h, x:x + w]
        roi_color = img[y:y + h, x:x + w]

        # Finding eyes inside the face box
        eyes = eye_cascade.detectMultiScale(roi_gray, maxSize=((w / 2),
                                                               (h / 2)))
        for (ex, ey, ew, eh) in eyes:
            # Have a lighter and darker colored box so that you see it anywhere.
            cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0),
                          1)
            cv2.rectangle(roi_color,
                          (ex - 1, ey - 1), (ex + ew + 1, ey + eh + 1),
                          (11, 86, 11), 1)

    msg = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')

    display_pub.publish(msg)


def track():
    def leave_subs_n_pubs():
        subscription.unregister()
        display_pub.unregister()

    # Creating the node
    rospy.init_node('head_camera_listener', anonymous=True)

    # Publisher for Baxter's head display
    global display_pub
    display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=0)

    # Subscribing to head camera
    rospy.on_shutdown(leave_subs_n_pubs)

    # global subscription
    subscription = rospy.Subscriber('/cameras/head_camera/image', Image, send)
    rospy.spin()


if __name__ == '__main__':
    track()
