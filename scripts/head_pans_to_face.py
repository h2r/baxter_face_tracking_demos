#!/usr/bin/env python

from cv_bridge import CvBridge
import cv2
import rospy
from sensor_msgs.msg import Image
from baxter_interface import Head
from baxter_interface import CameraController
from math import pi


class HeadPan:
    def __init__(self):

        head_cam = CameraController('head_camera')
        head_cam.resolution = (1280, 800)
        head_cam.open()

        # Creating the head variable to mess with later
        self.head = Head()
        self.head.set_pan(angle=0.0)

        # Field of view
        self.CENTER_X = int(1280 / 2)
        self.FOV = pi / 3

        # Range away from center you want the arm to stop within
        self.FACE_RANGE = 50

        self.FACE_CASCADE = cv2.CascadeClassifier(
            'src/baxter_face_tracking_demos/src' +
            '/haarcascade_frontalface_default.xml')
        self.EYE_CASCADE = cv2.CascadeClassifier(
            'src/baxter_face_tracking_demos/src/haarcascade_eye.xml')

        # Publisher for Baxter's head display
        self.display_pub = rospy.Publisher('/robot/xdisplay', Image,
                                           queue_size=0)

        self.subscription = rospy.Subscriber('/cameras/head_camera/image',
                                             Image, self.send)
        rospy.on_shutdown(self.leave_subs_n_pubs)

    def leave_subs_n_pubs(self):
        self.head.set_pan(angle=0)
        self.subscription.unregister()
        self.display_pub.unregister()

    def send(self, data):

        # Getting the image
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Finding the faces in the image
        faces = self.FACE_CASCADE.detectMultiScale(
            gray, scaleFactor=1.35, minNeighbors=4, minSize=(10, 10),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        known_face_x = False
        # Make it an ridiculous value so it won't affect anything
        dif_x = self.CENTER_X * 2

        # Iterating through all faces
        for (x, y, w, h) in faces:

            # Have a lighter and darker colored box so that you see it anywhere.
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
            cv2.rectangle(img, (x - 1, y - 1), (x + w + 1, y + h + 1),
                          (119, 0, 0), 1)
            roi_gray = gray[y:y + h, x:x + w]
            roi_color = img[y:y + h, x:x + w]

            # Finding eyes inside the face box
            eyes = self.EYE_CASCADE.detectMultiScale(roi_gray,
                                                     maxSize=((w / 2), (h / 2)))
            for (ex, ey, ew, eh) in eyes:
                # Have a lighter and darker colored box so that you see it
                # anywhere.
                cv2.rectangle(roi_color, (ex, ey), (ex + ew, ey + eh),
                              (0, 255, 0), 1)
                cv2.rectangle(roi_color,
                              (ex - 1, ey - 1), (ex + ew + 1, ey + eh + 1),
                              (11, 86, 11), 1)

                temp_dif_x = (x + (w / 2)) - self.CENTER_X
                # If the face is closer than the last one found, then set it
                # as the face to go to
                if temp_dif_x < dif_x:
                    known_face_x = True
                    dif_x = temp_dif_x

        cur_pan = self.head.pan()

        # If you found a face, the head is not moving, and that face is in a
        # legal position, then turn the head to center that face.
        if known_face_x and not self.head.panning() and abs(
                (-1 * (dif_x * (self.FOV / 2) / self.CENTER_X) + cur_pan)) < \
                (pi / 2):

            if dif_x > self.FACE_RANGE:
                self.head.set_pan(
                    angle=cur_pan + -1 * (dif_x * (self.FOV / 2)) /
                                    self.CENTER_X)
            elif dif_x < (-1 * self.FACE_RANGE):
                self.head.set_pan(
                    angle=cur_pan + -1 * (dif_x * (self.FOV / 2)) /
                                    self.CENTER_X)

        msg = CvBridge().cv2_to_imgmsg(img, encoding='bgr8')

        self.display_pub.publish(msg)


def track():
    # Creating the node
    rospy.init_node('head_camera_listener', anonymous=True)

    # Creating the class for this
    HeadPan()

    rospy.spin()


if __name__ == '__main__':
    track()
