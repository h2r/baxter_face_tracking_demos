#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from baxter_interface import CameraController
from sensor_msgs.msg import Image
from std_msgs.msg import String
from time import sleep
from ein.msg import EinConsole
from sys import maxint


class RotAndTranFaceFollowArmNoDisplay:
    def __init__(self, arm, starting_pose):
        # Range away from center you want the arm to stop within
        self.FACE_RANGE = 50
        # Denominator for movement, when rotational.
        self.MOVEMENT_DENOMINATOR = 24
        # Speed of movement, when translational. (strings/chars of integers
        # only)
        self.MOVEMENT_SPEED = '1'

        self.is_stuck = False
        self.face_count = 0

        # Starts at the end point so that it will see a new face automatically
        self.face_time_counter = 80

        # Finding center point
        desired_resolution = (640, 400)

        # If the arm's camera is not available, the other arm's camera will be
        # closed.
        try:
            CameraController(arm + '_hand_camera')
        except AttributeError:
            if arm == 'right':
                left_cam = CameraController('left_hand_camera')
                left_cam.close()
            elif arm == 'left':
                right_cam = CameraController('right_hand_camera')
                right_cam.close()

        cam = CameraController(arm + '_hand_camera')
        cam.resolution = desired_resolution
        cam.open()
        (x, y) = desired_resolution

        # Getting the center of the camera
        self.CENTER_X = x / 2
        self.CENTER_Y = y / 2

        # Position of last face seen.  Initialized as something impossible.
        self.last_face_x = -maxint
        self.last_face_y = -maxint


        self.FACE_CASCADE = cv2.CascadeClassifier(
            'src/baxter_face_tracking_demos/src' +
            '/haarcascade_frontalface_default.xml')
        self.EYE_CASCADE = cv2.CascadeClassifier(
            'src/baxter_face_tracking_demos/src/haarcascade_eye.xml')

        self.hand_pub = rospy.Publisher(
            '/ein/' + arm + '/forth_commands', String, queue_size=0)
        sleep(3)
        self.console_sub = rospy.Subscriber('/ein/' + arm + '/console',
                                            EinConsole, self.callback)
        self.cam_sub = rospy.Subscriber('/cameras/' + arm +
                                        '_hand_camera/image', Image,
                                        self.follow)

        rospy.on_shutdown(self.leave_subs_n_pubs)
        
        # Assume starting position
        self.hand_pub.publish(String(starting_pose +
                                     ' createEEPose moveEeToPoseWord'))
        # It takes time to get there
        sleep(1)

        # Setting good camera settings
        exposure_and_gain = '65 40'
        self.hand_pub.publish(String(exposure_and_gain +
                                     ' 1024 1024 2048 fixCameraLighting'))

    def leave_subs_n_pubs(self):
        self.hand_pub.publish(String('goHome'))
        self.cam_sub.unregister()
        self.console_sub.unregister()
        self.hand_pub.unregister()

    def callback(self, msg):
        if "ikClient says pose request is invalid." in msg.msg:
            self.is_stuck = not self.is_stuck

    def follow(self, data):



        # Converting the data to an image for OpenCV
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding='bgr8')

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Finding the faces in the image
        faces = self.FACE_CASCADE.detectMultiScale(
            gray, scaleFactor=1.25, minNeighbors=4, minSize=(10,10),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE)

        known_face_center = False
        dif_x = self.CENTER_X * 2
        dif_y = self.CENTER_Y * 2

        for (x, y, w, h) in faces:

            roi_gray = gray[y:y + h, x:x + w]

            eyes = self.EYE_CASCADE.detectMultiScale(roi_gray, 
                scaleFactor=1.1, minNeighbors=2)

            for (ex, ey, ew, eh) in eyes:

                temp_dif_x = (x + (w / 2)) - self.CENTER_X
                temp_dif_y = (y + (h / 2)) - self.CENTER_Y
                # If the face is closer than the last one found, then set it
                # as the face to go to.
                if temp_dif_x < dif_x:
                    # If it has eyes then it's definitely a face
                    known_face_center = True
                    dif_x = temp_dif_x
                    dif_y = temp_dif_y

                # TODO FIX THESE LINES.  MAYBE TAKE OUT THE PART ABOUT CHECKING IF ITS NEAR
                # Uses the size of the box as a way to see how far away
                # someone is.  If someone is farther away, they should have a
                #  wider range where the person can move without the counter
                # increasing.
                if self.face_time_counter == 80 or abs(self.last_face_x - (x +
                        (w / 2))) > self.FACE_RANGE * self.FACE_RANGE * \
                        self.FACE_RANGE / (3 * w) or abs(self.last_face_y - (
                                y + (h / 2))) > self.FACE_RANGE * \
                        self.FACE_RANGE * self.FACE_RANGE / (3 * h):
                # if self.face_time_counter == 80:
                    self.face_count += 1
                    self.last_face_x = (x + (w / 2))
                    self.last_face_y = (y + (h / 2))

                # Reset the face time counter every time you see a face
                self.face_time_counter = 0

        if self.face_time_counter < 80:
            self.face_time_counter += 1

        # If it saw a face, see if the arm needs to move
        if known_face_center:

            # If out of face_range, the arm will try to center itself on the
            # face
            if self.is_stuck:
                # Translational
                if dif_x > self.FACE_RANGE:
                    self.hand_pub.publish(String(
                        "( localYDown ) " + self.MOVEMENT_SPEED +
                        " replicateWord"))
                elif dif_x < -self.FACE_RANGE:
                    self.hand_pub.publish(String(
                        "( localYUp ) " + self.MOVEMENT_SPEED +
                        " replicateWord"))

                if dif_y > self.FACE_RANGE:
                    self.hand_pub.publish(String(
                        "( localXUp ) " + self.MOVEMENT_SPEED +
                        " replicateWord"))
                elif dif_y < -self.FACE_RANGE:
                    self.hand_pub.publish(String(
                        "( localXDown ) " + self.MOVEMENT_SPEED +
                        " replicateWord"))

            else:
                # Rotational
                if dif_x > self.FACE_RANGE:
                    self.hand_pub.publish(
                        String('( oXUp ) ' + str(
                            int(dif_x / self.MOVEMENT_DENOMINATOR)) +
                               ' replicateWord'))
                elif dif_x < -self.FACE_RANGE:
                    self.hand_pub.publish(
                        String('( oXDown ) ' + str(
                            int(abs(dif_x / self.MOVEMENT_DENOMINATOR))) +
                               ' replicateWord'))

                if dif_y > self.FACE_RANGE:
                    self.hand_pub.publish(
                        String('( oYUp ) ' + str(
                            int(dif_y / self.MOVEMENT_DENOMINATOR)) +
                               ' replicateWord'))
                elif dif_y < -self.FACE_RANGE:
                    self.hand_pub.publish(
                        String('( oYDown ) ' + str(
                            int(abs(dif_y / self.MOVEMENT_DENOMINATOR))) +
                               ' replicateWord'))