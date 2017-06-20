#!/usr/bin/env python
import rospy
from rot_and_tran_face_follow_class import RotAndTranFaceFollowArm


def setup():
    rospy.init_node('left_arm_face_follower', anonymous=True)
    RotAndTranFaceFollowArm('left', '0.656098 0.867744 0.563245 -0.053797 ' +
                                   '0.709533 0.025162 0.702165')
    rospy.spin()


if __name__ == '__main__':
    setup()
