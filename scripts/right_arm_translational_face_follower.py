#!/usr/bin/env python
import rospy
from translational_follow_class import TranslationalFollowArm


def setup():
    rospy.init_node('right_arm_translational_face_follower', anonymous=True)
    TranslationalFollowArm('right', '0.802374 -0.791327 0.529881 -0.031081 ' +
                           '0.710642 0.017939 0.702638')
    rospy.spin()


if __name__ == '__main__':
    setup()
