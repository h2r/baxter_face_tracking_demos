#!/usr/bin/env python
import rospy
from rotational_follow_class import RotationalFollowArm


def setup():
    rospy.init_node('right_arm_rotational_face_follower', anonymous=True)
    # RotationalFollowArm('right', '0.802374 -0.791327 0.529881 -0.031081 ' +
    #                     '0.710642 0.017939 0.702638')

    # TODO Check if the following is a better starting position
    RotationalFollowArm('right', '0.418192 -0.607819 0.204781 -0.047957 ' +
                                 '0.513668 0.066214 0.854085')

    rospy.spin()


if __name__ == '__main__':
    setup()
