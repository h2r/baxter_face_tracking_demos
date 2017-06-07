#!/usr/bin/env python
import rospy
from rotational_follow_class import RotationalFollowArm


def setup():
    rospy.init_node('left_arm_rotational_face_follower', anonymous=True)
    # RotationalFollowArm('left', '0.656098 0.867744 0.563245 -0.053797 ' +
    #                     '0.709533 0.025162 0.702165')

    # TODO Check if the following is a better starting position
    RotationalFollowArm('left', '0.276241 0.652758 0.235748 -0.039501 '
                                '0.559008 -0.013021 0.828118')
    rospy.spin()


if __name__ == '__main__':
    setup()
