#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState


printJointStates = False


def joint_state_callback(data):
    rospy.loginfo('Position: %s', data.position)


def listener():
    rospy.init_node('pos_hook', anonymous=False)
    rospy.Subscriber("/amr_dd/joint_states", JointState, joint_state_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()