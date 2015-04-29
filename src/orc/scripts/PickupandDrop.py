#!/usr/bin/env python
import rospy
import math
from orc.msg import JointSet



joint = JointSet()

def handleAngleMessage():
	

def find_block ():
	joint.joint_name="base_to_shoulder"
	joint.angle=-1.3
	joint_pub.publish(joint)

	joint.joint_name = "shoulder_to_wrist"
    joint.angle = 0.8
    joint_pub_.publish(joint)

    joint.joint_name = "wrist_to_gripper"
    joint.angle = 0.8
    joint_pub_.publish(joint)


def grip():
	joint.joint_name = "base_to_shoulder"
    joint.angle = -1.3
    joint_pub_.publish(joint)

    joint.joint_name = "shoulder_to_wrist"
    joint.angle = 0.8
    joint_pub_.publish(joint)

    joint.joint_name = "wrist_to_gripper"
    joint.angle = 0.55
    joint_pub_.publish(joint)


def pick_up():
	
	joint.joint_name = "base_to_shoulder"
	joint.angle = 2.0
	joint_pub_.publish(joint)

    joint.joint_name = "shoulder_to_wrist"
    joint.angle = 1.24
    joint_pub_.publish(joint)

    joint.joint_name = "wrist_to_gripper"
    joint.angle = 0.55
    joint_pub_.publish(joint)


def drop():
	joint.joint_name = "base_to_shoulder"
    joint.angle = 2.0
    joint_pub_.publish(joint)

    joint.joint_name = "shoulder_to_wrist"
    joint.angle = 1.24
    joint_pub_.publish(joint)

    joint.joint_name = "wrist_to_gripper"
    joint.angle = 0.8
    joint_pub_.publish(joint)


def main():
	global joint_pub_
	rospy.init_node('Pickup_and_drop')
	joint_pub_=rospy.Publisher("/joint_set",100)
