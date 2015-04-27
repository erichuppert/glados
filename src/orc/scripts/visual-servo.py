#!/usr/bin/python2
import rospy
import math
from geometry_msgs.msg import Twist, Point
from orc.msg import TouchState

TARGET_BLOCK_SIZE = 1100
ALIGNED_WIDTH = 250

DESIRED_ROTATION = -0.04 # Experimental
ROTATION_GAIN = 2

FORWARD_GAIN = .5
DESIRED_DIST = 0.15 # Experimental
STRAIGHTNESS = 100

def handle_block_msg(block_msg):
    angle_error = (block_msg.y - DESIRED_ROTATION)
    distance_error = (block_msg.x - DESIRED_DIST)
    print "error is " + str(angle_error)
    rotational_vel = angle_error*ROTATION_GAIN
    msg = Twist()
    msg.linear.x = math.cos(angle_error)**STRAIGHTNESS * FORWARD_GAIN * distance_error
    msg.angular.z = rotational_vel
    print msg
    vel_pub.publish(msg)

def handle_bump_msg(bump_msg):
    if (bump_msg.gripper):
        vel_pub.publish(Twist())

def main():
    global vel_pub
    rospy.init_node("visual_servo")
    nearest_block_sub = rospy.Subscriber("nearest_block", Point, handle_block_msg)
    bump_sub = rospy.Subscriber("touch", TouchState, handle_bump_msg)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rospy.spin()

if __name__ == "__main__":
    main()
