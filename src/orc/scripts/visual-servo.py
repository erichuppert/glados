#!/usr/bin/python2
import rospy
import math
from geometry_msgs.msg import Twist
from orc.msg import NearestBlock

TARGET_BLOCK_SIZE = 400
IMAGE_WIDTH = 640
ROTATION_GAIN = 1


def handle_block_msg(block_msg):
    block_size, loc_x, loc_y = block_msg.size, block_msg.x_location, block_msg.y_location
    print "Size: %d pixels, Location: (%d, %d)" % (block_size, loc_x, loc_y)
    angle_error = loc_x - IMAGE_WIDTH/2
    rotation_vel = angle_error * ROTATION_GAIN
    
def main():
    nearest_block_sub = rospy.Subscriber("nearest_block", NearestBlock, handle_block_msg)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rospy.spin()

if __name__ == "__main__":
    main()
