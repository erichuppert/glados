#!/usr/bin/python2
import rospy
import math
from geometry_msgs.msg import Twist, Point
from orc.msg import TouchState
from orc.srv import VisualServoResponse, VisualServo

TARGET_BLOCK_SIZE = 1100
ALIGNED_WIDTH = 250

DESIRED_ROTATION = -0.04 # Experimental
ROTATION_GAIN = 2

FORWARD_GAIN = .5
DESIRED_DIST = 0.15 # Experimental
STRAIGHTNESS = 100

paused = true
done = false

def handle_block_msg(block_msg):
    if not paused:
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
    global done
    if (bump_msg.gripper):
        vel_pub.publish(Twist())
    # call the pickup and drop service here
    if not paused:
        done = true
        pickup_block()

def pickup_block():
  states = ["findblock","grip","pickup","drop"]
    for state in states:
        rospy.wait_for_service('PickupAndDrop')
        try:
            pickupAndDrop = rospy.ServiceProxy('PickupAndDrop',PickupBlock)
            pickupAndDrop(state)
        except rospy.ServiceException, e:
        print "Service call failed"



def drop_arm():
    # use the pick_up_and_drop service to move the arm to the down state
    rospy.wait_for_service('PickupAndDrop')
    try:
        pickupAndDrop = rospy.ServiceProxy('PickupAndDrop',PickupBlock)
            pickupAndDrop("findblock")
        except rospy.ServiceException, e:
        print "Service call failed"
        
def visual_servo_service(req):
    global paused, done
    drop_arm()
    paused = false
    done = false
    loop = rospy.rate(30)
    while not done:
        loop.sleep()
    return VisualServoResponse()
        
def main():
    global vel_pub
    rospy.init_node("visual_servo")
    nearest_block_sub = rospy.Subscriber("nearest_block", Point, handle_block_msg)
    bump_sub = rospy.Subscriber("touch", TouchState, handle_bump_msg)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    s = rospy.Service('VisualServo', VisualServo, visual_servo_service)
    rospy.spin()

if __name__ == "__main__":
    main()
