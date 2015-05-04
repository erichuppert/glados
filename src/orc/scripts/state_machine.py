#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point
from orc.srv import OpenCloseDoorResponse, OpenCloseDoor, OpenCloseDoorRequest, VisualServo, VisualServoRequest

blocks_held = 0

DOOR_OPEN = 1
DOOR_CLOSED = 0

EXPLORER_ON = 1
EXPLORER_OFF = 0

def change_explorer_state(val):
    msg = UInt16()
    msg.data = val
    explorer_state_pub.publish(msg)

def pickup_block():
    # this should synchronously call visual servoing to start, which should make the arm
    # pickup the block when the bump sensors are triggered

    rospy.wait_for_service('VisualServo')
    try:
        visual_servo_service = rospy.ServiceProxy('VisualServo',visual_servo_service)
        visual_servo_service()
    except rospy.ServiceException, e:
        print "Service call failed"
    return True
    
def change_door_state(state):
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(2) # not sure about this.
    except rospy.ServiceException, e:
        print "Service call failed"

def handle_nearest_block_msg(msg):
    change_explorer_state(EXPLORER_OFF)
    # after we release the tower, we don't close the door, so make sure its closed before we put a block inside
    change_door_state(DOOR_CLOSED)
    if pickup_block():
        blocks_held += 1
        if blocks_held == 6:
            change_door_state(DOOR_OPEN)
            blocks_held = 0
    change_explorer_state(EXPLORER_ON)
    

def main():
    global nearest_block_sub, explorer_state_pub
    rospy.init_node('state_machine')
    explorer_state_pub = rospy.Publisher("explorer_state", UInt16, queue_size=10)
    nearest_block_sub = rospy.Subscriber("nearest_block", Point, handle_nearest_block_msg, queue_size=1)
    change_explorer_state(EXPLORER_ON)
    rospy.spin()
    
    
if __name__ == "__main__":
    main()
