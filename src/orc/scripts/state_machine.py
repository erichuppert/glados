#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point

blocks_held = 0

DOOR_OPEN = 1
DOOR_CLOSED = 0

WALL_FOLLOW_ON = 1
WALL_FOLLOW_OFF = 0

def change_wall_follow_state(val):
    msg = UInt16()
    msg.data = val
    wall_follow_state_pub.publish(msg)

def pickup_block():
    # this should synchronously call visual servoing to start, which should make the arm
    # pickup the block when the bump sensors are triggered
    pass

def change_door_state():
    # this should synchronously open or close the gate using a service (that needs to be
    # built)
    pass

def handle_nearest_block_msg(msg):
    change_wall_follow_state(WALL_FOLLOW_OFF)
    # after we release the tower, we don't close the door, so make sure its closed before we put a block inside
    change_door_state(DOOR_CLOSED)
    if pickup_block():
        blocks_held += 1
        if blocks_held == 6:
            change_door_state(DOOR_OPEN)
            blocks_held = 0
    change_wall_follow_state(WALL_FOLLOW_ON)
    

def main():
    global nearest_block_sub, wall_follow_state_pub
    rospy.init_node('state_machine')
    wall_follow_state_pub = rospy.Publisher("wall_follow_state", UInt16, queue_size=10)
    nearest_block_sub = rospy.Subscriber("nearest_block", Point, handle_nearest_block_msg, queue_size=1)
    change_wall_follow_state(WALL_FOLLOW_ON)
    
    
if __name__ == "__main__":
    main()
