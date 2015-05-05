#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point, PointStamped
from orc.srv import OpenCloseDoorResponse, OpenCloseDoor, OpenCloseDoorRequest, VisualServo, VisualServoRequest, Collides, CollidesResponse

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
        visual_servo_service = rospy.ServiceProxy('VisualServo',VisualServo)
        visual_servo_service()
    except rospy.ServiceException, e:
        print "Service call failed"
    return True
    
def change_door_state(state):
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(state) # not sure about this.
    except rospy.ServiceException, e:
        print "Service call failed"

def will_colide_with(point_msg):
    collision_service = rospy.ServiceProxy("collides", Collides)
    odometry_point = listener.transformPoint("odom", point_msg)
    collision_reponse = collision_service(odometry_point)
    return collision_response.collides

def handle_nearest_block_msg(msg):
    global blocks_held
    if not will_colide_with(msg):
        change_explorer_state(EXPLORER_OFF)
        # after we release the tower, we don't close the door, so make sure its closed before we put a block inside
        if pickup_block():
            blocks_held += 1
            if blocks_held == 6:
                change_door_state(DOOR_OPEN)
                blocks_held = 0
        change_explorer_state(EXPLORER_ON)
    

def main():
    global nearest_block_sub, explorer_state_pub, listener
    rospy.init_node('state_machine')
    explorer_state_pub = rospy.Publisher("explorer_state", UInt16, queue_size=10)
    nearest_block_sub = rospy.Subscriber("nearest_block", PointStamped, handle_nearest_block_msg, queue_size=1)
    listener = tf.TransformListener();
    change_explorer_state(EXPLORER_ON)
    rospy.spin()
    
    
if __name__ == "__main__":
    main()
