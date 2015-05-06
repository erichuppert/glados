#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Pose
from orc.srv import OpenCloseDoorResponse, OpenCloseDoor, OpenCloseDoorRequest, VisualServo, VisualServoRequest, Waypoint
import tf
import math

DOOR_OPEN = 1
DOOR_CLOSED = 0

def change_door_state(state):
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(state) # not sure about this.
    except rospy.ServiceException, e:
        print "Door service call failed"

def go_to_point():
    wp = [(0.6  , 0.6 ,  0      , True ),
          (0.6  , 0.6 ,  0.785  , True ),
          (1.5  , 0.6 ,  0.523  , True ),
          (2.7  , 1.0 ,  0      , True ),
          (3.0  , 1   , -0.785  , True ),
          (3.7  , 1.0 ,  0      , True ),
          (4.3  , 1.5 ,  0.785  , True ),
          (4.3  , 2.3 ,  1.5707 , True ),
          (3.5  , 2.7 ,  3.141  , True ),
          (2.8  , 1.5 , -1.5707 , False),
          (2.0  , 1.5 ,  3.141  , False),
          (2.0  , 2.0 ,  1.5707 , True ),
          (1.5  , 2.0 ,  3.141  , True ),
          (1.0  , 2.5 ,  1.5707 , True ),
          (0.95 , 2.6 ,  2.35   , True )
    ]
    wp = [(x-0.6,y-0.6,theta,servo) for (x,y,theta,servo) in wp]

    n_blocks = 0

    rospy.wait_for_service("waypoints")
    waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
    rospy.wait_for_service('VisualServo')
    visual_servo_service = rospy.ServiceProxy('VisualServo',VisualServo)
    for (x,y,theta,servo) in wp:
        print "Going to %f %f %f" % (x,y,theta)
        waypoint_service(False,servo,x,y,theta)
        if servo:
            print "Going to visual servo"
            visual_servo_service()
            waypoint_service(False,False,x,y,theta)
            n_blocks += 1
        if n_blocks >= 4:
            n_blocks = 0
            change_door_state(DOOR_OPEN)


def main():
	rospy.init_node('state_machine')
	go_to_point()
if __name__ == '__main__':
	main()
