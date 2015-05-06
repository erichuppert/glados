#!/usr/bin/python2
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion, Pose
from orc.srv import OpenCloseDoorResponse, OpenCloseDoor, OpenCloseDoorRequest, VisualServo, VisualServoRequest,
import tf
import math

def change_door_state(state):
    rospy.wait_for_service('open_close_door')
    try:
        open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
        open_close_door(state) # not sure about this.
    except rospy.ServiceException, e:
        print "Door service call failed"

def go_to_point():
	wp = [(0.6,0.6,0),(0.6,0.6,+1.5707),(1.7,1.0,0),(2.7,1.0,0),(3.3,0.7,-0.785),(3.7,1.0,+0)
	,(4.3,1.5,+1.5707),(4.2,2.3,+1.5707),(3.5,2.6,+3.141),(2.8,1.5,-1.5707),(2.0,1.5,3.141),(2.0,2.0,1.5707),(1.5,2,+3.141)
	,(1,2.5,1.5707),(0.95,2.6,+2.35)]
	
	for (x,y,theta) in wp:
		rospy.wait_for_service("waypoints")
        waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
        waypoint_service(False,True,x,y,theta)
        rospy.wait_for_service('VisualServo')
    	try:
        	visual_servo_service = rospy.ServiceProxy('VisualServo',VisualServo)
        	visual_servo_service()
    	except rospy.ServiceException, e:
       	 print "Visual Servo service call failed"
       	rospy.wait_for_service("waypoints")
        waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
        waypoint_service(False,False,x,y,theta)
        if (x,y,theta) in [(3.7,1.0,+0), (1.5,2,+3.141), (0.95,2.6,+2.35)]:
        	change_door_state(state)


def main():
	rospy.init_node('state_machine')
	go_to_point()
if __name__ == '__main__':
	main()
