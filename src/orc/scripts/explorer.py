#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
from orc.srv import Collides
from orc.srv import Waypoint

from tf.transformations import euler_from_quaternion
import message_filters

import random

from math import cos,sin,acos,pi,atan,atan2

SLEEPING = 0
AWAKE = 1
state = SLEEPING

angle_to_index = lambda theta,s: int(len(s.ranges)/(s.angle_max-s.angle_min) * (theta - s.angle_min))
index_to_angle = lambda i,s: i * (s.angle_max - s.angle_min) / len(s.ranges) + s.angle_min

M_PER_INCH = 0.0254 # meters/inch
ROBOT_LENGTH_INCHES = 14 # inches
ROBOT_WIDTH_INCHES = 19 # inches
ROBOT_LENGTH = ROBOT_LENGTH_INCHES*M_PER_INCH # meters
ROBOT_WIDTH = ROBOT_WIDTH_INCHES*M_PER_INCH # meters
RADIUS_SIMPLE = ((ROBOT_WIDTH/2.0)**2 + (ROBOT_LENGTH/2.0)**2)/2.0

ANGLE_DELTA = 0.1

forward_speed = 0.05
skip = True
def update(odom):
    global skip
    if state == SLEEPING: # Don't do anything if we're sleeping
        return

    skip = not skip
    if skip:
        return

    print "going to random waypoint"
    pose = odom.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,r_theta = euler_from_quaternion(quat)

    (rx,ry) = (pose.position.x,pose.position.y)
    tries = 1000
    rospy.wait_for_service("collides")
    collides = rospy.ServiceProxy("collides",Collides)
    rospy.wait_for_service("waypoints")
    waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
    dest_theta = random.random()*2*pi
    while tries > 0:
        r,theta = random.random()*0.5,random.random()*pi - pi/2.0
        x,y = rx+r*cos(r_theta+theta),ry+r*sin(r_theta+theta)
        print rx,ry,x,y,r_theta,theta
        if not collides(x,y,dest_theta).collides:
            waypoint_service(False,True,x,y,dest_theta)
            return
        tries -= 1
    print "no point found :("

    waypoint_service(False,True,rx,ry,dest_theta)

def change_state(new_state):
    global state, vel_pub
    state = new_state.data
    rospy.wait_for_service("waypoints")
    waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
    if state == SLEEPING:
        waypoint_service(True,False,0,0,0)
        vel_pub.publish(Twist())

def main():
    global vel_pub,state
    rospy.init_node("explorer")

    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    odo_sub = rospy.Subscriber("odom", Odometry, update, queue_size=1)
    rospy.Subscriber("explorer_state", UInt16, change_state)
    rospy.spin()

if __name__ == '__main__':
    main()
