#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
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
ROBOT_LENGTH_INCHES = 10 # inches
ROBOT_WIDTH_INCHES = 15 # inches
ROBOT_LENGTH = ROBOT_LENGTH_INCHES*M_PER_INCH # meters
ROBOT_WIDTH = ROBOT_WIDTH_INCHES*M_PER_INCH # meters
RADIUS_SIMPLE = ((ROBOT_WIDTH/2.0)**2 + (ROBOT_LENGTH/2.0)**2)/2.0

ANGLE_DELTA = 0.1

forward_speed = 0.05
def update(scan,odom):
    if state == SLEEPING: # Don't do anything if we're sleeping
        return

    print "going to random waypoint"
    pose = odom.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,r_theta = euler_from_quaternion(quat)

    (rx,ry) = (pose.position.x,pose.position.y)

    valid_scan = [(scan.ranges[i], index_to_angle(i,scan)) for i in range(0,len(scan.ranges)) if scan.range_min < scan.ranges[i] < scan.range_max]
    if len(valid_scan) <= 5:
        v = forward_speed
        omega = 0
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = omega
        vel_pub.publish(vel)
    else:
        collides = True
        while collides:
            # Choose random point
            r,theta = 2.0,(random.random()*pi/2.0)-pi/4.0
            r,theta = random.choice(valid_scan)
            dest_theta = random.random()*2*pi
            alpha = random.random()
            # Does path collide with any point?
            (y_min,y_max,x_min,x_max) = (-ROBOT_WIDTH/2.0, ROBOT_WIDTH/2.0, -ROBOT_LENGTH/2.0, alpha*r+ROBOT_LENGTH/2.0)
            collides = False
            for r2,theta2 in valid_scan:
                t_theta = theta2-theta
                (x,y) = (r2*cos(t_theta),r2*sin(t_theta))

                # Check if trajectory collides
                if x_min < x < x_max and y_min < y < y_max:
                    collides = True
                    break

                # Check if we can rotate at the destination
                if -RADIUS_SIMPLE < x < RADIUS_SIMPLE and alpha*r-RADIUS_SIMPLE < y < alpha*r+RADIUS_SIMPLE:
                    collides = True
                    break

                # Check if robot can rotate
                for delta in range(int(theta/ANGLE_DELTA)):
                    t_theta = theta2-delta*ANGLE_DELTA
                    (x,y) = (r2*cos(t_theta),r2*sin(t_theta))
                    if -ROBOT_LENGTH/2.0 < x < ROBOT_LENGTH/2.0 and -ROBOT_WIDTH/2.0 < y < ROBOT_WIDTH/2.0:
                        collides = True
                        break
                if collides:
                    break

        (x,y) = (rx+alpha*r*cos(theta+r_theta),ry+alpha*r*sin(theta+r_theta))
        print (x,y),(rx,ry)
        rospy.wait_for_service("waypoints")
        waypoint_service = rospy.ServiceProxy("waypoints",Waypoint)
        waypoint_service(False,True,x,y,dest_theta)

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

    scan_sub = message_filters.Subscriber("pcl_scan", LaserScan, queue_size=1)
    odom_sub = message_filters.Subscriber("odom", Odometry, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([scan_sub,odom_sub], 1, 0.1)
    ts.registerCallback(update)

    rospy.Subscriber("explorer_state", UInt16, change_state)
    rospy.spin()

if __name__ == '__main__':
    main()
