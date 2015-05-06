#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from orc.srv import Collides, CollidesResponse

from tf.transformations import euler_from_quaternion
import message_filters

import random
from threading import Lock
from math import *

index_to_angle = lambda i,s: i * (s.angle_max - s.angle_min) / len(s.ranges) + s.angle_min

HISTORY_MAX_SIZE = 200
history = []
odom = None
l = Lock()

# Keep track of points/odometry
#
def update(scan,odometry):
    global history,odom
    pose = odometry.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,r_theta = euler_from_quaternion(quat)

    (rx,ry) = (pose.position.x,pose.position.y)

    valid_scan = [(scan.ranges[i], index_to_angle(i,scan)) for i in range(0,len(scan.ranges)) if scan.range_min < scan.ranges[i] < scan.range_max]

    with l:
        odom = (rx,ry,r_theta)
        history.extend([
            (rx+r*cos(theta+r_theta),ry+r*sin(theta+r_theta))
            for (r,theta) in valid_scan
        ])
        if len(history) > HISTORY_MAX_SIZE:
            history = history[len(history)-HISTORY_MAX_SIZE:]

# Does going to a waypoint cause us to collide?
#
M_PER_INCH = 0.0254 # meters/inch
ROBOT_LENGTH_INCHES = 10 # inches
ROBOT_WIDTH_INCHES = 15 # inches
ROBOT_LENGTH = ROBOT_LENGTH_INCHES*M_PER_INCH # meters
ROBOT_WIDTH = ROBOT_WIDTH_INCHES*M_PER_INCH # meters
RADIUS_SIMPLE = ((ROBOT_WIDTH/2.0)**2 + (ROBOT_LENGTH/2.0)**2)/2.0
ANGLE_DELTA = 0.1

ang = lambda theta: (theta+pi)%(2*pi) - pi
def transform((ox,oy),theta,(x,y)):
    theta = -theta
    x,y = x-ox,y-oy
    return (x*cos(theta)-y*sin(theta), x*sin(theta) + y*cos(theta))

def collides(req):
    global odom,history
    x,y,theta = req.x,req.y,req.theta
    print "Request for collision detection received"
    if odom is None:
        return CollidesResponse(False)
    with l:
        hist = history[:]
        rx,ry,r_theta = odom

    distance = sqrt((x-rx)**2 + (y-ry)**2)
    d_theta = atan2((y-ry),(x-rx))
    x_min,x_max,y_min,y_max = (-ROBOT_LENGTH/2.0,distance+ROBOT_LENGTH/2.0,-ROBOT_WIDTH/2.0,ROBOT_WIDTH/2.0)
    # Does path collide with any point?
    for x2,y2 in hist:
        # Transform into new frame
        t_x,t_y = transform((rx,ry), d_theta, (x2,y2))
        # Check if trajectory collides
        if x_min < t_x < x_max and y_min < t_y < y_max:
            print "Collides on trajectory, point: ", x2, y2, t_x, t_y
            return CollidesResponse(True)
        # Check if we can rotate at the destination
        s = 1 if ang(d_theta-r_theta) > 0 else -1
        n_angles = int(abs((d_theta-r_theta)/ANGLE_DELTA))
        for delta in range(n_angles):
            t_x,t_y = transform((rx,ry), r_theta + s*delta*ANGLE_DELTA, (x2,y2))
            if -ROBOT_LENGTH/2.0 < t_x < ROBOT_LENGTH/2.0 and -ROBOT_WIDTH/2.0 < t_y < ROBOT_WIDTH/2.0:
                print "Collides on rotation, point: ", x2, y2, t_x, t_y
                return CollidesResponse(True)
        # Check if robot can rotate
        s = 1 if ang(theta-d_theta) > 0 else -1
        n_angles = int(abs((theta-d_theta)/ANGLE_DELTA))
        for delta in range(n_angles):
            t_x,t_y = transform((x,y), d_theta + s*delta*ANGLE_DELTA, (x2,y2))
            if -ROBOT_LENGTH/2.0 < t_x < ROBOT_LENGTH/2.0 and -ROBOT_WIDTH/2.0 < t_y < ROBOT_WIDTH/2.0:
                return CollidesResponse(True)
    return CollidesResponse(False)

def main():
    rospy.init_node("local_collision")

    scan_sub = message_filters.Subscriber("scan", LaserScan, queue_size=1)
    odom_sub = message_filters.Subscriber("odom", Odometry, queue_size=1)
    ts = message_filters.ApproximateTimeSynchronizer([scan_sub,odom_sub], 1, 0.1)
    ts.registerCallback(update)

    rospy.Service("collides", Collides, collides)

    rospy.spin()

if __name__ == '__main__':
    main()
