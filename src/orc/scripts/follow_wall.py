#!/usr/bin/env python2
import rospy
from threading import Lock

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry,Twist
from std_msgs.msg import UInt16
import message_filters

from numpy import polyfit as pfit
from math import cos,sin,acos

SLEEPING = 0
AWAKE = 1
state = SLEEPING

points = []

def error((a,b,c), position, theta):
    distance = abs(a*position.x + b*position.y + c)
    x = cos(theta)
    y = sin(theta)
    dot = (x*(-b) + y*a)
    if dot < 0:
        a = -a
        b = -b
        dot = (x*(-b) + y*a)
    sign = 1 if (x*a+y*b) > 0 else -1
    diff = acos(dot)*sign
    return distance,diff

ANGLE_MIN = -pi/2
ANGLE_MAX = 0
EPSILON = 0.001
angle_to_index = lambda theta,s: int(len(s.ranges)/(s.angle_max-s.angle_min) * (theta - s.angle_min))
index_to_angle = lambda i,s: i * (s.angle_max - s.angle_min) / len(s.ranges) + s.angle_min

k_a = 0.2
k_d = k_a*5
desired_distance = 0.4
forward_speed = 0.05
def update(odometry, scan):
    if state == SLEEPING: # Don't do anything if we're sleeping
        return

    # Find line
    min_index = angle_to_index(ANGLE_MIN, scan)
    max_index = angle_to_index(ANGLE_MAX, scan)
    y = [scan.ranges(i)*sin(index_to_angle(i)) for i in range(min_index,max_index+1)]
    x = [scan.ranges(i)*cos(index_to_angle(i)) for i in range(min_index,max_index+1)]
    a,c = pfit(x,y,1)
    line = (a,-1,c)
    u = sum(i**2 for i in line)
    line = tuple(i/u for i in line)

    pose = odometry.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,theta = euler_from_quaternion(quat)
    distance_error,theta_error = error(line,pose.position,theta)
    theta_i = k_d * (desired_distance - distance_error)
    omega = -k_a * (theta_i - angle_error)
    v = forward_speed

    vel = Twist()
    vel.linear.x = v
    vel.angular.z = omega
    vel_pub.Publish(vel)

def change_state(new_state):
    global state, vel_pub
    state = new_state.data
    if state == SLEEPING:
        vel_pub.Publish(Twist())

def main():
    global vel_pub
    rospy.init_node("wall_follow")
    vel_pub = rospy.Published("cmd_vel", Twist, queue_size=5)
    odom_sub = message_filters.Subscriber("odom", Odometry)
    scan_sub = message_filters.Subscriber("scan", LaserScan)
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub])
    ts.registerCallback(update)
    rospy.Subscriber("wall_follow_state", UInt16, change_state)
    rospy.spin()

if __name__ == '__main__':
    main()
