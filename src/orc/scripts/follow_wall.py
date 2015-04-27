#!/usr/bin/env python2
import rospy
from threading import Lock

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from tf.transformations import euler_from_quaternion
import message_filters

from numpy import polyfit as pfit
from math import cos,sin,acos,pi,atan

SLEEPING = 0
AWAKE = 1
state = SLEEPING

points = []

ANGLE_MIN = -pi/2
ANGLE_MAX = -pi/4
EPSILON = 0.001
angle_to_index = lambda theta,s: int(len(s.ranges)/(s.angle_max-s.angle_min) * (theta - s.angle_min))
index_to_angle = lambda i,s: i * (s.angle_max - s.angle_min) / len(s.ranges) + s.angle_min

k_a = 0.5
k_d = k_a*5
desired_distance = 0.4
forward_speed = 0.05
def update(scan):
    if state == SLEEPING: # Don't do anything if we're sleeping
        return

    # Find line
    min_index = angle_to_index(ANGLE_MIN, scan)
    max_index = angle_to_index(ANGLE_MAX, scan)
    y = [scan.ranges[i]*sin(index_to_angle(i,scan)) for i in range(min_index,max_index+1) if scan.range_min < scan.ranges[i] < scan.range_max]
    x = [scan.ranges[i]*cos(index_to_angle(i,scan)) for i in range(min_index,max_index+1) if scan.range_min < scan.ranges[i] < scan.range_max]
    if len(x) <= 5:
        return
    a,b = pfit(x,y,1)
    distance_error = -b
    theta_error = -atan(a)
    #theta_i = k_d * (desired_distance - distance_error)
    theta_i = 0
    omega = k_a * (theta_i - theta_error)
    v = forward_speed

    print "%f\t%f\t%f\t%f" % (distance_error,theta_error,theta_i,omega)
    vel = Twist()
    vel.linear.x = v
    vel.angular.z = omega
    vel_pub.publish(vel)

def change_state(new_state):
    global state, vel_pub
    state = new_state.data
    if state == SLEEPING:
        vel_pub.Publish(Twist())

def main():
    global vel_pub,state
    rospy.init_node("wall_follow")
    state=AWAKE
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    scan_sub = rospy.Subscriber("pcl_scan", LaserScan, update)
    rospy.Subscriber("wall_follow_state", UInt16, change_state)
    rospy.spin()

if __name__ == '__main__':
    main()
