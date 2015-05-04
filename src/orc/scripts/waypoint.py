#!/usr/bin/env python2
import rospy
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from Queue import Queue
from math import atan2,sin,cos,sqrt,pi
from orc.srv import Waypoint,WaypointResponse

l = Lock()
straightness = 100
d_threshold = 0.01
a_threshold = 0.1
ka = 0.5
kd = 0.8

distance_error = lambda p1,p2: sqrt(sum((x1-x2)**2 for x1,x2 in zip(p1[:2],p2[:2])))
angle_error = lambda p1,p2: atan2(sin(p2[2]-p1[2]), cos(p2[2]-p1[2]))
_sign = lambda i: -1 if i < 0 else 1

def angleTo(f, to):
    dx = to[0]-f[0]
    dy = to[1]-f[1]
    angle = atan2(dy,dx)
    diff = angle-f[2]
    return atan2(sin(diff),cos(diff))

def odom_update(odometry):
    global q,current_wp,vel_pub

    with l:
        if current_wp is None and q.empty():
            return
        current_wp = current_wp or q.get_nowait()
        wp = current_wp

    pose = odometry.pose.pose
    quat = (pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w)
    _,_,theta = euler_from_quaternion(quat)

    robot = (pose.position.x,pose.position.y,theta)
    v,omega = 0,0
    angle_to_wp = angleTo(robot,wp)
    d_angle = angle_error(robot,wp)
    distance = distance_error(robot,wp)

    if distance > d_threshold:
        sign = _sign(cos(angle_to_wp))
        v = kd*distance*abs(cos(angle_to_wp)**straightness)*sign
        if sign == -1:
            angle_to_wp -= pi
            angle_to_wp = atan2(sin(angle_to_wp),cos(angle_to_wp))
        omega = ka*angle_to_wp
        rospy.loginfo("Distance: %f" % distance)
    elif abs(d_angle) > a_threshold and q.empty():
        current_wp = (pose.position.x,pose.position.y,theta)
        v = 0
        rospy.loginfo(d_angle)
        omega = ka*d_angle
    else:
        current_wp = None
        if q.empty():
            vel_pub.publish(Twist())
        rospy.loginfo("DONE")
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = omega
    vel_pub.publish(msg)

def new_waypoint(waypoint):
    global q,current_wp
    rospy.loginfo("New waypoint received!")
    with q.mutex,l:
        if waypoint.reset:
            print "RESET received"
            q.queue.clear()
            current_wp = None
            return WaypointResponse()
    q.put((waypoint.x,waypoint.y,waypoint.theta))
    if (waypoint.block):
        loop = rospy.Rate(30)
        while not (q.empty() and current_wp is None):
            loop.sleep()
    return WaypointResponse()

def main():
    global q,vel_pub,current_wp
    q = Queue()
    current_wp = None
    rospy.init_node("waypoint_nav")
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rospy.Subscriber("odom", Odometry, odom_update)
    rospy.Service("waypoints", Waypoint, new_waypoint)
    rospy.spin()

if __name__ == '__main__':
    main()
