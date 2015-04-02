#!/usr/bin/env python2
import rospy
from threading import Lock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path,Odometry
from tf.transformations import euler_from_quaternion
from Queue import Queue
from threading import Lock
from math import atan2,sin,cos,sqrt,pi

l = Lock()
straightness = 1000
d_threshold = 0.01
a_threshold = 0.005
ka = -1.0
kd = 1.0

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
            vel_pub.publish(Twist())
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
    elif abs(d_angle) > a_threshold:
        v = 0
        rospy.loginfo(d_angle)
        omega = ka*d_angle
    else:
        current_wp = None
        rospy.loginfo("DONE")
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = omega
    vel_pub.publish(msg)

def new_path(path):
    global q
    rospy.loginfo("New path received!")
    with q.mutex,l:
        q.queue.clear()
        current_wp = None
    for pose in (p.pose for p in path.poses):
        quat = (pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w)
        _,_,theta = euler_from_quaternion(quat)
        rospy.loginfo(pose)
        rospy.loginfo(theta)
        q.put((pose.position.x,pose.position.y,theta))

def main():
    global q,vel_pub,current_wp
    q = Queue()
    current_wp = None
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)
    rospy.Subscriber("odom", Odometry, odom_update)
    rospy.Subscriber("waypoints", Path, new_path)
    rospy.init_node("waypoint_nav")
    rospy.spin()

if __name__ == '__main__':
    main()
