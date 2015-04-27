#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path,Odometry
from math import sqrt

_odom = (0,0,0)
d_threshold = 0.05
distance_error = lambda p1,p2: sqrt(sum((x1-x2)**2 for x1,x2 in zip(p1[:2],p2[:2])))

def odom_update(odometry):
    global p,path_pub
    pose = odometry.pose.pose
    robot = (pose.position.x,pose.position.y)
    distance = distance_error(robot,(0,1))
    if (distance < d_threshold):
        path_pub.publish(p)


def main():
    global p,path_pub
    path_pub = rospy.Publisher("waypoints", Path, queue_size=1);
    rospy.init_node("waypoint_go")

    p = Path()
    p1 = PoseStamped()
    p1.pose.position.x,p1.pose.position.y = 0,0
    p2 = PoseStamped()
    p2.pose.position.x,p2.pose.position.y = 1,0
    p3 = PoseStamped()
    p3.pose.position.x,p3.pose.position.y = 1.5,0.5
    p4 = PoseStamped()
    p4.pose.position.x,p4.pose.position.y = 1,1
    p5 = PoseStamped()
    p5.pose.position.x,p5.pose.position.y = 0,1
    p.poses = [p1,p2,p3,p4,p5]

    rospy.Subscriber("odom", Odometry, odom_update)
    path_pub.publish(p)
    rospy.spin()

if __name__ == '__main__':
    main()
