#!/usr/bin/python2
import rospy, math
from orc.srv import OpenCloseDoor, OpenCloseDoorResponse
from orc.msg import JointSet
from sensor_msgs.msg import JointState

OPEN_ANGLE = math.pi/2.0
CLOSED_ANGLE = 0.0

EPSILON = 0.1
DOOR_NAME = "door"
door_angle = -0.1

def open_close_door(req):
    if req.state == 1:
        desired_angle = OPEN_ANGLE
    else:
        desired_angle = CLOSED_ANGLE

    loop = rospy.Rate(30)
    while abs(door_angle - desired_angle) > EPSILON:
        print door_angle,desired_angle
        joint = JointSet()
        joint.joint_name=DOOR_NAME
        joint.angle=desired_angle
        joint_state_pub.publish(joint)
        loop.sleep()
    return OpenCloseDoorResponse()

def handle_joint_msg(msg):
    global door_angle
    door_servo_index = msg.name.index(DOOR_NAME)
    door_angle = msg.position[door_servo_index]

def main():
    global joint_state_pub
    rospy.init_node('door_open_close_server')
    s = rospy.Service('open_close_door', OpenCloseDoor, open_close_door)
    joint_state_pub = rospy.Publisher("joint_set", JointSet, queue_size=10)
    rospy.Subscriber("joint_states", JointState, handle_joint_msg)
    rospy.spin()

if __name__ == '__main__':
    main()
