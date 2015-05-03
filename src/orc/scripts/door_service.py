#!/usr/bin/python2
import rospy, math
from orc.srv import OpenCloseDoor, OpenCloseDoorResponse
from orc.msg import JointSet
from sensor_msgs.msg import JointState

OPEN_ANGLE = math.pi/2.0
CLOSED_ANGLE = 0.0
HALF_ANGLE = math.pi/4.0

EPSILON = 0.1
DOOR_NAME = "door"
door_angle = -0.1

state_vals = {
    0 : OPEN_ANGLE,
    1 : CLOSED_ANGLE,
    2 : HALF_ANGLE
}

def open_close_door(req):
    loop = rospy.Rate(30)
    desired_angle = state_vals[req.state]
    while abs(door_angle - desired_angle) > EPSILON:
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
