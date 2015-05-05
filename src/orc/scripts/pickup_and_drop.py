#!/usr/bin/env python2
import rospy
import math
from orc.msg import JointSet
from sensor_msgs.msg import JointState
from orc.srv import PickupBlock, PickupBlockResponse, OpenCloseDoor
from threading import Lock

state= "waiting"
joint = JointSet()
l = Lock()

def pickupAndDrop(req):
    global state
    print "Moving arm to state: %s" % req.state
    with l:
        if req.state not in stateAngles or state != "waiting":
            return
        state = req.state

    loop = rospy.Rate(30)
    while state != "waiting":
        loop.sleep()
    #calling door service
    if req.state == "drop":
        rospy.wait_for_service('open_close_door')
        try:
            open_close_door = rospy.ServiceProxy('open_close_door',OpenCloseDoor)
            open_close_door(2)
            open_close_door(0)
        except rospy.ServiceException, e:
            print "Door service call failed"
    return PickupBlockResponse()

def handleAngleMessage(jointState):
    global state,angleStates
    angleStates = dict(zip(jointState.name,jointState.position))
    if state == "waiting":
        return
    for (name, angle) in zip(joint_names,stateAngles[state]):
        set_angle(name,angle)
    if checkAngles():
    	# move to next state
        state = "waiting"

angleStates = {
    "base_to_shoulder" : None,
    "shoulder_to_wrist": None,
    "wrist_to_gripper" : None
}

desiredStates = {
    "base_to_shoulder" : None,
    "shoulder_to_wrist": None,
    "wrist_to_gripper" : None
}

def checkAngles():
    angleChecks = [angleStates[x] is not None and desiredStates[x] is not None and abs(desiredStates[x] - angleStates[x]) < 0.01 for x in desiredStates]
    return all(angleChecks)

def set_angle(name,angle):
    global desiredStates
    desiredStates[name] = angle
    joint.joint_name=name
    joint.angle=angle
    joint_pub_.publish(joint)

joint_names = ["base_to_shoulder", "shoulder_to_wrist", "wrist_to_gripper"]
stateAngles = {
    "findblock" : (-1.3,0.80,0.8),
    "grip"      : (-1.3,0.80,0.4),
    "pickup"    : ( 2.0,1.24,0.4),
    "drop"      : ( 2.0,1.24,0.8)
}

def main():
	global joint_pub_
	rospy.init_node('Pickup_and_drop_server')
        s = rospy.Service('PickupAndDrop', PickupBlock, pickupAndDrop)
	joint_pub_=rospy.Publisher("joint_set",JointSet, queue_size=100)
        rospy.Subscriber("joint_states",JointState, handleAngleMessage)
        rospy.spin()
if __name__ == "__main__":
    main()
